"""HW3 EKF-SLAM run: square laps -> (reset) -> octagon laps, from an unknown start.
Needs hw2_perception_launch.py (hw3 apriltag config) and motor_control (or use hw3_launch.py).

ros2 run robot_control hw3_slam --ros-args -p square_laps:=1 -p octagon_laps:=1 [-p dry_run:=true]

The map frame of each phase is the robot pose when the phase starts. Logs go to
HW3/logs/run_<time>/{square,octagon}/; write each phase's measured start into
its start.yaml (arena frame) for report_run.py.
"""
import os
import time

import yaml
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from robot_control import ol_node
from robot_control.hw2_nav import PosePublisher, spin_in_background
from robot_control.map_eval import cov_ellipse
from robot_control.ol_model import DEFAULT_CALIB_DIR, Calibration, ModelError
from robot_control.se2 import yaw_to_quat
from robot_control.slam_run import Hw3Session, RunOptions
from robot_control.stop_look import NavigationHalt
from robot_control.tag_listener import TagListener
from robot_control.tag_obs import filter_window

SRC = '~/ros2_ws/src/CSE276A'
DEFAULT_SLAM = f'{SRC}/robot_control/config/hw3/slam.yaml'
DEFAULT_LOG_DIR = f'{SRC}/HW3/logs'
FRAME = 'map'


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def landmark_markers(est, stamp, k_sigma=2.0):
    """Arrow (normal), flat cylinder (k-sigma xy ellipse) and id text per landmark."""
    clear = Marker()
    clear.header.frame_id, clear.action = FRAME, Marker.DELETEALL
    markers = [clear]
    for lm in est.landmarks():
        cov = est.landmark_cov(lm.id)
        a, b, ang = cov_ellipse(cov[:2, :2], k_sigma)
        for j, (kind, yaw, scale, color) in enumerate((
                (Marker.ARROW, lm.yaw, (0.15, 0.03, 0.03), (0.1, 0.6, 1.0, 1.0)),
                (Marker.CYLINDER, ang, (max(2 * a, 0.01), max(2 * b, 0.01), 0.01),
                 (1.0, 0.6, 0.1, 0.5)),
                (Marker.TEXT_VIEW_FACING, 0.0, (0.0, 0.0, 0.1), (1.0, 1.0, 1.0, 1.0)))):
            m = Marker()
            m.header.frame_id, m.header.stamp = FRAME, stamp
            m.ns, m.id, m.type, m.action = 'landmarks', 3 * lm.id + j, kind, Marker.ADD
            m.pose.position = Point(x=lm.x, y=lm.y, z=0.0)
            qx, qy, qz, qw = yaw_to_quat(yaw)
            o = m.pose.orientation
            o.x, o.y, o.z, o.w = qx, qy, qz, qw
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b, m.color.a = color
            m.text = str(lm.id)
            markers.append(m)
    return markers


class SlamPublisher(PosePublisher):
    """/hw3/pose + TF map -> base_link, and /hw3/landmarks after every look."""

    def __init__(self, node):
        super().__init__(node, 'hw3/pose', FRAME)
        self.markers = node.create_publisher(MarkerArray, 'hw3/landmarks', 10)

    def on_look(self, goal_index, records, estimator):
        super().on_look(goal_index, records, estimator)
        msg = MarkerArray()
        msg.markers = landmark_markers(estimator, self.node.get_clock().now().to_msg())
        self.markers.publish(msg)


class Hw3SlamNode(ol_node.MotorNode):
    def __init__(self):
        super().__init__('hw3_slam')
        self.floor = self.param('floor', 'study_wood2_hw2_ext')
        self.paths = {
            'calib_dir': os.path.expanduser(self.param('calib_dir', DEFAULT_CALIB_DIR)),
            'slam_config': os.path.expanduser(self.param('slam_config', DEFAULT_SLAM)),
        }
        self.log_root = os.path.expanduser(self.param('log_dir', DEFAULT_LOG_DIR))
        self.dry_run = bool(self.param('dry_run', False))
        self.z_sign = int(self.param('tag_z_sign', 1))
        self.run_ = self.runner(self.float_param('settle_s', 1.0))
        self.opts = RunOptions(
            square_size=self.float_param('square_size', 1.5),
            square_laps=int(self.param('square_laps', 3)),
            octagon_laps=int(self.param('octagon_laps', 3)),
            reset=bool(self.param('reset_between_phases', True)),
            phase_pause_s=self.float_param('phase_pause_s', 30.0),
            wp_pause_s=self.float_param('wp_pause_s', 1.0))
        self.cfg = load_yaml(self.paths['slam_config'])
        self.ids = set(self.cfg['detector_ids'])
        self.calib = Calibration.for_floor(self.paths['calib_dir'], self.floor)
        self.tags = TagListener(self, self.z_sign)
        self.publisher = SlamPublisher(self)
        print(f'calibration: {self.calib.path}\nslam config: {self.paths["slam_config"]}\n'
              f'detector ids: {sorted(self.ids)} (the filter starts with no landmarks)\n'
              f'options: {self.opts}')

    def look(self):
        """Stand still for one window and return the averaged tag observations."""
        look_cfg = self.cfg['look']
        since = self.tags.now()
        time.sleep(look_cfg['window_s'])
        raw, _ = self.tags.collect(since)
        accepted, rejected = filter_window(raw, self.ids, look_cfg)
        seen = ', '.join(f'{i}@{o.dist:.2f}m' for i, o in sorted(accepted.items()))
        print(f'  look: {len(raw)} frames, tags [{seen}]'
              + (f', rejected {rejected}' if rejected else ''))
        return accepted, rejected

    def execute(self, step):
        print(f'  {step.describe()}')
        _, timing = self.run_.run_step(step.kind, step.dir, step.cmd, step.T)
        return timing

    def session(self):
        return Hw3Session(self.cfg, self.calib, self.look, self.execute, self.opts,
                          self.log_root, [self.publisher],
                          {'floor': self.floor, 'tag_z_sign': self.z_sign, **self.paths})

    def run(self):
        s = self.session()
        if self.dry_run:
            s.dry_run()
            return
        try:
            s.run()
            print('done')
        finally:
            if s.run_dir:
                print(f'log: {s.run_dir}')


def main(args=None):
    ol_node.init(args)
    node, executor = None, None
    try:
        node = Hw3SlamNode()
        executor = spin_in_background(node)
        node.run()
    except NavigationHalt as e:
        print(f'HALT: {e}')
    except (ModelError, ValueError, KeyError, OSError) as e:
        print(f'ERROR: {e}')
    except KeyboardInterrupt:
        print('\nAborted.')
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        ol_node.shutdown(node)
    return 0


if __name__ == '__main__':
    main()
