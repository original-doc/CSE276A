"""HW4 path planning run: plan (safe = GVD, fast = visibility graph), then drive it.
Needs perception (hw4 apriltag config) and motor_control (or use hw4_launch.py).

ros2 run robot_control hw4_nav --ros-args -p planner:=safe [-p dry_run:=true]

World (arena, box, tags, start, goal) in config/hw4/world.yaml: measure the start
before every run. Logs go to HW4/logs/run_<time>_<planner>/.
"""
import os
import time

import yaml
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

from robot_control import ol_node
from robot_control.hw2_nav import PosePublisher, spin_in_background
from robot_control.ol_model import Calibration, DEFAULT_CALIB_DIR, ModelError
from robot_control.path_utils import densify
from robot_control.plan_run import PlanSession, PLANNERS, RunOptions
from robot_control.se2 import yaw_to_quat
from robot_control.stop_look import NavigationHalt
from robot_control.tag_listener import TagListener
from robot_control.tag_obs import filter_window

SRC = '~/ros2_ws/src/CSE276A'
DEFAULT_WORLD = f'{SRC}/robot_control/config/hw4/world.yaml'
DEFAULT_NAV = f'{SRC}/robot_control/config/hw4/nav.yaml'
DEFAULT_LOG_DIR = f'{SRC}/HW4/logs'
FRAME = 'world'
COLORS = {'safe': (0.16, 0.47, 0.84), 'fast': (0.92, 0.41, 0.20)}


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def marker(ns, mid, kind, stamp, color, scale):
    m = Marker()
    m.header.frame_id, m.header.stamp = FRAME, stamp
    m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def box_marker(ns, mid, rect, height, stamp, color):
    m = marker(ns, mid, Marker.CUBE, stamp, color, (rect.w, rect.h, height))
    m.pose.position = Point(x=rect.cx, y=rect.cy, z=height / 2)
    return m


def plan_markers(world, plan, wp_spacing, stamp):
    """Arena outline, boxes, inflated boxes, tags, planned polyline and waypoints."""
    clear = Marker()
    clear.header.frame_id, clear.action = FRAME, Marker.DELETEALL
    out = [clear]
    a = world.arena
    outline = marker('arena', 0, Marker.LINE_STRIP, stamp, (1.0, 1.0, 1.0, 1.0),
                     (0.01, 0.0, 0.0))
    outline.points = [Point(x=x, y=y, z=0.0) for x, y in a.corners() + a.corners()[:1]]
    out.append(outline)
    r = plan.stats['r']
    for k, o in enumerate(world.obstacles):
        out.append(box_marker('obstacles', k, o, 0.3, stamp, (0.32, 0.32, 0.31, 1.0)))
        out.append(box_marker('inflated', k, o.inflate(r), 0.01, stamp, (0.96, 0.77, 0.68, 0.4)))
    for lm in world.landmarks.landmarks.values():
        m = marker('tags', lm.id, Marker.ARROW, stamp, (0.1, 0.6, 1.0, 1.0), (0.15, 0.03, 0.03))
        m.pose.position = Point(x=lm.x, y=lm.y, z=0.12)
        q = m.pose.orientation
        q.x, q.y, q.z, q.w = yaw_to_quat(lm.yaw)
        out.append(m)
    color = (*COLORS[plan.kind], 1.0)
    path = marker('plan', 0, Marker.LINE_STRIP, stamp, color, (0.03, 0.0, 0.0))
    path.points = [Point(x=x, y=y, z=0.02) for x, y in plan.points]
    wps = marker('waypoints', 0, Marker.SPHERE_LIST, stamp, color, (0.06, 0.06, 0.06))
    wps.points = [Point(x=x, y=y, z=0.02) for x, y in densify(plan.points, wp_spacing)[1:]]
    return out + [path, wps]


class PlanPublisher(PosePublisher):
    """/hw4/pose + TF world -> base_link; /hw4/plan (latched) after every (re)plan."""

    def __init__(self, node, world, wp_spacing):
        super().__init__(node, 'hw4/pose', FRAME)
        self.world = world
        self.wp_spacing = wp_spacing
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers = node.create_publisher(MarkerArray, 'hw4/plan', qos)

    def on_plan(self, plan):
        msg = MarkerArray()
        msg.markers = plan_markers(self.world, plan, self.wp_spacing,
                                   self.node.get_clock().now().to_msg())
        self.markers.publish(msg)


class Hw4NavNode(ol_node.MotorNode):
    def __init__(self):
        super().__init__('hw4_nav')
        self.planner = self.param('planner', '')
        if self.planner not in PLANNERS:
            raise ValueError(f'set -p planner:=safe or planner:=fast (got {self.planner!r})')
        self.floor = self.param('floor', 'study_wood2_hw2_ext')
        self.paths = {
            'calib_dir': os.path.expanduser(self.param('calib_dir', DEFAULT_CALIB_DIR)),
            'world': os.path.expanduser(self.param('world', DEFAULT_WORLD)),
            'nav_config': os.path.expanduser(self.param('nav_config', DEFAULT_NAV)),
        }
        self.log_root = os.path.expanduser(self.param('log_dir', DEFAULT_LOG_DIR))
        self.dry_run = bool(self.param('dry_run', False))
        self.z_sign = int(self.param('tag_z_sign', 1))
        self.run_ = self.runner(self.float_param('settle_s', 1.0))
        self.opts = RunOptions(wp_pause_s=self.float_param('wp_pause_s', 1.0))
        self.cfg = load_yaml(self.paths['nav_config'])
        self.world_data = load_yaml(self.paths['world'])
        self.ids = set(self.cfg['detector_ids'])
        self.calib = Calibration.for_floor(self.paths['calib_dir'], self.floor)
        self.tags = TagListener(self, self.z_sign)
        run_config = {'floor': self.floor, 'tag_z_sign': self.z_sign, **self.paths}
        self.sess = PlanSession(
            self.cfg, self.world_data, self.planner, self.calib, self.look, self.execute,
            self.log_root, self.opts, run_config=run_config, on_plan=self.on_plan)
        self.publisher = PlanPublisher(self, self.sess.world, self.sess.ex.wp_spacing)
        self.sess.extra_logs.append(self.publisher)
        print(f'planner: {self.planner}\ncalibration: {self.calib.path}\n'
              f'world: {self.paths["world"]}\nnav config: {self.paths["nav_config"]}\n'
              f'start: {self.sess.world.start}  goal: {self.sess.world.goal}')

    def on_plan(self, plan):
        self.publisher.on_plan(plan)

    def look(self):
        """Stand still for one window and return the averaged tag observations."""
        look_cfg = self.cfg['look']
        since = self.tags.now()
        time.sleep(look_cfg['window_s'])
        raw, _ = self.tags.collect(since)
        accepted, rejected = filter_window(raw, self.ids, look_cfg)
        seen = ', '.join(f'{i}@{o.dist:.2f}m' for i, o in sorted(accepted.items()))
        self.sess.say(f'  look: {len(raw)} frames, tags [{seen}]'
                      + (f', rejected {rejected}' if rejected else ''))
        return accepted, rejected

    def execute(self, step):
        self.sess.say(f'  {step.describe()}')
        _, timing = self.run_.run_step(step.kind, step.dir, step.cmd, step.T)
        return timing

    def run(self):
        if self.dry_run:
            self.sess.dry_run()
            return
        try:
            self.sess.run()
        except NavigationHalt:
            pass                # already printed and saved in console.txt
        finally:
            if self.sess.run_dir:
                print(f'log: {self.sess.run_dir}')


def main(args=None):
    ol_node.init(args)
    node, executor = None, None
    try:
        node = Hw4NavNode()
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
