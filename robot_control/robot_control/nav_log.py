"""CSV logs of one run (no ROS).

<dir>/trajectory.csv    estimated pose every dt s (report: 2D trajectory)
<dir>/steps.csv         every primitive (smoothness, total distance)
<dir>/observations.csv  every tag in every look (noise handling)
<dir>/waypoints.csv     estimate at each waypoint (waypoint errors)
<dir>/config.yaml       parameters and map used (HW2)

HW3 (SlamCsvLog, one <dir> per phase: run_<time>/square, run_<time>/octagon) adds
<dir>/landmarks.csv     every landmark in the state after every look
<dir>/map_lap<k>.yaml   map and observation counts at the end of lap k
<dir>/final_state.npz   full s, Sigma, ids at the end
"""
import csv
import math
import os
import time
from collections import Counter
from datetime import datetime

import numpy as np
import yaml

from robot_control.ekf import sigmas
from robot_control.estimator import GATE, INIT, REJECTED, SKIP_INIT, UPDATE
from robot_control.ol_primitive import wheel_cmds
from robot_control.se2 import Pose2, wrap_angle
from robot_control.stop_look import NavLog

TRAJ_DT = 0.5


def interpolate(a, b, f):
    """Pose between a and b; f in [0, 1]."""
    return Pose2(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y),
                 wrap_angle(a.theta + f * wrap_angle(b.theta - a.theta)))


class TrajectorySampler:
    """Emits the pose on a fixed time grid: held while still, interpolated while moving."""

    def __init__(self, write_row, dt=TRAJ_DT):
        self.write_row = write_row      # write_row(t, pose, sig, source)
        self.dt = dt
        self.next_t = 0.0
        self.pose = None
        self.sig = None

    def hold(self, t_end):
        while self.pose is not None and self.next_t <= t_end:
            self.write_row(self.next_t, self.pose, self.sig, 'hold')
            self.next_t += self.dt

    def set(self, t, pose, sig, source):
        """New estimate at time t (start, or after a tag update)."""
        self.hold(t)
        self.pose, self.sig = pose, sig
        self.write_row(t, pose, sig, source)

    def move(self, t0, t1, pose_after, sig_after):
        """Motion from the current pose to pose_after during [t0, t1]."""
        self.hold(t0)
        start = self.pose
        while self.next_t <= t1:
            f = (self.next_t - t0) / (t1 - t0) if t1 > t0 else 1.0
            self.write_row(self.next_t, interpolate(start, pose_after, f), sig_after, 'predict')
            self.next_t += self.dt
        self.pose, self.sig = pose_after, sig_after


def pose_cells(p):
    return [f'{p.x:.4f}', f'{p.y:.4f}', f'{math.degrees(p.theta):.2f}']


def sigma_cells(sig):
    return [f'{sig[0]:.4f}', f'{sig[1]:.4f}', f'{math.degrees(sig[2]):.2f}']


def new_run_dir(root, now=None):
    """root/run_<YYYYmmdd_HHMMSS> (not created)."""
    return os.path.join(root, f'run_{now or datetime.now():%Y%m%d_%H%M%S}')


def write_yaml(path, data):
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


class CsvNavLog(NavLog):
    """lap_index[i] = lap of goal i+1 (HW3); without it the lap column is empty."""

    def __init__(self, run_dir, start_pose, start_sig, clock=time.monotonic, lap_index=None):
        os.makedirs(run_dir, exist_ok=True)
        self.dir = run_dir
        self.clock = clock
        self.lap_index = lap_index
        self.t_ref = clock()
        self.files = []
        self.traj = self.open('trajectory.csv', ['t', 'x', 'y', 'theta_deg', 'sx', 'sy',
                                                 'stheta_deg', 'source'])
        self.steps = self.open('steps.csv', ['wp', 'lap', 'kind', 'dir', 'amount', 'cmd', 'L',
                                             'R', 'T', 't_start', 't_stop', 'x0', 'y0',
                                             'theta0_deg', 'x1', 'y1', 'theta1_deg'])
        self.obs = self.open('observations.csv', ['t', 'wp', 'lap', 'id', 'x', 'y', 'psi_deg',
                                                  'dist', 'oblique_deg', 'count', 'action',
                                                  'accepted', 'd2', 'reason'])
        self.wps = self.open('waypoints.csv', ['t', 'wp', 'lap', 'goal_x', 'goal_y',
                                               'goal_theta_deg', 'reached', 'tag_based', 'x',
                                               'y', 'theta_deg', 'sx', 'sy', 'stheta_deg',
                                               'iters', 'tags', 'note'])
        self.sampler = TrajectorySampler(self.write_traj)
        self.sampler.set(0.0, start_pose, start_sig, 'start')

    def open(self, name, header):
        f = open(os.path.join(self.dir, name), 'w', newline='')
        self.files.append(f)
        w = csv.writer(f)
        w.writerow(header)
        return w

    def now(self):
        return self.clock() - self.t_ref

    def lap(self, goal_index):
        return self.lap_index[goal_index - 1] if self.lap_index else ''

    def flush(self):
        for f in self.files:
            f.flush()

    def write_traj(self, t, pose, sig, source):
        self.traj.writerow([f'{t:.2f}', *pose_cells(pose), *sigma_cells(sig), source])

    def write_config(self, data):
        write_yaml(os.path.join(self.dir, 'config.yaml'), data)

    # ---------- NavLog hooks ----------

    def on_look(self, goal_index, records, estimator):
        t = self.now()
        for r in records:
            o = r.obs
            cells = (['', '', '', '', '', '', ''] if o is None else
                     [o.id, f'{o.x:.4f}', f'{o.y:.4f}', f'{math.degrees(o.psi):.2f}',
                      f'{o.dist:.3f}', f'{math.degrees(o.oblique):.1f}', o.count])
            self.obs.writerow([f'{t:.2f}', goal_index, self.lap(goal_index), *cells, r.action,
                               int(r.accepted), f'{r.d2:.2f}', r.reason])
        if any(r.accepted for r in records):
            self.sampler.set(t, estimator.pose(), sigmas(estimator.pose_cov()), 'update')
        self.flush()

    def on_step(self, goal_index, step, pose_before, pose_after, P, timing=None):
        L, R = wheel_cmds(step.kind, step.dir, step.cmd)
        t1 = self.now()
        t0, t1 = (timing[0] - self.t_ref, timing[1] - self.t_ref) if timing else (t1 - step.T, t1)
        self.steps.writerow([goal_index, self.lap(goal_index), step.kind, step.dir,
                             f'{step.amount:.4f}', step.cmd, f'{L:.3f}', f'{R:.3f}',
                             f'{step.T:.3f}', f'{t0:.2f}', f'{t1:.2f}',
                             *pose_cells(pose_before), *pose_cells(pose_after)])
        self.sampler.move(t0, t1, pose_after, sigmas(P))
        self.flush()

    def on_waypoint(self, result):
        g = result.goal
        self.wps.writerow([f'{self.now():.2f}', result.index, self.lap(result.index),
                           *pose_cells(g),
                           int(result.reached), int(result.tag_based), *pose_cells(result.pose),
                           *sigma_cells(result.sigmas), result.iters,
                           ' '.join(str(i) for i in result.tags_used), result.note])
        self.flush()

    def close(self):
        self.sampler.hold(self.now())
        for f in self.files:
            f.close()


def lap_stats(records):
    """Action counts and mean d2 of the accepted updates."""
    counts = Counter(r.action for r in records)
    d2 = [r.d2 for r in records if r.action == UPDATE]
    return {**{a: counts[a] for a in (UPDATE, INIT, GATE, SKIP_INIT, REJECTED)},
            'mean_d2': float(np.mean(d2)) if d2 else None}


class SlamCsvLog(CsvNavLog):
    """CsvNavLog for one HW3 phase, plus the map: landmarks.csv, map_lap<k>.yaml, final npz."""

    def __init__(self, phase_dir, estimator, lap_index, clock=time.monotonic):
        super().__init__(phase_dir, estimator.pose(), sigmas(estimator.pose_cov()), clock,
                         lap_index)
        self.estimator = estimator
        self.lms = self.open('landmarks.csv', ['t', 'lap', 'id', 'x', 'y', 'psi_deg', 'sxx',
                                               'sxy', 'syy', 'spsi_deg', 'n_updates'])
        self.records = {}       # lap -> TagRecords of its looks
        self.looks = Counter()  # lap -> number of looks

    def on_look(self, goal_index, records, estimator):
        super().on_look(goal_index, records, estimator)
        lap = self.lap(goal_index)
        self.records.setdefault(lap, []).extend(records)
        self.looks[lap] += 1
        self.write_landmarks(self.now(), lap, estimator.snapshot())
        self.flush()

    def write_landmarks(self, t, lap, snap):
        for m in snap['landmarks']:
            c = m['cov']
            self.lms.writerow([f'{t:.2f}', lap, m['id'], f'{m["x"]:.4f}', f'{m["y"]:.4f}',
                               f'{math.degrees(m["yaw"]):.2f}', f'{c[0][0]:.3e}',
                               f'{c[0][1]:.3e}', f'{c[1][1]:.3e}',
                               f'{math.degrees(math.sqrt(max(c[2][2], 0.0))):.2f}',
                               m['n_updates']])

    def on_waypoint(self, result):
        super().on_waypoint(result)
        i = result.index
        if i == len(self.lap_index) or self.lap_index[i] != self.lap_index[i - 1]:
            self.write_lap_map(self.lap(i))

    def write_lap_map(self, lap):
        stats = {'looks': self.looks[lap], **lap_stats(self.records.get(lap, []))}
        write_yaml(os.path.join(self.dir, f'map_lap{lap}.yaml'),
                   {'lap': lap, 't': round(self.now(), 2), 'stats': stats,
                    **self.estimator.snapshot()})

    def close(self):
        super().close()
        s, Sigma, ids = self.estimator.state()
        np.savez(os.path.join(self.dir, 'final_state.npz'), s=s, Sigma=Sigma,
                 ids=np.array(ids, dtype=int))


class TeeLog(NavLog):
    """Forwards every hook to several logs."""

    def __init__(self, *logs):
        self.logs = logs

    def on_look(self, *args):
        for log in self.logs:
            log.on_look(*args)

    def on_step(self, *args):
        for log in self.logs:
            log.on_step(*args)

    def on_waypoint(self, *args):
        for log in self.logs:
            log.on_waypoint(*args)
