"""Replay a real HW2 run through the EKF-SLAM estimator, without the map (Mac, no ROS).

    /opt/anaconda3/bin/python HW3/replay_hw2.py [HW2/logs/run_20261024_055456]
        [--slam-config robot_control/config/hw3/slam.yaml] [--init-max-dist 2.5] [--out f.md]

Feeds steps.csv (nominal moves) and observations.csv (every tag with values) in
time order to SlamEstimator, then compares the tags it mapped with the
hand-measured HW2 map. The HW2 start is the world origin, so no transform.
"""
import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'robot_control'))

from robot_control import map_eval  # noqa: E402
from robot_control.ekf import noise_from_config  # noqa: E402
from robot_control.estimator import SlamEstimator, init_limits_from_config  # noqa: E402
from robot_control.landmark_map import load_map  # noqa: E402
from robot_control.stop_look import Move  # noqa: E402
from robot_control.tag_obs import make_obs  # noqa: E402

DEFAULT_RUN = ROOT / 'HW2' / 'logs' / 'run_20261024_055456'
DEFAULT_SLAM = ROOT / 'robot_control' / 'config' / 'hw3' / 'slam.yaml'
DEFAULT_MAP = ROOT / 'robot_control' / 'config' / 'hw2' / 'landmarks.yaml'


@dataclass
class Event:
    t: float
    wp: int
    kind: str           # 'look' | 'step'
    data: object        # {id: TagObs} or Move


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def step_move(row):
    """steps.csv row -> nominal Move (SI, ccw > 0)."""
    amount = float(row['amount'])
    if row['kind'] == 'drive':
        return Move('drive', amount)
    return Move('spin', math.radians(amount) * (1.0 if row['dir'] == 'ccw' else -1.0))


def look_events(rows):
    """observations.csv rows -> one look Event per time stamp (rows without values dropped)."""
    looks = {}
    for r in rows:
        if not r['id']:
            continue
        t = float(r['t'])
        obs = make_obs(int(r['id']), t, float(r['x']), float(r['y']),
                       math.radians(float(r['psi_deg'])), int(r['count']))
        looks.setdefault((t, int(r['wp'])), {})[obs.id] = obs
    return [Event(t, wp, 'look', obs) for (t, wp), obs in looks.items()]


def load_events(run_dir):
    """Looks and steps in time order; a look logged at a step's start came first."""
    steps = [Event(float(r['t_start']), int(r['wp']), 'step', step_move(r))
             for r in read_csv(os.path.join(run_dir, 'steps.csv'))]
    looks = look_events(read_csv(os.path.join(run_dir, 'observations.csv')))
    return sorted(looks + steps, key=lambda e: (e.t, e.kind != 'look'))


def replay(events, est):
    """Run the events through est -> [(event, records (None for a step), pose after)]."""
    out = []
    for e in events:
        records = None
        if e.kind == 'step':
            est.predict(e.data)
        else:
            records = est.update(e.data)
        out.append((e, records, est.pose()))
    return out


def pose_at(history, t):
    """Estimate after the last event at or before t."""
    pose = None
    for e, _, p in history:
        if e.t > t + 1e-6:
            break
        pose = p
    return pose


def make_estimator(slam_cfg, init_max_dist=None):
    limits = init_limits_from_config(slam_cfg['slam'])
    if init_max_dist is not None:
        limits = type(limits)(init_max_dist, limits.max_oblique)
    return SlamEstimator(noise_from_config(slam_cfg['ekf']), limits)


def fmt_pose(p):
    return f'({p.x:+.3f}, {p.y:+.3f}, {math.degrees(p.theta):+.1f}°)'


def look_table(looks):
    lines = ['| t (s) | wp | tag | dist (m) | action | d² | pose after look |',
             '|---|---|---|---|---|---|---|']
    for e, records, pose in looks:
        for r in records or []:
            d2 = f'{r.d2:.2f}' if r.action in ('update', 'gate') else ''
            lines.append(f'| {e.t:.2f} | {e.wp} | {r.obs.id} | {r.obs.dist:.3f} | {r.action} '
                         f'{r.reason} | {d2} | {fmt_pose(pose)} |')
    return lines


def waypoint_table(looks, hw2_wps):
    """Replay estimate at each HW2 waypoint time vs what HW2 (known map) estimated."""
    lines = ['| wp | HW2 estimate (map known) | SLAM replay (no map) | difference |',
             '|---|---|---|---|']
    for w in hw2_wps:
        hw2 = map_eval.Pose2(float(w['x']), float(w['y']), math.radians(float(w['theta_deg'])))
        s = pose_at(looks, float(w['t']))
        lines.append(f'| {w["wp"]} | {fmt_pose(hw2)} | {fmt_pose(s)} | '
                     f'{math.hypot(s.x - hw2.x, s.y - hw2.y) * 100:.1f} cm, '
                     f'{math.degrees(map_eval.wrap_angle(s.theta - hw2.theta)):+.1f}° |')
    return lines


def landmark_table(est, truth):
    snap = est.snapshot()
    me = map_eval.landmark_errors(map_eval.snapshot_poses(snap), truth)
    sig = {m['id']: m for m in snap['landmarks']}
    lines = ['| tag | SLAM estimate (x, y, ψ) | ± σ (x, y, ψ) | HW2 map | error | updates |',
             '|---|---|---|---|---|---|']
    for e in me.errors:
        m, t = sig[e.id], truth[e.id]
        c = m['cov']
        lines.append(
            f'| {e.id} | ({m["x"]:+.3f}, {m["y"]:+.3f}, {math.degrees(m["yaw"]):+.1f}°) | '
            f'({math.sqrt(c[0][0]) * 100:.1f} cm, {math.sqrt(c[1][1]) * 100:.1f} cm, '
            f'{math.degrees(math.sqrt(c[2][2])):.1f}°) | {fmt_pose(t)} | '
            f'dx {e.dx * 100:+.1f}, dy {e.dy * 100:+.1f} cm ({e.pos * 100:.1f} cm), '
            f'dψ {math.degrees(e.yaw):+.1f}° | {m["n_updates"]} |')
    s = mean_line('mean error', me)
    if len(me.errors) >= 2:
        aligned, T = map_eval.align(map_eval.snapshot_poses(snap), truth)
        s += [mean_line(f'after best rigid fit (T = {fmt_pose(T)})',
                        map_eval.landmark_errors(aligned, truth))[0]]
    if me.missing:
        s.append(f'- never mapped: {me.missing}')
    return lines + [''] + s


def mean_line(label, me):
    m = map_eval.mean_error(me)
    return [f'- {label}: mean {m["mean"] * 100:.1f} cm, rms {m["rms"] * 100:.1f} cm, '
            f'max {m["max"] * 100:.1f} cm, mean |ψ| {math.degrees(m["mean_yaw"]):.1f}°']


def report(run_dir, slam_cfg, truth, init_max_dist=None):
    est = make_estimator(slam_cfg, init_max_dist)
    history = replay(load_events(run_dir), est)
    limits = est.limits
    head = [f'## {os.path.basename(os.path.normpath(run_dir))}: init_max_dist '
            f'{limits.max_dist:.2f} m, init_max_oblique {math.degrees(limits.max_oblique):.0f}°',
            '']
    return '\n'.join(head + ['### Landmarks', ''] + landmark_table(est, truth)
                     + ['', '### Waypoints', '']
                     + waypoint_table(history, read_csv(os.path.join(run_dir, 'waypoints.csv')))
                     + ['', '### Every look', ''] + look_table(history))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dir', nargs='?', default=str(DEFAULT_RUN))
    ap.add_argument('--slam-config', default=str(DEFAULT_SLAM))
    ap.add_argument('--map', default=str(DEFAULT_MAP))
    ap.add_argument('--init-max-dist', type=float)
    ap.add_argument('--out')
    a = ap.parse_args()
    with open(a.slam_config) as f:
        cfg = yaml.safe_load(f)
    text = report(a.run_dir, cfg, map_eval.truth_poses(load_map(a.map)), a.init_max_dist)
    print(text)
    if a.out:
        Path(a.out).write_text(text + '\n')


if __name__ == '__main__':
    main()
