"""HW3 simulation: 3 m x 3 m arena with 8 tags, full square -> octagon run.

The rover moves in the arena frame; each SLAM phase only sees its own start
frame. The truth (arena tags, phase start poses) is used only to score.
"""
import math
import os
from collections import Counter
from dataclasses import dataclass, field

import numpy as np
import yaml
from sim_rover import SimNoise, SimRover, make_calib

from robot_control import map_eval
from robot_control.ekf import noise_from_config
from robot_control.estimator import SlamEstimator, init_limits_from_config
from robot_control.landmark_map import Landmark, LandmarkMap
from robot_control.nav_log import SlamCsvLog, TeeLog, new_run_dir, write_yaml
from robot_control.paths import corner_offset, inscribed_octagon_side, make_phases
from robot_control.se2 import Pose2
from robot_control.slam_run import LapEnd, run_goals, run_phases
from robot_control.stop_look import Navigator, NavLog, params_from_config

HERE = os.path.dirname(__file__)
SLAM_YAML = os.path.join(HERE, '..', 'config', 'hw3', 'slam.yaml')
ARENA = 3.0
# id, x, y, outward normal (deg), arena frame (spec 2.2)
TAGS = [(0, 1, 0, 90), (1, 2, 0, 90), (2, 3, 1, 180), (3, 3, 2, 180),
        (4, 2, 3, -90), (5, 1, 3, -90), (6, 0, 2, 0), (7, 0, 1, 0)]


def load_config():
    with open(SLAM_YAML) as f:
        return yaml.safe_load(f)


def arena_map():
    return LandmarkMap(0.166, {i: Landmark(i, float(x), float(y), math.radians(yaw))
                               for i, x, y, yaw in TAGS})


def centred_start(n):
    """Start P that centres the n x n square (and the octagon) in the arena, heading +x."""
    margin = (ARENA - n) / 2
    return Pose2(margin + corner_offset(n, inscribed_octagon_side(n)), margin, 0.0)


class SimClock:
    """Fake time: 1 s per look, T + 1 s settle per step."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


@dataclass
class PhaseOutcome:
    name: str
    start: Pose2                        # true phase start in the arena
    results: list
    laps: dict = field(default_factory=dict)    # lap -> estimator.snapshot()
    final: dict = None
    steps: int = 0
    drive_m: float = 0.0
    actions: Counter = field(default_factory=Counter)     # TagRecord.action counts


class ActionCounter(NavLog):
    def __init__(self, counter):
        self.counter = counter

    def on_look(self, goal_index, records, estimator):
        self.counter.update(r.action for r in records)


def model_noise():
    """Sim tag noise equal to the filter's R model (slam.yaml obs: a, b, c, e)."""
    o = load_config()['ekf']['obs']
    return SimNoise(obs_xy=o['a'], obs_xy_b=o['b'], obs_psi=math.radians(o['c_deg']),
                    obs_psi_e=math.radians(o['e_deg']))


def landmark_errors(snapshot, start, truth):
    """truth {id: Landmark} -> id -> (position error m, |yaw error| rad), in the arena."""
    est = map_eval.poses_to_arena(start, map_eval.snapshot_poses(snapshot))
    me = map_eval.landmark_errors(est, {i: lm.pose() for i, lm in truth.items()})
    return {e.id: (e.pos, abs(e.yaw)) for e in me.errors}


def mean_sigma(snapshot):
    """Mean over landmarks of sqrt(Sxx + Syy) (m)."""
    return float(np.mean([math.sqrt(m['cov'][0][0] + m['cov'][1][1])
                          for m in snapshot['landmarks']]))


def mean_error(snapshot, start, truth):
    return float(np.mean([e for e, _ in landmark_errors(snapshot, start, truth).values()]))


def run_hw3(tmp_path, seed, n=1.5, square_laps=3, octagon_laps=3, reset=True,
            log_root=None, noise=None):
    """Full run in simulation -> list[PhaseOutcome]. log_root: also write SlamCsvLog dirs."""
    cfg = load_config()
    params = params_from_config(cfg)
    ekf_noise, limits = noise_from_config(cfg['ekf']), init_limits_from_config(cfg['slam'])
    truth = arena_map()
    rover = SimRover(truth, centred_start(n), np.random.default_rng(seed), noise)
    calib = make_calib(tmp_path)
    clock = SimClock()
    run_dir = new_run_dir(log_root) if log_root else None
    outcomes, starts = [], []

    def look():
        clock.t += 1.0
        return rover.look()

    def execute(step):
        t0 = clock.t
        clock.t += step.T
        rover.execute(step)
        t1 = clock.t
        clock.t += 1.0
        return t0, t1

    def new_estimator():
        starts.append(rover.true)
        return SlamEstimator(ekf_noise, limits)

    def run_phase(phase, est):
        out = PhaseOutcome(phase.name, starts[-1], [])
        lap_end = LapEnd(phase.lap_index, est,
                         lambda lap, e: out.laps.__setitem__(lap, e.snapshot()))
        log = NavLog()
        if run_dir:
            log = SlamCsvLog(os.path.join(run_dir, phase.name), est, phase.lap_index, clock)
            write_start(log.dir, out.start)
        n_steps = len(rover.steps)
        nav = Navigator(est, calib, params, look, execute,
                        TeeLog(log, lap_end, ActionCounter(out.actions)))
        try:
            out.results = run_goals(nav, phase.goals)
        finally:
            if run_dir:
                log.close()
        out.final, out.steps = est.snapshot(), len(rover.steps) - n_steps
        out.drive_m = sum(s.amount for s in rover.steps[n_steps:] if s.kind == 'drive')
        outcomes.append(out)
        return out.results

    if run_dir:
        os.makedirs(run_dir, exist_ok=True)
        write_yaml(os.path.join(run_dir, 'config.yaml'),
                   {'sim': {'seed': seed, 'n': n}, 'slam': cfg})
    run_phases(make_phases(n, square_laps, octagon_laps), new_estimator, run_phase, reset)
    return outcomes, truth, run_dir


def write_start(phase_dir, start):
    """start.yaml as the user would write it after measuring (here: the sim truth)."""
    write_yaml(os.path.join(phase_dir, 'start.yaml'),
               {'x': round(start.x, 4), 'y': round(start.y, 4),
                'theta_deg': round(math.degrees(start.theta), 2)})
