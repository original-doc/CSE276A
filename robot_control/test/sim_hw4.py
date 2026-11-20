"""HW4 closed loop in simulation: PlanSession + SimRover in the 12-tag arena with the box.

measured_noise() = HW3 run errors (drives +20 %, veer left 5 deg/m, coarse spins -14 %,
tag noise = the filter's R model).
"""
import math
import os
from dataclasses import dataclass

import numpy as np
import yaml
from sim_hw3 import SimClock
from sim_rover import make_calib, SimNoise, SimRover

from robot_control.plan_run import PlanSession, RunOptions
from robot_control.stop_look import NavigationHalt, NavLog
from robot_control.world import parse_world, segment_clearance

HERE = os.path.dirname(__file__)
NAV_YAML = os.path.join(HERE, '..', 'config', 'hw4', 'nav.yaml')
WORLD_YAML = os.path.join(HERE, '..', 'config', 'hw4', 'world.yaml')


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def measured_noise():
    o = load_yaml(NAV_YAML)['ekf']['obs']
    return SimNoise(drive_scale=1.20, drift=math.radians(5.0), spin_scale=0.86,
                    obs_xy=o['a'], obs_xy_b=o['b'], obs_psi=math.radians(o['c_deg']),
                    obs_psi_e=math.radians(o['e_deg']))


class TagsPerLook(NavLog):
    def __init__(self):
        self.counts = []

    def on_look(self, goal_index, records, estimator):
        self.counts.append(sum(r.obs is not None for r in records))


@dataclass
class Hw4Outcome:
    kind: str
    seed: int
    halted: str             # '' or the NavigationHalt message
    summary: object         # RunSummary (None if halted)
    end_error: float        # true end position to G (m)
    min_clearance: float    # true path to walls / box (m)
    tags_per_look: list
    drive_m: float          # true distance driven


def true_min_clearance(trail, world):
    pts = [(p.x, p.y) for p in trail]
    if len(pts) == 1:
        return segment_clearance(pts[0], pts[0], world)
    return min(segment_clearance(a, b, world) for a, b in zip(pts, pts[1:]))


def run_hw4(tmp_path, kind, seed, noise=None, log_root=None, cfg=None):
    cfg = cfg or load_yaml(NAV_YAML)
    world_data = load_yaml(WORLD_YAML)
    world = parse_world(world_data)
    rover = SimRover(world.landmarks, world.start, np.random.default_rng(seed),
                     noise or measured_noise(), occluders=world.obstacles)
    clock = SimClock()
    tags = TagsPerLook()

    def look():
        clock.t += 1.0
        return rover.look()

    def execute(step):
        t0 = clock.t
        clock.t += step.T
        rover.execute(step)
        return t0, clock.t

    session = PlanSession(cfg, world_data, kind, make_calib(tmp_path), look, execute,
                          str(log_root or tmp_path / 'logs'), RunOptions(countdown_s=0),
                          extra_logs=[tags], run_config={'sim': {'seed': seed}},
                          sleep=lambda s: None, out=lambda s: None, clock=clock)
    halted, summary = '', None
    try:
        summary = session.run()
    except NavigationHalt as e:
        halted = str(e)
    end = rover.true
    drive_m = 0.0
    for a, b in zip(rover.trail, rover.trail[1:]):
        drive_m += math.hypot(b.x - a.x, b.y - a.y)
    return Hw4Outcome(kind, seed, halted, summary, math.dist((end.x, end.y), world.goal),
                      true_min_clearance(rover.trail, world), tags.counts, drive_m)
