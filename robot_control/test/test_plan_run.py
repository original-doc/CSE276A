import math
import os

import numpy as np
import pytest
import yaml
from sim_hw3 import SimClock
from sim_rover import SimRover, make_calib

from robot_control.ekf import noise_from_config
from robot_control.estimator import LocalizationEstimator
from robot_control.plan_run import (
    drive_guard, ExecOptions, face_tag_heading, goal_params, plan_from, plan_goals,
    PlanSession, RunOptions)
from robot_control.se2 import Pose2
from robot_control.stop_look import Move, NavigationHalt, Navigator, params_from_config, Replan
from robot_control.world import parse_world

HERE = os.path.dirname(__file__)
NAV_YAML = os.path.join(HERE, '..', 'config', 'hw4', 'nav.yaml')
WORLD_YAML = os.path.join(HERE, '..', 'config', 'hw4', 'world.yaml')


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


CFG, WORLD_DATA = load(NAV_YAML), load(WORLD_YAML)
WORLD = parse_world(WORLD_DATA)
EX = ExecOptions.from_config(CFG['exec'])
OPTS = RunOptions(countdown_s=2, wp_pause_s=0.5, dry_looks=2)


class Rig:
    """Sim rover + fake clock / sleep / print for a PlanSession."""

    def __init__(self, tmp_path, kind, guard=None, seed=0):
        self.rover = SimRover(WORLD.landmarks, WORLD.start, np.random.default_rng(seed))
        self.clock, self.lines, self.executed, self.pos_tols = SimClock(), [], [], []
        self.plans = []
        self.session = PlanSession(
            CFG, WORLD_DATA, kind, make_calib(tmp_path), self.look, self.execute,
            str(tmp_path / 'logs'), OPTS, run_config={'floor': 'sim'},
            on_plan=self.plans.append, guard=guard, sleep=self.sleep,
            out=self.lines.append, clock=self.clock)

    def look(self):
        self.clock.t += 1.0
        nav = self.session.nav
        if nav is not None:
            self.pos_tols.append((nav.p.pos_tol, nav.p.ang_tol))
        return self.rover.look()

    def execute(self, step):
        self.executed.append(step)
        t0 = self.clock.t
        self.clock.t += step.T
        self.rover.execute(step)
        return t0, self.clock.t

    def sleep(self, s):
        self.clock.t += s


# ---------- guard ----------

def test_drive_guard():
    guard = drive_guard(WORLD, WORLD.r_guard, overshoot=0.25)
    toward_box = Pose2(1.5, 0.6, math.pi / 2)            # box face at y = 1.325
    assert guard(toward_box, Move('drive', 0.3))         # ends 0.425 m short (x1.25: 0.35)
    # 0.5 m plan stops 0.225 m short, but 25 % overshoot reaches 0.1 m < r_guard
    assert not guard(toward_box, Move('drive', 0.5))
    assert not guard(Pose2(0.5, 0.5, math.pi), Move('drive', 0.4))   # into the left wall


def _noise():
    return noise_from_config(CFG['ekf'])


def test_guard_none_keeps_old_behaviour(tmp_path):
    # Without a guard the same refused drive is executed.
    calib = make_calib(tmp_path)
    est = LocalizationEstimator(WORLD.landmarks, Pose2(0.5, 0.5, 0.0), _noise())
    executed = []
    nav = Navigator(est, calib, params_from_config(CFG), lambda: ({}, {}), executed.append)
    assert nav.act(1, Move('drive', 0.3)) is not None and len(executed) == 1
    nav.guard = lambda pose, move: False
    nav.act(1, Move('spin', 0.5))                        # spins are never checked
    assert len(executed) == 2
    with pytest.raises(Replan):
        nav.act(1, Move('drive', 0.3))
    assert len(executed) == 2


# ---------- planning for (re)plans ----------

def test_goal_params():
    p = params_from_config(CFG)
    mid, last = goal_params(p, EX, False), goal_params(p, EX, True)
    assert (mid.pos_tol, mid.ang_tol) == pytest.approx((0.08, math.radians(15)))
    assert mid.near_dist == p.near_dist > 0              # pass a waypoint without turning back
    assert last.pos_tol == pytest.approx(0.05) and last.ang_tol == pytest.approx(math.radians(15))
    assert last.near_dist == 0.0                         # an overshot goal is driven back to
    assert mid.max_drive == p.max_drive                  # everything else unchanged
    no_face = goal_params(p, ExecOptions.from_config({**CFG['exec'], 'goal_face_tag': False}),
                          True)
    assert no_face.ang_tol == pytest.approx(math.pi)


def test_face_tag_heading():
    # From G the nearest tags are 1 (2, 0) and 2 (3, 1), both 0.707 m away.
    h = face_tag_heading(WORLD, WORLD.goal, math.radians(60), 0.25)
    assert math.degrees(h) == pytest.approx(-135.0) or math.degrees(h) == pytest.approx(45.0)
    # Left of the box the nearest tag is 11 (its face); tag 9 behind the box is hidden.
    h = face_tag_heading(WORLD, (1.0, 1.5), math.radians(60), 0.25)
    assert h == pytest.approx(0.0)
    # Tag 11 (0.325 m) closer than min_dist: the wall tags 6 / 7 (1.118 m) instead.
    h = face_tag_heading(WORLD, (1.0, 1.5), math.radians(60), 0.4)
    assert abs(math.degrees(h)) == pytest.approx(153.43, abs=0.01)
    plan, _ = plan_from(WORLD, 'fast', CFG['plan'], (0.5, 2.5))
    goals = plan_goals(plan, EX, goal_heading=1.0)
    assert goals[-1].theta == 1.0 and goals[-2].theta != 1.0


@pytest.mark.parametrize('kind', ['safe', 'fast'])
def test_plan_from_falls_back_to_r_guard(kind):
    plan, used = plan_from(WORLD, kind, CFG['plan'], (0.5, 2.5))
    assert not used and plan.stats['r'] == pytest.approx(WORLD.r_inflate)
    start = (1.5, 1.15)                                  # 0.175 m below the box
    plan, used = plan_from(WORLD, kind, CFG['plan'], start)
    assert used and plan.stats['r'] == pytest.approx(WORLD.r_guard)
    assert plan.points[0] == start
    with pytest.raises(NavigationHalt):
        plan_from(WORLD, kind, CFG['plan'], (1.5, 1.2))  # 0.125 m < r_guard


# ---------- full session ----------

@pytest.mark.parametrize('kind', ['safe', 'fast'])
def test_run_reaches_goal_and_logs(tmp_path, kind):
    rig = Rig(tmp_path, kind)
    s = rig.session.run()
    assert s.run_dir.endswith(f'_{kind}')
    files = set(os.listdir(s.run_dir))
    assert {'config.yaml', 'console.txt', 'plan.yaml', 'plan.npz', 'trajectory.csv',
            'steps.csv', 'observations.csv', 'waypoints.csv', 'summary.yaml'} <= files
    summary = load(os.path.join(s.run_dir, 'summary.yaml'))
    assert summary['counts'] == {k: s.counts.get(k, 0) for k in ('drive', 'spin', 'look')}
    assert summary['goal_reached'] and summary['replans'] == 0
    assert s.replans == 0 and all(r.reached for r in s.results)
    assert math.dist((rig.rover.true.x, rig.rover.true.y), WORLD.goal) < 0.1
    rec = load(os.path.join(s.run_dir, 'plan.yaml'))
    assert rec['planner'] == kind and rec['replans'] == [] and not rec['used_r_guard']
    assert len(rec['waypoints']) == len(s.results)
    assert rec['waypoints'][-1][:2] == pytest.approx(list(WORLD.goal))
    npz = np.load(os.path.join(s.run_dir, 'plan.npz'))
    assert ('gvd' in npz) == (kind == 'safe') and ('vertices' in npz) == (kind == 'fast')
    cfg = load(os.path.join(s.run_dir, 'config.yaml'))
    assert cfg['planner'] == kind and cfg['world'] == WORLD_DATA
    assert len(rig.plans) == 1


def test_waypoint_tolerances(tmp_path):
    rig = Rig(tmp_path, 'fast')
    s = rig.session.run()
    mid = (pytest.approx(0.08), pytest.approx(math.radians(15)))
    goal = (pytest.approx(0.05), pytest.approx(math.radians(15)))
    assert mid in rig.pos_tols and goal in rig.pos_tols
    assert rig.pos_tols[-1] == goal
    last = s.results[-1]
    assert math.dist((last.pose.x, last.pose.y), WORLD.goal) <= 0.05


class RefuseFirst:
    """Guard that refuses the first n drives (forces replans)."""

    def __init__(self, n):
        self.n = n
        self.poses = []

    def __call__(self, pose, move):
        if self.n > 0:
            self.n -= 1
            self.poses.append(pose)
            return False
        return True


@pytest.mark.parametrize('kind', ['safe', 'fast'])
def test_replan_from_current_pose(tmp_path, kind):
    guard = RefuseFirst(1)
    rig = Rig(tmp_path, kind, guard=guard)
    s = rig.session.run()
    assert s.replans == 1 and len(s.plans) == 2
    refused = guard.poses[0]
    assert s.plans[1].points[0] == pytest.approx((refused.x, refused.y))
    rec = load(os.path.join(s.run_dir, 'plan.yaml'))
    assert len(rec['replans']) == 1
    assert rec['replans'][0]['pose'][:2] == pytest.approx([refused.x, refused.y], abs=1e-4)
    assert s.results[-1].reached
    assert any('REPLAN 1' in line for line in rig.lines)


def test_too_many_replans_halts(tmp_path):
    rig = Rig(tmp_path, 'fast', guard=lambda pose, move: False)
    with pytest.raises(NavigationHalt):
        rig.session.run()
    assert sum('REPLAN' in line for line in rig.lines) == EX.max_replans + 1
    assert any(line.startswith('HALT') for line in rig.lines)
    assert not any(s.kind == 'drive' for s in rig.executed)


def test_dry_run_never_moves(tmp_path):
    rig = Rig(tmp_path, 'safe')
    plan, est = rig.session.dry_run()
    assert rig.executed == [] and plan.kind == 'safe'
    assert math.dist((est.pose().x, est.pose().y), (0.5, 2.5)) < 0.05
    assert sum(line.startswith('estimate') for line in rig.lines) == OPTS.dry_looks


def test_unknown_planner(tmp_path):
    with pytest.raises(ValueError):
        PlanSession(CFG, WORLD_DATA, 'short', None, None, None, str(tmp_path))
