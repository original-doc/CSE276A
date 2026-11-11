import dataclasses
import math
import os

import numpy as np
import pytest
import yaml

from robot_control.estimator import GATE, TagRecord
from robot_control.landmark_map import Landmark
from robot_control.ol_model import Calibration
from robot_control.se2 import Pose2
from robot_control.step_timing import min_step
from robot_control.stop_look import (
    Move, NavigationHalt, Navigator, NavLog, below_min_step, calibrated_spin, near_skip,
    next_move, params_from_config, position_done, split_spin, to_step,
)
from robot_control.tag_obs import make_obs

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'hw3', 'slam.yaml')) as f:
    P = params_from_config(yaml.safe_load(f))
CALIB = Calibration.load(os.path.join(HERE, '..', 'config', 'open_loop',
                                      'study_wood2_hw2_ext.yaml'))
r = math.radians


# fine 0.25 + coarse 0.35 with a gap in between, as configured before 2026-11-06 091125
SPLIT = dataclasses.replace(P, fine_max=r(7.0), coarse_min=r(25.0), spin_levels=(0.35, 0.25))


def test_hw3_config_values():
    assert P.spin_levels == (0.35,)                  # one level, timed by its pulse model
    assert P.fine_max == 0.0 and P.coarse_min == 0.0
    assert math.degrees(P.min_split) == pytest.approx(45.0)
    assert P.near_dist == 0.3
    assert math.degrees(P.align_min) == pytest.approx(8.0)
    assert math.degrees(P.ang_tol) == pytest.approx(8.0)
    for d in ('ccw', 'cw'):
        assert CALIB.pulse(d, 0.35) is not None
    assert 7.0 < min_step(CALIB, 'spin', 'ccw', P.spin_levels) < 8.0
    assert 8.0 < min_step(CALIB, 'spin', 'cw', P.spin_levels) < 10.0


@pytest.mark.parametrize('deg', [5, 8, 9.08, 20.4, 26, 45, 90, -9.08, -32])
def test_single_level_never_splits(deg):
    assert calibrated_spin(Move('spin', r(deg)), P, CALIB) == Move('spin', r(deg))


# the same split with the coarse level only (smallest steps 7.3 deg ccw / 9.0 deg cw)
SPLIT_COARSE = dataclasses.replace(SPLIT, spin_levels=(0.35,))


@pytest.mark.parametrize('params, deg, piece', [
    # fine level present: smallest steps 3.05 deg ccw / 4.55 deg cw (as on 2026-11-06)
    (SPLIT, 8, 4), (SPLIT, 10.7, 5.35), (SPLIT, 20.4, 6.8), (SPLIT, 24, 6), (SPLIT, 25, 25),
    (SPLIT, -9.08, -9.08), (SPLIT, -15, -5), (SPLIT, -20, -20 / 3),
    (SPLIT_COARSE, 8, 8), (SPLIT_COARSE, 20.4, 10.2), (SPLIT_COARSE, 24, 8),
    (SPLIT_COARSE, -15, -15), (SPLIT_COARSE, -20, -10), (SPLIT_COARSE, -24, -12)])
def test_split_pieces_are_never_below_the_smallest_step(params, deg, piece):
    # run 091125 octagon goal 8: -9.08 deg -> 2 x -4.54 deg, 0.01 deg below the smallest cw
    # step -> "remaining spin is below the smallest step" -> start vertex "reached" 0.5 m away
    m = calibrated_spin(Move('spin', r(deg)), params, CALIB)
    assert math.degrees(m.value) == pytest.approx(piece)
    assert not below_min_step(m, CALIB, params)


def test_split_without_calibration_as_before():
    assert math.degrees(calibrated_spin(Move('spin', r(20.4)), SPLIT).value) == pytest.approx(6.8)


@pytest.mark.parametrize('deg', [8, 9.08, 10.7, 12, 20.4, 23, 25, 26, 30, 32, 44, 45, 60, 90,
                                 135, 180, -9.08, -10, -26, -32, -45, -90])
def test_every_turn_gets_a_usable_step(deg):
    # 2026-11-06: 20.4 deg and cw 26-32 deg had "no calibrated level" (fine level too short,
    # coarse level's min_T too long) and vertices were skipped; now one pulse of 0.35
    step = to_step(calibrated_spin(Move('spin', r(deg)), P, CALIB), CALIB, P)
    assert step.T is not None, deg
    assert step.cmd == 0.35
    assert step.T >= CALIB.pulse(step.dir, 0.35).min_T


def test_turns_below_the_smallest_pulse_are_dropped_not_executed():
    assert below_min_step(Move('spin', r(5.0)), CALIB, P)
    # 0.5 m from the vertex, 5 deg off: drive, do not stop
    ok = lambda m: not below_min_step(m, CALIB, P)      # noqa: E731
    m = next_move(Pose2(0, 0, r(5.0)), Pose2(0.5, 0, 0), P, ok)
    assert m.kind == 'drive'


def test_run_091125_octagon_goal_8_now_drives_to_the_start():
    pose = Pose2(-0.3133, 0.3849, r(-41.78))
    goal = Pose2(0.0, 0.0, 0.0)
    ok = lambda m: not below_min_step(m, CALIB, P)      # noqa: E731
    m = calibrated_spin(next_move(pose, goal, P, ok), P, CALIB)
    assert m.kind == 'spin' and math.degrees(m.value) == pytest.approx(-9.08, abs=0.01)
    assert to_step(m, CALIB, P).T is not None
    assert not position_done(pose, goal, P)


def test_position_done():
    goal = Pose2(0.0, 0.0, 0.0)
    assert position_done(Pose2(0.04, 0.0, 0.0), goal, P)
    assert position_done(Pose2(0.2, 0.0, 0.0), goal, P)      # 20 cm past it: near_skip
    assert not position_done(Pose2(-0.2, 0.0, 0.0), goal, P)  # 20 cm short, straight ahead
    assert not position_done(Pose2(-0.5, 0.0, 0.0), goal, P)


def test_calibrated_spin_leaves_drives_and_none():
    assert calibrated_spin(Move('drive', 0.3), P) == Move('drive', 0.3)
    assert calibrated_spin(None, P) is None


def test_split_spin_needs_both_parts_big():
    # corner turn of 88 deg: the 37.5 deg first part is below min_split -> one turn
    tag = Landmark(0, 1.3, 0.05, math.pi)       # just ahead, will leave the view
    m = split_spin(Pose2(0, 0, 0), Move('spin', r(88)), [tag], P)
    assert m == Move('spin', r(88))
    # tag 30 deg to the left: keep it at the view edge -> 30 + 35 = 65 deg, then 105 deg
    left = Landmark(1, 1.5, 1.5 * math.tan(r(30)), r(-150))
    m = split_spin(Pose2(0, 0, 0), Move('spin', r(170)), [left], P)
    assert math.degrees(m.value) == pytest.approx(65.0)


def test_near_goal_skips_big_turn():
    # run 1 wp2: 6 cm beside the vertex, heading 99.5 deg, goal heading 180
    goal = Pose2(1.0607, 1.5, math.pi)
    pose = Pose2(0.9992, 1.4973, r(99.53))
    assert near_skip(0.062, r(-96.5), P)
    m = next_move(pose, goal, P)
    assert m.kind == 'spin' and math.degrees(m.value) == pytest.approx(80.5, abs=0.1)
    # a short drive straight ahead is still made
    m = next_move(Pose2(1.0, 1.5, math.pi), Pose2(0.9, 1.5, math.pi), P)
    assert m.kind == 'drive'


class Gated:
    def pose(self):
        return Pose2(0, 0, 0)

    def pose_cov(self):
        return np.zeros((3, 3))

    def predict(self, move):
        pass

    def update(self, accepted):
        return [TagRecord(make_obs(0, 0.0, 1.0, 0.0, math.pi), GATE, 50.0, 'gate')]

    def landmarks(self):
        return []


def test_the_halting_look_is_logged():
    looks = []

    class Log(NavLog):
        def on_look(self, goal_index, records, estimator):
            looks.append(records)

    nav = Navigator(Gated(), None, P, lambda: ({}, {}), None, Log())
    nav.update_from_look(1)
    with pytest.raises(NavigationHalt):
        nav.update_from_look(1)
    assert len(looks) == 2


class StillAt:
    """Estimator stuck at one pose; no tags."""

    def __init__(self, pose):
        self.p = pose

    def pose(self):
        return self.p

    def pose_cov(self):
        return np.zeros((3, 3))

    def predict(self, move):
        pass

    def update(self, accepted):
        return []

    def landmarks(self):
        return []


@pytest.mark.parametrize('x, reached', [(0.0, True), (0.2, True), (-0.5, False)])
def test_below_min_step_ends_a_goal_only_when_the_position_is_done(x, reached):
    goal = Pose2(0.0, 0.0, 0.0)
    nav = Navigator(StillAt(Pose2(x, 0.0, 0.0)), CALIB, P, lambda: ({}, {}), lambda s: None)
    nav.choose_move = lambda goal, tag_based, searched: (Move('spin', r(2.0)), None)
    res = nav.run_goal(1, goal)
    assert res.reached is reached
    assert 'below the smallest step' in res.note


def test_near_goal_no_turnaround_after_a_pose_jump():
    # run_085837 vertex 4: a 22 cm correction left the vertex 27 cm behind-left (121 deg);
    # with near_dist 0.15 it turned 121 deg, drove back and turned -118 deg (30 deg short)
    goal = Pose2(-0.4393, 0.0, 0.0)
    assert next_move(Pose2(-0.3174, -0.2413, r(-4.14)), goal, P) is None    # heading within tol
    m = next_move(Pose2(-0.3174, -0.2413, r(-20.0)), goal, P)
    assert m.kind == 'spin' and math.degrees(m.value) == pytest.approx(20.0)
