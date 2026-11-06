import math
import os

import numpy as np
import pytest
import yaml

from robot_control.estimator import GATE, TagRecord
from robot_control.landmark_map import Landmark
from robot_control.ol_model import Calibration
from robot_control.se2 import Pose2
from robot_control.stop_look import (
    Move, NavigationHalt, Navigator, NavLog, calibrated_spin, near_skip, next_move,
    params_from_config, split_spin, to_step,
)
from robot_control.tag_obs import make_obs

HERE = os.path.dirname(__file__)
with open(os.path.join(HERE, '..', 'config', 'hw3', 'slam.yaml')) as f:
    P = params_from_config(yaml.safe_load(f))
CALIB = Calibration.load(os.path.join(HERE, '..', 'config', 'open_loop',
                                      'study_wood2_hw2_ext.yaml'))
r = math.radians


def test_hw3_config_values():
    assert math.degrees(P.fine_max) == pytest.approx(7.0)
    assert math.degrees(P.coarse_min) == pytest.approx(25.0)
    assert math.degrees(P.min_split) == pytest.approx(45.0)
    assert P.near_dist == 0.15


@pytest.mark.parametrize('deg, piece', [(5, 5), (7, 7), (8, 4), (10.7, 5.35), (20.4, 6.8),
                                        (24, 6), (25, 25), (45, 45), (90, 90), (-15, -5)])
def test_calibrated_spin_pieces(deg, piece):
    m = calibrated_spin(Move('spin', r(deg)), P)
    assert math.degrees(m.value) == pytest.approx(piece)


@pytest.mark.parametrize('deg', [5, 7, 10.7, 12, 20.4, 23, 25, 30, 44, 45, 60, 90, 135, 180])
def test_every_turn_gets_a_usable_step(deg):
    # 2026-11-06: 20.4 deg had no level ("no calibrated level") and a vertex was skipped;
    # fine spins longer than ~0.35 s turned 1.5-2.4x the plan
    step = to_step(calibrated_spin(Move('spin', r(deg)), P), CALIB, P)
    assert step.T is not None, deg
    if step.cmd == 0.25:
        assert step.T <= 0.32 and not step.warn
    else:
        assert step.T >= 0.2         # coarse from ~25 deg (short T measured 0.7-1.0x)


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
