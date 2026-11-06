import math

import numpy as np
import pytest
from test_stop_look import MAP, NOISE, P, localizer

from robot_control import ekf
from robot_control.estimator import GATE, INIT, REJECTED, SKIP_INIT, TagRecord, UPDATE
from robot_control.se2 import Pose2, to_child
from robot_control.stop_look import Move, NavigationHalt, Navigator
from robot_control.tag_obs import make_obs


def true_obs(lm, pose, dx=0.0):
    x, y = to_child(pose, lm.x, lm.y)
    return make_obs(lm.id, 0.0, x + dx, y, lm.yaw - pose.theta, 10)


def test_record_accepted_follows_action():
    assert TagRecord(None, UPDATE, 0.0).accepted
    assert TagRecord(None, INIT, 0.0).accepted
    for a in (GATE, SKIP_INIT, REJECTED):
        assert not TagRecord(None, a, 0.0).accepted


def test_localization_matches_hw2_ekf():
    start = Pose2(0.0, 0.0, 0.0)
    est = localizer(MAP, start)
    est.predict(Move('drive', 0.5))
    x, Pm = ekf.predict_drive(ekf.state_from_pose(start), ekf.initial_covariance(NOISE),
                              0.5, NOISE)
    np.testing.assert_allclose(est.x, x)
    np.testing.assert_allclose(est.pose_cov(), Pm)
    obs = true_obs(MAP.get(0), Pose2(0.5, 0.0, 0.0), dx=0.01)
    records = est.update({0: obs})
    r = ekf.update(x, Pm, obs, MAP.get(0), NOISE)
    assert [rec.action for rec in records] == [UPDATE]
    np.testing.assert_allclose(est.x, r.x)
    assert est.landmarks() == list(MAP.landmarks.values())


def test_localization_gate_and_unknown_tag():
    est = localizer(MAP)
    bad = true_obs(MAP.get(0), Pose2(0.0, 0.0, 0.0), dx=0.5)
    unknown = make_obs(42, 0.0, 1.0, 0.0, math.pi, 10)
    records = est.update({0: bad, 42: unknown})
    by_id = {r.obs.id: r for r in records}
    assert by_id[0].action == GATE and by_id[0].d2 > NOISE.gate
    assert by_id[42].action == REJECTED and by_id[42].reason == 'not in map'
    np.testing.assert_allclose(est.x, [0.0, 0.0, 0.0])


class ScriptedEstimator:
    """Returns pre-set record actions, one list per look."""

    def __init__(self, looks):
        self.looks = list(looks)

    def pose(self):
        return Pose2(0.0, 0.0, 0.0)

    def pose_cov(self):
        return np.zeros((3, 3))

    def predict(self, move):
        pass

    def update(self, accepted):
        obs = make_obs(0, 0.0, 1.0, 0.0, math.pi, 10)
        return [TagRecord(obs, a, 0.0) for a in self.looks.pop(0)]

    def landmarks(self):
        return []


def navigator(looks):
    return Navigator(ScriptedEstimator(looks), None, P, lambda: ({}, {}), None)


def test_halt_after_two_all_gated_looks():
    nav = navigator([[GATE], [GATE, INIT]])
    nav.update_from_look(1)
    with pytest.raises(NavigationHalt):
        nav.update_from_look(1)


def test_init_only_look_does_not_count_as_rejected():
    nav = navigator([[GATE], [INIT], [UPDATE], [GATE], [SKIP_INIT]])
    for _ in range(5):
        nav.update_from_look(1)
    assert nav.all_rejected_looks == 1


def test_only_updates_count_as_used():
    nav = navigator([[INIT, UPDATE]])
    assert nav.update_from_look(1) == [0]
    nav = navigator([[INIT]])
    assert nav.update_from_look(1) == []
