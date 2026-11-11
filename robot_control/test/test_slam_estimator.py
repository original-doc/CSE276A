import math
import os

import numpy as np
import pytest
import yaml

from robot_control import slam
from robot_control.ekf import noise_from_config
from robot_control.estimator import (
    GATE, INIT, SKIP_INIT, UPDATE, SlamEstimator, init_limits_from_config,
)
from robot_control.se2 import Pose2, to_child, wrap_angle
from robot_control.stop_look import Move, NavigationHalt, Navigator, params_from_config
from robot_control.tag_obs import make_obs

with open(os.path.join(os.path.dirname(__file__), '..', 'config', 'hw3', 'slam.yaml')) as f:
    CFG = yaml.safe_load(f)
NOISE = noise_from_config(CFG['ekf'])
LIMITS = init_limits_from_config(CFG['slam'])
P = params_from_config(CFG)

TAG_A = Pose2(1.5, 0.3, math.pi)                # in front of the start
TAG_B = Pose2(2.0, -0.6, math.radians(150))


def obs_of(robot, tag, tag_id, dx=0.0):
    x, y = to_child(robot, tag.x, tag.y)
    return make_obs(tag_id, 0.0, x + dx, y, wrap_angle(tag.theta - robot.theta), 10)


def look(*obs):
    return {o.id: o for o in obs}


def test_config_loads():
    assert LIMITS.max_dist == 2.8 and LIMITS.max_oblique == pytest.approx(math.radians(45))
    assert P.max_drive == 0.75 and P.pos_tol == 0.05
    assert CFG['detector_ids'] == list(range(8))


def test_starts_at_origin_with_no_landmarks():
    est = SlamEstimator(NOISE, LIMITS)
    assert est.pose() == Pose2(0.0, 0.0, 0.0)
    np.testing.assert_allclose(est.pose_cov(), np.zeros((3, 3)))
    assert est.landmarks() == [] and est.ids == []
    s, Sigma, ids = est.state()
    assert s.shape == (3,) and Sigma.shape == (3, 3) and ids == []


def test_landmarks_added_in_order_of_first_sight_with_any_id():
    est = SlamEstimator(NOISE, LIMITS)
    start = Pose2(0.0, 0.0, 0.0)
    records = est.update(look(obs_of(start, TAG_B, 42), obs_of(start, TAG_A, 7)))
    assert [(r.obs.id, r.action) for r in records] == [(7, INIT), (42, INIT)]   # nearest first
    assert est.ids == [7, 42] and slam.landmark_count(est.s) == 2
    lm = {m.id: m for m in est.landmarks()}
    assert lm[7].x == pytest.approx(TAG_A.x) and lm[42].yaw == pytest.approx(TAG_B.theta)
    records = est.update(look(obs_of(start, TAG_A, 7)))
    assert [r.action for r in records] == [UPDATE]
    assert est.n_updates == {7: 1, 42: 0}


def test_known_tags_update_before_new_tags_are_added():
    est = SlamEstimator(NOISE, LIMITS)
    est.update(look(obs_of(Pose2(0.0, 0.0, 0.0), TAG_A, 0)))
    est.predict(Move('drive', 0.5))
    true = Pose2(0.6, 0.0, 0.0)                  # drove 10 cm farther than planned
    obs_a, obs_b = obs_of(true, TAG_A, 0), obs_of(true, TAG_B, 1)
    s_pred, Sigma_pred = est.s.copy(), est.Sigma.copy()
    records = est.update(look(obs_b, obs_a))
    assert [r.action for r in records] == [UPDATE, INIT]
    corrected = slam.update(s_pred, Sigma_pred, obs_a, 0, NOISE).x
    expected = slam.augment(corrected, Sigma_pred, obs_b, NOISE)[0]
    np.testing.assert_allclose(est.s[6:9], expected[6:9])
    from_predicted = slam.inverse_obs(s_pred[:3], obs_b)
    b = est.landmarks()[1]
    assert math.hypot(b.x - TAG_B.x, b.y - TAG_B.y) < math.hypot(
        from_predicted[0] - TAG_B.x, from_predicted[1] - TAG_B.y)


def test_init_limits_skip_far_and_oblique_tags():
    est = SlamEstimator(NOISE, LIMITS)
    start = Pose2(0.0, 0.0, 0.0)
    far = obs_of(start, Pose2(3.0, 0.0, math.pi), 3)            # > init_max_dist 2.8
    oblique = obs_of(start, Pose2(1.0, 0.0, math.radians(130)), 4)    # 50 deg off
    records = {r.obs.id: r for r in est.update(look(far, oblique))}
    assert records[3].action == SKIP_INIT and 'dist' in records[3].reason
    assert records[4].action == SKIP_INIT and 'oblique' in records[4].reason
    assert est.ids == [] and len(est.s) == 3
    near = obs_of(start, Pose2(2.7, 0.0, math.pi), 3)
    assert [r.action for r in est.update(look(near))] == [INIT]
    assert est.ids == [3]


def test_gate_rejection_leaves_state_unchanged():
    est = SlamEstimator(NOISE, LIMITS)
    start = Pose2(0.0, 0.0, 0.0)
    est.update(look(obs_of(start, TAG_A, 0)))
    s, Sigma, _ = est.state()
    records = est.update(look(obs_of(start, TAG_A, 0, dx=0.5)))
    assert records[0].action == GATE and records[0].d2 > NOISE.gate
    np.testing.assert_array_equal(est.s, s)
    np.testing.assert_array_equal(est.Sigma, Sigma)
    assert est.n_updates[0] == 0


def test_navigator_halts_after_two_looks_with_every_known_tag_gated():
    est = SlamEstimator(NOISE, LIMITS)
    start = Pose2(0.0, 0.0, 0.0)
    looks = [look(obs_of(start, TAG_A, 0)),
             look(obs_of(start, TAG_A, 0, dx=0.5), obs_of(start, TAG_B, 1)),
             look(obs_of(start, TAG_A, 0, dx=0.5))]
    nav = Navigator(est, None, P, lambda: (looks.pop(0), {}), None)
    nav.update_from_look(1)     # init A
    nav.update_from_look(1)     # A gated (B added: does not reset the count)
    with pytest.raises(NavigationHalt):
        nav.update_from_look(1)


def test_snapshot_has_every_landmark_with_covariance():
    est = SlamEstimator(NOISE, LIMITS)
    start = Pose2(0.0, 0.0, 0.0)
    est.update(look(obs_of(start, TAG_A, 5), obs_of(start, TAG_B, 2)))
    snap = est.snapshot()
    assert [m['id'] for m in snap['landmarks']] == [5, 2]
    for m in snap['landmarks']:
        cov = np.array(m['cov'])
        np.testing.assert_allclose(cov, est.landmark_cov(m['id']))
        assert np.linalg.eigvalsh(cov).min() > 0
    yaml.safe_dump(snap)        # plain python types only
