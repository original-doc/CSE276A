import math
import os

import numpy as np
import pytest
import yaml

from robot_control import slam
from robot_control.ekf import noise_from_config
from robot_control.se2 import Pose2, to_child, wrap_angle
from robot_control.tag_obs import make_obs

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw2')
with open(os.path.join(CONFIG, 'nav.yaml')) as f:
    NOISE = noise_from_config(yaml.safe_load(f)['ekf'])
ANGLES = [2, 5]     # angle components of a state with one landmark


def obs_of(robot, lm, tag_id=0):
    """Exact observation of landmark pose lm (Pose2) from robot pose (Pose2)."""
    x, y = to_child(robot, lm.x, lm.y)
    return make_obs(tag_id, 0.0, x, y, wrap_angle(lm.theta - robot.theta), 10)


def numeric_jacobian(f, x, angles, eps=1e-6):
    J = np.zeros((3, len(x)))
    for i in range(len(x)):
        d = np.zeros(len(x))
        d[i] = eps
        diff = f(x + d) - f(x - d)
        for a in angles:
            diff[a] = wrap_angle(diff[a])
        J[:, i] = diff / (2 * eps)
    return J


def is_psd(S):
    return np.allclose(S, S.T, atol=1e-12) and np.linalg.eigvalsh(S).min() > -1e-12


def empty():
    return np.zeros(3), np.zeros((3, 3))


def with_one_landmark(robot=Pose2(0.3, -0.2, 0.4), lm=Pose2(1.5, 0.5, math.radians(200))):
    s, S = empty()
    s[:] = robot
    s, S = slam.predict_spin(s, S, 0.3, NOISE)      # some robot uncertainty
    return slam.augment(s, S, obs_of(slam.ekf.pose_from_state(s), lm), NOISE)


# ---------- Jacobians ----------

@pytest.mark.parametrize('theta_deg', [0, 30, 179.9, -120])
def test_obs_jacobian_matches_numeric(theta_deg):
    s = np.array([0.4, -0.3, math.radians(theta_deg), 1.7, 0.6, math.radians(-170)])
    H_r, H_j = slam.obs_jacobian(s, 0)
    J = numeric_jacobian(lambda v: slam.expected_obs(v, 0), s, [2])
    np.testing.assert_allclose(H_r, J[:, :3], atol=1e-6)
    np.testing.assert_allclose(H_j, J[:, 3:], atol=1e-6)


@pytest.mark.parametrize('theta_deg', [0, 75, -179.9])
def test_inverse_obs_jacobians_match_numeric(theta_deg):
    x_r = np.array([0.2, 0.1, math.radians(theta_deg)])
    f = np.array([1.2, -0.4, math.radians(160)])

    def g(v):
        return slam.inverse_obs(v[:3], make_obs(0, 0.0, v[3], v[4], v[5]))

    G_r, G_f = slam.inverse_obs_jacobians(x_r, make_obs(0, 0.0, *f))
    J = numeric_jacobian(g, np.concatenate([x_r, f]), [2])
    np.testing.assert_allclose(G_r, J[:, :3], atol=1e-6)
    np.testing.assert_allclose(G_f, J[:, 3:], atol=1e-6)


def test_inverse_obs_inverts_expected_obs():
    s, _ = with_one_landmark()
    o = slam.expected_obs(s, 0)
    back = slam.inverse_obs(s[:3], make_obs(0, 0.0, *o))
    np.testing.assert_allclose(back, s[3:6], atol=1e-12)


# ---------- predict ----------

def test_predict_drive_moves_robot_only_and_keeps_landmark_block():
    s, S = with_one_landmark()
    s2, S2 = slam.predict_drive(s, S, 0.5, NOISE)
    np.testing.assert_allclose(s2[3:], s[3:])
    np.testing.assert_allclose(s2[:2], s[:2] + 0.5 * np.array([math.cos(s[2]), math.sin(s[2])]))
    np.testing.assert_allclose(slam.landmark_cov(S2, 0), slam.landmark_cov(S, 0))
    F_r = slam.ekf.drive_jacobian(s[:3], 0.5)
    np.testing.assert_allclose(S2[:3, 3:], F_r @ S[:3, 3:])
    assert is_psd(S2)


def test_predict_spin_wraps_heading():
    s, S = empty()
    s[2] = math.radians(170)
    s, S = slam.predict_spin(s, S, math.radians(20), NOISE)
    assert s[2] == pytest.approx(math.radians(-170))
    assert S[2, 2] > 0 and S[0, 0] == 0


# ---------- augment ----------

def test_augment_dimension_and_cross_covariance():
    s, S = with_one_landmark()
    o = obs_of(Pose2(*s[:3]), Pose2(2.0, -1.0, math.pi / 2), 1)
    s2, S2 = slam.augment(s, S, o, NOISE)
    assert s2.shape == (9,) and S2.shape == (9, 9) and slam.landmark_count(s2) == 2
    np.testing.assert_allclose(S2[:6, :6], S)
    G_r, G_f = slam.inverse_obs_jacobians(s[:3], o)
    np.testing.assert_allclose(S2[6:, :3], G_r @ S[:3, :3])       # vs robot
    np.testing.assert_allclose(S2[6:, 3:6], G_r @ S[:3, 3:6])     # vs landmark 0
    assert np.abs(S2[6:, 3:6]).max() > 0
    assert is_psd(S2)
    assert slam.landmark_pose(s2, 1).x == pytest.approx(2.0)


def test_augment_from_exact_start_has_only_obs_noise():
    s, S = empty()
    o = make_obs(0, 0.0, 1.5, 0.0, math.pi)
    s, S = slam.augment(s, S, o, NOISE)
    np.testing.assert_allclose(slam.landmark_cov(S, 0), slam.ekf.obs_noise(o, NOISE), atol=1e-15)
    np.testing.assert_allclose(S[:3, :], 0.0)


# ---------- update ----------

def test_repeated_static_observations_shrink_landmark_covariance():
    s, S = with_one_landmark()
    lm, robot = Pose2(*s[3:6]), Pose2(*s[:3])
    traces = [np.trace(slam.landmark_cov(S, 0))]
    for _ in range(5):
        r = slam.update(s, S, obs_of(robot, lm), 0, NOISE)
        assert r.accepted
        s, S = r.x, r.P
        traces.append(np.trace(slam.landmark_cov(S, 0)))
        assert is_psd(S)
    assert all(b < a for a, b in zip(traces, traces[1:]))


def test_gate_rejects_and_leaves_state_unchanged():
    s, S = with_one_landmark()
    o = obs_of(Pose2(*s[:3]), Pose2(s[3] + 0.8, s[4], s[5]))
    r = slam.update(s, S, o, 0, NOISE)
    assert not r.accepted and r.d2 > NOISE.gate
    assert r.x is s and r.P is S


def test_update_wraps_angles_near_pi():
    # robot at 179 deg sees a tag whose normal is at -178 deg (= 182 deg)
    s, S = empty()
    s[2] = math.radians(179)
    s, S = slam.predict_spin(s, S, 0.0, NOISE)
    lm = Pose2(-1.5, 0.0, math.radians(-178))
    s, S = slam.augment(s, S, obs_of(Pose2(*s[:3]), lm), NOISE)
    assert s[5] == pytest.approx(math.radians(-178))
    robot_true = Pose2(0.0, 0.0, math.radians(-179.5))
    r = slam.update(s, S, obs_of(robot_true, lm), 0, NOISE)
    assert r.accepted and r.d2 < 3.0      # 1.5 deg off; unwrapped it would be ~358 deg
    assert all(-math.pi < r.x[i] <= math.pi for i in ANGLES)
    assert abs(wrap_angle(r.x[2] - math.radians(179))) < math.radians(2)


def test_loop_closure_corrects_landmarks_seen_on_the_way():
    """Robot drives 10 % farther than commanded; seeing landmark 0 again pulls
    both the robot and landmark 1 (added on the way) towards the truth."""
    lm0, lm1 = Pose2(0.0, 1.0, -math.pi / 2), Pose2(2.5, 1.0, -math.pi / 2)
    true = Pose2(0.0, 0.0, 0.0)
    s, S = empty()
    s, S = slam.augment(s, S, obs_of(true, lm0, 0), NOISE)
    for step in range(3):
        s, S = slam.predict_drive(s, S, 1.0, NOISE)
        true = Pose2(true.x + 1.1, 0.0, 0.0)
        if step == 1:
            s, S = slam.augment(s, S, obs_of(true, lm1, 1), NOISE)
    err_before = math.hypot(s[6] - lm1.x, s[7] - lm1.y)
    robot_before = abs(s[0] - true.x)
    r = slam.update(s, S, obs_of(true, lm0, 0), 0, NOISE)
    assert r.accepted
    s, S = r.x, r.P
    assert abs(s[0] - true.x) < robot_before / 2
    assert math.hypot(s[6] - lm1.x, s[7] - lm1.y) < err_before / 2
    assert is_psd(S)


def test_covariance_stays_symmetric_psd_over_random_run():
    rng = np.random.default_rng(0)
    landmarks = [Pose2(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
                 for _ in range(4)]
    s, S = empty()
    known = 0
    for _ in range(60):
        if rng.random() < 0.5:
            s, S = slam.predict_drive(s, S, rng.uniform(0.05, 0.6), NOISE)
        else:
            s, S = slam.predict_spin(s, S, rng.uniform(-2.0, 2.0), NOISE)
        k = int(rng.integers(len(landmarks)))
        o = obs_of(Pose2(*s[:3]), landmarks[k], k)
        o = make_obs(k, 0.0, o.x + rng.normal(0, 0.01), o.y + rng.normal(0, 0.01),
                     o.psi + rng.normal(0, 0.02))
        if k == known:          # add landmarks in id order so id == state index
            s, S = slam.augment(s, S, o, NOISE)
            known += 1
        elif k < known:
            r = slam.update(s, S, o, k, NOISE)
            s, S = r.x, r.P
        assert is_psd(S), np.linalg.eigvalsh(S).min()
    assert known >= 2
