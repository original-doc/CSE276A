import math

import numpy as np
import pytest
import yaml
from sim_hw3 import arena_map

from robot_control.map_eval import (
    align, best_fit_rigid, cov_ellipse, cov_to_arena, landmark_errors, load_start, mean_error,
    poses_to_arena, rigid_to_pose, snapshot_poses, to_arena, truth_poses,
)
from robot_control.se2 import Pose2, compose, inverse, wrap_angle

TRUTH = truth_poses(arena_map())
START = Pose2(1.189, 0.75, math.radians(3.0))


def in_start_frame(poses, start=START):
    """What a perfect SLAM run from `start` would estimate."""
    inv = inverse(start)
    return {i: compose(inv, p) for i, p in poses.items()}


def test_to_arena_inverts_start_frame():
    est = in_start_frame(TRUTH)
    back = poses_to_arena(START, est)
    for i, p in TRUTH.items():
        assert back[i] == pytest.approx(p, abs=1e-12)
    assert to_arena(START, Pose2(0, 0, 0)) == START


def test_cov_to_arena_rotates():
    cov = np.diag([0.04, 0.01])                 # long along map x
    c = cov_to_arena(Pose2(0, 0, math.pi / 2), cov)
    np.testing.assert_allclose(c, np.diag([0.01, 0.04]), atol=1e-15)
    c = cov_to_arena(Pose2(5, 5, 0.7), cov)
    assert np.trace(c) == pytest.approx(0.05)
    np.testing.assert_allclose(c, c.T)


def test_landmark_errors_values_and_missing_extra():
    est = poses_to_arena(START, in_start_frame(TRUTH))
    del est[4]
    est[1] = Pose2(est[1].x + 0.03, est[1].y - 0.04, est[1].theta + math.radians(2))
    est[9] = Pose2(0, 0, 0)
    me = landmark_errors(est, TRUTH)
    assert me.missing == [4] and me.extra == [9]
    assert [e.id for e in me.errors] == [0, 1, 2, 3, 5, 6, 7]
    e1 = next(e for e in me.errors if e.id == 1)
    assert e1.pos == pytest.approx(0.05) and math.degrees(e1.yaw) == pytest.approx(2.0)
    s = mean_error(me)
    assert s['n'] == 7
    assert s['mean'] == pytest.approx(0.05 / 7)
    assert s['rms'] == pytest.approx(math.sqrt(0.05 ** 2 / 7))
    assert s['max'] == pytest.approx(0.05)


def test_yaw_error_wraps():
    me = landmark_errors({0: Pose2(0, 0, math.radians(179))}, {0: Pose2(0, 0, math.radians(-179))})
    assert math.degrees(me.errors[0].yaw) == pytest.approx(-2.0)


def test_mean_error_empty():
    s = mean_error(landmark_errors({}, TRUTH))
    assert s['n'] == 0 and math.isnan(s['mean'])


@pytest.mark.parametrize('theta_deg', [0.0, 17.0, -120.0, 180.0])
def test_best_fit_recovers_known_rigid_transform(theta_deg):
    rng = np.random.default_rng(1)
    est = rng.uniform(-2, 2, size=(6, 2))
    T = Pose2(0.4, -1.3, math.radians(theta_deg))
    R_true = np.array([[math.cos(T.theta), -math.sin(T.theta)],
                       [math.sin(T.theta), math.cos(T.theta)]])
    truth = est @ R_true.T + [T.x, T.y]
    R, t, res = best_fit_rigid(est, truth)
    np.testing.assert_allclose(R, R_true, atol=1e-12)
    np.testing.assert_allclose(res, 0.0, atol=1e-12)
    got = rigid_to_pose(R, t)
    assert (got.x, got.y) == pytest.approx((T.x, T.y), abs=1e-12)
    assert abs(wrap_angle(got.theta - T.theta)) < 1e-12
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_best_fit_never_reflects():
    est = np.array([[0, 0], [1, 0], [0, 1], [1, 1.2]])
    mirrored = est * [1, -1]
    R, _, res = best_fit_rigid(est, mirrored)
    assert np.linalg.det(R) == pytest.approx(1.0)
    assert res.max() > 0.1


def test_align_removes_wrong_start_measurement():
    # perfect map, but the start was measured 5 cm / 2 deg off
    est = in_start_frame(TRUTH)
    wrong = Pose2(START.x + 0.05, START.y, START.theta + math.radians(2))
    in_arena = poses_to_arena(wrong, est)
    assert mean_error(landmark_errors(in_arena, TRUTH))['mean'] > 0.04
    aligned, T = align(in_arena, TRUTH)
    s = mean_error(landmark_errors(aligned, TRUTH))
    assert s['mean'] < 1e-9 and s['mean_yaw'] < 1e-9


def test_align_keeps_shape_error():
    est = dict(TRUTH)
    est[2] = Pose2(est[2].x + 0.1, est[2].y, est[2].theta)
    aligned, _ = align(est, TRUTH)
    s = mean_error(landmark_errors(aligned, TRUTH))
    assert 0.0 < s['mean'] < 0.1 / 8 * 2         # spread over the map, not removed


def test_align_needs_two_landmarks():
    with pytest.raises(ValueError):
        align({0: TRUTH[0]}, TRUTH)


def test_load_start_and_snapshot(tmp_path):
    p = tmp_path / 'start.yaml'
    p.write_text(yaml.safe_dump({'x': 1.2, 'y': 0.75, 'theta_deg': 90}))
    assert load_start(p) == pytest.approx(Pose2(1.2, 0.75, math.pi / 2))
    snap = {'landmarks': [{'id': 3, 'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'cov': []}]}
    assert snapshot_poses(snap) == {3: Pose2(1.0, 2.0, 0.5)}


def test_cov_ellipse_axes_and_angle():
    a, b, ang = cov_ellipse(np.diag([0.01, 0.04]), k=2.0)
    assert (a, b) == pytest.approx((0.4, 0.2))
    assert abs(math.cos(ang)) == pytest.approx(0.0, abs=1e-12)       # major along y
    R = np.array([[math.cos(0.5), -math.sin(0.5)], [math.sin(0.5), math.cos(0.5)]])
    a, b, ang = cov_ellipse(R @ np.diag([0.09, 0.01]) @ R.T, k=1.0)
    assert (a, b) == pytest.approx((0.3, 0.1))
    assert math.tan(ang) == pytest.approx(math.tan(0.5))
