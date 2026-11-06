"""Score an estimated landmark map against the tape-measured truth (no ROS).

The estimate lives in the phase's map frame (its start); T_arena_start is the
measured start pose in the arena. Two numbers for the report:
  1. errors after mapping with the measured start (includes start-measurement error)
  2. errors after the best rigid fit (only the shape of the map)
"""
import math
from dataclasses import dataclass, field

import numpy as np
import yaml

from robot_control.se2 import Pose2, compose, wrap_angle


@dataclass(frozen=True)
class LandmarkError:
    id: int
    dx: float           # estimate - truth, arena (m)
    dy: float
    yaw: float          # estimate - truth, wrapped (rad)

    @property
    def pos(self):
        return math.hypot(self.dx, self.dy)


@dataclass
class MapErrors:
    errors: list                                    # LandmarkError, sorted by id
    missing: list = field(default_factory=list)     # in the truth, never estimated
    extra: list = field(default_factory=list)       # estimated, not in the truth


def load_start(path):
    """start.yaml {x, y, theta_deg} -> Pose2."""
    with open(path) as f:
        d = yaml.safe_load(f)
    return Pose2(float(d['x']), float(d['y']), math.radians(float(d['theta_deg'])))


def snapshot_poses(snapshot):
    """estimator.snapshot() / map_lap yaml -> {id: Pose2} in the map frame."""
    return {m['id']: Pose2(m['x'], m['y'], m['yaw']) for m in snapshot['landmarks']}


def truth_poses(landmark_map):
    """LandmarkMap -> {id: Pose2}."""
    return {i: lm.pose() for i, lm in landmark_map.landmarks.items()}


def to_arena(T_arena_start, pose):
    return compose(T_arena_start, pose)


def poses_to_arena(T_arena_start, poses):
    return {i: to_arena(T_arena_start, p) for i, p in poses.items()}


def rotation(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def cov_to_arena(T_arena_start, cov_xy):
    """2x2 position covariance, map frame -> arena."""
    R = rotation(T_arena_start.theta)
    return R @ np.asarray(cov_xy) @ R.T


def cov_ellipse(cov_xy, k=2.0):
    """k-sigma ellipse of a 2x2 covariance -> (major semi-axis, minor semi-axis, angle rad)."""
    vals, vecs = np.linalg.eigh(np.asarray(cov_xy, dtype=float))
    vals = np.clip(vals, 0.0, None)
    major = vecs[:, 1]
    return (k * math.sqrt(vals[1]), k * math.sqrt(vals[0]),
            math.atan2(major[1], major[0]))


def landmark_errors(est, truth):
    """est, truth: {id: Pose2} in the same frame -> MapErrors."""
    common = sorted(set(est) & set(truth))
    errors = [LandmarkError(i, est[i].x - truth[i].x, est[i].y - truth[i].y,
                            wrap_angle(est[i].theta - truth[i].theta)) for i in common]
    return MapErrors(errors, sorted(set(truth) - set(est)), sorted(set(est) - set(truth)))


def mean_error(map_errors):
    """Position mean (the number the PDF asks for), RMS, max; mean |yaw| (rad)."""
    pos = np.array([e.pos for e in map_errors.errors])
    yaw = np.array([abs(e.yaw) for e in map_errors.errors])
    if not len(pos):
        return {'n': 0, 'mean': math.nan, 'rms': math.nan, 'max': math.nan,
                'mean_yaw': math.nan}
    return {'n': len(pos), 'mean': float(pos.mean()), 'rms': float(np.sqrt((pos ** 2).mean())),
            'max': float(pos.max()), 'mean_yaw': float(yaw.mean())}


def best_fit_rigid(est_xy, truth_xy):
    """2D least-squares rotation + translation (Umeyama, no scale): truth ~ R est + t.

    Returns (R, t, residuals) with residuals[i] = |R est_i + t - truth_i|.
    """
    P, Q = np.asarray(est_xy, dtype=float), np.asarray(truth_xy, dtype=float)
    p0, q0 = P.mean(axis=0), Q.mean(axis=0)
    U, _, Vt = np.linalg.svd((P - p0).T @ (Q - q0))
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])     # no reflection
    R = Vt.T @ D @ U.T
    t = q0 - R @ p0
    return R, t, np.linalg.norm(P @ R.T + t - Q, axis=1)


def rigid_to_pose(R, t):
    return Pose2(float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]))


def align(est, truth):
    """Best rigid fit of est onto truth (common ids) -> (aligned est {id: Pose2}, T).

    Needs at least 2 common landmarks.
    """
    common = sorted(set(est) & set(truth))
    if len(common) < 2:
        raise ValueError(f'need >= 2 common landmarks to align, have {len(common)}')
    R, t, _ = best_fit_rigid([(est[i].x, est[i].y) for i in common],
                             [(truth[i].x, truth[i].y) for i in common])
    T = rigid_to_pose(R, t)
    return {i: compose(T, p) for i, p in est.items()}, T
