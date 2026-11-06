"""EKF-SLAM math with pose landmarks (numpy, no ROS, no policy).

State s = [x, y, theta, x_1, y_1, psi_1, ..., x_N, y_N, psi_N] in map,
covariance Sigma (3+3N square). Landmark k (0-based) is a tag: centre + outward
normal. Observation of a tag in base_link: f = [x_b, y_b, psi_b] (TagObs).
The robot part reuses ekf.py (HW2).
"""
import math

import numpy as np

from robot_control import ekf
from robot_control.landmark_map import Landmark
from robot_control.se2 import Pose2, wrap_angle

ROBOT = slice(0, 3)


def landmark_count(s):
    return (len(s) - 3) // 3


def landmark_slice(k):
    return slice(3 + 3 * k, 6 + 3 * k)


def landmark_pose(s, k):
    x, y, psi = s[landmark_slice(k)]
    return Pose2(float(x), float(y), wrap_angle(float(psi)))


def landmark_cov(Sigma, k):
    sl = landmark_slice(k)
    return Sigma[sl, sl]


def as_landmark(s, k, tag_id=-1):
    """Landmark k as a landmark_map.Landmark (for ekf.py and the planner)."""
    p = landmark_pose(s, k)
    return Landmark(tag_id, p.x, p.y, p.theta)


def wrap_state(s):
    """Wrap the robot heading and every landmark normal."""
    s = s.copy()
    for i in range(2, len(s), 3):
        s[i] = wrap_angle(s[i])
    return s


def symmetrize(Sigma):
    return (Sigma + Sigma.T) / 2


# ---------- predict ----------

def propagate(Sigma, F_r, Q):
    """Sigma after a robot-only motion: only the robot row/column blocks change."""
    S = Sigma.copy()
    S[ROBOT, ROBOT] = F_r @ Sigma[ROBOT, ROBOT] @ F_r.T + Q
    S[ROBOT, 3:] = F_r @ Sigma[ROBOT, 3:]
    S[3:, ROBOT] = S[ROBOT, 3:].T
    return S


def predict_drive(s, Sigma, d, noise):
    x_r = s[ROBOT]
    s_new = s.copy()
    s_new[ROBOT] = ekf.drive_model(x_r, d)
    return s_new, propagate(Sigma, ekf.drive_jacobian(x_r, d), ekf.drive_noise(x_r, d, noise))


def predict_spin(s, Sigma, a, noise):
    s_new = s.copy()
    s_new[2] = wrap_angle(s[2] + a)
    return s_new, propagate(Sigma, np.eye(3), ekf.spin_noise(a, noise))


# ---------- update ----------

def expected_obs(s, k):
    """h: where landmark k should appear in base_link."""
    return ekf.expected_obs(s[ROBOT], as_landmark(s, k))


def obs_jacobian(s, k):
    """(H_r, H_j): d h / d robot, d h / d landmark k."""
    H_r = ekf.obs_jacobian(s[ROBOT], as_landmark(s, k))
    c, sn = math.cos(s[2]), math.sin(s[2])
    H_j = np.array([[c, sn, 0.0], [-sn, c, 0.0], [0.0, 0.0, 1.0]])
    return H_r, H_j


def full_obs_jacobian(s, k):
    """3 x (3+3N), zero except the robot and landmark-k blocks."""
    H_r, H_j = obs_jacobian(s, k)
    H = np.zeros((3, len(s)))
    H[:, ROBOT] = H_r
    H[:, landmark_slice(k)] = H_j
    return H


def update(s, Sigma, obs, k, noise):
    """One observation of landmark k. Rejected (unchanged) when d2 > noise.gate."""
    z = np.array([obs.x, obs.y, obs.psi])
    H = full_obs_jacobian(s, k)
    R = ekf.obs_noise(obs, noise)
    nu = ekf.innovation(z, expected_obs(s, k))
    S = H @ Sigma @ H.T + R
    d2 = float(nu @ np.linalg.solve(S, nu))
    if d2 > noise.gate:
        return ekf.UpdateResult(s, Sigma, False, d2)
    K = Sigma @ H.T @ np.linalg.inv(S)
    s_new = wrap_state(s + K @ nu)
    I_KH = np.eye(len(s)) - K @ H
    Sigma_new = symmetrize(I_KH @ Sigma @ I_KH.T + K @ R @ K.T)    # Joseph form
    return ekf.UpdateResult(s_new, Sigma_new, True, d2)


# ---------- augment (new landmark) ----------

def inverse_obs(x_r, obs):
    """g: landmark pose in map from the robot pose and one observation."""
    c, sn = math.cos(x_r[2]), math.sin(x_r[2])
    return np.array([x_r[0] + c * obs.x - sn * obs.y,
                     x_r[1] + sn * obs.x + c * obs.y,
                     wrap_angle(x_r[2] + obs.psi)])


def inverse_obs_jacobians(x_r, obs):
    """(G_r, G_f): d g / d robot, d g / d observation."""
    c, sn = math.cos(x_r[2]), math.sin(x_r[2])
    G_r = np.array([[1.0, 0.0, -sn * obs.x - c * obs.y],
                    [0.0, 1.0, c * obs.x - sn * obs.y],
                    [0.0, 0.0, 1.0]])
    G_f = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
    return G_r, G_f


def augment(s, Sigma, obs, noise):
    """Append a new landmark initialised from obs; keeps its cross-covariances."""
    x_r = s[ROBOT]
    G_r, G_f = inverse_obs_jacobians(x_r, obs)
    n = len(s)
    s_new = np.concatenate([s, inverse_obs(x_r, obs)])
    Sigma_new = np.zeros((n + 3, n + 3))
    Sigma_new[:n, :n] = Sigma
    cross = G_r @ Sigma[ROBOT, :]                      # new vs robot and old landmarks
    Sigma_new[n:, :n] = cross
    Sigma_new[:n, n:] = cross.T
    Sigma_new[n:, n:] = G_r @ Sigma[ROBOT, ROBOT] @ G_r.T + G_f @ ekf.obs_noise(obs, noise) @ G_f.T
    return s_new, Sigma_new
