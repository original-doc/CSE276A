"""State estimators used by the stop-look-correct Navigator (no ROS).

An estimator turns executed moves (predict) and one look's tag observations
(update) into a robot pose estimate, and tells the planner which landmarks
it knows. HW2: known map (LocalizationEstimator). HW3: EKF-SLAM.
"""
import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from robot_control import ekf, slam
from robot_control.se2 import Pose2

# TagRecord.action values
UPDATE = 'update'           # used to correct the state
GATE = 'gate'               # rejected by the Mahalanobis gate
INIT = 'init'               # new landmark added to the state
SKIP_INIT = 'skip_init'     # new landmark, observation not good enough to add it yet
REJECTED = 'rejected'       # not usable (filter_window, or not in the map)


@dataclass
class TagRecord:
    obs: object         # TagObs, None when filter_window rejected the tag
    action: str
    d2: float
    reason: str = ''

    @property
    def accepted(self):
        return self.action in (UPDATE, INIT)


class Estimator(Protocol):
    def pose(self):
        """Pose2 estimate."""

    def pose_cov(self):
        """3x3 covariance of the pose."""

    def predict(self, move):
        """Move('drive'|'spin', value) that was just executed."""

    def update(self, accepted):
        """{id: TagObs} from one look -> list[TagRecord]."""

    def landmarks(self):
        """Landmarks known so far (Landmark objects), for planning."""


class LocalizationEstimator:
    """EKF on the robot pose with a known landmark map (HW2)."""

    def __init__(self, landmark_map, start_pose, noise):
        self.map = landmark_map
        self.noise = noise
        self.x = ekf.state_from_pose(start_pose)
        self.P = ekf.initial_covariance(noise)

    def pose(self):
        return ekf.pose_from_state(self.x)

    def pose_cov(self):
        return self.P

    def predict(self, move):
        if move.kind == 'drive':
            self.x, self.P = ekf.predict_drive(self.x, self.P, move.value, self.noise)
        else:
            self.x, self.P = ekf.predict_spin(self.x, self.P, move.value, self.noise)

    def update(self, accepted):
        """EKF update per tag, nearest first."""
        records = []
        for obs in sorted(accepted.values(), key=lambda o: o.dist):
            lm = self.map.get(obs.id)
            if lm is None:
                records.append(TagRecord(obs, REJECTED, 0.0, 'not in map'))
                continue
            r = ekf.update(self.x, self.P, obs, lm, self.noise)
            if r.accepted:
                self.x, self.P = r.x, r.P
                records.append(TagRecord(obs, UPDATE, r.d2))
            else:
                records.append(TagRecord(obs, GATE, r.d2, 'gate'))
        return records

    def landmarks(self):
        return list(self.map.landmarks.values())


@dataclass(frozen=True)
class InitLimits:
    """A new landmark is added only from an observation this close and frontal."""

    max_dist: float     # m
    max_oblique: float  # rad


def init_limits_from_config(cfg):
    """`slam:` section of slam.yaml -> InitLimits."""
    return InitLimits(float(cfg['init_max_dist']), math.radians(cfg['init_max_oblique_deg']))


class SlamEstimator:
    """EKF-SLAM: robot pose + every tag seen so far. Starts at the map origin, no landmarks."""

    def __init__(self, noise, init_limits, start_pose=Pose2(0.0, 0.0, 0.0)):
        self.noise = noise
        self.limits = init_limits
        self.s = ekf.state_from_pose(start_pose)
        self.Sigma = ekf.initial_covariance(noise)
        self.ids = []           # tag id of landmark k
        self.n_updates = {}     # tag id -> accepted updates

    def pose(self):
        return ekf.pose_from_state(self.s)

    def pose_cov(self):
        return self.Sigma[:3, :3].copy()

    def index(self, tag_id):
        return self.ids.index(tag_id)

    def predict(self, move):
        if move.kind == 'drive':
            self.s, self.Sigma = slam.predict_drive(self.s, self.Sigma, move.value, self.noise)
        else:
            self.s, self.Sigma = slam.predict_spin(self.s, self.Sigma, move.value, self.noise)

    def update(self, accepted):
        """Known tags first (nearest first), then add new ones from the corrected pose."""
        by_dist = sorted(accepted.values(), key=lambda o: o.dist)
        known = [o for o in by_dist if o.id in self.ids]
        new = [o for o in by_dist if o.id not in self.ids]
        return [self.correct(o) for o in known] + [self.add(o) for o in new]

    def correct(self, obs):
        r = slam.update(self.s, self.Sigma, obs, self.index(obs.id), self.noise)
        if not r.accepted:
            return TagRecord(obs, GATE, r.d2, 'gate')
        self.s, self.Sigma = r.x, r.P
        self.n_updates[obs.id] += 1
        return TagRecord(obs, UPDATE, r.d2)

    def init_problem(self, obs):
        """Why this observation is not good enough to add a landmark ('' = it is)."""
        if obs.dist > self.limits.max_dist:
            return f'dist {obs.dist:.2f} > {self.limits.max_dist:.2f} m'
        if obs.oblique > self.limits.max_oblique:
            return (f'oblique {math.degrees(obs.oblique):.0f} > '
                    f'{math.degrees(self.limits.max_oblique):.0f} deg')
        return ''

    def add(self, obs):
        problem = self.init_problem(obs)
        if problem:
            return TagRecord(obs, SKIP_INIT, 0.0, problem)
        self.s, self.Sigma = slam.augment(self.s, self.Sigma, obs, self.noise)
        self.ids.append(obs.id)
        self.n_updates[obs.id] = 0
        return TagRecord(obs, INIT, 0.0)

    def landmarks(self):
        return [slam.as_landmark(self.s, k, tag_id) for k, tag_id in enumerate(self.ids)]

    def landmark_cov(self, tag_id):
        return slam.landmark_cov(self.Sigma, self.index(tag_id))

    def state(self):
        """(s, Sigma, ids): copies of the full state."""
        return self.s.copy(), self.Sigma.copy(), list(self.ids)

    def snapshot(self):
        """Plain-python summary for logs: robot pose and every landmark with its covariance."""
        p = self.pose()
        return {
            'robot': {'x': p.x, 'y': p.y, 'theta': p.theta,
                      'cov': np.asarray(self.pose_cov()).tolist()},
            'landmarks': [{'id': lm.id, 'x': lm.x, 'y': lm.y, 'yaw': lm.yaw,
                           'cov': self.landmark_cov(lm.id).tolist(),
                           'n_updates': self.n_updates[lm.id]}
                          for lm in self.landmarks()],
        }
