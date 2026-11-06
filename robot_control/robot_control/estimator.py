"""State estimators used by the stop-look-correct Navigator (no ROS).

An estimator turns executed moves (predict) and one look's tag observations
(update) into a robot pose estimate, and tells the planner which landmarks
it knows. HW2: known map (LocalizationEstimator). HW3: EKF-SLAM.
"""
from dataclasses import dataclass
from typing import Protocol

from robot_control import ekf

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
