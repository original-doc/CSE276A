"""Simulated rover for closed-loop tests: imperfect primitives + noisy tag views."""
import math
from dataclasses import dataclass

from robot_control.ol_model import Calibration, PulseModel
from robot_control.se2 import Pose2, angle_between, to_child, wrap_angle
from robot_control.tag_obs import make_obs


@dataclass
class SimNoise:
    drive_scale: float = 1.03       # systematic: drives 3 % too far
    drive_sigma: float = 0.01       # m per drive
    drift: float = math.radians(1.5)    # rad per m driven (veers left)
    spin_scale: float = 0.95        # turns 5 % short
    spin_sigma: float = math.radians(2.0)
    obs_xy: float = 0.003           # m, per look (already averaged)
    obs_psi: float = math.radians(1.0)
    obs_xy_b: float = 0.0           # extra xy std per m^2 of distance (0 = constant)
    obs_psi_e: float = 0.0          # extra psi std per m of distance
    slow_fine_T: float = math.inf   # fine spins (cmd 0.25) at least this long ...
    slow_fine_scale: float = 1.0    # ... turn this much more (2026-11-06: 1.5-2.4x)
    half_fov: float = math.radians(45.0)
    max_dist: float = 3.0
    max_oblique: float = math.radians(60.0)


class SimRover:
    def __init__(self, landmark_map, start, rng, noise=None):
        self.map = landmark_map
        self.true = start
        self.rng = rng
        self.n = noise or SimNoise()
        self.steps = []

    def execute(self, step):
        self.steps.append(step)
        if step.kind == 'drive':
            self.drive(step.amount)
        else:
            sign = 1.0 if step.dir == 'ccw' else -1.0
            slow_fine = step.cmd == 0.25 and step.T is not None and step.T >= self.n.slow_fine_T
            scale = self.n.slow_fine_scale if slow_fine else 1.0
            self.spin(sign * math.radians(step.amount) * scale)

    def drive(self, d):
        n = self.n
        actual = d * n.drive_scale + self.rng.normal(0.0, n.drive_sigma)
        dth = n.drift * actual
        mid = self.true.theta + dth / 2
        self.true = Pose2(self.true.x + actual * math.cos(mid),
                          self.true.y + actual * math.sin(mid),
                          wrap_angle(self.true.theta + dth))

    def spin(self, a):
        actual = a * self.n.spin_scale + self.rng.normal(0.0, self.n.spin_sigma)
        self.true = Pose2(self.true.x, self.true.y, wrap_angle(self.true.theta + actual))

    def visible(self, lm):
        x, y = to_child(self.true, lm.x, lm.y)
        d = math.hypot(x, y)
        oblique = angle_between(math.cos(lm.yaw), math.sin(lm.yaw),
                                self.true.x - lm.x, self.true.y - lm.y)
        return (0.25 <= d <= self.n.max_dist and abs(math.atan2(y, x)) <= self.n.half_fov
                and oblique <= self.n.max_oblique)

    def look(self):
        seen = {}
        for lm in self.map.landmarks.values():
            if not self.visible(lm):
                continue
            x, y = to_child(self.true, lm.x, lm.y)
            d = math.hypot(x, y)
            s_xy = self.n.obs_xy + self.n.obs_xy_b * d * d
            s_psi = self.n.obs_psi + self.n.obs_psi_e * d
            seen[lm.id] = make_obs(
                lm.id, 0.0, x + self.rng.normal(0.0, s_xy), y + self.rng.normal(0.0, s_xy),
                lm.yaw - self.true.theta + self.rng.normal(0.0, s_psi), 10)
        return seen, {}


def make_calib(tmp_path):
    """Coarse levels like study_wood2, plus fine levels (min 1.5 cm / 3 deg).

    The coarse spin level also carries a pulse model (like study_wood2_hw2_ext after
    HW3/fit_spin_pulse.py), so HW3's single-level spin config can turn from ~6.5 deg.
    """
    c = Calibration.load(tmp_path / 'sim.yaml', 'sim', create=True)
    for T in (2.0, 3.0, 4.0):
        c.drive_trials().append({'cmd': 0.3, 'T': T, 'x': 0.396 * T - 0.024, 'y': 0.0})
    for T in (0.2, 0.4, 0.6):
        c.drive_trials().append({'cmd': 0.2, 'T': T, 'x': 0.1 * T - 0.005, 'y': 0.0})
    for d in ('ccw', 'cw'):
        for T in (0.4, 0.8, 1.6):
            c.spin_trials(d).append({'cmd': 0.35, 'T': T, 'angle': 125.0 * T - 6.0})
        for T in (0.2, 0.4, 0.6):
            c.spin_trials(d).append({'cmd': 0.25, 'T': T, 'angle': 20.0 * T - 1.0})
        c.set_pulse(d, 0.35, PulseModel(125.0, -6.0, 0.1, 0.4, 1.6))   # 'short' below 0.32 s
    return c
