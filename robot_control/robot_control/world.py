"""HW4 world: arena, box obstacles, robot size and 2-D geometry helpers (no ROS).

Frame = arena (bottom-left corner origin), as in HW3. Format: config/hw4/world.yaml.
"""
import math
from dataclasses import dataclass

import yaml

from robot_control.landmark_map import LandmarkMap, parse_map
from robot_control.se2 import Pose2, wrap_angle

TAG_TOL = 0.01          # m, obstacle tag must sit on a box face
TAG_NEAR = 0.10         # m, landmarks this close to a box belong to it
TAG_YAW_TOL_DEG = 10.0
EPS = 1e-9


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle."""

    cx: float
    cy: float
    w: float
    h: float

    @property
    def x0(self):
        return self.cx - self.w / 2

    @property
    def x1(self):
        return self.cx + self.w / 2

    @property
    def y0(self):
        return self.cy - self.h / 2

    @property
    def y1(self):
        return self.cy + self.h / 2

    def corners(self):
        """Counter-clockwise from bottom-left."""
        return [(self.x0, self.y0), (self.x1, self.y0),
                (self.x1, self.y1), (self.x0, self.y1)]

    def inflate(self, r):
        """Square-corner inflation: push every side out by r."""
        return Rect(self.cx, self.cy, self.w + 2 * r, self.h + 2 * r)


@dataclass(frozen=True)
class RobotSize:
    length: float
    width: float
    extra: float = 0.0


@dataclass(frozen=True)
class Margins:
    r_margin: float
    guard_margin: float


@dataclass
class World:
    arena: Rect
    obstacles: list         # [Rect]
    landmarks: LandmarkMap
    robot: RobotSize
    margins: Margins
    start: Pose2
    goal: tuple             # (x, y)

    @property
    def r_robot(self):
        """Radius of the disc that covers the robot body."""
        return math.hypot(self.robot.length, self.robot.width) / 2 + self.robot.extra

    @property
    def r_inflate(self):
        return self.r_robot + self.margins.r_margin

    @property
    def r_guard(self):
        return self.r_robot + self.margins.guard_margin


# ---------- geometry ----------

def point_rect_dist(p, rect):
    """Distance from p to a solid rectangle (0 inside)."""
    dx = max(rect.x0 - p[0], 0.0, p[0] - rect.x1)
    dy = max(rect.y0 - p[1], 0.0, p[1] - rect.y1)
    return math.hypot(dx, dy)


def point_segment_dist(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _clip_interval(a, b, rect, strict):
    """Liang-Barsky: parameter interval [t0, t1] of line a + t(b - a) inside rect.

    strict=True uses the open rectangle (a line lying on a side is outside).
    Returns None when the line misses the rectangle.
    """
    t0, t1 = -math.inf, math.inf
    for p0, d, lo, hi in ((a[0], b[0] - a[0], rect.x0, rect.x1),
                          (a[1], b[1] - a[1], rect.y0, rect.y1)):
        if abs(d) < EPS:
            inside = lo + EPS < p0 < hi - EPS if strict else lo - EPS <= p0 <= hi + EPS
            if not inside:
                return None
            continue
        ta, tb = (lo - p0) / d, (hi - p0) / d
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
    return t0, t1


def segment_hits_rect(a, b, rect):
    """Segment ab touches the closed rectangle."""
    iv = _clip_interval(a, b, rect, strict=False)
    return iv is not None and max(iv[0], 0.0) <= min(iv[1], 1.0) + EPS


def segment_enters_rect(a, b, rect):
    """Segment ab passes through the open interior (grazing a side or corner is not)."""
    iv = _clip_interval(a, b, rect, strict=True)
    if iv is None:
        return False
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    return (min(iv[1], 1.0) - max(iv[0], 0.0)) * length > EPS * 1e3


def segment_rect_dist(a, b, rect):
    if segment_hits_rect(a, b, rect):
        return 0.0
    d = min(point_rect_dist(a, rect), point_rect_dist(b, rect))
    return min(d, *(point_segment_dist(c, a, b) for c in rect.corners()))


def wall_dist(p, arena):
    """Distance from p to the arena boundary (0 outside the arena)."""
    d = min(p[0] - arena.x0, arena.x1 - p[0], p[1] - arena.y0, arena.y1 - p[1])
    return max(d, 0.0)


def clearance(p, world):
    """Min distance from p to the walls and all obstacles."""
    return min([wall_dist(p, world.arena)]
               + [point_rect_dist(p, o) for o in world.obstacles])


def segment_clearance(a, b, world):
    # Arena is convex: distance to each wall is linear along ab, so its min is at an end.
    d = min(wall_dist(a, world.arena), wall_dist(b, world.arena))
    return min([d] + [segment_rect_dist(a, b, o) for o in world.obstacles])


# ---------- loading ----------

def _face_normal(p, rect):
    """Outward normal angle of the box face nearest to p."""
    faces = [(abs(p[1] - rect.y0), -math.pi / 2), (abs(p[0] - rect.x1), 0.0),
             (abs(p[1] - rect.y1), math.pi / 2), (abs(p[0] - rect.x0), math.pi)]
    return min(faces)[1]


def check_obstacle_tags(obstacles, landmarks):
    """Raise if a tag near a box is not on its face or does not face outward."""
    for k, box in enumerate(obstacles):
        for lm in landmarks.landmarks.values():
            p = (lm.x, lm.y)
            if point_rect_dist(p, box) > TAG_NEAR:
                continue
            depth = min(p[0] - box.x0, box.x1 - p[0], p[1] - box.y0, box.y1 - p[1])
            if point_rect_dist(p, box) > TAG_TOL or depth > TAG_TOL:
                raise ValueError(f'tag {lm.id} is not on a face of obstacle {k} ({box})')
            yaw_err = abs(wrap_angle(lm.yaw - _face_normal(p, box)))
            if yaw_err > math.radians(TAG_YAW_TOL_DEG):
                raise ValueError(f'tag {lm.id} does not face out of obstacle {k}')


def parse_world(data):
    a = data['arena']
    arena = Rect((a['x0'] + a['x1']) / 2, (a['y0'] + a['y1']) / 2,
                 a['x1'] - a['x0'], a['y1'] - a['y0'])
    obstacles = [Rect(float(o['cx']), float(o['cy']), float(o['w']), float(o['h']))
                 for o in data.get('obstacles') or []]
    landmarks = parse_map(data)
    check_obstacle_tags(obstacles, landmarks)
    s, g = data['start'], data['goal']
    return World(
        arena=arena,
        obstacles=obstacles,
        landmarks=landmarks,
        robot=RobotSize(**data['robot']),
        margins=Margins(**data['margins']),
        start=Pose2(float(s['x']), float(s['y']), math.radians(float(s['theta_deg']))),
        goal=(float(g['x']), float(g['y'])),
    )


def load_world(path):
    with open(path) as f:
        return parse_world(yaml.safe_load(f) or {})
