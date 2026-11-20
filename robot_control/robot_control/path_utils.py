"""Polyline helpers shared by the HW4 planners and the executor (no ROS)."""
import math
from dataclasses import dataclass, field

from robot_control.se2 import Pose2
from robot_control.world import clearance, point_segment_dist, segment_clearance


@dataclass
class Plan:
    kind: str               # 'safe' | 'fast'
    points: list            # [(x, y)], start .. goal
    length: float
    min_clearance: float    # robot centre to nearest wall / obstacle
    stats: dict = field(default_factory=dict)   # nodes, edges, expanded, time_s, ...
    extra: dict = field(default_factory=dict)   # planner data for plots


def make_plan(kind, points, world, stats=None, extra=None):
    points = [tuple(map(float, p)) for p in points]
    return Plan(kind, points, path_length(points), path_min_clearance(points, world),
                dict(stats or {}), dict(extra or {}))


def path_length(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def path_min_clearance(points, world):
    if len(points) == 1:
        return clearance(points[0], world)
    return min(segment_clearance(a, b, world) for a, b in zip(points, points[1:]))


def simplify(points, world, r, tol=0.03):
    """Douglas-Peucker; a chord replaces its points only if it keeps clearance >= r."""
    if len(points) <= 2:
        return list(points)
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < 2:
            continue
        a, b = points[i], points[j]
        devs = [point_segment_dist(points[k], a, b) for k in range(i + 1, j)]
        k_max = i + 1 + max(range(len(devs)), key=devs.__getitem__)
        if devs[k_max - i - 1] <= tol and segment_clearance(a, b, world) >= r:
            continue
        keep.add(k_max)
        stack += [(i, k_max), (k_max, j)]
    return [points[k] for k in sorted(keep)]


def densify(points, spacing):
    """Split every segment longer than spacing into equal parts."""
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(a, b) / spacing - 1e-9))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
                for k in range(1, n + 1)]
    return out


def waypoint_headings(points):
    """Heading at each point = direction of the next segment (last: of the last one)."""
    seg = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])]
    return seg + seg[-1:]


def to_goals(points):
    """Navigator goals for points[1:] (points[0] is where the robot starts)."""
    headings = waypoint_headings(points)
    return [Pose2(p[0], p[1], th) for p, th in zip(points[1:], headings[1:])]
