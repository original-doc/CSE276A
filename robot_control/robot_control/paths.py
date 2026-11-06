"""HW3 routes: square and inscribed octagon, in the phase's own start frame (no ROS).

The start of a phase is (0, 0, 0): the robot's pose when the phase begins. Both
routes go counter-clockwise, pass through the start heading 0, and each vertex
heading points along the next side. Nothing here knows where the start is in
the arena.
"""
import math
from dataclasses import dataclass

from robot_control.se2 import Pose2, wrap_angle


@dataclass(frozen=True)
class Phase:
    name: str           # 'square' | 'octagon'
    goals: list         # Pose2 vertices, start frame
    lap_index: list     # lap (1-based) of each goal


def inscribed_octagon_side(n):
    """Side of the regular octagon whose four straight sides lie on an n x n square."""
    return n * math.tan(math.radians(22.5))


def corner_offset(n, m):
    """Distance from the start (middle of the octagon's bottom side) to the square corner."""
    return (n - m) / 2


def square_lap(n, a):
    """One square lap: 4 corners then back to the start; the start is a from the corner."""
    h = math.pi / 2
    return [Pose2(n - a, 0.0, h), Pose2(n - a, n, 2 * h), Pose2(-a, n, -h),
            Pose2(-a, 0.0, 0.0), Pose2(0.0, 0.0, 0.0)]


def octagon_lap(m):
    """One octagon lap of side m: vertex k = previous + m * (cos, sin)(45 (k-1) deg)."""
    goals, x, y = [], 0.0, 0.0
    for k in range(1, 9):
        side = math.radians(45.0 * (k - 1))
        x, y = x + m * math.cos(side), y + m * math.sin(side)
        goals.append(Pose2(x, y, wrap_angle(math.radians(45.0 * k))))
    goals[-1] = Pose2(0.0, 0.0, 0.0)        # exact start, no rounding drift
    return goals


def repeat(lap, laps):
    """lap repeated laps times -> (goals, lap_index)."""
    goals, lap_index = [], []
    for k in range(1, laps + 1):
        goals += lap
        lap_index += [k] * len(lap)
    return goals, lap_index


def make_phases(square_size, square_laps, octagon_laps):
    """Square phase then octagon phase; a phase with 0 laps is left out."""
    m = inscribed_octagon_side(square_size)
    laps = [('square', square_lap(square_size, corner_offset(square_size, m)), square_laps),
            ('octagon', octagon_lap(m), octagon_laps)]
    return [Phase(name, *repeat(lap, n)) for name, lap, n in laps if n > 0]
