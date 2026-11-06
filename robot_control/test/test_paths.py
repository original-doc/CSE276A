import math

import pytest

from robot_control.paths import (
    corner_offset, inscribed_octagon_side, make_phases, octagon_lap, repeat, square_lap,
)
from robot_control.se2 import Pose2, wrap_angle

N = 1.5
M = inscribed_octagon_side(N)
A = corner_offset(N, M)


def sides(start, lap):
    """(length, direction) of each side, starting from `start`."""
    out, prev = [], start
    for g in lap:
        out.append((math.hypot(g.x - prev.x, g.y - prev.y),
                    math.atan2(g.y - prev.y, g.x - prev.x)))
        prev = g
    return out


def turns(start, lap):
    """Heading change at each vertex (from arriving heading to the vertex heading)."""
    out, prev_heading = [], start.theta
    for g in lap:
        out.append(wrap_angle(g.theta - prev_heading))
        prev_heading = g.theta
    return out


def test_sizes_match_spec():
    assert M == pytest.approx(0.6213, abs=1e-4)
    assert A == pytest.approx(0.4393, abs=1e-4)


@pytest.mark.parametrize('lap', [square_lap(N, A), octagon_lap(M)])
def test_each_lap_returns_to_start_heading_zero(lap):
    assert lap[-1] == Pose2(0.0, 0.0, 0.0)


def test_square_sides_and_turns():
    lap = square_lap(N, A)
    assert len(lap) == 5
    lengths = [d for d, _ in sides(Pose2(0, 0, 0), lap)]
    assert lengths == pytest.approx([N - A, N, N, N, A])
    t = turns(Pose2(0, 0, 0), lap)
    assert t == pytest.approx([math.pi / 2] * 4 + [0.0])
    next_dirs = [d for _, d in sides(Pose2(0, 0, 0), lap)[1:]] + [0.0]   # next lap starts at 0
    for g, d in zip(lap, next_dirs):
        assert abs(wrap_angle(g.theta - d)) < 1e-9      # vertex heading = next side


def test_octagon_sides_and_turns():
    lap = octagon_lap(M)
    assert len(lap) == 8
    s = sides(Pose2(0, 0, 0), lap)
    assert [d for d, _ in s] == pytest.approx([M] * 8)
    assert [wrap_angle(d) for _, d in s] == pytest.approx(
        [wrap_angle(math.radians(45 * k)) for k in range(8)], abs=1e-9)
    assert turns(Pose2(0, 0, 0), lap) == pytest.approx([math.pi / 4] * 8)


def test_octagon_flat_sides_lie_on_square_sides():
    sq = square_lap(N, A)
    xmin, xmax = min(p.x for p in sq), max(p.x for p in sq)
    ymin, ymax = min(p.y for p in sq), max(p.y for p in sq)
    pts = [Pose2(0, 0, 0)] + octagon_lap(M)
    for a, b in zip(pts, pts[1:]):
        flat_x = abs(a.x - b.x) < 1e-9
        flat_y = abs(a.y - b.y) < 1e-9
        if flat_x:
            assert a.x == pytest.approx(xmin) or a.x == pytest.approx(xmax)
        if flat_y:
            assert a.y == pytest.approx(ymin) or a.y == pytest.approx(ymax)
    assert sum(abs(a.x - b.x) < 1e-9 or abs(a.y - b.y) < 1e-9
               for a, b in zip(pts, pts[1:])) == 4
    for p in pts:                                       # octagon inside the square
        assert xmin - 1e-9 <= p.x <= xmax + 1e-9 and ymin - 1e-9 <= p.y <= ymax + 1e-9


def test_repeat_lap_index():
    goals, idx = repeat(['a', 'b'], 3)
    assert goals == ['a', 'b'] * 3
    assert idx == [1, 1, 2, 2, 3, 3]
    assert repeat(['a'], 0) == ([], [])


def test_make_phases_default_run():
    phases = make_phases(N, 3, 3)
    assert [p.name for p in phases] == ['square', 'octagon']
    sq, oc = phases
    assert len(sq.goals) == 15 and sq.lap_index == [1] * 5 + [2] * 5 + [3] * 5
    assert len(oc.goals) == 24 and oc.lap_index[-1] == 3
    assert sq.goals[:5] == square_lap(N, A)
    assert oc.goals[:8] == octagon_lap(M)


def test_make_phases_skips_zero_laps():
    assert [p.name for p in make_phases(N, 0, 2)] == ['octagon']
    assert [p.name for p in make_phases(N, 1, 0)] == ['square']
    assert make_phases(N, 0, 0) == []
