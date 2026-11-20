import math
import os

import numpy as np
import pytest

from robot_control.path_utils import (
    densify, make_plan, path_length, path_min_clearance, simplify, to_goals,
    waypoint_headings)
from robot_control.world import load_world, segment_clearance

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4', 'world.yaml')
WORLD = load_world(CONFIG)       # box x, y in [1.325, 1.675]


def line(a, b, step=0.02):
    n = round(math.dist(a, b) / step)
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(n)]


def around_box():
    # Grid-like path: down x = 1.0, right along y = 1.0, up x = 2.0 (0.325 m from the box).
    return line((1.0, 1.5), (1.0, 1.0)) + line((1.0, 1.0), (2.0, 1.0)) + \
        line((2.0, 1.0), (2.0, 1.5)) + [(2.0, 1.5)]


def test_length_and_clearance():
    pts = [(0.5, 0.5), (0.5, 2.5), (2.5, 2.5)]
    assert path_length(pts) == pytest.approx(4.0)
    assert path_min_clearance(pts, WORLD) == pytest.approx(0.5)
    assert path_min_clearance([(1.5, 1.0)], WORLD) == pytest.approx(0.325)
    assert path_min_clearance([(0.5, 1.5), (2.5, 1.5)], WORLD) == 0.0


def test_make_plan():
    p = make_plan('fast', [(0.5, 0.5), (0.5, 2.5)], WORLD, stats={'nodes': 2})
    assert p.length == pytest.approx(2.0)
    assert p.min_clearance == pytest.approx(0.5)
    assert p.stats == {'nodes': 2} and p.extra == {}


def test_simplify_straight_grid_path():
    # Staircase within 1 cm of the line y = 2.5 -> 2 points.
    pts = [(0.5 + 0.02 * k, 2.5 + (0.01 if k % 2 else 0.0)) for k in range(101)]
    out = simplify(pts, WORLD, r=0.2)
    assert out == [pts[0], pts[-1]]


def test_simplify_keeps_corners():
    out = simplify(around_box(), WORLD, r=0.2)
    assert np.allclose(out, [(1.0, 1.5), (1.0, 1.0), (2.0, 1.0), (2.0, 1.5)])


def test_simplify_never_cuts_through_obstacle():
    # Huge tol: only the clearance test stops chords through the box.
    pts = around_box()
    out = simplify(pts, WORLD, r=0.3, tol=10.0)
    assert len(out) < len(pts)
    assert out[0] == pts[0] and out[-1] == pts[-1]
    assert all(segment_clearance(a, b, WORLD) >= 0.3 for a, b in zip(out, out[1:]))


def test_simplify_short_inputs():
    assert simplify([(0.5, 0.5)], WORLD, 0.2) == [(0.5, 0.5)]
    assert simplify([(0.5, 0.5), (1.0, 0.5)], WORLD, 0.2) == [(0.5, 0.5), (1.0, 0.5)]


def test_densify():
    pts = [(0.0, 0.0), (1.2, 0.0), (1.2, 0.3)]
    out = densify(pts, 0.5)
    assert np.allclose(out, [(0, 0), (0.4, 0), (0.8, 0), (1.2, 0), (1.2, 0.3)])
    assert all(math.dist(a, b) <= 0.5 + 1e-9 for a, b in zip(out, out[1:]))
    assert path_length(out) == pytest.approx(path_length(pts))
    assert np.allclose(densify([(0, 0), (1.0, 0)], 0.5), [(0, 0), (0.5, 0), (1.0, 0)])


def test_headings_and_goals():
    pts = [(0.5, 2.5), (1.0, 2.0), (1.0, 1.0), (2.0, 1.0)]
    hd = waypoint_headings(pts)
    assert hd == pytest.approx([-math.pi / 4, -math.pi / 2, 0.0, 0.0])
    goals = to_goals(pts)
    assert len(goals) == 3                                   # start not included
    assert (goals[0].x, goals[0].y) == (1.0, 2.0)
    assert [g.theta for g in goals] == pytest.approx(hd[1:])  # face the next segment
