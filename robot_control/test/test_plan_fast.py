import copy
import itertools
import math
import os

import pytest
import yaml

from robot_control.path_utils import NoPath, path_length
from robot_control.plan_fast import (
    graph_vertices, plan_fast, side_of, visibility_graph, visible)
from robot_control.world import load_world, parse_world, segment_clearance, segment_enters_rect

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4', 'world.yaml')
WORLD = load_world(CONFIG)
S, G = (0.5, 2.5), (2.5, 0.5)


def world_with(obstacles, landmarks_keep=8):
    with open(CONFIG) as f:
        d = yaml.safe_load(f)
    d = copy.deepcopy(d)
    d['obstacles'] = obstacles
    d['landmarks'] = d['landmarks'][:landmarks_keep]     # wall tags only
    return parse_world(d)


def brute_force_length(world, r):
    """Shortest valid S -> (up to 2 corners) -> G, by trying every sequence."""
    corners = [c for o in world.obstacles for c in o.inflate(r).corners()]
    best = math.inf
    for k in range(3):
        for mid in itertools.permutations(corners, k):
            pts = [S, *mid, G]
            if all(visible(a, b, world, r) for a, b in zip(pts, pts[1:])):
                best = min(best, path_length(pts))
    return best


def check_valid(plan, world, r):
    for a, b in zip(plan.points, plan.points[1:]):
        assert segment_clearance(a, b, world) >= r - 1e-9


def test_default_layout_length():
    r = WORLD.r_inflate
    p = plan_fast(WORLD)
    corner = (1.325 - r, 1.325 - r)                 # bottom-left inflated corner
    expected = math.dist(S, corner) + math.dist(corner, G)
    assert p.length == pytest.approx(expected)
    assert p.length == pytest.approx(3.03, abs=0.01)
    assert len(p.points) == 3
    assert p.points[1] == pytest.approx(corner)
    assert side_of(p.points[1], S, G) < 0           # right of S -> G (tie_side default)
    check_valid(p, WORLD, r)
    assert r - 1e-9 <= p.min_clearance <= r * math.sqrt(2) + 1e-9


def test_tie_side_left():
    r = WORLD.r_inflate
    p = plan_fast(WORLD, {'tie_side': 'left'})
    assert p.points[1] == pytest.approx((1.675 + r, 1.675 + r))
    assert p.length == pytest.approx(plan_fast(WORLD).length)


def test_stats_and_extra():
    p = plan_fast(WORLD)
    assert p.kind == 'fast'
    assert p.stats['nodes'] == 6                    # S, G, 4 corners
    assert p.stats['edges'] == len(p.extra['edges'])
    assert p.stats['expanded'] == len(p.extra['expanded'])
    assert p.extra['expanded'][0] == 0 and p.extra['expanded'][-1] == 1
    assert p.stats['time_s'] >= 0


def test_all_edges_avoid_inflated_box():
    r = WORLD.r_inflate
    p = plan_fast(WORLD)
    big = WORLD.obstacles[0].inflate(r)
    verts = p.extra['vertices']
    for i, j in p.extra['edges']:
        assert not segment_enters_rect(verts[i], verts[j], big)
    # S-G straight line is blocked; diagonal corners do not see each other
    assert (0, 1) not in p.extra['edges']


def test_no_obstacle_goes_straight():
    w = world_with([])
    p = plan_fast(w)
    assert p.points == [S, G]
    assert p.length == pytest.approx(math.dist(S, G))


def test_obstacle_off_the_line_goes_straight():
    w = world_with([{'cx': 2.3, 'cy': 2.3, 'w': 0.35, 'h': 0.35}])
    assert len(plan_fast(w).points) == 2


@pytest.mark.parametrize('box', [
    {'cx': 1.5, 'cy': 1.5, 'w': 0.5, 'h': 0.5},
    {'cx': 1.3, 'cy': 1.6, 'w': 0.35, 'h': 0.35},
    {'cx': 1.6, 'cy': 1.4, 'w': 0.6, 'h': 0.3},
])
def test_moved_or_resized_box_is_optimal(box):
    w = world_with([box])
    r = w.r_inflate
    p = plan_fast(w)
    check_valid(p, w, r)
    assert p.length == pytest.approx(brute_force_length(w, r))


def test_two_boxes_drop_hidden_corners():
    # Overlapping inflated boxes: corners inside the other box are not vertices.
    # The taller right box covers the left box's two right corners.
    w = world_with([{'cx': 1.3, 'cy': 1.5, 'w': 0.3, 'h': 0.3},
                    {'cx': 1.7, 'cy': 1.5, 'w': 0.3, 'h': 0.4}])
    r = w.r_inflate
    verts = graph_vertices(w, r, S, G)
    assert len(verts) == 2 + 8 - 2
    right_box = w.obstacles[1].inflate(r)
    assert all(not (right_box.x0 < x < right_box.x1 and right_box.y0 < y < right_box.y1)
               for x, y in verts)
    p = plan_fast(w)
    check_valid(p, w, r)
    assert p.length == pytest.approx(brute_force_length(w, r))


def test_corner_near_wall_dropped():
    w = world_with([{'cx': 0.4, 'cy': 1.5, 'w': 0.3, 'h': 0.3}])
    verts = graph_vertices(w, w.r_inflate, S, G)
    assert all(v[0] >= w.r_inflate - 1e-9 for v in verts)
    assert len(verts) == 2 + 2                      # left corners are within r of the wall


def test_replan_from_other_start():
    p = plan_fast(WORLD, start=(1.0, 2.0))
    assert p.points[0] == (1.0, 2.0) and p.points[-1] == G
    check_valid(p, WORLD, WORLD.r_inflate)


def test_start_inside_inflated_box_fails_but_smaller_r_works():
    start = (1.5, 1.15)                             # 0.175 m below the box
    with pytest.raises(NoPath):
        plan_fast(WORLD, start=start)
    p = plan_fast(WORLD, start=start, r=WORLD.r_guard)
    check_valid(p, WORLD, WORLD.r_guard)


def test_blocked_arena_raises():
    w = world_with([{'cx': 1.5, 'cy': 1.5, 'w': 3.0, 'h': 0.3}])   # wall to wall
    with pytest.raises(NoPath):
        plan_fast(w)


def test_visibility_graph_symmetric():
    r = WORLD.r_inflate
    verts = graph_vertices(WORLD, r, S, G)
    adj = visibility_graph(verts, WORLD, r)
    for i, nbrs in adj.items():
        for j, d in nbrs:
            assert (i, d) in adj[j]


def test_turn_cost_prefers_fewer_bends():
    # Default layout: every path bends once, so the cost grows by one bend (5 s * 0.2 m/s).
    p0 = plan_fast(WORLD, {'turn_cost_s': 0.0})
    p1 = plan_fast(WORLD, {'turn_cost_s': 5.0, 'speed': 0.2})
    assert p1.points == p0.points                   # only one bend possible here
    assert p1.stats['cost'] == pytest.approx(p0.stats['cost'] + 1.0)
