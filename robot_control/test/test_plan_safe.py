import copy
import os

import pytest
import yaml

from robot_control.path_utils import NoPath
from robot_control.plan_fast import plan_fast
from robot_control.plan_safe import path_side, plan_safe
from robot_control.world import (
    clearance, load_world, parse_world, segment_clearance, segment_rect_dist)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4', 'world.yaml')
WORLD = load_world(CONFIG)
S, G = (0.5, 2.5), (2.5, 0.5)


@pytest.fixture(scope='module')
def plans():
    return {side: plan_safe(WORLD, {'tie_side': side}) for side in ('right', 'left')}


def other_world(obstacles):
    with open(CONFIG) as f:
        d = copy.deepcopy(yaml.safe_load(f))
    d['landmarks'] = d['landmarks'][:8]
    d['obstacles'] = obstacles
    return parse_world(d)


def box_clearance(points, box):
    return min(segment_rect_dist(a, b, box) for a, b in zip(points, points[1:]))


def check_valid(plan, world, r):
    for a, b in zip(plan.points, plan.points[1:]):
        assert segment_clearance(a, b, world) >= r - 1e-9


def test_default_path(plans):
    p = plans['right']
    assert p.kind == 'safe'
    assert p.points[0] == S and p.points[-1] == G
    assert 3.6 <= p.length <= 3.8
    check_valid(p, WORLD, WORLD.r_inflate)
    assert p.min_clearance == pytest.approx(0.5, abs=1e-6)      # at S and G
    assert len(p.points) < p.stats['points_raw'] / 5            # simplified


def test_far_from_box(plans):
    box = WORLD.obstacles[0]
    safe = box_clearance(plans['right'].points, box)
    fast = box_clearance(plan_fast(WORLD).points, box)
    assert safe >= 0.6                                          # ~ half of 1.325 m
    assert safe > 2 * fast


def test_tie_side(plans):
    assert path_side(plans['right'].points, S, G) == -1         # below-left of the box
    assert path_side(plans['left'].points, S, G) == 1
    assert plans['right'].length == pytest.approx(plans['left'].length, abs=0.03)


def test_stats_and_extra(plans):
    p = plans['right']
    st = p.stats
    assert st['nodes'] == int(p.extra['gvd'].sum())
    assert st['expanded'] == len(p.extra['expanded']) <= st['nodes']
    assert st['edges'] > st['nodes']
    assert st['time_s'] >= st['field_time_s'] > 0
    assert p.extra['D'].shape == p.extra['gvd'].shape == (152, 152)
    assert all(p.extra['gvd'][c] for c in p.extra['expanded'])


def test_start_off_gvd():
    start = (0.35, 2.0)                                         # near the left wall
    p = plan_safe(WORLD, start=start)
    assert p.points[0] == start and p.points[-1] == G
    check_valid(p, WORLD, WORLD.r_inflate)
    assert box_clearance(p.points, WORLD.obstacles[0]) >= 0.6


def test_start_too_close():
    start = (1.5, 1.15)                                         # 0.175 m below the box
    assert WORLD.r_guard < clearance(start, WORLD) < WORLD.r_inflate
    with pytest.raises(NoPath):
        plan_safe(WORLD, start=start)
    p = plan_safe(WORLD, start=start, r=WORLD.r_guard)
    check_valid(p, WORLD, WORLD.r_guard)


def test_moved_box():
    w = other_world([{'cx': 1.2, 'cy': 1.7, 'w': 0.5, 'h': 0.4}])
    p = plan_safe(w)
    assert p.points[0] == S and p.points[-1] == G
    check_valid(p, w, w.r_inflate)


def test_blocked_raises():
    w = other_world([{'cx': 1.5, 'cy': 1.5, 'w': 3.0, 'h': 0.3}])   # wall to wall
    with pytest.raises(NoPath):
        plan_safe(w)
