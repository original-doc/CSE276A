import copy
import math
import os

import pytest
import yaml

from robot_control.world import (
    clearance, load_world, parse_world, point_rect_dist, Rect, segment_clearance,
    segment_enters_rect, segment_hits_rect, segment_rect_dist, wall_dist)

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4', 'world.yaml')
BOX = Rect(1.5, 1.5, 0.35, 0.35)     # x, y in [1.325, 1.675]


def raw_config():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


def test_rect_corners_and_inflate():
    assert BOX.corners()[0] == pytest.approx((1.325, 1.325))
    assert BOX.corners()[2] == pytest.approx((1.675, 1.675))
    big = BOX.inflate(0.2)
    assert (big.x0, big.x1, big.y0, big.y1) == pytest.approx((1.125, 1.875, 1.125, 1.875))


def test_point_rect_dist():
    assert point_rect_dist((1.5, 1.5), BOX) == 0.0                     # inside
    assert point_rect_dist((1.675, 1.6), BOX) == pytest.approx(0.0)    # on side
    assert point_rect_dist((2.0, 1.5), BOX) == pytest.approx(0.325)    # beside a face
    assert point_rect_dist((1.775, 1.775), BOX) == pytest.approx(0.1 * math.sqrt(2))  # corner


def test_segment_rect_dist():
    assert segment_rect_dist((0.0, 1.5), (3.0, 1.5), BOX) == 0.0       # crosses
    assert segment_rect_dist((0.0, 2.0), (3.0, 2.0), BOX) == pytest.approx(0.325)
    # endpoints far, segment passes near a corner
    # line y = x + 1.5; nearest corner (1.325, 1.675)
    assert segment_rect_dist((0.5, 2.0), (2.0, 3.5), BOX) == pytest.approx(1.15 / math.sqrt(2))
    assert segment_rect_dist((2.0, 1.5), (3.0, 1.5), BOX) == pytest.approx(0.325)  # endpoint


def test_segment_enters_rect():
    assert segment_enters_rect((0.0, 1.5), (3.0, 1.5), BOX)            # through
    assert segment_enters_rect((1.5, 1.5), (3.0, 3.0), BOX)            # starts inside
    assert not segment_enters_rect((0.0, 1.675), (3.0, 1.675), BOX)    # along a side
    assert not segment_enters_rect((1.0, 1.0), (1.325, 1.325), BOX)    # ends at corner
    # diagonal through one corner only: (1.675, 1.675) on line x + y = 3.35
    assert not segment_enters_rect((1.0, 2.35), (2.35, 1.0), BOX)
    assert segment_enters_rect((1.0, 2.34), (2.34, 1.0), BOX)          # 1 cm inside
    assert not segment_hits_rect((0.0, 2.0), (3.0, 2.0), BOX)
    assert segment_hits_rect((0.0, 1.675), (3.0, 1.675), BOX)          # touching counts


def test_corner_grazing_between_inflated_corners():
    # The fast path: S -> inflated corner -> G must not count as entering.
    big = BOX.inflate(0.208)
    c = big.corners()[0]
    assert not segment_enters_rect((0.5, 2.5), c, big)
    assert not segment_enters_rect(c, (2.5, 0.5), big)


def test_wall_and_clearance():
    w = load_world(CONFIG)
    assert wall_dist((0.5, 2.5), w.arena) == pytest.approx(0.5)
    assert wall_dist((-0.1, 1.0), w.arena) == 0.0
    assert clearance((0.5, 2.5), w) == pytest.approx(0.5)
    assert clearance((1.5, 1.0), w) == pytest.approx(0.348)            # box closer than wall
    assert segment_clearance((0.5, 0.5), (0.5, 2.5), w) == pytest.approx(0.5)
    assert segment_clearance((0.5, 1.5), (2.5, 1.5), w) == 0.0


def test_default_world():
    w = load_world(CONFIG)
    assert w.landmarks.ids() == list(range(12))
    assert len(w.obstacles) == 1 and w.obstacles[0] == Rect(1.5, 1.5, 0.31, 0.304)
    assert w.r_robot == pytest.approx(0.128, abs=1e-3)
    assert w.r_inflate == pytest.approx(0.188, abs=1e-3)     # r_margin 0.06
    assert w.r_guard == pytest.approx(0.158, abs=1e-3)
    assert w.start.theta == pytest.approx(-math.pi / 4)
    assert w.goal == (2.5, 0.5)


def test_extra_and_margins_from_config():
    d = raw_config()
    d['robot']['extra'] = 0.02
    d['margins']['r_margin'] = 0.1
    w = parse_world(d)
    assert w.r_inflate == pytest.approx(0.128 + 0.02 + 0.1, abs=1e-3)


@pytest.mark.parametrize('change', ['bigger_box', 'smaller_box', 'moved_tag', 'wrong_yaw'])
def test_tags_must_match_box(change):
    d = copy.deepcopy(raw_config())
    if change == 'bigger_box':
        d['obstacles'][0].update(w=0.45, h=0.45)
    elif change == 'smaller_box':
        d['obstacles'][0].update(w=0.26, h=0.26)
    elif change == 'moved_tag':
        d['landmarks'][9]['x'] = 1.70
    else:
        d['landmarks'][8]['yaw_deg'] = -90
    with pytest.raises(ValueError):
        parse_world(d)


def test_moved_box_with_tags_ok():
    d = raw_config()
    d['obstacles'][0].update(cx=1.2)
    for lm in d['landmarks'][8:]:
        lm['x'] -= 0.3
    assert parse_world(d).obstacles[0].cx == 1.2
