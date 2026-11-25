import math

import numpy as np
import pytest
from sim_hw4 import load_yaml, measured_noise, NAV_YAML, run_hw4, WORLD_YAML
from sim_rover import SimRover

from robot_control.plan_run import config_for
from robot_control.se2 import Pose2
from robot_control.world import parse_world

WORLD = parse_world(load_yaml(WORLD_YAML))
NAV = load_yaml(NAV_YAML)
SEEDS = range(10)


def test_box_hides_tags_behind_it():
    rover = SimRover(WORLD.landmarks, Pose2(1.0, 1.5, 0.0), np.random.default_rng(0),
                     measured_noise(), occluders=WORLD.obstacles)
    seen, _ = rover.look()
    assert 9 in seen                        # the face towards the robot
    assert 2 not in seen and 3 not in seen  # wall tags behind the box (x = 3)
    open_view = SimRover(WORLD.landmarks, Pose2(1.0, 1.5, 0.0), np.random.default_rng(0),
                         measured_noise())
    assert {2, 3} <= set(open_view.look()[0])       # same view without the box


@pytest.fixture(scope='module', params=['safe', 'fast'])
def outcomes(request, tmp_path_factory):
    return [run_hw4(tmp_path_factory.mktemp(f'{request.param}{s}'), request.param, s)
            for s in SEEDS]


def test_no_halt_and_no_collision(outcomes):
    for o in outcomes:
        assert o.halted == '', (o.kind, o.seed, o.halted)
        # the true path (not the estimate) never touches a wall or the box
        assert o.min_clearance >= WORLD.r_robot, (o.kind, o.seed, o.min_clearance)


def test_reaches_goal(outcomes):
    errors = []
    for o in outcomes:
        if o.halted:
            continue                    # reported by test_no_halt_and_no_collision
        last = o.summary.results[-1]
        assert last.reached and last.tag_based
        goal_pos = config_for(NAV, o.kind)['exec']['goal_pos']
        est = math.dist((last.pose.x, last.pose.y), WORLD.goal)
        assert est <= goal_pos + 1e-9                   # the filter says: within goal_pos
        assert o.end_error <= goal_pos + 0.03, (o.kind, o.seed, o.end_error)   # + estimate error
        errors.append(o.end_error)
    limit = 0.04 if outcomes[0].kind == 'safe' else 0.06     # fast: goal_pos 0.08
    assert np.mean(errors) <= limit


def test_replans_are_logged(outcomes):
    for o in outcomes:
        if o.halted:
            continue
        s = o.summary
        assert s.replans == len(s.plans) - 1 <= 3
        assert s.counts['look'] >= len(s.results)
