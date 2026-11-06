import math
import os

import numpy as np
import pytest
import yaml
from sim_hw3 import (
    centred_start, landmark_errors, mean_error, mean_sigma, model_noise, run_hw3,
)

from robot_control.paths import corner_offset, inscribed_octagon_side
from robot_control.se2 import Pose2

ALL_IDS = list(range(8))


def test_centred_start_matches_spec():
    p = centred_start(1.5)
    assert (p.x, p.y, p.theta) == pytest.approx((1.189, 0.75, 0.0), abs=1e-3)
    n = 2.0
    p = centred_start(n)
    assert p.x - corner_offset(n, inscribed_octagon_side(n)) == pytest.approx(0.5)


@pytest.mark.parametrize('seed', range(5))
def test_full_run_square_then_octagon(tmp_path, seed):
    outcomes, truth, _ = run_hw3(tmp_path, seed)
    assert [o.name for o in outcomes] == ['square', 'octagon']
    for o in outcomes:
        assert all(r.reached for r in o.results), [r.note for r in o.results if not r.reached]
        assert sorted(lm['id'] for lm in o.final['landmarks']) == ALL_IDS
        assert sorted(o.laps) == [1, 2, 3]
        assert mean_error(o.final, o.start, truth.landmarks) < 0.10
        assert mean_sigma(o.laps[3]) < mean_sigma(o.laps[1])


def test_octagon_phase_starts_from_scratch(tmp_path):
    outcomes, truth, _ = run_hw3(tmp_path, 0, square_laps=1, octagon_laps=1)
    square, octagon = outcomes
    assert octagon.start != centred_start(1.5)          # where the square phase really ended
    assert math.hypot(octagon.start.x - 1.189, octagon.start.y - 0.75) < 0.1
    first_lm = octagon.laps[1]['landmarks']
    assert len(first_lm) == 8
    # its own frame: errors are small only after mapping with the octagon start
    assert mean_error(octagon.final, octagon.start, truth.landmarks) < 0.10


def test_no_reset_keeps_one_map(tmp_path):
    outcomes, truth, _ = run_hw3(tmp_path, 0, square_laps=1, octagon_laps=1, reset=False)
    square, octagon = outcomes
    assert octagon.start == square.start
    assert [m['id'] for m in octagon.final['landmarks']] == [
        m['id'] for m in square.final['landmarks']]
    assert mean_sigma(octagon.final) < mean_sigma(square.final)


def test_skipped_phase(tmp_path):
    outcomes, _, _ = run_hw3(tmp_path, 0, square_laps=0, octagon_laps=1)
    assert [o.name for o in outcomes] == ['octagon']


def test_filter_noise_model_run_is_consistent(tmp_path):
    """Sim tag noise = the filter's R: errors stay within ~2 sigma, maps still improve."""
    for seed in range(3):
        outcomes, truth, _ = run_hw3(tmp_path, seed, noise=model_noise())
        for o in outcomes:
            assert len(o.final['landmarks']) == 8
            assert mean_sigma(o.laps[3]) < mean_sigma(o.laps[1])
            errs = landmark_errors(o.final, o.start, truth.landmarks)
            assert np.mean([e for e, _ in errs.values()]) < 2 * mean_sigma(o.final)


def test_logs_written_per_phase(tmp_path):
    outcomes, _, run_dir = run_hw3(tmp_path, 1, square_laps=2, octagon_laps=1,
                                   log_root=str(tmp_path / 'logs'))
    assert sorted(os.listdir(run_dir)) == ['config.yaml', 'octagon', 'square']
    sq = os.listdir(os.path.join(run_dir, 'square'))
    for f in ('trajectory.csv', 'steps.csv', 'observations.csv', 'landmarks.csv',
              'waypoints.csv', 'map_lap1.yaml', 'map_lap2.yaml', 'final_state.npz',
              'start.yaml'):
        assert f in sq
    assert 'map_lap1.yaml' in os.listdir(os.path.join(run_dir, 'octagon'))
    with open(os.path.join(run_dir, 'octagon', 'start.yaml')) as f:
        start = yaml.safe_load(f)
    s = outcomes[1].start
    assert Pose2(start['x'], start['y'], math.radians(start['theta_deg'])) == pytest.approx(
        s, abs=1e-3)
    with open(os.path.join(run_dir, 'square', 'map_lap2.yaml')) as f:
        lap2 = yaml.safe_load(f)
    assert lap2['stats']['update'] > 0 and len(lap2['landmarks']) == 8
