import importlib.util
import math
import os

import pytest
from sim_hw3 import arena_map, mean_error, run_hw3

from robot_control.map_eval import truth_poses

HERE = os.path.dirname(__file__)
spec = importlib.util.spec_from_file_location(
    'report_run', os.path.join(HERE, '..', '..', 'HW3', 'report_run.py'))
report_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report_run)

TRUTH = truth_poses(arena_map())


@pytest.fixture(scope='module')
def sim_run(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('report')
    outcomes, truth, run_dir = run_hw3(tmp, 2, square_laps=2, octagon_laps=1,
                                       log_root=str(tmp / 'logs'))
    return outcomes, run_dir


def test_report_writes_figures_and_tables(sim_run):
    _, run_dir = sim_run
    text = report_run.report(run_dir, TRUTH)
    for phase in ('square', 'octagon'):
        files = os.listdir(os.path.join(run_dir, phase))
        assert {'map.png', 'sigma.png', 'report.md'} <= set(files)
    assert os.path.exists(os.path.join(run_dir, 'report.md'))
    assert '## Square vs octagon' in text and 'square/map.png' in text
    assert text.count('mean error, measured start') == 2


def test_lap_rows_match_the_simulation(sim_run):
    outcomes, run_dir = sim_run
    square = report_run.load_phase(os.path.join(run_dir, 'square'))
    rows = report_run.lap_rows(square, TRUTH)
    assert [r['lap'] for r in rows] == [1, 2]
    sim = outcomes[0]
    for r in rows:
        assert r['err'] == pytest.approx(mean_error(sim.laps[r['lap']], sim.start,
                                                    arena_map().landmarks), abs=2e-4)
        assert r['aligned'] <= r['err'] + 1e-9 or math.isnan(r['err'])
    assert rows[-1]['sigma'] < rows[0]['sigma']
    assert sum(r['update'] for r in rows) == sim.actions['update']


def test_placement_falls_back_to_rigid_fit(sim_run):
    _, run_dir = sim_run
    phase = report_run.load_phase(os.path.join(run_dir, 'octagon'))
    phase['start'] = None
    T, how = report_run.placement(phase, TRUTH)
    assert 'rigid fit' in how
    measured, aligned = report_run.map_errors(report_run.final_snapshot(phase), None, TRUTH)
    assert measured is None and aligned is not None
    assert any('rigid fit' in line for line in report_run.error_table(phase, TRUTH))


def obs_row(t, tag, action, dist='1.0', lap='1'):
    return {'t': f'{t:.2f}', 'id': str(tag), 'action': action, 'dist': dist, 'lap': lap}


def test_landmark_cases_count_reappearance():
    # tag 3: skipped once, added at t=2, seen at 3, missing at 4-5, back at 6, missing 7, back 8
    obs = [obs_row(1, 3, 'skip_init', '2.7'), obs_row(2, 3, 'init', '2.2'),
           obs_row(3, 3, 'update'), obs_row(6, 3, 'gate'), obs_row(8, 3, 'update'),
           obs_row(4, 5, 'init'), obs_row(5, 5, 'update'),
           {'t': '7.00', 'id': '', 'action': 'rejected', 'dist': '', 'lap': '1'}]
    lms = [{'t': f'{t:.2f}', 'id': '3'} for t in range(2, 9)]
    cases = report_run.landmark_cases({'obs': obs, 'lms': lms})
    c = cases[3]
    assert (c['init_t'], c['init_dist'], c['skip_init']) == (2.0, 2.2, 1)
    assert (c['update'], c['gate'], c['reappear']) == (2, 1, 2)
    assert cases[5]['reappear'] == 0


def test_spread_and_outward():
    assert report_run.spread([5.0, 5.1, 1.0], 0.5) == [5.0, 5.5, 1.0]
    dx, dy = report_run.outward(report_run.Pose2(3.0, 1.5, 0.0))
    assert dx > 0 and dy == 0


def test_motion():
    steps = [{'kind': 'drive', 'amount': '0.5'}, {'kind': 'spin', 'amount': '90'},
             {'kind': 'drive', 'amount': '0.25'}]
    assert report_run.motion(steps) == {'drive_m': 0.75, 'spin_deg': 90.0, 'drives': 2,
                                        'spins': 1}
