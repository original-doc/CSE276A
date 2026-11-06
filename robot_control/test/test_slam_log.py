import csv
import math
import os
from datetime import datetime

import numpy as np
import pytest
import yaml

from robot_control.estimator import GATE, INIT, REJECTED, SKIP_INIT, TagRecord, UPDATE
from robot_control.nav_log import SlamCsvLog, lap_stats, new_run_dir
from robot_control.ol_planner import Step
from robot_control.se2 import Pose2
from robot_control.stop_look import WaypointResult
from robot_control.tag_obs import make_obs


def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


class FakeClock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


class FakeEstimator:
    """Fixed pose; landmarks set by the test."""

    def __init__(self):
        self.p = Pose2(0.1, 0.2, math.radians(30))
        self.cov = np.diag([1e-4, 4e-4, 1e-3])
        self.lms = []       # (id, x, y, yaw, 3x3 cov, n_updates)

    def pose(self):
        return self.p

    def pose_cov(self):
        return self.cov

    def snapshot(self):
        return {'robot': {'x': self.p.x, 'y': self.p.y, 'theta': self.p.theta,
                          'cov': self.cov.tolist()},
                'landmarks': [{'id': i, 'x': x, 'y': y, 'yaw': yaw, 'cov': c.tolist(),
                               'n_updates': n} for i, x, y, yaw, c, n in self.lms]}

    def state(self):
        s = np.concatenate([[self.p.x, self.p.y, self.p.theta]]
                           + [[x, y, yaw] for _, x, y, yaw, _, _ in self.lms])
        return s, np.eye(len(s)) * 1e-3, [m[0] for m in self.lms]


def record(tag_id, action, d2=0.0):
    return TagRecord(make_obs(tag_id, 0.0, 1.0, 0.1, math.pi), action, d2)


def waypoint(i):
    return WaypointResult(i, Pose2(i, 0, 0), True, True, Pose2(i, 0, 0), (0.01, 0.01, 0.02), 2)


LAPS = [1, 1, 2, 2]
COV0 = np.array([[4e-4, 1e-5, 0.0], [1e-5, 9e-4, 0.0], [0.0, 0.0, 2.5e-3]])


def write_phase(tmp_path):
    clock, est = FakeClock(), FakeEstimator()
    log = SlamCsvLog(str(tmp_path / 'square'), est, LAPS, clock)
    # lap 1: tag 3 added, tag 5 too far; then a step and an update
    est.lms = [(3, 1.5, 0.3, math.pi, COV0, 0)]
    clock.t += 1
    log.on_look(1, [record(3, INIT), record(5, SKIP_INIT),
                    TagRecord(None, REJECTED, 0.0, 'tag 9: only 2 frames')], est)
    log.on_step(1, Step('drive', 'fwd', 0.5, 0.3, 1.3), est.p, est.p, est.cov, (11.0, 12.3))
    clock.t = 13.0
    est.lms = [(3, 1.51, 0.29, math.pi, COV0 / 2, 1)]
    log.on_look(1, [record(3, UPDATE, 2.0)], est)
    log.on_waypoint(waypoint(1))
    log.on_look(2, [record(3, UPDATE, 4.0)], est)
    log.on_waypoint(waypoint(2))            # end of lap 1
    # lap 2: one gated look
    clock.t = 20.0
    log.on_look(3, [record(3, GATE, 30.0)], est)
    log.on_waypoint(waypoint(3))
    log.on_waypoint(waypoint(4))            # end of lap 2
    log.close()
    return log.dir, est


def test_lap_stats():
    st = lap_stats([record(1, UPDATE, 2.0), record(1, UPDATE, 4.0), record(2, GATE, 40.0),
                    record(3, INIT)])
    assert st == {'update': 2, 'init': 1, 'gate': 1, 'skip_init': 0, 'rejected': 0,
                  'mean_d2': 3.0}
    assert lap_stats([])['mean_d2'] is None


def test_new_run_dir_name():
    assert new_run_dir('/logs', datetime(2026, 11, 10, 9, 5, 7)) == '/logs/run_20261110_090507'


def test_observations_have_lap_and_action(tmp_path):
    d, _ = write_phase(tmp_path)
    obs = read_csv(os.path.join(d, 'observations.csv'))
    assert [(o['wp'], o['lap'], o['id'], o['action'], o['accepted']) for o in obs] == [
        ('1', '1', '3', 'init', '1'), ('1', '1', '5', 'skip_init', '0'),
        ('1', '1', '', 'rejected', '0'), ('1', '1', '3', 'update', '1'),
        ('2', '1', '3', 'update', '1'), ('3', '2', '3', 'gate', '0')]
    assert obs[2]['reason'] == 'tag 9: only 2 frames'
    steps = read_csv(os.path.join(d, 'steps.csv'))
    assert steps[0]['lap'] == '1' and steps[0]['kind'] == 'drive'
    wps = read_csv(os.path.join(d, 'waypoints.csv'))
    assert [w['lap'] for w in wps] == ['1', '1', '2', '2']


def test_landmarks_csv_matches_snapshot(tmp_path):
    d, _ = write_phase(tmp_path)
    rows = read_csv(os.path.join(d, 'landmarks.csv'))
    assert len(rows) == 4                       # one landmark after each of 4 looks
    first, second = rows[0], rows[1]
    assert first['t'] == '1.00' and first['lap'] == '1' and first['n_updates'] == '0'
    assert float(first['x']) == pytest.approx(1.5) and float(first['psi_deg']) == 180.0
    assert float(first['sxx']) == pytest.approx(COV0[0, 0])
    assert float(first['sxy']) == pytest.approx(COV0[0, 1])
    assert float(first['syy']) == pytest.approx(COV0[1, 1])
    assert float(first['spsi_deg']) == pytest.approx(math.degrees(0.05), abs=0.01)
    assert float(second['x']) == pytest.approx(1.51) and second['n_updates'] == '1'
    assert rows[-1]['lap'] == '2'


def test_map_lap_files(tmp_path):
    d, est = write_phase(tmp_path)
    assert sorted(f for f in os.listdir(d) if f.startswith('map_lap')) == [
        'map_lap1.yaml', 'map_lap2.yaml']
    with open(os.path.join(d, 'map_lap1.yaml')) as f:
        m1 = yaml.safe_load(f)
    assert m1['lap'] == 1 and m1['t'] == 3.0
    assert m1['stats'] == {'looks': 3, 'update': 2, 'init': 1, 'gate': 0, 'skip_init': 1,
                           'rejected': 1, 'mean_d2': 3.0}
    lm = m1['landmarks'][0]
    assert lm['id'] == 3 and lm['x'] == pytest.approx(1.51)
    np.testing.assert_allclose(lm['cov'], COV0 / 2)
    assert m1['robot']['theta'] == pytest.approx(est.p.theta)
    with open(os.path.join(d, 'map_lap2.yaml')) as f:
        m2 = yaml.safe_load(f)
    assert m2['stats']['gate'] == 1 and m2['stats']['update'] == 0
    assert m2['stats']['mean_d2'] is None


def test_final_state_npz(tmp_path):
    d, est = write_phase(tmp_path)
    data = np.load(os.path.join(d, 'final_state.npz'))
    s, Sigma, ids = est.state()
    np.testing.assert_allclose(data['s'], s)
    np.testing.assert_allclose(data['Sigma'], Sigma)
    assert data['ids'].tolist() == [3]


def test_trajectory_starts_at_estimator_pose(tmp_path):
    d, est = write_phase(tmp_path)
    traj = read_csv(os.path.join(d, 'trajectory.csv'))
    assert traj[0]['source'] == 'start'
    assert float(traj[0]['x']) == pytest.approx(est.p.x)
    assert float(traj[0]['sy']) == pytest.approx(0.02)
