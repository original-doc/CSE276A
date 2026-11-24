import importlib.util
import math
import os

import pytest
import yaml
from test_plan_run import RefuseFirst, Rig

REPORT_RUN = os.path.join(os.path.dirname(__file__), '..', '..', 'HW4', 'report_run.py')
spec = importlib.util.spec_from_file_location('hw4_report_run', REPORT_RUN)
report_run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report_run)


@pytest.fixture(scope='module')
def logs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp('hw4')
    safe = Rig(tmp / 'a', 'safe').session.run()
    fast = Rig(tmp / 'b', 'fast', guard=RefuseFirst(1)).session.run()     # one replan
    with open(os.path.join(fast.run_dir, 'end.yaml'), 'w') as f:
        yaml.safe_dump({'x': 2.53, 'y': 0.46}, f)
    return tmp, safe, fast


def test_latest_runs(logs):
    tmp, safe, fast = logs
    assert report_run.latest_runs(tmp / 'a' / 'logs') == {'safe': report_run.Path(safe.run_dir)}


def test_metrics(logs):
    _, safe, fast = logs
    m = report_run.metrics(report_run.load_run(fast.run_dir))
    assert m['planned length (m)'] == pytest.approx(fast.plans[0].length, abs=1e-3)
    assert m['replans'] == 1
    assert m['looks'] == fast.counts['look'] and m['drives'] == fast.counts['drive']
    assert m['time (s)'] == pytest.approx(fast.elapsed_s, abs=0.01)
    assert m['end error, measured (cm)'] == pytest.approx(100 * math.dist((2.53, 0.46),
                                                                          (2.5, 0.5)))
    assert m['end error, estimate (cm)'] <= 8.0 + 1e-6      # fast goal_pos 0.08 (nav.yaml fast:)
    assert m['actual min clearance, est. (m)'] > 0.128
    m = report_run.metrics(report_run.load_run(safe.run_dir))
    assert 'end error, measured (cm)' not in m and m['replans'] == 0


def test_report_files(logs, tmp_path):
    _, safe, fast = logs
    runs = report_run.report([fast.run_dir, safe.run_dir], figures_dir=tmp_path,
                             report_md=tmp_path / 'report.md')
    assert [r['kind'] for r in runs] == ['safe', 'fast']
    for s in (safe, fast):
        assert os.path.getsize(os.path.join(s.run_dir, 'report.png')) > 10_000
        assert os.path.exists(os.path.join(s.run_dir, 'report.md'))
    assert os.path.getsize(tmp_path / 'report_compare.png') > 10_000
    text = (tmp_path / 'report.md').read_text()
    assert '| | safe | fast |' in text and 'end error, measured' in text
    assert 'no end.yaml' in text                                    # safe has none
