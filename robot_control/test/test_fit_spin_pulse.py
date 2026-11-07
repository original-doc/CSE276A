import csv
import math

import pytest

from robot_control.fit_spin_pulse import fit_pulse, main, samples
from robot_control.ol_model import Calibration


def write_steps(path, rows):
    """rows (dir, T, planned, actual) -> steps.csv; the next row's theta0 holds the correction."""
    header = ['wp', 'lap', 'kind', 'dir', 'amount', 'cmd', 'L', 'R', 'T', 't_start', 't_stop',
              'x0', 'y0', 'theta0_deg', 'x1', 'y1', 'theta1_deg']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        theta = 0.0
        for k, (d, T, planned, actual) in enumerate(rows):
            sign = 1.0 if d == 'ccw' else -1.0
            w.writerow([k, 1, 'spin', d, planned, 0.35, 0, 0, T, 0, 0, 0, 0, theta, 0, 0,
                        theta + sign * planned])
            theta += sign * actual                     # the look after it corrects the heading
        w.writerow([99, 1, 'drive', 'fwd', 0.5, 0.3, 0, 0, 1.5, 0, 0, 0, 0, theta, 0.5, 0, theta])


def test_samples_and_fit(tmp_path):
    rows = [('ccw', T, 100 * T, 150 * T - 8) for T in (0.2, 0.3, 0.4, 0.6)]
    rows += [('cw', 0.3, 30, 40), ('cw', 0.5, 60, 70)]
    (tmp_path / 'sq').mkdir()
    write_steps(tmp_path / 'sq' / 'steps.csv', rows)
    s = samples([str(tmp_path / 'sq' / 'steps.csv')], 0.35)
    assert len(s['ccw']) == 4 and len(s['cw']) == 2
    T, actual, planned = s['ccw'][0]
    assert (T, planned, actual) == pytest.approx((0.2, 20.0, 22.0))
    pm = fit_pulse(s['ccw'], 0.1)
    assert (pm.rate, pm.offset) == pytest.approx((150.0, -8.0), abs=1e-6)
    assert pm.min_T == pytest.approx(8 / 150 + 0.05)     # dead time + 50 ms
    assert pm.n == 4 and pm.rms == pytest.approx(0.0, abs=1e-6)
    assert fit_pulse(s['cw'], 0.1) is None                # 2 samples: not enough


def test_main_writes_the_calibration(tmp_path):
    (tmp_path / 'run_a' / 'octagon').mkdir(parents=True)
    write_steps(tmp_path / 'run_a' / 'octagon' / 'steps.csv',
                [('ccw', T, 100 * T, 140 * T - 6) for T in (0.2, 0.3, 0.4)])
    c = Calibration.load(tmp_path / 'f.yaml', 'f', create=True)
    for T in (0.4, 0.8):
        c.spin_trials('ccw').append({'cmd': 0.35, 'T': T, 'angle': 125.0 * T - 6.0})
    c.save()
    main([str(tmp_path / 'run_a'), '--calib', str(tmp_path / 'f.yaml'), '--write'])
    pm = Calibration.load(tmp_path / 'f.yaml').pulse('ccw', 0.35)
    assert pm is not None and pm.rate == pytest.approx(140.0, abs=1e-3)
    assert math.isclose(pm.offset, -6.0, abs_tol=1e-3)
