"""Fit the coarse spin level as single pulses from closed-loop run logs (Mac, no ROS).

    ~/anaconda3/bin/python HW3/fit_spin_pulse.py [HW3/logs/run_*] [--cmd 0.35] [--min-T 0.1]
        [--calib robot_control/config/open_loop/study_wood2_hw2_ext.yaml] [--write]

Every spin step in steps.csv that was followed by a tag correction gives one sample:
actual angle = planned angle + heading correction at the next look, against the
COMMANDED pulse length T. Per direction a line  deg = rate * T + offset  is fitted and
(with --write) stored as spin: <dir>: pulse: {<cmd>: ...} in the calibration file, where
it replaces the open-loop trial fit and time_scale for that level (see ol_model.py).
min_T is raised to the fitted dead time (-offset / rate) + 0.05 s, so a pulse always
contains at least 50 ms of actual turning.
"""
import argparse
import csv
import glob
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'robot_control'))

from robot_control.ol_model import DIRS, Calibration, PulseModel  # noqa: E402

DEFAULT_CALIB = ROOT / 'robot_control' / 'config' / 'open_loop' / 'study_wood2_hw2_ext.yaml'
MOTION_S = 0.05      # s of actual turning the shortest pulse must contain


def wrap_deg(a):
    return ((a + 180.0) % 360.0) - 180.0


def samples(step_files, cmd):
    """{dir: [(T, actual deg, planned deg)]} from steps.csv files."""
    out = {d: [] for d in DIRS}
    for path in step_files:
        with open(path) as f:
            steps = list(csv.DictReader(f))
        for r, n in zip(steps, steps[1:]):
            if r['kind'] != 'spin' or abs(float(r['cmd']) - cmd) > 1e-9:
                continue
            dh = wrap_deg(float(n['theta0_deg']) - float(r['theta1_deg']))
            if abs(dh) < 1e-6:
                continue                    # no look correction: actual angle unknown
            sign = 1.0 if r['dir'] == 'ccw' else -1.0
            planned = float(r['amount'])
            out[r['dir']].append((float(r['T']), planned + sign * dh, planned))
    return out


def fit_pulse(pts, min_T):
    """Least-squares deg = rate * T + offset -> PulseModel; needs 3 samples, 2 durations."""
    if len(pts) < 3 or len({round(p[0], 3) for p in pts}) < 2:
        return None
    T = np.array([p[0] for p in pts])
    A = np.array([p[1] for p in pts])
    (rate, offset), *_ = np.linalg.lstsq(np.c_[T, np.ones_like(T)], A, rcond=None)
    rms = float(np.sqrt(np.mean((A - (rate * T + offset)) ** 2)))
    dead = max(-offset / rate, 0.0) if rate > 0 else 0.0
    return PulseModel(float(rate), float(offset), float(max(min_T, dead + MOTION_S)),
                      float(T.min()), float(T.max()), rms, len(pts))


def report(d, cmd, pts, pm, calib):
    lines = [f'spin {d} cmd {cmd:g}: {len(pts)} samples']
    if pm is None:
        return lines + ['  not enough samples (need 3 with 2 durations)']
    lines.append(f'  pulse: deg = {pm.rate:.1f} * T {pm.offset:+.1f}   rms {pm.rms:.1f} deg, '
                 f'T {pm.t_min:.2f}-{pm.t_max:.2f} s, min_T {pm.min_T:.2f} s '
                 f'-> smallest turn {pm.min_deg:.1f} deg')
    try:
        fits = calib.spin_fits(d)
        old = next(f for c, f in fits.items() if abs(c - cmd) < 1e-9)
        scale = float(calib.data['spin'][d].get('time_scale', 1.0))
        row = ['  commanded T -> old model (trial fit / time_scale) vs pulse:']
        for T in (0.15, 0.25, 0.4, 0.7, 1.2):
            row.append(f'    {T:.2f} s: {old.at(T / scale):6.1f} deg  vs {pm.at(T):6.1f} deg')
        lines += row
    except (StopIteration, KeyError):
        pass
    return lines


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('runs', nargs='*', help='run dirs or globs (default HW3/logs/run_*)')
    ap.add_argument('--cmd', type=float, default=0.35)
    ap.add_argument('--min-T', type=float, default=0.1, dest='min_T')
    ap.add_argument('--calib', default=str(DEFAULT_CALIB))
    ap.add_argument('--write', action='store_true', help='store the models in --calib')
    a = ap.parse_args()
    runs = a.runs or [str(ROOT / 'HW3' / 'logs' / 'run_*')]
    files = sorted(f for pat in runs for f in glob.glob(os.path.join(pat, '*', 'steps.csv'))
                   + glob.glob(os.path.join(pat, 'steps.csv')))
    if not files:
        raise SystemExit(f'no steps.csv under {runs}')
    print(f'{len(files)} steps.csv files')
    calib = Calibration.load(a.calib)
    models = {}
    for d, pts in samples(files, a.cmd).items():
        pm = fit_pulse(pts, a.min_T)
        print('\n'.join(report(d, a.cmd, pts, pm, calib)))
        if pm is not None:
            models[d] = pm
    if a.write and models:
        for d, pm in models.items():
            calib.set_pulse(d, a.cmd, pm)
        calib.save()
        print(f'written: {calib.path}')


if __name__ == '__main__':
    main()
