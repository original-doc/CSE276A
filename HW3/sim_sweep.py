"""Sweep the square size n in simulation (Mac, no ROS); m = n * tan 22.5 deg.

    /opt/anaconda3/bin/python HW3/sim_sweep.py [--seeds 10] [--sizes 1.0 1.5 2.0]
        [--out table.md] [--log-dir <dir>]

Each run: square x laps -> reset -> octagon x laps, robot centred in the 3 x 3 m
arena. Two tag-noise settings: 'sim' (SimNoise default, 3 mm) and 'model'
(noise = the filter's R). --log-dir also writes one full log set (n = 1.5,
seed 0, model noise) for report_run.py.
"""
import argparse
import math
import os
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / 'robot_control'), str(ROOT / 'robot_control' / 'test')]

from sim_hw3 import landmark_errors, mean_sigma, model_noise, run_hw3  # noqa: E402

from robot_control.paths import inscribed_octagon_side  # noqa: E402
from robot_control.stop_look import NavigationHalt  # noqa: E402

NOISES = {'sim': lambda: None, 'model': model_noise}


def phase_metrics(o, truth):
    errs = {k: np.mean([e for e, _ in landmark_errors(o.laps[k], o.start, truth).values()])
            for k in sorted(o.laps)}
    final = landmark_errors(o.final, o.start, truth)
    return {
        'reached': np.mean([r.reached for r in o.results]),
        'n_lm': len(o.final['landmarks']),
        'err_lap1': errs.get(1, np.nan), 'err_last': errs[max(errs)] if errs else np.nan,
        'err_max': max(e for e, _ in final.values()),
        'yaw_err_deg': math.degrees(np.mean([y for _, y in final.values()])),
        'sig_lap1': mean_sigma(o.laps[1]) if 1 in o.laps else np.nan,
        'sig_last': mean_sigma(o.final),
        'steps': o.steps, 'drive_m': o.drive_m,
        'gate': o.actions['gate'], 'skip_init': o.actions['skip_init'],
    }


def sweep(sizes, seeds, laps):
    """(n, noise, phase) -> list of per-seed metrics; halts counted separately."""
    rows, halts = {}, {}
    for n in sizes:
        for noise_name, noise in NOISES.items():
            for seed in range(seeds):
                with tempfile.TemporaryDirectory() as tmp:
                    try:
                        outcomes, truth, _ = run_hw3(Path(tmp), seed, n, laps, laps,
                                                     noise=noise())
                    except NavigationHalt:
                        halts[(n, noise_name)] = halts.get((n, noise_name), 0) + 1
                        continue
                for o in outcomes:
                    rows.setdefault((n, noise_name, o.name), []).append(
                        phase_metrics(o, truth.landmarks))
    return rows, halts


def table(rows, halts, seeds):
    cm = 100.0
    lines = ['| n (m) | m (m) | noise | phase | runs | reached | landmarks | '
             'err lap 1 (cm) | err lap 3 (cm) | max err (cm) | yaw err (deg) | '
             'σ lap 1 (cm) | σ lap 3 (cm) | err/σ | steps | drive (m) | gate | skip_init |',
             '|' + '---|' * 18]
    for (n, noise, phase), ms in sorted(rows.items()):
        def avg(k):
            return float(np.mean([m[k] for m in ms]))
        lines.append(
            f'| {n:.1f} | {inscribed_octagon_side(n):.2f} | {noise} | {phase} | '
            f'{len(ms)}/{seeds} | {avg("reached") * 100:.0f} % | {avg("n_lm"):.1f} | '
            f'{avg("err_lap1") * cm:.1f} | {avg("err_last") * cm:.1f} | '
            f'{avg("err_max") * cm:.1f} | {avg("yaw_err_deg"):.1f} | '
            f'{avg("sig_lap1") * cm:.1f} | {avg("sig_last") * cm:.1f} | '
            f'{avg("err_last") / avg("sig_last"):.2f} | {avg("steps"):.0f} | '
            f'{avg("drive_m"):.1f} | {avg("gate"):.1f} | {avg("skip_init"):.1f} |')
    if halts:
        lines += ['', 'NavigationHalt: ' + ', '.join(
            f'n={n} {noise}: {k}' for (n, noise), k in sorted(halts.items()))]
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--sizes', type=float, nargs='+', default=[1.0, 1.5, 2.0])
    ap.add_argument('--seeds', type=int, default=10)
    ap.add_argument('--laps', type=int, default=3)
    ap.add_argument('--out', help='also write the table to this file')
    ap.add_argument('--log-dir', help='write one full sim log set here (n=1.5, seed 0)')
    a = ap.parse_args()
    rows, halts = sweep(a.sizes, a.seeds, a.laps)
    text = table(rows, halts, a.seeds)
    print(text)
    if a.out:
        Path(a.out).write_text(text + '\n')
    if a.log_dir:
        os.makedirs(a.log_dir, exist_ok=True)
        with tempfile.TemporaryDirectory() as tmp:
            _, _, run_dir = run_hw3(Path(tmp), 0, 1.5, a.laps, a.laps, log_root=a.log_dir,
                                    noise=model_noise())
        print(f'logs: {run_dir}')


if __name__ == '__main__':
    main()
