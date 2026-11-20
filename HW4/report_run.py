#!/usr/bin/env python3
"""Report figures and tables from hw4_nav runs (Mac, no ROS; report 3(d)).

    /opt/anaconda3/bin/python HW4/report_run.py [run_dir ...] [--log-dir HW4/logs]

No run_dir: the latest safe and the latest fast run in --log-dir. Per run writes
<run>/report.png (plan, estimated trajectory, replans, measured end) and
<run>/report.md; with a safe and a fast run also HW4/figures/report_compare.png and
HW4/report.md (side by side). Optional <run>/end.yaml {x, y}: tape-measured end.
"""
import argparse
import csv
import math
import os
import sys
from pathlib import Path

import matplotlib
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / 'robot_control'))
sys.path.insert(0, str(HERE))

import plot_plan  # noqa: E402
from robot_control.world import segment_clearance  # noqa: E402

DEFAULT_LOG_DIR = ROOT / 'HW4' / 'logs'
COLORS = {'safe': plot_plan.SAFE, 'fast': plot_plan.FAST}


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def latest_runs(log_dir):
    """{'safe': dir, 'fast': dir} of the newest run of each planner."""
    out = {}
    for d in sorted(Path(log_dir).glob('run_*_*')):
        kind = d.name.rsplit('_', 1)[-1]
        if kind in ('safe', 'fast') and (d / 'plan.yaml').exists():
            out[kind] = d
    return out


def load_run(run_dir):
    d = Path(run_dir)
    world, plan = plot_plan.load_run_plan(d)
    run = {'dir': d, 'world': world, 'plan': plan, 'record': read_yaml(d / 'plan.yaml'),
           'steps': read_csv(d / 'steps.csv'), 'traj': read_csv(d / 'trajectory.csv'),
           'summary': read_yaml(d / 'summary.yaml') if (d / 'summary.yaml').exists() else {},
           'end': read_yaml(d / 'end.yaml') if (d / 'end.yaml').exists() else None}
    run['kind'] = plan.kind
    return run


def drives(steps):
    """Estimated (start, end) of every drive: pose before -> predicted pose after."""
    return [((float(s['x0']), float(s['y0'])), (float(s['x1']), float(s['y1'])))
            for s in steps if s['kind'] == 'drive']


def metrics(run):
    world, plan, s = run['world'], run['plan'], run['summary']
    segs = drives(run['steps'])
    est_clear = min((segment_clearance(a, b, world) for a, b in segs), default=float('nan'))
    last = run['traj'][-1] if run['traj'] else None
    est_end = (float(last['x']), float(last['y'])) if last else (float('nan'),) * 2
    m = {
        'planned length (m)': plan.length,
        'driven, nominal (m)': sum(float(st['amount']) for st in run['steps']
                                   if st['kind'] == 'drive'),
        'time (s)': s.get('elapsed_s', float('nan')),
        'drives': s.get('counts', {}).get('drive', len(segs)),
        'spins': s.get('counts', {}).get('spin',
                                         sum(st['kind'] == 'spin' for st in run['steps'])),
        'looks': s.get('counts', {}).get('look', float('nan')),
        'replans': len(run['record'].get('replans', [])),
        'planned min clearance (m)': plan.min_clearance,
        'actual min clearance, est. (m)': est_clear,
        'actual body margin, est. (m)': est_clear - world.r_robot,
        'end error, estimate (cm)': 100 * math.dist(est_end, world.goal),
    }
    if run['end'] is not None:
        m['end error, measured (cm)'] = 100 * math.dist(
            (float(run['end']['x']), float(run['end']['y'])), world.goal)
    return m


def plan_stats(run):
    st = run['plan'].stats
    keys = ('nodes', 'edges', 'expanded', 'time_s', 'r')
    return {k: st[k] for k in keys if k in st}


# ---------- figures ----------

def draw_run(ax, run):
    world, color = run['world'], COLORS[run['kind']]
    plot_plan.draw_world(ax, world)
    plot_plan.draw_path(ax, run['plan'], color, label=f'{run["kind"]} plan')
    for k, rp in enumerate(run['record'].get('replans', [])):
        xy = list(zip(*rp['points']))
        ax.plot(*xy, '--', color=color, lw=1.5, label='replanned' if k == 0 else None)
        ax.plot(rp['pose'][0], rp['pose'][1], 'X', ms=10, color=plot_plan.INK,
                label='replan point' if k == 0 else None, zorder=9)
    tx = [float(r['x']) for r in run['traj']]
    ty = [float(r['y']) for r in run['traj']]
    ax.plot(tx, ty, '-', color=plot_plan.INK, lw=1, label='estimated trajectory', zorder=8)
    if run['end'] is not None:
        ax.plot(float(run['end']['x']), float(run['end']['y']), 'D', ms=7, mfc='white',
                mec=plot_plan.INK, label='measured end', zorder=9)
    plot_plan.legend_below(ax)


def plot_run(run, out_path):
    fig, ax = plt.subplots(figsize=(6.4, 7.0))
    draw_run(ax, run)
    m = metrics(run)
    plot_plan.add_text(ax, f"planned {m['planned length (m)']:.2f} m, "
                           f"driven {m['driven, nominal (m)']:.2f} m\n"
                           f"time {m['time (s)']:.0f} s, replans {m['replans']}\n"
                           f"end error (est.) {m['end error, estimate (cm)']:.1f} cm")
    ax.set_title(f'{run["kind"]} run: {run["dir"].name}')
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_compare(runs, out_path):
    fig, axes = plt.subplots(1, len(runs), figsize=(6.4 * len(runs), 7.0))
    for ax, run in zip(axes, runs):
        draw_run(ax, run)
        ax.set_title(run['kind'])
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------- tables ----------

def fmt(v):
    if isinstance(v, float):
        return 'n/a' if math.isnan(v) else f'{v:.3f}'
    return str(v)


def table(runs, rows_of):
    header = '| | ' + ' | '.join(r['kind'] for r in runs) + ' |'
    lines = [header, '|---' * (len(runs) + 1) + '|']
    cols = [rows_of(r) for r in runs]
    keys = list(dict.fromkeys(k for c in cols for k in c))     # union, in order
    for key in keys:
        lines.append(f'| {key} | ' + ' | '.join(fmt(c.get(key, float('nan'))) for c in cols)
                     + ' |')
    return lines


def markdown(runs):
    out = ['# HW4 run report', '']
    for r in runs:
        out.append(f'- {r["kind"]}: `{r["dir"].name}`'
                   + ('' if r['end'] else ' (no end.yaml: measured end error missing)'))
    out += ['', '## Result', ''] + table(runs, metrics)
    out += ['', '## Planner statistics', ''] + table(runs, plan_stats)
    return '\n'.join(out) + '\n'


def report(run_dirs, figures_dir=HERE / 'figures', report_md=HERE / 'report.md'):
    runs = [load_run(d) for d in run_dirs]
    for run in runs:
        plot_run(run, run['dir'] / 'report.png')
        (run['dir'] / 'report.md').write_text(markdown([run]))
    kinds = sorted({r['kind'] for r in runs}, reverse=True)       # safe, fast
    if kinds == ['safe', 'fast'] and len(runs) == 2:
        runs = sorted(runs, key=lambda r: kinds.index(r['kind']))
        Path(figures_dir).mkdir(parents=True, exist_ok=True)
        plot_compare(runs, Path(figures_dir) / 'report_compare.png')
        Path(report_md).write_text(markdown(runs))
    return runs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='*', help='run directories (default: latest safe + fast)')
    ap.add_argument('--log-dir', default=str(DEFAULT_LOG_DIR))
    args = ap.parse_args(argv)
    dirs = args.runs or list(latest_runs(args.log_dir).values())
    if not dirs:
        sys.exit(f'no runs in {args.log_dir}')
    runs = report(dirs)
    for r in runs:
        print(f'{r["kind"]}: {os.path.relpath(r["dir"] / "report.png")}')
    print(markdown(runs))


if __name__ == '__main__':
    main()
