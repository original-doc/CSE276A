"""Report figures and tables from one hw3_slam run (Mac, no ROS).

    /opt/anaconda3/bin/python HW3/report_run.py [HW3/logs/run_<time>[/square]]
        [--truth HW3/truth_map.yaml]

Give a run directory (both phases + square vs octagon) or one phase directory.
Each phase needs <phase>/start.yaml {x, y, theta_deg}: its measured start in the
arena. Without it the map is placed with the best rigid fit only (marked as such).

Writes per phase: map.png (arena: trajectory, vertices, estimated tags with 2σ
ellipses, lap-1 estimate, truth), sigma.png (σ of every tag over time), report.md;
and run_dir/report.md with all phases and the comparison.
"""
import argparse
import csv
import glob
import math
import os
import sys
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Ellipse, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'robot_control'))

from robot_control import map_eval  # noqa: E402
from robot_control.landmark_map import load_map  # noqa: E402
from robot_control.se2 import Pose2  # noqa: E402

DEFAULT_LOGS = ROOT / 'HW3' / 'logs'
DEFAULT_TRUTH = ROOT / 'HW3' / 'truth_map.yaml'
ARENA = 3.0
PHASES = ('square', 'octagon')
# categorical slots in fixed order (validated: dataviz validate_palette.js, light)
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6',
          '#e34948']
INK, MUTED, GRID = '#1f1f1e', '#8a8984', '#e4e3df'
SEEN = ('update', 'gate', 'init', 'skip_init')


# ---------- loading ----------

def read_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def read_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def latest_run(log_dir):
    runs = sorted(glob.glob(os.path.join(log_dir, 'run_*')))
    if not runs:
        raise SystemExit(f'no run_* in {log_dir}')
    return runs[-1]


def load_phase(phase_dir):
    def path(name):
        return os.path.join(phase_dir, name)
    laps = {}
    for f in glob.glob(path('map_lap*.yaml')):
        m = read_yaml(f)
        laps[int(m['lap'])] = m
    return {
        'name': os.path.basename(os.path.normpath(phase_dir)), 'dir': phase_dir,
        'traj': read_csv(path('trajectory.csv')), 'steps': read_csv(path('steps.csv')),
        'obs': read_csv(path('observations.csv')), 'lms': read_csv(path('landmarks.csv')),
        'wps': read_csv(path('waypoints.csv')), 'laps': dict(sorted(laps.items())),
        'start': map_eval.load_start(path('start.yaml')) if os.path.exists(path('start.yaml'))
        else None,
    }


def phase_dirs(path):
    """A run dir -> its phase dirs; a phase dir -> itself."""
    subs = [os.path.join(path, p) for p in PHASES if os.path.isdir(os.path.join(path, p))]
    return subs or [path]


def final_snapshot(phase):
    return phase['laps'][max(phase['laps'])] if phase['laps'] else None


# ---------- numbers ----------

def placement(phase, truth):
    """Map -> arena transform: the measured start, or the rigid fit of the final map."""
    if phase['start'] is not None:
        return phase['start'], 'measured start'
    _, T = map_eval.align(map_eval.snapshot_poses(final_snapshot(phase)), truth)
    return T, 'best rigid fit (no start.yaml)'


def map_errors(snapshot, start, truth):
    """(errors with the measured start or None, errors after the best rigid fit)."""
    est = map_eval.snapshot_poses(snapshot)
    measured = None
    if start is not None:
        measured = map_eval.landmark_errors(map_eval.poses_to_arena(start, est), truth)
    aligned = None
    if len(set(est) & set(truth)) >= 2:
        aligned = map_eval.landmark_errors(map_eval.align(est, truth)[0], truth)
    return measured, aligned


def mean_sigma(snapshot):
    """Mean over landmarks of sqrt(Sxx + Syy) (m)."""
    lms = snapshot['landmarks']
    return float(np.mean([math.sqrt(m['cov'][0][0] + m['cov'][1][1]) for m in lms])) \
        if lms else math.nan


def lap_rows(phase, truth):
    rows = []
    for lap, snap in phase['laps'].items():
        measured, aligned = map_errors(snap, phase['start'], truth)
        rows.append({'lap': lap, 'n': len(snap['landmarks']),
                     'err': map_eval.mean_error(measured)['mean'] if measured else math.nan,
                     'aligned': map_eval.mean_error(aligned)['mean'] if aligned else math.nan,
                     'sigma': mean_sigma(snap), **snap['stats']})
    return rows


def look_times(phase):
    """Times of the looks that left a trace (a tag seen, or a non-empty map)."""
    ts = {float(r['t']) for r in phase['obs']} | {float(r['t']) for r in phase['lms']}
    return sorted(ts)


def landmark_cases(phase):
    """Per tag: first init (t, dist, lap), skip_init count, updates, gates, re-appearances.

    Re-appearance = seen in a look after at least one look (since its init) without it.
    """
    seen, cases = {}, {}
    for r in phase['obs']:
        if not r['id'] or r['action'] not in SEEN:
            continue
        i, t = int(r['id']), float(r['t'])
        seen.setdefault(i, set()).add(t)
        c = cases.setdefault(i, {'init_t': None, 'init_dist': None, 'init_lap': None,
                                 'skip_init': 0, 'update': 0, 'gate': 0, 'reappear': 0})
        if r['action'] == 'init':
            c['init_t'], c['init_dist'], c['init_lap'] = t, float(r['dist']), r['lap']
        elif r['action'] in c:
            c[r['action']] += 1
    times = look_times(phase)
    for i, c in cases.items():
        if c['init_t'] is None:
            continue
        after = [t in seen[i] for t in times if t >= c['init_t']]
        c['reappear'] = sum(1 for a, b in zip(after, after[1:]) if b and not a)
    return dict(sorted(cases.items()))


def motion(steps):
    drives = [float(s['amount']) for s in steps if s['kind'] == 'drive']
    spins = [float(s['amount']) for s in steps if s['kind'] == 'spin']
    return {'drive_m': sum(drives), 'spin_deg': sum(spins), 'drives': len(drives),
            'spins': len(spins)}


# ---------- markdown ----------

def cm(v):
    return '—' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v * 100:.1f}'


def error_table(phase, truth):
    snap = final_snapshot(phase)
    measured, aligned = map_errors(snap, phase['start'], truth)
    al = {e.id: e for e in aligned.errors} if aligned else {}
    T, how = placement(phase, truth)
    est = map_eval.poses_to_arena(T, map_eval.snapshot_poses(snap))
    cov = {m['id']: np.array(m['cov']) for m in snap['landmarks']}
    lines = [f'Placement in the arena: {how} {fmt_pose(T)}', '',
             '| tag | estimate (arena) | truth | 2σ ellipse (cm) | error, measured start (cm) |'
             ' error, rigid fit (cm) | ψ error (°) |', '|---|---|---|---|---|---|---|']
    meas = {e.id: e for e in measured.errors} if measured else {}
    for i in sorted(est):
        a, b, _ = map_eval.cov_ellipse(map_eval.cov_to_arena(T, cov[i][:2, :2]))
        e = meas.get(i) or al.get(i)
        lines.append(f'| {i} | {fmt_pose(est[i])} | {fmt_pose(truth[i]) if i in truth else "—"}'
                     f' | {a * 100:.1f} × {b * 100:.1f} | {cm(meas[i].pos) if i in meas else "—"}'
                     f' | {cm(al[i].pos) if i in al else "—"} | '
                     f'{math.degrees(e.yaw):+.1f} |' if e else ' | — | — | — |')
    lines.append('')
    for label, me in (('measured start', measured), ('best rigid fit', aligned)):
        if me:
            s = map_eval.mean_error(me)
            lines.append(f'- **mean error, {label}: {cm(s["mean"])} cm** (rms {cm(s["rms"])}, '
                         f'max {cm(s["max"])} cm, mean |ψ| {math.degrees(s["mean_yaw"]):.1f}°)')
    scale, res = map_eval.similarity_errors(map_eval.snapshot_poses(snap), truth)
    if scale is not None:
        lines.append(f'- mean error after rigid fit **with scale**: {cm(res)} cm; the map is '
                     f'**{1 / scale:.3f}×** the truth (a uniform scale error, e.g. tag size or '
                     f'arena size)')
    me = measured or aligned
    if me and me.missing:
        lines.append(f'- never mapped: {me.missing}')
    if me and me.extra:
        lines.append(f'- mapped but not in the truth: {me.extra}')
    return lines


def lap_table(phase, truth):
    lines = ['| lap | landmarks | mean error (cm) | after rigid fit (cm) | mean σ (cm) | looks |'
             ' update | init | gate | skip_init | mean d² |', '|' + '---|' * 11]
    for r in lap_rows(phase, truth):
        d2 = '—' if r['mean_d2'] is None else f'{r["mean_d2"]:.2f}'
        lines.append(f'| {r["lap"]} | {r["n"]} | {cm(r["err"])} | {cm(r["aligned"])} | '
                     f'{cm(r["sigma"])} | {r["looks"]} | {r["update"]} | {r["init"]} | '
                     f'{r["gate"]} | {r["skip_init"]} | {d2} |')
    return lines


def cases_table(phase):
    lines = ['| tag | first added (t s, dist m, lap) | skip_init | updates | gated | '
             're-appeared |', '|---|---|---|---|---|---|']
    for i, c in landmark_cases(phase).items():
        first = (f'{c["init_t"]:.1f}, {c["init_dist"]:.2f}, lap {c["init_lap"]}'
                 if c['init_t'] is not None else 'never')
        lines.append(f'| {i} | {first} | {c["skip_init"]} | {c["update"]} | {c["gate"]} | '
                     f'{c["reappear"]} |')
    return lines


def phase_markdown(phase, truth):
    m = motion(phase['steps'])
    wps = phase['wps']
    reached = sum(w['reached'] == '1' for w in wps)
    return '\n'.join(
        [f'## Phase `{phase["name"]}`', '', '![map](map.png)', '', '![sigma](sigma.png)', '',
         f'- vertices reached: {reached}/{len(wps)}; motion: {m["drive_m"]:.2f} m driven, '
         f'{m["spin_deg"]:.0f}° turned, {m["drives"]} drives + {m["spins"]} spins', '',
         '### Final map vs truth', ''] + error_table(phase, truth)
        + ['', '### Lap by lap (lap 1 builds the map; later laps re-observe it)', '']
        + lap_table(phase, truth)
        + ['', '### Landmark cases (new / out of view / re-appeared)', ''] + cases_table(phase))


def comparison_markdown(phases, truth):
    lines = ['## Square vs octagon (final maps)', '',
             '| phase | laps | landmarks | mean error (cm) | after rigid fit (cm) | '
             'with scale (cm) | map / truth | lap 1 error (cm) | mean σ lap 1 → last (cm) | '
             'drive (m) | turns |', '|' + '---|' * 11]
    for p in phases:
        rows = lap_rows(p, truth)
        first, last = rows[0], rows[-1]
        m = motion(p['steps'])
        scale, res = map_eval.similarity_errors(
            map_eval.snapshot_poses(final_snapshot(p)), truth)
        ratio = f'{1 / scale:.3f}' if scale else '—'
        lines.append(f'| {p["name"]} | {len(rows)} | {last["n"]} | {cm(last["err"])} | '
                     f'{cm(last["aligned"])} | {cm(res)} | {ratio} | {cm(first["err"])} | '
                     f'{cm(first["sigma"])} → '
                     f'{cm(last["sigma"])} | {m["drive_m"]:.1f} | {m["spins"]} |')
    return lines


def fmt_pose(p):
    return f'({p.x:.3f}, {p.y:.3f}, {math.degrees(p.theta):+.1f}°)'


# ---------- figures ----------

def arrow(ax, p, length, **kw):
    ax.annotate('', xy=(p.x + length * math.cos(p.theta), p.y + length * math.sin(p.theta)),
                xytext=(p.x, p.y), arrowprops=dict(arrowstyle='->', lw=1.5, **kw))


def outward(p, dist=16):
    """Label offset (points) pointing from the arena centre to p, so it sits outside."""
    dx, dy = p.x - ARENA / 2, p.y - ARENA / 2
    n = math.hypot(dx, dy) or 1.0
    return dist * 1.6 * dx / n, dist * dy / n


def spread(values, min_gap):
    """Nudge sorted label positions apart by at least min_gap; returns them in input order."""
    order = sorted(range(len(values)), key=lambda k: values[k])
    out, last = list(values), -math.inf
    for k in order:
        out[k] = max(values[k], last + min_gap)
        last = out[k]
    return out


def style(ax):
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelcolor=INK)


LAP_COLORS = ['#9ecae1', '#4292c6', '#08306b']      # light -> dark blue, lap 1 -> 3


def lap_color(lap, n_laps):
    """Colour of lap `lap` (1-based) when there are n_laps laps."""
    if n_laps <= 1:
        return LAP_COLORS[-1]
    return LAP_COLORS[round((lap - 1) * (len(LAP_COLORS) - 1) / (n_laps - 1))]


def lap_of_rows(phase):
    """Lap (1-based) of every trajectory row, from the lap end times in waypoints.csv."""
    ends = {}
    for w in phase['wps']:
        ends[int(w['lap'])] = float(w['t'])
    bounds = sorted(ends.items())
    laps = []
    for r in phase['traj']:
        t = float(r['t'])
        laps.append(next((lap for lap, t_end in bounds if t <= t_end), bounds[-1][0]))
    return laps


def plot_map(phase, truth, out_path):
    T, how = placement(phase, truth)
    fig, ax = plt.subplots(figsize=(7, 7))
    style(ax)
    ax.add_patch(Rectangle((0, 0), ARENA, ARENA, fill=False, ec=MUTED, lw=1.5))
    traj = [map_eval.to_arena(T, Pose2(float(r['x']), float(r['y']),
                                       math.radians(float(r['theta_deg']))))
            for r in phase['traj']]
    laps = lap_of_rows(phase)
    n_laps = max(laps)
    # One colour per lap; each segment starts at the last point of the previous lap.
    for lap in range(1, n_laps + 1):
        idx = [k for k, l in enumerate(laps) if l == lap]
        if not idx:
            continue
        if idx[0] > 0:
            idx = [idx[0] - 1] + idx
        ax.plot([traj[k].x for k in idx], [traj[k].y for k in idx], '-',
                color=lap_color(lap, n_laps), lw=2, label=f'trajectory lap {lap}',
                zorder=2 + lap)
    goals = [map_eval.to_arena(T, Pose2(float(w['goal_x']), float(w['goal_y']), 0.0))
             for w in phase['wps']]
    ax.scatter([g.x for g in goals], [g.y for g in goals], s=60, facecolors='none',
               edgecolors=INK, lw=1.2, label='vertices', zorder=6)
    for i, t in truth.items():
        ax.scatter([t.x], [t.y], s=90, marker='s', color=INK, zorder=5,
                   label='truth (tape)' if i == min(truth) else None)
        arrow(ax, t, 0.18, color=INK)
        ax.annotate(f'tag {i}', (t.x, t.y), textcoords='offset points', color=INK,
                    xytext=outward(t), ha='center', va='center', fontsize=9)
    snaps = list(phase['laps'].items())
    # Map after every lap but the last: hollow circles in that lap's colour.
    for lap, snap in snaps[:-1]:
        est = map_eval.poses_to_arena(T, map_eval.snapshot_poses(snap))
        ax.scatter([p.x for p in est.values()], [p.y for p in est.values()], s=40,
                   facecolors='none', edgecolors=lap_color(lap, n_laps), lw=1.5, zorder=7,
                   label=f'map after lap {lap}')
    last_lap, final = snaps[-1]
    fc = lap_color(last_lap, n_laps)
    est = map_eval.poses_to_arena(T, map_eval.snapshot_poses(final))
    for k, m in enumerate(final['landmarks']):
        p = est[m['id']]
        a, b, ang = map_eval.cov_ellipse(map_eval.cov_to_arena(T, np.array(m['cov'])[:2, :2]))
        ax.add_patch(Ellipse((p.x, p.y), 2 * a, 2 * b, angle=math.degrees(ang), fill=False,
                             ec=fc, lw=1.5, zorder=7,
                             label=f'2\u03c3 ellipse after lap {last_lap}' if k == 0 else None))
        ax.scatter([p.x], [p.y], s=40, color=fc, ec='white', lw=1, zorder=8,
                   label=f'map after lap {last_lap} (final)' if k == 0 else None)
        arrow(ax, p, 0.15, color=fc)
    ax.set_aspect('equal')
    ax.set_xlim(-0.3, ARENA + 0.3)
    ax.set_ylim(-0.3, ARENA + 0.3)
    ax.set_xlabel('x (m)', color=INK)
    ax.set_ylabel('y (m)', color=INK)
    ax.set_title(f'{phase["name"]}: trajectory and map by lap (arena frame; {how})', color=INK,
                 fontsize=11)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.09), ncol=3, fontsize=8,
              frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def sigma_series(phase):
    """id -> (times, sqrt(Sxx + Syy)) from landmarks.csv."""
    out = {}
    for r in phase['lms']:
        ts, ss = out.setdefault(int(r['id']), ([], []))
        ts.append(float(r['t']))
        ss.append(math.sqrt(float(r['sxx']) + float(r['syy'])))
    return dict(sorted(out.items()))


def lap_starts(phase):
    """Time each lap after the first starts (from waypoints.csv)."""
    ends, starts = {}, []
    for w in phase['wps']:
        ends[int(w['lap'])] = float(w['t'])
    for lap in sorted(ends)[:-1]:
        starts.append((lap + 1, ends[lap]))
    return starts


def plot_sigma(phase, out_path):
    fig, ax = plt.subplots(figsize=(8.5, 4.5))
    style(ax)
    series = sigma_series(phase)
    for i, (ts, ss) in series.items():
        ax.plot(ts, [s * 100 for s in ss], '-', color=SERIES[i % len(SERIES)], lw=2,
                label=f'tag {i}')
    top = max((max(ss) for _, ss in series.values()), default=0.0) * 100
    ends = spread([ss[-1] * 100 for _, ss in series.values()], 0.045 * top)
    for (i, (ts, _)), y in zip(series.items(), ends):
        ax.annotate(f'tag {i}', (ts[-1], y), textcoords='offset points', xytext=(5, 0),
                    color=INK, fontsize=8, va='center', annotation_clip=False)
    for lap, t in lap_starts(phase):
        ax.axvline(t, color=MUTED, lw=1, ls='--')
        ax.annotate(f'lap {lap}', (t, 1), xycoords=('data', 'axes fraction'),
                    textcoords='offset points', xytext=(3, -12), color=MUTED, fontsize=8)
    ax.set_ylim(bottom=0)
    ax.set_xlabel('time (s)', color=INK)
    ax.set_ylabel('σ position, √(Σxx+Σyy) (cm)', color=INK)
    ax.set_title(f'{phase["name"]}: landmark uncertainty over time', color=INK, fontsize=11)
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.18), ncol=8, fontsize=8,
              frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


# ---------- main ----------

def report(path, truth):
    """Write figures and report.md files; returns the run-level markdown."""
    phases = [load_phase(d) for d in phase_dirs(path)]
    parts = [f'# HW3 run `{os.path.basename(os.path.normpath(path))}`', '']
    for p in phases:
        if not p['laps']:
            parts += [f'## Phase `{p["name"]}`: no map_lap*.yaml (run stopped in lap 1?)', '']
            continue
        plot_map(p, truth, os.path.join(p['dir'], 'map.png'))
        plot_sigma(p, os.path.join(p['dir'], 'sigma.png'))
        text = phase_markdown(p, truth)
        Path(p['dir'], 'report.md').write_text(text + '\n')
        parts += [text.replace('](map.png)', f']({p["name"]}/map.png)')
                  .replace('](sigma.png)', f']({p["name"]}/sigma.png)'), '']
    done = [p for p in phases if p['laps']]
    if len(done) > 1:
        parts += comparison_markdown(done, truth)
    text = '\n'.join(parts)
    if len(phases) > 1:
        Path(path, 'report.md').write_text(text + '\n')
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('path', nargs='?', help='run_<time> or a phase dir (default: latest run)')
    ap.add_argument('--truth', default=str(DEFAULT_TRUTH))
    a = ap.parse_args()
    path = a.path or latest_run(str(DEFAULT_LOGS))
    print(report(path, map_eval.truth_poses(load_map(a.truth))))


if __name__ == '__main__':
    main()
