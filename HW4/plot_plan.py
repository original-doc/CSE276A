#!/usr/bin/env python3
"""HW4 planning figures (report 3(c)): runs both planners on world.yaml and saves PNGs.

  python HW4/plot_plan.py [--world robot_control/config/hw4/world.yaml] [--out HW4/figures]
  python HW4/plot_plan.py --run HW4/logs/run_<time>_<planner>   # the plan a run used

Figures: plan_safe.png, plan_fast.png, plan_compare.png, expand_safe.png, expand_fast.png
(--run: plan_<planner>.png and expand_<planner>.png in <run>/figures/)
"""
import argparse
import os
import sys
from pathlib import Path

import matplotlib
import numpy as np
import yaml

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap, ListedColormap  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robot_control'))

from robot_control.grid import Grid  # noqa: E402
from robot_control.path_utils import make_plan  # noqa: E402
from robot_control.plan_fast import plan_fast  # noqa: E402
from robot_control.plan_safe import plan_safe  # noqa: E402
from robot_control.world import load_world, parse_world, segment_rect_dist  # noqa: E402

DEFAULT_WORLD = ROOT / 'robot_control' / 'config' / 'hw4' / 'world.yaml'
DEFAULT_OUT = ROOT / 'HW4' / 'figures'
DEFAULT_CFG = {'grid_res': 0.02, 'simplify_tol': 0.03, 'tie_side': 'right',
               'tie_tol': 0.02, 'turn_cost_s': 0.0}

SAFE = '#2a78d6'        # categorical slot 1
FAST = '#eb6834'        # categorical slot 2
INK = '#0b0b0b'
INK_2 = '#52514e'
BOX = '#52514e'
GRID_LINE = '#d9d8d4'
NEAR_CMAP = LinearSegmentedColormap.from_list('near', ['#a3a29b', '#ffffff'])  # dark = close


def ramp(name):
    """Sequential map without its near-white end (marks stay visible on white)."""
    return ListedColormap(plt.get_cmap(name)(np.linspace(0.3, 1.0, 256)))


# ---------- shared drawing ----------

def draw_world(ax, world, tags=True):
    a = world.arena
    ax.add_patch(Rectangle((a.x0, a.y0), a.w, a.h, fill=False, ec=INK, lw=1.5))
    for o in world.obstacles:
        ax.add_patch(Rectangle((o.x0, o.y0), o.w, o.h, fc=BOX, ec=INK, lw=1, zorder=4))
    if tags:
        for lm in world.landmarks.landmarks.values():
            dx, dy = 0.09 * np.cos(lm.yaw), 0.09 * np.sin(lm.yaw)
            ax.plot(lm.x, lm.y, 's', ms=4, color=INK, zorder=5)
            ax.annotate(str(lm.id), (lm.x + dx * 1.6, lm.y + dy * 1.6), fontsize=7,
                        color=INK_2, ha='center', va='center', zorder=5)
    s, g = world.start, world.goal
    ax.plot(s.x, s.y, 'o', ms=9, mfc='white', mec=INK, mew=1.5, zorder=6)
    ax.plot(*g, '*', ms=13, mfc='white', mec=INK, mew=1.2, zorder=6)
    ax.annotate('S', (s.x, s.y), (6, 6), textcoords='offset points', fontsize=9, color=INK)
    ax.annotate('G', g, (6, 6), textcoords='offset points', fontsize=9, color=INK)
    ax.set_xlim(a.x0 - 0.15, a.x1 + 0.15)
    ax.set_ylim(a.y0 - 0.15, a.y1 + 0.15)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.grid(color=GRID_LINE, lw=0.5)
    ax.set_axisbelow(True)


def draw_path(ax, plan, color, label=None, lw=2.5):
    xy = np.array(plan.points)
    ax.plot(xy[:, 0], xy[:, 1], '-', color=color, lw=lw, label=label, zorder=7,
            solid_capstyle='round')
    ax.plot(xy[1:-1, 0], xy[1:-1, 1], 'o', ms=5, mfc=color, mec='white', mew=1, zorder=8)


def legend_below(ax):
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=8,
              frameon=False)


def box_clearance(plan, world):
    return min(segment_rect_dist(a, b, o) for o in world.obstacles
               for a, b in zip(plan.points, plan.points[1:]))


def stats_text(plan):
    st = plan.stats
    return (f"length {plan.length:.3f} m\n"
            f"min clearance {plan.min_clearance:.3f} m\n"
            f"nodes {st['nodes']}, edges {st['edges']}\n"
            f"expanded {st['expanded']}\n"
            f"plan time {st['time_s'] * 1000:.0f} ms")


def add_text(ax, text):
    ax.text(0.02, 0.02, text, transform=ax.transAxes, fontsize=8, color=INK, va='bottom',
            bbox=dict(fc='white', ec=GRID_LINE, alpha=0.9), zorder=10)


def grid_extent(grid):
    return [grid.ox, grid.ox + grid.nx * grid.res, grid.oy, grid.oy + grid.ny * grid.res]


def cell_xy(grid, cells):
    cells = np.asarray(cells)
    return (grid.ox + (cells[:, 0] + 0.5) * grid.res, grid.oy + (cells[:, 1] + 0.5) * grid.res)


# ---------- figures ----------

def fig_safe(plan, world):
    grid, D, gvd = plan.extra['grid'], plan.extra['D'], plan.extra['gvd']
    r = plan.stats['r']
    fig, ax = plt.subplots(figsize=(7, 6.4))
    im = ax.imshow(np.ma.masked_equal(D, 0).T, origin='lower', extent=grid_extent(grid),
                   cmap=NEAR_CMAP, vmin=0, vmax=float(D.max()), interpolation='nearest')
    fig.colorbar(im, ax=ax, shrink=0.8, label='brushfire distance D (m)')
    ext = grid_extent(grid)
    xs = np.linspace(ext[0], ext[1], D.shape[0])
    ys = np.linspace(ext[2], ext[3], D.shape[1])
    ax.contourf(xs, ys, ((D > 0) & (D < r)).T.astype(float), levels=[0.5, 1.5], colors='none',
                hatches=['////'], zorder=1)
    ax.contour(xs, ys, D.T, levels=[r], colors=INK_2, linewidths=0.6, zorder=1)
    x, y = cell_xy(grid, np.argwhere(gvd))
    ax.plot(x, y, 's', ms=1.6, color='#86b6ef', label='GVD cells', zorder=3)
    draw_world(ax, world)
    draw_path(ax, plan, SAFE, label='safe path')
    ax.add_patch(Rectangle((0, 0), 0, 0, fc='white', ec=INK_2, hatch='////',
                           label=f'D < r_inflate ({r:.3f} m): too close'))
    legend_below(ax)
    add_text(ax, stats_text(plan) + f"\nbox clearance {box_clearance(plan, world):.3f} m")
    ax.set_title('Safest path: brushfire distance field + GVD + Dijkstra')
    return fig


def fig_fast(plan, world):
    r = plan.stats['r']
    verts = plan.extra['vertices']
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    a = world.arena
    ax.add_patch(Rectangle((a.x0 + r, a.y0 + r), a.w - 2 * r, a.h - 2 * r, fill=False,
                           ec=INK_2, lw=0.8, ls='--', label='arena shrunk by r'))
    for k, o in enumerate(world.obstacles):
        big = o.inflate(r)
        ax.add_patch(Rectangle((big.x0, big.y0), big.w, big.h, fc='#f6c4ae', ec=FAST, lw=1,
                               alpha=0.6, zorder=2,
                               label=f'inflated box (r = {r:.3f} m)' if k == 0 else None))
    for n, (i, j) in enumerate(plan.extra['edges']):
        ax.plot([verts[i][0], verts[j][0]], [verts[i][1], verts[j][1]], '-', color='#a3a29b',
                lw=0.8, zorder=3, label='visibility edges' if n == 0 else None)
    v = np.array(verts)
    ax.plot(v[:, 0], v[:, 1], 'o', ms=5, color=INK_2, zorder=5)
    draw_world(ax, world)
    draw_path(ax, plan, FAST, label='fast path')
    legend_below(ax)
    add_text(ax, stats_text(plan) + f"\nbox clearance {box_clearance(plan, world):.3f} m")
    ax.set_title('Shortest path: visibility graph + Dijkstra')
    return fig


def fig_compare(safe, fast, world):
    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    draw_world(ax, world)
    for plan, color in ((safe, SAFE), (fast, FAST)):
        label = (f'{plan.kind}: {plan.length:.2f} m, '
                 f'box clearance {box_clearance(plan, world):.2f} m')
        draw_path(ax, plan, color, label=label)
    legend_below(ax)
    ax.set_title('Safe vs fast path')
    return fig


def fig_expand_safe(plan, world):
    grid = plan.extra['grid']
    order = plan.extra['expanded']
    fig, ax = plt.subplots(figsize=(7, 6.4))
    x, y = cell_xy(grid, np.argwhere(plan.extra['gvd']))
    ax.plot(x, y, 's', ms=1.6, color=GRID_LINE, label='GVD cells (not expanded)', zorder=2)
    x, y = cell_xy(grid, order)
    sc = ax.scatter(x, y, c=np.arange(len(order)), cmap=ramp('Blues'), s=5, marker='s', zorder=3)
    fig.colorbar(sc, ax=ax, shrink=0.8, label='expansion order')
    draw_world(ax, world)
    draw_path(ax, plan, INK, label='final path', lw=1.2)
    legend_below(ax)
    add_text(ax, f"expanded {len(order)} of {plan.stats['nodes']} GVD cells\n"
                 f"searches {plan.stats['searches']}, tie switched {plan.stats['tie_switched']}")
    ax.set_title('Safe planner: Dijkstra expansion order on the GVD')
    return fig


def fig_expand_fast(plan, world):
    verts = plan.extra['vertices']
    order = plan.extra['expanded']
    fig, ax = plt.subplots(figsize=(7, 6.4))
    for i, j in plan.extra['edges']:
        ax.plot([verts[i][0], verts[j][0]], [verts[i][1], verts[j][1]], '-', color=GRID_LINE,
                lw=0.8, zorder=2)
    v = np.array([verts[i] for i in order])
    sc = ax.scatter(v[:, 0], v[:, 1], c=np.arange(len(order)), cmap=ramp('Oranges'), s=120,
                    edgecolors=INK, zorder=6)
    fig.colorbar(sc, ax=ax, shrink=0.8, label='expansion order')
    for n, (x, y) in enumerate(v, 1):
        ax.annotate(str(n), (x, y), (8, -12), textcoords='offset points', fontsize=9,
                    color=INK, zorder=9)
    draw_world(ax, world)
    draw_path(ax, plan, INK, label='final path', lw=1.2)
    legend_below(ax)
    add_text(ax, f"expanded {len(order)} of {plan.stats['nodes']} vertices\n"
                 "label = expansion order")
    ax.set_title('Fast planner: Dijkstra expansion order on the visibility graph')
    return fig


def make_figures(world, cfg, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    safe, fast = plan_safe(world, cfg), plan_fast(world, cfg)
    figs = {
        'plan_safe.png': fig_safe(safe, world),
        'plan_fast.png': fig_fast(fast, world),
        'plan_compare.png': fig_compare(safe, fast, world),
        'expand_safe.png': fig_expand_safe(safe, world),
        'expand_fast.png': fig_expand_fast(fast, world),
    }
    paths = []
    for name, fig in figs.items():
        fig.tight_layout()
        fig.savefig(out_dir / name, dpi=150)
        plt.close(fig)
        paths.append(out_dir / name)
    return paths, safe, fast


def load_run_plan(run_dir):
    """(world, Plan) from a run's config.yaml, plan.yaml and plan.npz (first plan)."""
    run_dir = Path(run_dir)
    with open(run_dir / 'config.yaml') as f:
        world = parse_world(yaml.safe_load(f)['world'])
    with open(run_dir / 'plan.yaml') as f:
        rec = yaml.safe_load(f)
    npz = np.load(run_dir / 'plan.npz')
    if rec['planner'] == 'safe':
        res, nx, ny, ox, oy = npz['grid']
        extra = {'grid': Grid(float(res), int(nx), int(ny), float(ox), float(oy)),
                 'D': npz['D'], 'label': npz['label'], 'gvd': npz['gvd'],
                 'expanded': [tuple(c) for c in npz['expanded']]}
    else:
        extra = {'vertices': [tuple(v) for v in npz['vertices']],
                 'edges': [tuple(e) for e in npz['edges']],
                 'expanded': list(npz['expanded'])}
    return world, make_plan(rec['planner'], rec['points'], world, rec['stats'], extra)


def run_figures(run_dir, out_dir=None):
    world, plan = load_run_plan(run_dir)
    out_dir = Path(out_dir or Path(run_dir) / 'figures')
    out_dir.mkdir(parents=True, exist_ok=True)
    draw = {'safe': (fig_safe, fig_expand_safe), 'fast': (fig_fast, fig_expand_fast)}
    paths = []
    for name, fn in zip(('plan', 'expand'), draw[plan.kind]):
        fig = fn(plan, world)
        fig.tight_layout()
        paths.append(out_dir / f'{name}_{plan.kind}.png')
        fig.savefig(paths[-1], dpi=150)
        plt.close(fig)
    return paths


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--world', default=str(DEFAULT_WORLD))
    ap.add_argument('--out', default=str(DEFAULT_OUT))
    ap.add_argument('--tie-side', default=DEFAULT_CFG['tie_side'], choices=['right', 'left'])
    ap.add_argument('--run', help='redraw the plan of this run directory instead')
    args = ap.parse_args(argv)
    if args.run:
        for p in run_figures(args.run, None if args.out == str(DEFAULT_OUT) else args.out):
            print('saved', os.path.relpath(p))
        return
    cfg = dict(DEFAULT_CFG, tie_side=args.tie_side)
    paths, safe, fast = make_figures(load_world(args.world), cfg, args.out)
    for plan in (safe, fast):
        print(f'{plan.kind}: length {plan.length:.3f} m, min clearance '
              f'{plan.min_clearance:.3f} m, {len(plan.points)} points')
    for p in paths:
        print('saved', os.path.relpath(p))


if __name__ == '__main__':
    main()
