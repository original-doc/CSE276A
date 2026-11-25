"""Safest-path planner: brushfire -> GVD -> Dijkstra on GVD cells (no ROS).

GVD cells are locally as far as possible from the two nearest obstacles (lec12
p.9), so the shortest path on the GVD is the shortest among the safest paths.
"""
import math
import time

import numpy as np

from robot_control.graph_search import dijkstra
from robot_control.grid import (
    ascend_to_gvd, brushfire, gvd_cells, make_grid, NB8, obstacle_cells)
from robot_control.path_utils import make_plan, NoPath, simplify
from robot_control.world import clearance, side_of


def gvd_graph(gvd, res):
    """neighbors(cell) over 8-adjacent GVD cells; cost = distance between centers."""
    nx, ny = gvd.shape

    def neighbors(c):
        i, j = c
        return [((i + di, j + dj), res * math.hypot(di, dj)) for di, dj in NB8
                if 0 <= i + di < nx and 0 <= j + dj < ny and gvd[i + di, j + dj]]
    return neighbors


def signed_dist_to_line(grid, a, b):
    """Per-cell signed distance to the line a -> b (> 0 on the left)."""
    i, j = np.indices((grid.nx, grid.ny))
    x = grid.ox + (i + 0.5) * grid.res
    y = grid.oy + (j + 0.5) * grid.res
    return ((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])) / math.dist(a, b)


def path_side(points, a, b):
    """+1 / -1: side of line a -> b where the path strays farthest; 0 if on the line."""
    far = max(points, key=lambda p: abs(side_of(p, a, b)))
    s = side_of(far, a, b) / math.dist(a, b)
    return 0 if abs(s) < 1e-6 else (1 if s > 0 else -1)


def plan_safe(world, cfg=None, start=None, r=None):
    """Safest path from start (default world.start) to world.goal.

    cfg keys: grid_res, simplify_tol, tie_side ('right' | 'left', seen from start
    looking at goal), tie_tol (m). Raises NoPath.
    """
    cfg = cfg or {}
    r = world.r_inflate if r is None else r
    s = (start[0], start[1]) if start is not None else (world.start.x, world.start.y)
    g = world.goal
    if clearance(s, world) < r or clearance(g, world) < r:
        raise NoPath(f'start {s} or goal {g} is closer than r={r:.3f} to an obstacle')
    t0 = time.perf_counter()

    grid = make_grid(world, cfg.get('grid_res', 0.02))
    labels = obstacle_cells(grid, world)
    D, label = brushfire(grid, labels)
    t_field = time.perf_counter() - t0
    gvd = gvd_cells(D, label, r)

    up = ascend_to_gvd(D, gvd, grid.cell_of(*s))
    down = ascend_to_gvd(D, gvd, grid.cell_of(*g))

    def search(mask):
        return dijkstra(up[-1], down[-1], gvd_graph(mask, grid.res))

    res = search(gvd)
    if res.path is None:
        raise NoPath(f'GVD does not connect start {s} to goal')
    n_searches, switched = 1, False

    # Equal-length detours on both sides: prefer tie_side if it costs < tie_tol more.
    want = -1 if cfg.get('tie_side', 'right') == 'right' else 1
    got = path_side([grid.cell_center(*c) for c in res.path], s, g)
    if got == -want:
        other_side = signed_dist_to_line(grid, s, g) * want < -2 * grid.res
        res2 = search(gvd & ~other_side)
        n_searches = 2
        if res2.path is not None and res2.cost - res.cost < cfg.get('tie_tol', 0.02):
            res, switched = res2, True

    cells = up + res.path[1:] + down[::-1][1:]
    raw = [s] + [grid.cell_center(*c) for c in cells[1:-1]] + [g]
    points = simplify(raw, world, r, cfg.get('simplify_tol', 0.03))

    stats = {
        'grid': [grid.nx, grid.ny],
        'free_cells': int((labels == -1).sum()),
        'nodes': int(gvd.sum()),
        'edges': sum(len(gvd_graph(gvd, grid.res)(tuple(c))) for c in np.argwhere(gvd)) // 2,
        'expanded': len(res.expanded),
        'cost': res.cost,
        'searches': n_searches,
        'tie_switched': switched,
        'points_raw': len(raw),
        'points': len(points),
        'field_time_s': t_field,
        'time_s': time.perf_counter() - t0,
        'r': r,
    }
    extra = {
        'grid': grid,
        'D': D,
        'label': label,
        'gvd': gvd,
        'expanded': res.expanded,
        'cell_path': cells,
        'raw_points': raw,
    }
    return make_plan('safe', points, world, stats, extra)
