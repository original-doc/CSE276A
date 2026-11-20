import copy
import os
from collections import deque

import numpy as np
import pytest
import yaml

from robot_control.grid import (
    ascend_to_gvd, brushfire, FREE, gvd_cells, make_grid, NB8, obstacle_cells)
from robot_control.path_utils import NoPath
from robot_control.world import clearance, load_world, parse_world

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config', 'hw4', 'world.yaml')
WORLD = load_world(CONFIG)
RES = 0.02


@pytest.fixture(scope='module')
def field():
    grid = make_grid(WORLD, RES)
    labels = obstacle_cells(grid, WORLD)
    D, label = brushfire(grid, labels)
    gvd = gvd_cells(D, label, WORLD.r_inflate)
    return grid, labels, D, label, gvd


def components(mask):
    """8-connected components of a boolean mask."""
    seen = np.zeros(mask.shape, dtype=bool)
    comps = []
    for start in zip(*np.nonzero(mask)):
        if seen[start]:
            continue
        seen[start] = True
        comp, queue = [], deque([start])
        while queue:
            i, j = queue.popleft()
            comp.append((i, j))
            for di, dj in NB8:
                n = (i + di, j + dj)
                if 0 <= n[0] < mask.shape[0] and 0 <= n[1] < mask.shape[1] \
                        and mask[n] and not seen[n]:
                    seen[n] = True
                    queue.append(n)
        comps.append(set(comp))
    return comps


def near_gvd(gvd, cell):
    i, j = cell
    return gvd[cell] or any(gvd[i + di, j + dj] for di, dj in NB8)


def test_grid_geometry():
    grid = make_grid(WORLD, RES)
    assert (grid.nx, grid.ny) == (152, 152)         # 150 arena cells + wall ring
    assert grid.cell_center(1, 1) == pytest.approx((0.01, 0.01))
    assert grid.cell_of(0.01, 0.01) == (1, 1)
    assert grid.cell_of(1.5, 1.5) == (76, 76)
    assert grid.cell_of(-5.0, 9.0) == (0, 151)      # clamped
    i, j = grid.cell_of(0.73, 2.41)
    assert np.allclose(grid.cell_center(i, j), (0.73, 2.41), atol=RES / 2)


def test_obstacle_labels(field):
    grid, labels, *_ = field
    assert labels[5, 0] == 0 and labels[-1, 5] == 1 and labels[5, -1] == 2 and labels[0, 5] == 3
    assert labels[grid.cell_of(1.5, 1.5)] == 4
    assert labels[grid.cell_of(1.33, 1.33)] == 4    # just inside the box corner
    assert labels[grid.cell_of(1.29, 1.5)] == FREE
    assert labels[grid.cell_of(0.5, 0.5)] == FREE


def test_brushfire_distance_matches_clearance(field):
    grid, labels, D, *_ = field
    err = np.array([D[i, j] - clearance(grid.cell_center(i, j), WORLD)
                    for i, j in zip(*np.nonzero(labels == FREE))])
    # Box edges cut cells in half here: D within half a cell, never above the truth.
    assert err.max() <= 1e-9
    assert err.min() >= -RES / 2 - 1e-9


def test_brushfire_labels(field):
    grid, _, _, label, _ = field
    assert label[grid.cell_of(0.2, 1.5)] == 3       # left wall
    assert label[grid.cell_of(2.8, 1.5)] == 1       # right wall
    assert label[grid.cell_of(1.5, 0.2)] == 0       # bottom wall
    assert label[grid.cell_of(1.5, 2.8)] == 2       # top wall
    assert label[grid.cell_of(1.5, 1.1)] == 4       # box
    assert label[grid.cell_of(1.1, 1.5)] == 4


def test_gvd_shape(field):
    grid, _, D, _, gvd = field
    assert near_gvd(gvd, grid.cell_of(0.5, 2.5))    # S on the corner bisector
    assert near_gvd(gvd, grid.cell_of(2.5, 0.5))    # G
    # Between the box's left face (x = 1.325) and the left wall: x = 0.6625
    assert near_gvd(gvd, grid.cell_of(0.6625, 1.5))
    assert not gvd[grid.cell_of(1.0, 1.5)]
    assert (D[gvd] >= WORLD.r_inflate).all()


def test_gvd_connected(field):
    grid, _, _, _, gvd = field
    comps = components(gvd)
    assert len(comps) == 1
    ring = [grid.cell_of(x, y) for x, y in ((0.66, 1.5), (2.34, 1.5), (1.5, 0.66), (1.5, 2.34))]
    assert all(any(c in comps[0] for c in [r] + [(r[0] + a, r[1] + b) for a, b in NB8])
               for r in ring)


def test_ascend_from_off_gvd(field):
    grid, _, D, _, gvd = field
    start = grid.cell_of(0.3, 2.0)                  # near the left wall, off the GVD
    assert not gvd[start]
    path = ascend_to_gvd(D, gvd, start)
    assert path[0] == start and gvd[path[-1]]
    assert all(D[b] >= D[a] for a, b in zip(path, path[1:]))   # clearance never drops


def test_ascend_on_gvd_is_one_cell(field):
    _, _, D, _, gvd = field
    cell = tuple(np.argwhere(gvd)[0])
    assert ascend_to_gvd(D, gvd, cell) == [cell]


def test_ascend_stuck_raises():
    D = np.zeros((5, 5))
    D[2, 2] = 1.0
    with pytest.raises(NoPath):
        ascend_to_gvd(D, np.zeros((5, 5), dtype=bool), (2, 2))


def other_world(obstacles):
    with open(CONFIG) as f:
        d = yaml.safe_load(f)
    d = copy.deepcopy(d)
    d['landmarks'] = d['landmarks'][:8]
    d['obstacles'] = obstacles
    return parse_world(d)


@pytest.mark.parametrize('obstacles', [
    # edges on cell borders
    [{'cx': 1.0, 'cy': 2.0, 'w': 0.3, 'h': 0.3}, {'cx': 2.0, 'cy': 1.0, 'w': 0.4, 'h': 0.4}],
    # edges anywhere inside cells
    [{'cx': 1.4567, 'cy': 1.6123, 'w': 0.3311, 'h': 0.4199}],
])
def test_other_layouts(obstacles):
    w = other_world(obstacles)
    grid = make_grid(w, RES)
    labels = obstacle_cells(grid, w)
    D, label = brushfire(grid, labels)
    assert label[grid.cell_of(w.obstacles[0].cx, w.obstacles[0].y0 - 0.1)] == 4
    gvd = gvd_cells(D, label, w.r_inflate)
    assert len(components(gvd)) == 1
    free = labels == FREE
    err = np.array([D[c] - clearance(grid.cell_center(*c), w) for c in zip(*np.nonzero(free))])
    # < 1 cell; slightly above the truth (< RES / 4) only diagonal to a box corner
    assert err.min() > -RES and err.max() < RES / 4
