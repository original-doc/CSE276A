"""Occupancy grid, brushfire distance field and GVD for the safe planner (no ROS).

Brushfire: fire starts on every obstacle cell at once and spreads outward; each
free cell remembers whose fire reached it first (label) and how far (D).
GVD: free cells where the nearest obstacle changes = equally far from two obstacles.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

from robot_control.path_utils import NoPath
from robot_control.world import point_rect_dist

FREE = -1
WALLS = ('bottom', 'right', 'top', 'left')      # labels 0-3; boxes are 4, 5, ...
NB8 = [(di, dj) for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj]


@dataclass(frozen=True)
class Grid:
    """nx x ny cells of size res; cell (i, j) covers [ox + i res, ox + (i+1) res) in x."""

    res: float
    nx: int
    ny: int
    ox: float
    oy: float

    def cell_center(self, i, j):
        return (self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res)

    def cell_of(self, x, y):
        i = min(max(math.floor((x - self.ox) / self.res), 0), self.nx - 1)
        j = min(max(math.floor((y - self.oy) / self.res), 0), self.ny - 1)
        return (i, j)

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny


def make_grid(world, res):
    """Cells tile the arena, plus a one-cell ring outside it for the walls."""
    a = world.arena
    nx = round((a.x1 - a.x0) / res) + 2
    ny = round((a.y1 - a.y0) / res) + 2
    return Grid(res, nx, ny, a.x0 - res, a.y0 - res)


def obstacle_cells(grid, world):
    """labels[i, j] = obstacle id (walls 0-3, box k = 4 + k), FREE if empty."""
    labels = np.full((grid.nx, grid.ny), FREE, dtype=int)
    labels[:, 0] = 0                    # bottom
    labels[-1, :] = 1                   # right
    labels[:, -1] = 2                   # top
    labels[0, :] = 3                    # left
    for k, box in enumerate(world.obstacles):
        for i in range(grid.nx):
            for j in range(grid.ny):
                # cell overlaps the box interior
                if point_rect_dist(grid.cell_center(i, j), box) < grid.res / 2 - 1e-9:
                    labels[i, j] = 4 + k
    return labels


def brushfire(grid, labels):
    """Multi-source Dijkstra from all obstacle cells (8-neighbour).

    Propagates the distance to the nearest obstacle cell center, then subtracts half
    a cell (that center lies about half a cell inside the obstacle). D is within one
    cell of the true clearance, and mostly below it. Returns (D, label); D = 0 on obstacles.
    Exact ties go to the smaller label, so the result does not depend on queue order
    (keeps mirror-symmetric layouts symmetric).
    """
    D = np.full(labels.shape, np.inf)
    src = {}
    heap = []
    for i, j in zip(*np.nonzero(labels != FREE)):
        D[i, j] = 0.0
        src[(i, j)] = (i, j)
        heap.append((0.0, i, j))
    heapq.heapify(heap)
    label = labels.copy()
    while heap:
        d, i, j = heapq.heappop(heap)
        if d > D[i, j]:
            continue
        si, sj = src[(i, j)]
        for di, dj in NB8:
            ni, nj = i + di, j + dj
            if not grid.inside(ni, nj) or labels[ni, nj] != FREE:
                continue
            nd = grid.res * math.hypot(ni - si, nj - sj)
            tie = abs(nd - D[ni, nj]) <= 1e-12 and label[si, sj] < label[ni, nj]
            if nd < D[ni, nj] - 1e-12 or tie:
                D[ni, nj] = nd
                src[(ni, nj)] = (si, sj)
                label[ni, nj] = label[si, sj]
                heapq.heappush(heap, (nd, ni, nj))
    free = labels == FREE
    D[free] -= grid.res / 2
    return D, label


def gvd_cells(D, label, r_free):
    """Cells with D >= r_free whose label differs from a 4-neighbour's (2 cells wide)."""
    edge = np.zeros(D.shape, dtype=bool)
    dx = label[1:, :] != label[:-1, :]
    dy = label[:, 1:] != label[:, :-1]
    edge[1:, :] |= dx
    edge[:-1, :] |= dx
    edge[:, 1:] |= dy
    edge[:, :-1] |= dy
    return edge & (D >= r_free)


def ascend_to_gvd(D, gvd, cell):
    """Climb D (best 8-neighbour each step) from cell until a GVD cell. Returns the cells."""
    path = [tuple(cell)]
    while not gvd[path[-1]]:
        i, j = path[-1]
        nbrs = [(i + di, j + dj) for di, dj in NB8
                if 0 <= i + di < D.shape[0] and 0 <= j + dj < D.shape[1]]
        best = max(nbrs, key=lambda c: D[c])
        if D[best] <= D[i, j]:
            raise NoPath(f'stuck at a local maximum of D at cell {(i, j)} before the GVD')
        path.append(best)
    return path
