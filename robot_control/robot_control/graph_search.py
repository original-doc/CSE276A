"""Dijkstra shortest path on an implicit graph (no ROS).

Dijkstra = A* with heuristic 0 (lec12 p.16-20). Shared by the safe (GVD grid)
and fast (visibility graph) planners.
"""
import heapq
import itertools
import math
from typing import List, NamedTuple, Optional


class SearchResult(NamedTuple):
    path: Optional[list]    # start .. goal, None if unreachable
    cost: float             # inf if unreachable
    expanded: List          # nodes in the order they left the queue


def _trace(parent, goal):
    path = [goal]
    while parent[path[-1]] is not None:
        path.append(parent[path[-1]])
    return path[::-1]


def dijkstra(start, goal, neighbors, tie_key=None, eps=1e-9):
    """Shortest path from start to goal.

    neighbors(u) -> [(v, cost)], cost >= 0. Nodes must be hashable.
    tie_key(u) -> number: among paths whose cost differs by <= eps, the one through
    the predecessor with the smaller tie_key wins (makes equal-length solutions
    reproducible).
    """
    tie = tie_key or (lambda n: 0)
    dist = {start: 0.0}
    parent = {start: None}
    done = set()
    expanded = []
    order = itertools.count()       # keeps heap comparisons off the nodes
    heap = [(0.0, tie(start), next(order), start)]
    while heap:
        d, _, _, u = heapq.heappop(heap)
        if u in done:
            continue                # stale entry
        done.add(u)
        expanded.append(u)
        if u == goal:
            return SearchResult(_trace(parent, goal), d, expanded)
        for v, w in neighbors(u):
            if w < 0:
                raise ValueError(f'negative edge cost {w} from {u} to {v}')
            if v in done:
                continue
            nd = d + w
            old = dist.get(v, math.inf)
            better = nd < old - eps
            tie_win = abs(nd - old) <= eps and tie(u) < tie(parent[v])
            if better or tie_win:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(heap, (nd, tie(v), next(order), v))
    return SearchResult(None, math.inf, expanded)
