import math

import pytest

from robot_control.graph_search import dijkstra


def adj(edges, directed=False):
    g = {}
    for u, v, w in edges:
        g.setdefault(u, []).append((v, w))
        g.setdefault(v, [])
        if not directed:
            g[v].append((u, w))
    return lambda u: g[u]


# Hand-checked: A-B-D = 1 + 5 = 6, A-C-D = 4 + 1 = 5, A-B-C-D = 1 + 2 + 1 = 4.
SMALL = adj([('A', 'B', 1), ('A', 'C', 4), ('B', 'C', 2), ('B', 'D', 5), ('C', 'D', 1)])

# Diamond: S -> L -> G and S -> R -> G, both cost 2.
DIAMOND = adj([('S', 'L', 1), ('S', 'R', 1), ('L', 'G', 1), ('R', 'G', 1)])


def test_small_graph_by_hand():
    r = dijkstra('A', 'D', SMALL)
    assert r.path == ['A', 'B', 'C', 'D']
    assert r.cost == pytest.approx(4)
    assert r.expanded == ['A', 'B', 'C', 'D']      # dist 0, 1, 3, 4


def test_start_is_goal():
    r = dijkstra('A', 'A', SMALL)
    assert r.path == ['A'] and r.cost == 0 and r.expanded == ['A']


def test_unreachable():
    g = adj([('A', 'B', 1), ('C', 'D', 1)])
    r = dijkstra('A', 'D', g)
    assert r.path is None and r.cost == math.inf
    assert sorted(r.expanded) == ['A', 'B']        # searched everything reachable


def test_negative_cost_rejected():
    with pytest.raises(ValueError):
        dijkstra('A', 'B', adj([('A', 'B', -1)]))


@pytest.mark.parametrize('prefer', ['L', 'R'])
def test_tie_key_picks_side(prefer):
    r = dijkstra('S', 'G', DIAMOND, tie_key=lambda n: 0 if n == prefer else 1)
    assert r.path == ['S', prefer, 'G']
    assert r.cost == pytest.approx(2)


def test_tie_key_within_eps():
    # R side is shorter by 1e-12 only: still counts as a tie.
    g = adj([('S', 'L', 1), ('S', 'R', 1 - 1e-12), ('L', 'G', 1), ('R', 'G', 1)])
    r = dijkstra('S', 'G', g, tie_key=lambda n: 0 if n == 'L' else 1)
    assert r.path == ['S', 'L', 'G']
    r = dijkstra('S', 'G', g, tie_key=lambda n: 0 if n == 'L' else 1, eps=0)
    assert r.path == ['S', 'R', 'G']


def test_tie_key_does_not_beat_shorter():
    g = adj([('S', 'L', 1), ('S', 'R', 1), ('L', 'G', 1.1), ('R', 'G', 1)])
    r = dijkstra('S', 'G', g, tie_key=lambda n: 0 if n == 'L' else 1)
    assert r.path == ['S', 'R', 'G']


def grid_neighbors(n):
    def nb(c):
        i, j = c
        out = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if (di or dj) and 0 <= i + di < n and 0 <= j + dj < n:
                    out.append(((i + di, j + dj), math.hypot(di, dj)))
        return out
    return nb


def test_grid_expanded_unique_and_in_cost_order():
    nb = grid_neighbors(6)
    r = dijkstra((0, 0), (5, 3), nb)
    assert r.cost == pytest.approx(3 * math.sqrt(2) + 2)     # 3 diagonal + 2 straight
    assert len(r.expanded) == len(set(r.expanded))
    # Each expanded node's cost is >= the previous one's (Dijkstra order).
    costs = [dijkstra((0, 0), u, nb).cost for u in r.expanded]
    assert all(b >= a - 1e-9 for a, b in zip(costs, costs[1:]))
    assert r.path[0] == (0, 0) and r.path[-1] == (5, 3)
    assert r.expanded[-1] == (5, 3)
