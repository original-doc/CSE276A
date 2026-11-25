"""Shortest-path planner: visibility graph on inflated boxes + Dijkstra (no ROS).

In a polygon world the shortest path is straight segments that bend only at
obstacle vertices (lec12 p.9), so the visibility graph holds the exact optimum
(for square-corner inflation, a conservative cover of the true C-space).
"""
import math
import time

from robot_control.graph_search import dijkstra
from robot_control.path_utils import make_plan, NoPath
from robot_control.world import segment_enters_rect, side_of, wall_dist

EPS = 1e-9


def is_free(p, world, r):
    """p is at least r from the walls and outside every inflated box (sides allowed)."""
    if wall_dist(p, world.arena) < r - EPS:
        return False
    for o in world.obstacles:
        big = o.inflate(r)
        if big.x0 + EPS < p[0] < big.x1 - EPS and big.y0 + EPS < p[1] < big.y1 - EPS:
            return False
    return True


def graph_vertices(world, r, start, goal):
    """[start, goal] + free corners of every inflated box."""
    corners = [c for o in world.obstacles for c in o.inflate(r).corners()]
    return [tuple(start), tuple(goal)] + [c for c in corners if is_free(c, world, r)]


def visible(a, b, world, r):
    """ab stays in the shrunk arena and never enters an inflated box."""
    if not (is_free(a, world, r) and is_free(b, world, r)):
        return False                # arena is convex: endpoints inside => segment inside
    return not any(segment_enters_rect(a, b, o.inflate(r)) for o in world.obstacles)


def visibility_graph(vertices, world, r):
    """Adjacency {i: [(j, length)]} over all mutually visible vertex pairs."""
    adj = {i: [] for i in range(len(vertices))}
    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            if visible(vertices[i], vertices[j], world, r):
                d = math.dist(vertices[i], vertices[j])
                adj[i].append((j, d))
                adj[j].append((i, d))
    return adj


def plan_fast(world, cfg=None, start=None, r=None):
    """Shortest path from start (default world.start) to world.goal.

    cfg keys: tie_side ('right' | 'left', seen from start looking at goal),
    turn_cost_s and speed (time per bend, converted to metres; 0 = shortest distance).
    Raises NoPath.
    """
    cfg = cfg or {}
    r = world.r_inflate if r is None else r
    s = (start[0], start[1]) if start is not None else (world.start.x, world.start.y)
    g = world.goal
    t0 = time.perf_counter()

    if not is_free(s, world, r) or not is_free(g, world, r):
        raise NoPath(f'start {s} or goal {g} is closer than r={r:.3f} to an obstacle')
    verts = graph_vertices(world, r, s, g)
    adj = visibility_graph(verts, world, r)

    sign = -1.0 if cfg.get('tie_side', 'right') == 'right' else 1.0

    def tie_key(i):
        return 0 if side_of(verts[i], s, g) * sign >= 0 else 1

    bend = cfg.get('turn_cost_s', 0.0) * cfg.get('speed', 0.2)

    def neighbors(i):
        return [(j, d + (bend if j != 1 else 0.0)) for j, d in adj[i]]

    res = dijkstra(0, 1, neighbors, tie_key=tie_key)
    if res.path is None:
        raise NoPath(f'goal not visible-connected to start {s}')

    n_edges = sum(len(v) for v in adj.values()) // 2
    stats = {
        'nodes': len(verts),
        'edges': n_edges,
        'candidate_pairs': len(verts) * (len(verts) - 1) // 2,
        'expanded': len(res.expanded),
        'cost': res.cost,
        'time_s': time.perf_counter() - t0,
        'r': r,
    }
    extra = {
        'vertices': verts,
        'edges': sorted((i, j) for i in adj for j, _ in adj[i] if i < j),
        'expanded': res.expanded,
        'vertex_path': res.path,
    }
    return make_plan('fast', [verts[i] for i in res.path], world, stats, extra)
