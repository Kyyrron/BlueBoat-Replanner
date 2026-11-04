"""Routes inside the mission area (MATH.md §3.3).

`connect(start, goal)`:
1. the shortest Dubins path whose samples lie in F_margin;
2. else a visibility-graph route over the inward-offset reflex vertices: each route node gets
   the heading of its outgoing segment and consecutive node poses are joined by the shortest
   feasible Dubins path (MATH §3.3 as implemented);
3. else None (infeasible). A route never leaves the area.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from .areas import Area
from .dubins import dubins_candidates, prims_length, sample_prims


def prims_feasible(prims, area: Area, d: float, step: float) -> bool:
    if not prims:
        return True
    x, y, _ = sample_prims(prims, step)
    return area.all_feasible(np.column_stack([x, y]), d)


def shortest_feasible_dubins(start, goal, area: Area, d: float, rho: float, step: float):
    for length, word, prims in dubins_candidates(start, goal, rho):
        if prims_feasible(prims, area, d, step):
            return prims
    return None


def _segment_feasible(p, q, area: Area, d: float, step: float) -> bool:
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(1, int(math.ceil(L / step)))
    t = np.linspace(0.0, 1.0, n + 1)
    pts = np.column_stack([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])])
    return area.all_feasible(pts, d)


def visibility_route(p0, p1, area: Area, d: float, node_d: float, step: float):
    """Shortest polyline p0 -> p1 through inward reflex offsets, or None."""
    nodes = [np.asarray(p0, float), np.asarray(p1, float)]
    for q in area.reflex_offset_points(node_d):
        if area.all_feasible(q[None, :], d):
            nodes.append(q)
    n = len(nodes)
    adj = {i: [] for i in range(n)}
    for i in range(n):
        for j in range(i + 1, n):
            if _segment_feasible(nodes[i], nodes[j], area, d, step):
                w = float(np.hypot(*(nodes[j] - nodes[i])))
                adj[i].append((j, w))
                adj[j].append((i, w))
    dist = [math.inf] * n
    prev = [-1] * n
    dist[0] = 0.0
    heap = [(0.0, 0)]
    while heap:
        du, u = heapq.heappop(heap)
        if du > dist[u]:
            continue
        if u == 1:
            break
        for v, w in adj[u]:
            if du + w < dist[v] - 1e-12:
                dist[v] = du + w
                prev[v] = u
                heapq.heappush(heap, (dist[v], v))
    if not math.isfinite(dist[1]):
        return None
    path = [1]
    while path[-1] != 0:
        path.append(prev[path[-1]])
    return [nodes[i] for i in reversed(path)]


def connect(start, goal, area: Area, cfg):
    """Feasible primitive chain from pose `start` to pose `goal` inside F_margin, or None."""
    d = cfg.margin_m
    rho = cfg.r_turn_m
    step = cfg.geom_step_m
    prims = shortest_feasible_dubins(start, goal, area, d, rho, step)
    if prims is not None:
        return prims
    poly = visibility_route(start[:2], goal[:2], area, d, d + rho + 1e-3, step)
    if poly is None or len(poly) < 3:
        return None
    poses = [tuple(start)]
    for k in range(1, len(poly) - 1):
        nxt = poly[k + 1] - poly[k]
        poses.append((float(poly[k][0]), float(poly[k][1]), math.atan2(nxt[1], nxt[0])))
    poses.append(tuple(goal))
    chain = []
    for a, b in zip(poses[:-1], poses[1:]):
        piece = shortest_feasible_dubins(a, b, area, d, rho, step)
        if piece is None:
            return None
        chain.extend(piece)
    return chain


def route_length(prims) -> float:
    return prims_length(prims) if prims is not None else math.inf
