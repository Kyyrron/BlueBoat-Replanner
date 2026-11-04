"""Vectorised look predictor (MATH.md §5): path -> unique look keys (cell, folded sector g),
with the best per-look detection scale PD_r and the first time each key is reached.

Contract look geometry (C-5): b = psi + s*pi/2 (s = +1 port, -1 starboard); ground sample at
ground range r is p + r(cos b, sin b). Poses every resolution/2 along the path, ground
samples every resolution/2 across; reduction by sort + reduceat (no np.add.at, brief §9.1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Looks:
    cell: np.ndarray      # flat cell index
    g: np.ndarray         # folded sector
    pd: np.ndarray        # PD_r at the best range (plateau model)
    t: np.ndarray         # first time the key is reached (path seconds)

    @property
    def key(self) -> np.ndarray:
        return self.cell.astype(np.int64) * 64 + self.g

    def __len__(self):
        return len(self.cell)

    @staticmethod
    def empty() -> "Looks":
        z = np.zeros(0)
        return Looks(z.astype(np.int64), z.astype(int), z, z)


def plan_poses(plan, t_from: float, t_to: float, ds: float):
    """Poses (t, x, y, psi) of `plan` between t_from and t_to, every <= ds of arc length."""
    T, X, Y, P = [], [], [], []
    for seg, (a, b) in zip(plan.segments, plan.times):
        if b < t_from - 1e-9 or a > t_to + 1e-9:
            continue
        if seg.kind == "hold":
            x, y, th = seg.start_pose()
            tt = max(a, t_from)
            T.append(np.array([tt]))
            X.append(np.array([x]))
            Y.append(np.array([y]))
            P.append(np.array([th]))
            continue
        acc = 0.0
        for p in seg.prims:
            if p.length <= 0:
                continue
            pa = a + acc / plan.speed
            pb = pa + p.length / plan.speed
            acc += p.length
            lo, hi = max(pa, t_from), min(pb, t_to)
            if hi < lo:
                continue
            s0, s1 = (lo - pa) * plan.speed, (hi - pa) * plan.speed
            n = max(1, int(math.ceil((s1 - s0) / ds)))
            s = np.linspace(s0, s1, n + 1)
            x, y, th = p.pose(s)
            T.append(pa + s / plan.speed)
            X.append(np.atleast_1d(x))
            Y.append(np.atleast_1d(y))
            P.append(np.atleast_1d(th))
    if not T:
        z = np.zeros(0)
        return z, z, z, z
    return np.concatenate(T), np.concatenate(X), np.concatenate(Y), np.concatenate(P)


def looks_from_poses(t, x, y, psi, grid, model) -> Looks:
    """Unique (cell, g) looks of poses (t, x, y, psi) on `grid` (a BeliefSnapshot-like object
    with width, height, resolution, origin_x, origin_y)."""
    if len(t) == 0 or model.r_hi < model.r_lo:
        return Looks.empty()
    res = grid.resolution
    rs = np.arange(model.r_lo, model.r_hi + 1e-9, res / 2.0)
    pdr = model.pd_r(rs)
    cells, gs, pds, ts = [], [], [], []
    for side in (1.0, -1.0):
        b = psi + side * math.pi / 2.0
        k = model.sector_of(b)
        g = model.fold(k)
        cb, sb = np.cos(b), np.sin(b)
        gx = x[:, None] + rs[None, :] * cb[:, None]
        gy = y[:, None] + rs[None, :] * sb[:, None]
        col = np.floor((gx - grid.origin_x) / res).astype(np.int64)
        row = np.floor((gy - grid.origin_y) / res).astype(np.int64)
        ok = (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height)
        cells.append((row * grid.width + col)[ok])
        gs.append(np.broadcast_to(g[:, None], ok.shape)[ok])
        pds.append(np.broadcast_to(pdr[None, :], ok.shape)[ok])
        ts.append(np.broadcast_to(t[:, None], ok.shape)[ok])
    cell = np.concatenate(cells)
    g = np.concatenate(gs)
    pd = np.concatenate(pds)
    tt = np.concatenate(ts)
    keep = pd > 0
    cell, g, pd, tt = cell[keep], g[keep], pd[keep], tt[keep]
    if len(cell) == 0:
        return Looks.empty()
    key = cell.astype(np.int64) * 64 + g
    order = np.lexsort((tt, key))
    key, cell, g, pd, tt = key[order], cell[order], g[order], pd[order], tt[order]
    starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
    return Looks(cell[starts], g[starts], np.maximum.reduceat(pd, starts),
                 np.minimum.reduceat(tt, starts))


def plan_looks(plan, t_from: float, t_to: float, grid, model) -> Looks:
    t, x, y, psi = plan_poses(plan, t_from, t_to, grid.resolution / 2.0)
    return looks_from_poses(t, x, y, psi, grid, model)


def union(*parts: Looks) -> Looks:
    """Union of look sets: one entry per (cell, g), max pd, min t."""
    parts = [p for p in parts if len(p)]
    if not parts:
        return Looks.empty()
    cell = np.concatenate([p.cell for p in parts])
    g = np.concatenate([p.g for p in parts])
    pd = np.concatenate([p.pd for p in parts])
    t = np.concatenate([p.t for p in parts])
    key = cell.astype(np.int64) * 64 + g
    order = np.lexsort((t, key))
    key, cell, g, pd, t = key[order], cell[order], g[order], pd[order], t[order]
    starts = np.flatnonzero(np.r_[True, key[1:] != key[:-1]])
    return Looks(cell[starts], g[starts], np.maximum.reduceat(pd, starts),
                 np.minimum.reduceat(t, starts))


def subtract_keys(a: Looks, keys: np.ndarray) -> Looks:
    """Looks of `a` whose (cell, g) key is not in `keys`."""
    if len(a) == 0 or len(keys) == 0:
        return a
    m = ~np.isin(a.key, keys)
    return Looks(a.cell[m], a.g[m], a.pd[m], a.t[m])
