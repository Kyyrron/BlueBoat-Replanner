"""Path primitives and Dubins paths (MATH.md §3).

Primitives are analytic: `Straight` and `Arc` (constant curvature, left d=+1 / right d=-1),
plus `Hold` (stationary, a duration). Every Dubins candidate is integrated and kept only if
it reaches the goal pose within 1e-6 m / 1e-6 rad (self-check against formula errors).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

TWO_PI = 2.0 * math.pi


def _mod2pi(a: float) -> float:
    r = math.fmod(a, TWO_PI)
    return r + TWO_PI if r < 0 else r


@dataclass(frozen=True)
class Straight:
    x: float
    y: float
    th: float
    length: float

    def pose(self, s):
        s = np.asarray(s, dtype=float)
        c, si = math.cos(self.th), math.sin(self.th)
        return self.x + s * c, self.y + s * si, np.full_like(s, self.th)

    def end(self):
        x, y, th = self.pose(self.length)
        return float(x), float(y), float(th)


@dataclass(frozen=True)
class Arc:
    x: float
    y: float
    th: float
    radius: float
    d: int          # +1 left (CCW), -1 right (CW)
    length: float

    def pose(self, s):
        s = np.asarray(s, dtype=float)
        th_s = self.th + self.d * s / self.radius
        x = self.x + self.d * self.radius * (np.sin(th_s) - math.sin(self.th))
        y = self.y - self.d * self.radius * (np.cos(th_s) - math.cos(self.th))
        return x, y, th_s

    def end(self):
        x, y, th = self.pose(self.length)
        return float(x), float(y), float(th)


@dataclass(frozen=True)
class Hold:
    x: float
    y: float
    th: float
    duration: float
    length: float = 0.0

    def pose(self, s):
        s = np.asarray(s, dtype=float)
        return np.full_like(s, self.x), np.full_like(s, self.y), np.full_like(s, self.th)

    def end(self):
        return self.x, self.y, self.th


def chain_end(prims) -> tuple:
    return prims[-1].end()


def prims_length(prims) -> float:
    return float(sum(p.length for p in prims))


def straight_between(p0, p1) -> Straight:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return Straight(float(p0[0]), float(p0[1]), math.atan2(dy, dx), math.hypot(dx, dy))


# ------------------------------------------------------------------ Dubins
def _words(alpha, beta, d):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    out = {}
    # LSL
    tmp = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if tmp >= 0:
        p = math.sqrt(tmp)
        tmp1 = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (_mod2pi(-alpha + tmp1), p, _mod2pi(beta - tmp1))
    # RSR
    tmp = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if tmp >= 0:
        p = math.sqrt(tmp)
        tmp1 = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (_mod2pi(alpha - tmp1), p, _mod2pi(-beta + tmp1))
    # LSR
    tmp = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if tmp >= 0:
        p = math.sqrt(tmp)
        tmp1 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (_mod2pi(-alpha + tmp1), p, _mod2pi(-_mod2pi(beta) + tmp1))
    # RSL
    tmp = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if tmp >= 0:
        p = math.sqrt(tmp)
        tmp1 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (_mod2pi(alpha - tmp1), p, _mod2pi(beta - tmp1))
    # RLR
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(TWO_PI - math.acos(tmp))
        t = _mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + _mod2pi(p / 2.0))
        out["RLR"] = (t, p, _mod2pi(alpha - beta - t + _mod2pi(p)))
    # LRL
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(TWO_PI - math.acos(tmp))
        t = _mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out["LRL"] = (t, p, _mod2pi(_mod2pi(beta) - alpha - t + _mod2pi(p)))
    return out


def _build(word, seg, start, rho):
    x, y, th = start
    prims = []
    for letter, length in zip(word, seg):
        L = length * rho if letter != "S" else length * rho
        if L <= 1e-12:
            continue
        if letter == "S":
            pr = Straight(x, y, th, L)
        else:
            pr = Arc(x, y, th, rho, 1 if letter == "L" else -1, L)
        prims.append(pr)
        x, y, th = pr.end()
    return prims


def dubins_candidates(start, goal, rho: float, tol: float = 1e-6) -> list:
    """All valid Dubins paths from start (x, y, th) to goal, shortest first. Each item is
    (length, word, primitives). A zero-length path is returned when start == goal."""
    x0, y0, th0 = start
    x1, y1, th1 = goal
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    if D < 1e-12 and abs(math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))) < 1e-12:
        return [(0.0, "", [])]
    d = D / rho
    phi = math.atan2(dy, dx) if D > 0 else 0.0
    alpha = _mod2pi(th0 - phi)
    beta = _mod2pi(th1 - phi)
    out = []
    for word, seg in _words(alpha, beta, d).items():
        prims = _build(word, seg, (x0, y0, th0), rho)
        if not prims:
            continue
        ex, ey, eth = prims[-1].end()
        if math.hypot(ex - x1, ey - y1) > tol * max(1.0, D):
            continue
        if abs(math.atan2(math.sin(eth - th1), math.cos(eth - th1))) > 1e-6:
            continue
        out.append((prims_length(prims), word, prims))
    out.sort(key=lambda r: (r[0], r[1]))
    return out


def sample_prims(prims, step: float):
    """Points along a primitive chain every <= step (end points included)."""
    xs, ys, ths = [], [], []
    for p in prims:
        if p.length <= 0:
            x, y, th = p.pose(np.array([0.0]))
        else:
            n = max(1, int(math.ceil(p.length / step)))
            s = np.linspace(0.0, p.length, n + 1)
            x, y, th = p.pose(s)
        xs.append(np.atleast_1d(x))
        ys.append(np.atleast_1d(y))
        ths.append(np.atleast_1d(th))
    if not xs:
        return np.zeros(0), np.zeros(0), np.zeros(0)
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(ths)
