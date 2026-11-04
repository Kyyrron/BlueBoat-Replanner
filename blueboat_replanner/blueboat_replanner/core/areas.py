"""Mission-area geometry (MATH.md §3.1): validation and exact vectorised predicates.

Feasible set F_d = { p : p inside the polygon and dist(p, boundary) >= d }.
No polygon offsetting and no geometry library: numpy only.
"""

from __future__ import annotations

import math

import numpy as np

INCLUSION_TOL = 1e-6


class MissionError(ValueError):
    """Invalid or infeasible mission; the message says what and why."""


def _segments_intersect(p1, p2, p3, p4) -> bool:
    """Proper or touching intersection of segments p1p2 and p3p4."""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def on_seg(a, b, c):
        return (min(a[0], b[0]) - 1e-12 <= c[0] <= max(a[0], b[0]) + 1e-12 and
                min(a[1], b[1]) - 1e-12 <= c[1] <= max(a[1], b[1]) + 1e-12)

    d1, d2 = orient(p3, p4, p1), orient(p3, p4, p2)
    d3, d4 = orient(p1, p2, p3), orient(p1, p2, p4)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)) and d1 != 0 and d2 != 0 \
            and d3 != 0 and d4 != 0:
        return True
    if d1 == 0 and on_seg(p3, p4, p1):
        return True
    if d2 == 0 and on_seg(p3, p4, p2):
        return True
    if d3 == 0 and on_seg(p1, p2, p3):
        return True
    if d4 == 0 and on_seg(p1, p2, p4):
        return True
    return False


def signed_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


class Area:
    """One simple polygon in the map frame, normalised to counter-clockwise."""

    def __init__(self, vertices, min_area: float = 0.0):
        v = np.asarray(vertices, dtype=float)
        if v.ndim != 2 or v.shape[1] != 2:
            raise MissionError("area: vertices must be an (n, 2) array")
        if not np.all(np.isfinite(v)):
            raise MissionError("area: non-finite vertex")
        # drop consecutive duplicates, including a closing duplicate
        keep = [0]
        for i in range(1, len(v)):
            if math.hypot(*(v[i] - v[keep[-1]])) > 1e-6:
                keep.append(i)
        v = v[keep]
        if len(v) > 1 and math.hypot(*(v[0] - v[-1])) <= 1e-6:
            v = v[:-1]
        if len(v) < 3:
            raise MissionError("area: a polygon needs at least 3 distinct vertices")
        n = len(v)
        for i in range(n):
            for j in range(i + 1, n):
                if j == i + 1 or (i == 0 and j == n - 1):
                    continue
                if _segments_intersect(v[i], v[(i + 1) % n], v[j], v[(j + 1) % n]):
                    raise MissionError(f"area: polygon is not simple (edges {i} and {j} meet)")
        a = signed_area(v)
        if abs(a) < 1e-9:
            raise MissionError("area: polygon has zero area")
        if a < 0:
            v = v[::-1].copy()
        if abs(a) < min_area:
            raise MissionError(f"area: {abs(a):.1f} m2 is below the minimum {min_area:.1f} m2")
        self.vertices = v
        self.area = abs(a)
        self._a = v
        self._b = np.roll(v, -1, axis=0)

    # ------------------------------------------------------------- predicates
    def contains(self, pts) -> np.ndarray:
        """Even-odd ray test, vectorised over points (boundary points count as inside within
        the distance test of `feasible`)."""
        p = np.atleast_2d(np.asarray(pts, dtype=float))
        x, y = p[:, 0:1], p[:, 1:2]
        ax, ay = self._a[:, 0][None, :], self._a[:, 1][None, :]
        bx, by = self._b[:, 0][None, :], self._b[:, 1][None, :]
        cond = (ay > y) != (by > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = ax + (y - ay) * (bx - ax) / (by - ay)
        cross = cond & (x < xint)
        return (np.count_nonzero(cross, axis=1) % 2) == 1

    def boundary_distance(self, pts) -> np.ndarray:
        p = np.atleast_2d(np.asarray(pts, dtype=float))
        a = self._a[None, :, :]
        ab = (self._b - self._a)[None, :, :]
        ap = p[:, None, :] - a
        denom = np.sum(ab * ab, axis=2)
        t = np.clip(np.sum(ap * ab, axis=2) / denom, 0.0, 1.0)
        proj = a + t[:, :, None] * ab
        d = np.hypot(p[:, None, 0] - proj[:, :, 0], p[:, None, 1] - proj[:, :, 1])
        return d.min(axis=1)

    def feasible(self, pts, d: float) -> np.ndarray:
        """Points in F_d."""
        p = np.atleast_2d(np.asarray(pts, dtype=float))
        out = np.zeros(len(p), dtype=bool)
        if len(p) == 0:
            return out
        # chunk to bound memory (points x edges)
        step = max(1, 200000 // max(1, len(self._a)))
        for i in range(0, len(p), step):
            q = p[i:i + step]
            out[i:i + step] = self.contains(q) & (self.boundary_distance(q) >= d - INCLUSION_TOL)
        return out

    def all_feasible(self, pts, d: float) -> bool:
        return bool(np.all(self.feasible(pts, d)))

    # ------------------------------------------------------------- geometry
    def convex_hull(self) -> np.ndarray:
        pts = sorted(map(tuple, self.vertices))

        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower, upper = [], []
        for p in pts:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)
        for p in reversed(pts):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)
        return np.asarray(lower[:-1] + upper[:-1], dtype=float)

    def long_axis_direction(self) -> float:
        """Direction (rad, in [0, pi)) of the hull edge whose orthogonal width is minimal: lines
        parallel to it minimise the number of passes (MATH §3.2, 'longest extent')."""
        hull = self.convex_hull()
        best, best_theta = math.inf, 0.0
        for i in range(len(hull)):
            e = hull[(i + 1) % len(hull)] - hull[i]
            theta = math.atan2(e[1], e[0]) % math.pi
            n = np.array([-math.sin(theta), math.cos(theta)])
            proj = hull @ n
            width = float(proj.max() - proj.min())
            # deterministic tie-break: smaller width, then smaller angle
            if width < best - 1e-9 or (abs(width - best) <= 1e-9 and theta < best_theta):
                best, best_theta = width, theta
        return best_theta

    def reflex_vertices(self) -> list:
        v = self.vertices
        n = len(v)
        out = []
        for i in range(n):
            e1 = v[i] - v[i - 1]
            e2 = v[(i + 1) % n] - v[i]
            if e1[0] * e2[1] - e1[1] * e2[0] < -1e-12:      # right turn on a CCW polygon
                out.append(i)
        return out

    def reflex_offset_points(self, d: float) -> np.ndarray:
        """Each reflex vertex moved inward to distance d from both adjacent edges (MATH §3.3)."""
        v = self.vertices
        n = len(v)
        pts = []
        for i in self.reflex_vertices():
            e1 = v[i] - v[i - 1]
            e2 = v[(i + 1) % n] - v[i]
            n1 = np.array([-e1[1], e1[0]]) / np.hypot(*e1)
            n2 = np.array([-e2[1], e2[0]]) / np.hypot(*e2)
            c = float(n1 @ n2)
            if 1.0 + c < 1e-6:
                continue
            pts.append(v[i] + (d / (1.0 + c)) * (n1 + n2))
        return np.asarray(pts, dtype=float).reshape(-1, 2)

    def cross_track_extent(self, theta: float) -> tuple:
        n = np.array([-math.sin(theta), math.cos(theta)])
        proj = self.vertices @ n
        return float(proj.min()), float(proj.max())

    def along_track_extent(self, theta: float) -> tuple:
        u = np.array([math.cos(theta), math.sin(theta)])
        proj = self.vertices @ u
        return float(proj.min()), float(proj.max())
