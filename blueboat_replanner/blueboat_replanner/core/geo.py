"""Geodesy and frames (DESIGN.md §4).

`latlon_to_local_en` / `local_en_to_latlon` are verbatim copies of BlueBoat-MCS
`mcs/core/geo.py` l.38-52 (rev 1d5cb36), the project's single equirectangular pair (E-33).
The gate checks that their bodies and results are bit-identical to MCS when MCS is present.

`Frame2D` is the W (Control world = /blueboat/odom local ENU) <-> M (Belief map) transform
published by Belief (contract C-3): x_M = R(yaw)·x_W + t. Rotation is never assumed zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

EARTH_RADIUS_M = 6378137.0


# ----------------------------------------------------------------- lat/lon
def latlon_to_local_en(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Equirectangular projection of (lat, lon) to metres east/north of (lat0, lon0)."""
    east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * EARTH_RADIUS_M
    return east, north


def local_en_to_latlon(east: float, north: float, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


def wrap_pi(a):
    """Angle(s) to (-pi, pi], the same rule as Control (`path_update.wrap`)."""
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def wrap_scalar(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Frame2D:
    """x_M = R(yaw) x_W + (tx, ty); headings: psi_M = psi_W + yaw."""

    tx: float = 0.0
    ty: float = 0.0
    yaw: float = 0.0

    def world_to_map(self, x, y, psi=None):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        xm = c * x - s * y + self.tx
        ym = s * x + c * y + self.ty
        if psi is None:
            return xm, ym
        return xm, ym, wrap_pi(np.asarray(psi, dtype=float) + self.yaw)

    def map_to_world(self, x, y, psi=None):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx = np.asarray(x, dtype=float) - self.tx
        dy = np.asarray(y, dtype=float) - self.ty
        xw = c * dx + s * dy
        yw = -s * dx + c * dy
        if psi is None:
            return xw, yw
        return xw, yw, wrap_pi(np.asarray(psi, dtype=float) - self.yaw)

    def distance_to(self, other: "Frame2D", probe_radius: float = 50.0) -> float:
        """Largest displacement of points within probe_radius of the origin between the two
        transforms (used for the transform-jump log, DESIGN §4)."""
        dt = math.hypot(self.tx - other.tx, self.ty - other.ty)
        dyaw = abs(wrap_scalar(self.yaw - other.yaw))
        return dt + probe_radius * dyaw


def areas_to_map(areas_latlon, datum_lat: float, datum_lon: float) -> list:
    """Mission polygons [[lat, lon], ...] -> list of (n, 2) arrays in M (east, north)."""
    out = []
    for poly in areas_latlon:
        pts = [latlon_to_local_en(float(lat), float(lon), datum_lat, datum_lon)
               for lat, lon in poly]
        out.append(np.asarray(pts, dtype=float))
    return out
