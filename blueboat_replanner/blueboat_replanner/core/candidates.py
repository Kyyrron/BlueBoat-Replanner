"""Revisit legs and detours (MATH.md §10.2).

A REVISIT body passes the contact at ground range r* so that the look bearing is b, on side s:
heading psi = b - s*pi/2, closest-approach point q = m - r*(cos b, sin b), straight length
2*max(3*sigma_along, L_min/2) plus the settle lead-in l_in at both ends.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .dubins import Straight, sample_prims
from .spacing import altitude, r_geo


@dataclass(frozen=True)
class RevisitLeg:
    uid: int
    bearing: float      # directed look bearing b (rad, ENU)
    side: int           # +1 port, -1 starboard
    r_star: float
    body: Straight
    r_star_adjusted: bool

    @property
    def start_pose(self):
        return (self.body.x, self.body.y, self.body.th)

    @property
    def end_pose(self):
        return self.body.end()


def detection_band(cfg, h: float) -> tuple:
    """Detection band [r_lo, min(r_hi, r_geo)] (MATH §6)."""
    return cfg.pd_r_lo_m, min(cfg.pd_r_hi_m, r_geo(cfg.range_m, h))


def revisit_legs(contact, bearing: float, cfg, depth_m: float, area=None) -> list:
    """The four candidate legs for directed bearing `bearing` and its opposite, both sides.
    Only legs whose body lies in F_margin are returned when `area` is given."""
    h = altitude(depth_m, cfg.sensor_depth_m)
    lo, hi = detection_band(cfg, h)
    r = cfg.r_star_m
    adjusted = False
    if not (lo <= r <= hi):
        r = 0.5 * (lo + hi)
        adjusted = True
    m = np.array([contact.x, contact.y])
    out = []
    for b in (bearing, bearing + math.pi):
        b = math.atan2(math.sin(b), math.cos(b))
        for s in (1, -1):
            psi = b - s * math.pi / 2.0
            u = np.array([math.cos(psi), math.sin(psi)])
            sig = math.sqrt(max(0.0, float(u @ contact.cov @ u)))
            l_in = cfg.speed_mps * cfg.t_settle_s
            L = 2.0 * max(3.0 * sig, cfg.l_min_m / 2.0) + 2.0 * l_in
            q = m - r * np.array([math.cos(b), math.sin(b)])
            p0 = q - 0.5 * L * u
            body = Straight(float(p0[0]), float(p0[1]), math.atan2(u[1], u[0]), L)
            if area is not None:
                x, y, _ = sample_prims([body], cfg.geom_step_m)
                if not area.all_feasible(np.column_stack([x, y]), cfg.margin_m):
                    continue
            out.append(RevisitLeg(contact.uid, b, s, r, body, adjusted))
    return out
