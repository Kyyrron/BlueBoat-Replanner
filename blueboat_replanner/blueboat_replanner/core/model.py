"""Planner sensor model (MATH.md §5-§6). Placeholders until E3; configuration, not code.

PD_k(r, b, phi) = PD_r(r) * s_k * A_k(b, phi). Orientation phi takes the S sector-centre values
with a uniform prior. Folded look dependence: one look per (target, folded sector g = k mod S/2).
r is the ground range from the transducer; a ground point sits at r + lever arm from the boat's
centre line (MATH §5).
"""

from __future__ import annotations

import math

import numpy as np

from .spacing import r_geo

SECTOR_ROUND_DECIMALS = 9      # degrees rounded to 1e-9 before the sector floor (MATH §1)
# look keys are cell * KEY_STRIDE + g (MATH §5): valid while the folded sector count G is below
# it, which `evidence.validate_grid` checks for every grid
KEY_STRIDE = 64


def folded_count(sector_count: int) -> int:
    """G: the folded sector count (S/2 for even S, else S; MATH §1)."""
    S = int(sector_count)
    return S // 2 if S % 2 == 0 else S


def sector_index(bearing, sector_count: int, origin_deg: float):
    """Directed sector of a bearing [rad] (MATH §1; Belief topics.md §4 item 3): the angle is
    rounded to 1e-9 deg first, so a bearing on a sector edge goes to the upper sector."""
    deg = np.degrees(np.asarray(bearing, float))
    a = np.mod(np.round(deg - origin_deg, SECTOR_ROUND_DECIMALS), 360.0)
    k = np.floor(a / (360.0 / sector_count)).astype(int)
    return np.clip(k, 0, sector_count - 1)


class SensorModel:
    def __init__(self, cfg, class_names, sector_count: int, sector_origin_deg: float, h: float,
                 belief_model: dict | None = None):
        self.cfg = cfg
        self.names = tuple(class_names)
        self.S = int(sector_count)
        self.folded = self.S % 2 == 0
        self.G = folded_count(self.S)
        self.o = float(sector_origin_deg)
        self.fallbacks = []
        self.unmapped = []          # Belief class names with no aspect family (MATH §6)
        m, w, a, s = [], [], [], []
        for name in self.names:
            cm = cfg.classes.get(cfg.class_families.get(name, name))
            if cm is None:
                cm_m, cm_w, cm_a, cm_s = 0, 0.0, 1.0, 1.0
                self.fallbacks.append(f"class_{name}_isotropic")
                self.unmapped.append(name)
            else:
                cm_m, cm_w, cm_a, cm_s = cm.m, cm.w_rad, cm.a, cm.s
            m.append(cm_m)
            w.append(cm_w)
            a.append(cm_a)
            s.append(cm_s)
        self.m = np.array(m, dtype=int)
        self.w = np.array(w, dtype=float)
        self.a = np.array(a, dtype=float)
        self.s = np.array(s, dtype=float)
        if cfg.aspect_agnostic:
            self.m = np.zeros_like(self.m)
        self.K = len(self.names)
        self.mapped = np.array([n not in self.unmapped for n in self.names], dtype=bool)
        self.phi = np.radians(self.o + (np.arange(self.S) + 0.5) * 360.0 / self.S)
        self.pi_phi = np.full(self.S, 1.0 / self.S)
        self.pi_k = np.full(self.K, 1.0 / self.K)
        self.r_lo = cfg.pd_r_lo_m
        self.r_hi = min(cfg.pd_r_hi_m, r_geo(cfg.range_m, h))
        self.p = cfg.pd_plateau
        self.lever = cfg.lever_arm_m
        self.speed = cfg.speed_mps
        # turn gate (MATH §5): Belief's published switch and threshold when present
        tg = (belief_model or {}).get("turn_gating") or {}
        self.turn_gate = bool(tg["enabled"]) if isinstance(tg.get("enabled"), bool) \
            else bool(cfg.turn_gate)
        rate = tg.get("min_yaw_rate_rad_s")
        self.turn_min_rate = float(rate) if isinstance(rate, (int, float)) and rate > 0 \
            else float(cfg.turn_min_yaw_rate_rad_s)
        # A on the folded-sector grid: (G, K, S)
        self.A_g = np.stack([self.A(self.beta(g)) for g in range(self.G)])

    # ------------------------------------------------------------- sectors
    def beta(self, k: int) -> float:
        """Centre bearing of directed sector k."""
        return math.radians(self.o + (k + 0.5) * 360.0 / self.S)

    def sector_of(self, b):
        return sector_index(b, self.S, self.o)

    def fold(self, k):
        k = np.asarray(k, int)
        return k % self.G if self.folded else k

    # ------------------------------------------------------------- model
    def A(self, b) -> np.ndarray:
        """A_k(b, phi) for scalar bearing b: (K, S)."""
        out = np.ones((self.K, self.S))
        for k in range(self.K):
            mk = int(self.m[k])
            if mk == 0:
                continue
            P = 2.0 * math.pi / mk
            d = (b - self.phi) - P * np.round((b - self.phi) / P)
            out[k] = np.where(np.abs(d) <= self.w[k] + 1e-12, 1.0, self.a[k])
        return out

    def pd_r(self, r) -> np.ndarray:
        """PD_r at ground range r from the transducer (MATH §6: a box)."""
        r = np.asarray(r, float)
        return np.where((r >= self.r_lo) & (r <= self.r_hi), self.p, 0.0)
