"""Planner sensor model (MATH.md §6). Placeholders until E3; configuration, not code.

PD_k(r, b, phi) = PD_r(r) * s_k * A_k(b, phi). Orientation phi takes the S sector-centre values
with a uniform prior. Folded look dependence: one look per (target, folded sector g = k mod S/2).
"""

from __future__ import annotations

import math

import numpy as np

from .spacing import r_geo


class SensorModel:
    def __init__(self, cfg, class_names, sector_count: int, sector_origin_deg: float, h: float):
        self.cfg = cfg
        self.names = tuple(class_names)
        self.S = int(sector_count)
        self.folded = self.S % 2 == 0
        self.G = self.S // 2 if self.folded else self.S
        self.o = float(sector_origin_deg)
        self.fallbacks = []
        m, w, a, s = [], [], [], []
        for name in self.names:
            cm = cfg.classes.get(name)
            if cm is None:
                cm_m, cm_w, cm_a, cm_s = 0, 0.0, 1.0, 1.0
                self.fallbacks.append(f"class_{name}_isotropic")
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
        self.phi = np.radians(self.o + (np.arange(self.S) + 0.5) * 360.0 / self.S)
        self.pi_phi = np.full(self.S, 1.0 / self.S)
        self.pi_k = np.full(self.K, 1.0 / self.K)
        self.r_lo = cfg.pd_r_lo_m
        self.r_hi = min(cfg.pd_r_hi_m, r_geo(cfg.range_m, h))
        self.p = cfg.pd_plateau
        # A on the folded-sector grid: (G, K, S)
        self.A_g = np.stack([self.A(self.beta(g)) for g in range(self.G)])

    # ------------------------------------------------------------- sectors
    def beta(self, k: int) -> float:
        """Centre bearing of directed sector k."""
        return math.radians(self.o + (k + 0.5) * 360.0 / self.S)

    def sector_of(self, b):
        deg = np.degrees(np.asarray(b, float))
        return (np.floor(np.mod(deg - self.o, 360.0) / (360.0 / self.S))).astype(int) % self.S

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
        r = np.asarray(r, float)
        return np.where((r >= self.r_lo) & (r <= self.r_hi), self.p, 0.0)

    def p_look_g(self, g: int, pd: float = None) -> np.ndarray:
        """Per-look detection probability on folded sector g: (K, S)."""
        pd = self.p if pd is None else pd
        return pd * self.s[:, None] * self.A_g[g]
