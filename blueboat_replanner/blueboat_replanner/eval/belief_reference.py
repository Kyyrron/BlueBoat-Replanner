"""REFERENCE BELIEF — a stand-in written from the briefs and the contract `belief_grid/0`.

THIS IS NOT BlueBoat-Belief. It implements the semantics the Replanner brief (§9.2) and the
contract (C-3, C-4) describe, so that the development loop can run in the cloud without the
real core. Every result produced with it is labelled "reference-belief-conditional,
preliminary" and must be re-run with the real core (`belief_real.py`, in-machine) before any
report.

Semantics (each a reference choice, documented in docs/DEVLOOP.md):
- grid cells inside the mission area are valid; outside -> NaN;
- a LOOK = (cell, directed sector, pass); one per pass and sector (pings are not looks);
- miss update Q <- Q (1 - PD_b(r))^rho, rho = 1 for a sector new to the cell, rho_repeat
  otherwise (an independence factor; opposite sides are different sectors, as the brief says
  Belief does);
- contacts from detections: nearest-contact gating (3 sigma), new uid otherwise (never reused);
  detected / missed SECTOR sets (a missed look stored at its sector); a detection from a sector
  previously missed adds no evidence; confirmed = >= 2 detections on different passes;
- p_exists: odds update with PD_b / f_ref per detected new-sector look and
  (1 - PD_b) / (1 - f_ref) per missed look; pruned below 0.01 (uid not reused).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

CONTRACT_VERSION = "belief_grid/0"


@dataclass
class RefContact:
    uid: int
    x: float
    y: float
    cov: np.ndarray
    p_exists: float
    first_confidence: float
    first_sector: int
    max_confidence: float
    detected: set = field(default_factory=set)
    missed: set = field(default_factory=set)
    det_passes: set = field(default_factory=set)
    state: int = 0
    pending_miss: dict = field(default_factory=dict)   # pass -> sector

    def as_dict(self, n_classes: int) -> dict:
        return {"uid": self.uid, "x": self.x, "y": self.y,
                "covariance": self.cov.ravel().tolist(), "p_exists": self.p_exists,
                "state": self.state, "class_probabilities": [1.0 / n_classes] * n_classes,
                "sectors_detected": sorted(self.detected), "sectors_missed": sorted(self.missed),
                "n_looks_detected": len(self.det_passes),
                "first_confidence": self.first_confidence, "first_sector": self.first_sector,
                "max_confidence": self.max_confidence, "shadow_known": False,
                "has_shadow": False}


class ReferenceBelief:
    def __init__(self, area, resolution: float, datum, class_names, sector_count: int,
                 sector_origin_deg: float, prior_density_per_ha: float, depth_m: float,
                 pd_plateau: float = 0.7, pd_r_lo: float = 3.0, pd_r_hi: float = 25.0,
                 range_m: float = 20.0, sensor_depth_m: float = 0.22, rho_repeat: float = 0.3,
                 f_ref: float = 0.2, gate_sigma: float = 3.0, det_sigma: float = 0.5):
        v = area.vertices
        pad = 2.0
        self.origin_x = float(np.floor(v[:, 0].min() - pad))
        self.origin_y = float(np.floor(v[:, 1].min() - pad))
        self.res = float(resolution)
        self.width = int(math.ceil((v[:, 0].max() + pad - self.origin_x) / self.res))
        self.height = int(math.ceil((v[:, 1].max() + pad - self.origin_y) / self.res))
        H, W = self.height, self.width
        cols, rows = np.meshgrid(np.arange(W), np.arange(H))
        cx = self.origin_x + (cols + 0.5) * self.res
        cy = self.origin_y + (rows + 0.5) * self.res
        self.valid = area.contains(np.column_stack([cx.ravel(), cy.ravel()])).reshape(H, W)
        self.Q = np.ones((H, W))
        self.S = int(sector_count)
        self.o = float(sector_origin_deg)
        self.ac = np.zeros((H, W, self.S), dtype=np.int64)
        self.n_looks = np.zeros((H, W), dtype=np.int64)
        self.datum = datum
        self.class_names = list(class_names)
        self.density = float(prior_density_per_ha)
        self.depth = float(depth_m)
        self.pd = pd_plateau
        h = max(0.0, depth_m - sensor_depth_m)
        self.r_lo = pd_r_lo
        self.r_hi = min(pd_r_hi, math.sqrt(max(0.0, range_m ** 2 - h * h)))
        self.rho_rep = rho_repeat
        self.f_ref = f_ref
        self.gate = gate_sigma
        self.det_cov = np.eye(2) * det_sigma ** 2
        self.contacts: list[RefContact] = []
        self.next_uid = 1
        self.seq = 0
        self._looked = set()        # (cell, sector, pass)

    # ------------------------------------------------------------- geometry
    def sector_of(self, b):
        deg = np.degrees(np.asarray(b, float))
        return (np.floor(np.mod(deg - self.o, 360.0) / (360.0 / self.S))).astype(int) % self.S

    def cell_of(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.res))
        row = int(math.floor((y - self.origin_y) / self.res))
        return row, col

    def look_events(self, t, x, y, psi, pass_id):
        """Contract C-5 look geometry, computed independently of core.looks (reference)."""
        out = []
        rs = np.arange(self.r_lo, self.r_hi + 1e-9, self.res / 2.0)
        for i in range(len(t)):
            for side in (1, -1):
                b = psi[i] + side * math.pi / 2.0
                k = int(self.sector_of(b))
                for r in rs:
                    gx = x[i] + r * math.cos(b)
                    gy = y[i] + r * math.sin(b)
                    row, col = self.cell_of(gx, gy)
                    if 0 <= row < self.height and 0 <= col < self.width:
                        out.append((row, col, k, float(r), float(b), int(pass_id), float(t[i])))
        return out

    # ------------------------------------------------------------- updates
    def feed_looks(self, events):
        """Apply look events (one look per (cell, sector, pass)); register contact looks."""
        cells_this = {}
        for (row, col, k, r, b, pid, t) in events:
            if not self.valid[row, col]:
                continue
            key = (row, col, k, pid)
            if key in self._looked:
                continue
            self._looked.add(key)
            pd = self.pd if self.r_lo <= r <= self.r_hi else 0.0
            rho = 1.0 if self.ac[row, col, k] == 0 else self.rho_rep
            self.Q[row, col] *= (1.0 - pd) ** rho
            self.ac[row, col, k] = min(255, self.ac[row, col, k] + 1)
            self.n_looks[row, col] += 1
            cells_this.setdefault((row, col), (k, pid))
        for c in self.contacts:
            rc = self.cell_of(c.x, c.y)
            if rc in cells_this:
                k, pid = cells_this[rc]
                if pid not in c.det_passes:
                    c.pending_miss.setdefault(pid, k)

    def finalize_passes(self, current_pass):
        """Misses become final once their pass is over (and no detection came in it)."""
        for c in self.contacts:
            for pid in [p for p in c.pending_miss if p != current_pass]:
                k = c.pending_miss.pop(pid)
                if pid in c.det_passes:
                    continue
                if k not in c.detected:
                    c.missed.add(k)
                lr = (1.0 - self.pd) / (1.0 - self.f_ref)
                c.p_exists = _odds_update(c.p_exists, lr)
        self.contacts = [c for c in self.contacts if c.p_exists >= 0.01 or c.state == 1]

    def feed_detections(self, dets):
        """dets: (x, y, confidence, directed sector, pass id)."""
        for (x, y, q, k, pid) in dets:
            best, bd = None, math.inf
            for c in self.contacts:
                S = c.cov + self.det_cov
                d = np.array([x - c.x, y - c.y])
                m2 = float(d @ np.linalg.solve(S, d))
                if m2 <= self.gate ** 2 and m2 < bd:
                    best, bd = c, m2
            if best is None:
                c = RefContact(self.next_uid, float(x), float(y), self.det_cov.copy(), float(q),
                               float(q), int(k), float(q))
                c.detected.add(int(k))
                c.det_passes.add(pid)
                self.next_uid += 1
                self.contacts.append(c)
                continue
            c = best
            # position fusion (information form)
            Pi = np.linalg.inv(c.cov) + np.linalg.inv(self.det_cov)
            cov = np.linalg.inv(Pi)
            mu = cov @ (np.linalg.solve(c.cov, [c.x, c.y]) + np.linalg.solve(self.det_cov, [x, y]))
            c.x, c.y, c.cov = float(mu[0]), float(mu[1]), 0.5 * (cov + cov.T)
            c.max_confidence = max(c.max_confidence, float(q))
            new_pass = pid not in c.det_passes
            c.det_passes.add(pid)
            c.pending_miss.pop(pid, None)
            if k in c.missed:
                continue            # a detection from a previously missed sector adds no evidence
            if k not in c.detected:
                c.detected.add(int(k))
                c.p_exists = _odds_update(c.p_exists, self.pd / self.f_ref)
            if new_pass and len(c.det_passes) >= 2:
                c.state = 1

    # ------------------------------------------------------------- contract
    def to_grid_dict(self, stamp: float = 0.0) -> dict:
        self.seq += 1
        H, W = self.height, self.width
        cov = np.where(self.valid, 1.0 - self.Q, np.nan)
        p0 = self.density * self.res ** 2 / 1e4
        pb = np.where(self.valid, p0 * self.Q / (1.0 - p0 + p0 * self.Q), np.nan)
        n = len(self.class_names)
        return {
            "contract_version": CONTRACT_VERSION, "grid_seq": self.seq, "width": W,
            "height": H, "resolution": self.res, "origin_x": self.origin_x,
            "origin_y": self.origin_y, "datum_lat": self.datum[0], "datum_lon": self.datum[1],
            "odom_to_map_x": 0.0, "odom_to_map_y": 0.0, "odom_to_map_yaw": 0.0,
            "odom_to_map_sigma_m": 0.0, "odom_to_map_valid": True,
            "aspect_sector_count": self.S, "aspect_sector_origin_deg": self.o,
            "class_names": list(self.class_names), "prior_density_per_ha": self.density,
            "model_params_json": '{"schema": 1, "source": "reference_belief (NOT Belief)"}',
            "p_background": pb.ravel(), "p_target": pb.ravel(), "coverage": cov.ravel(),
            "n_looks": self.n_looks.ravel(), "n_pings": self.n_looks.ravel(),
            "n_aspects": (self.ac > 0).sum(axis=2).ravel(),
            "depth_m": np.where(self.valid, self.depth, np.nan).ravel(),
            "aspect_counts": np.minimum(self.ac, 255).ravel(), "aspect_log_miss": [],
            "contacts": [c.as_dict(n) for c in self.contacts], "stamp": float(stamp)}


def _odds_update(p, lr):
    p = min(max(p, 1e-9), 1 - 1e-9)
    o = p / (1 - p) * lr
    return float(o / (1 + o))
