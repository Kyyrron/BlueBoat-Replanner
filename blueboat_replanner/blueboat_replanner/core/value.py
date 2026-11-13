"""Contact posterior and value, background calibration and value (MATH.md §7-§9, §11.1).

All functions are pure and deterministic. Shapes: K classes x S orientations ("hypotheses").
Look sets are given in FOLDED sectors g with a per-look detection probability scale pd
(PD_r at the look's range; the class scale and aspect window are applied here).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import SensorModel


# =========================================================================== background
def calibrate_p(Q, n, s, pi, iterations: int = 60):
    """Solve sum_k pi_k (1 - p s_k)^n = Q for p in [0, 1] (MATH §9.1), vectorised over cells.
    Returns (p, clamped) — clamped where Q < Q_min = sum pi_k (1 - s_k)^n (p = 1)."""
    Q = np.asarray(Q, float)
    n = np.asarray(n, float)
    s = np.asarray(s, float)
    pi = np.asarray(pi, float)
    pi = pi / pi.sum()

    def M(p):
        return np.sum(pi[None, :] * (1.0 - p[:, None] * s[None, :]) ** n[:, None], axis=1)

    lo = np.zeros_like(Q)
    hi = np.ones_like(Q)
    qmin = M(hi)
    clamped = Q < qmin
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        above = M(mid) > Q            # residual still too high -> need larger p
        lo = np.where(above, mid, lo)
        hi = np.where(above, hi, mid)
    p = 0.5 * (lo + hi)
    p = np.where(clamped, 1.0, p)
    p = np.where(n <= 0, 0.0, p)
    return p, clamped


@dataclass
class Background:
    """Per eligible cell: q_i * w_i(k, phi) (the undetected-target mass by hypothesis)."""

    cells: np.ndarray        # flat cell indices (eligible)
    qw: np.ndarray           # (n_cells, K, S)
    seen: np.ndarray         # (n_cells, G) bool — folded sectors a detector searched (MATH §9)
    q: np.ndarray            # (n_cells,)
    clamped: int
    no_sector_fallback: int
    index: dict              # flat cell -> row
    searched_source: str = "aspect_log_miss"     # or "aspect_counts" [FALLBACK]
    pbg_rel_diff: float | None = None            # MATH §9.2 consistency value (None: no field)

    def info(self) -> dict:
        """The `calibration` block of a decision record (topics.md §5.1)."""
        return {"cells": int(len(self.cells)), "clamped": int(self.clamped),
                "fallback_no_sector": int(self.no_sector_fallback),
                "searched_source": self.searched_source,
                "p_background_median_rel_diff": self.pbg_rel_diff}


def build_background(snapshot, model: SensorModel, eligible_mask: np.ndarray, cfg) -> Background:
    """MATH §9: Q = 1 - coverage, S_i = the folded sectors a detector searched
    (`aspect_log_miss < 0`; `aspect_counts > 0` only when that field is absent), calibrated
    p_i, L_i, q_i, w_i."""
    H, W = snapshot.coverage.shape
    flat = np.flatnonzero(eligible_mask.ravel())
    cov = snapshot.coverage.ravel()[flat]
    Q = 1.0 - cov
    searched = snapshot.searched
    if searched is not None:
        ac = searched.reshape(H * W, snapshot.sector_count)[flat]
        source = "aspect_log_miss"
    else:
        ac = snapshot.aspect_counts.reshape(H * W, snapshot.sector_count)[flat] > 0
        source = "aspect_counts"
    G = model.G
    if model.folded:
        seen = np.zeros((len(flat), G), dtype=bool)
        for k in range(snapshot.sector_count):
            seen[:, k % G] |= ac[:, k]
    else:
        seen = ac.copy()
    n = seen.sum(axis=1).astype(float)
    nofall = (n == 0) & (Q < 1.0 - 1e-12)
    n_eff = np.where(nofall, 1.0, n)
    p, clamped = calibrate_p(Q, n_eff, model.s, model.pi_k, cfg.calibration_iterations)
    K, S = model.K, model.S
    logL = np.zeros((len(flat), K, S))
    for g in range(G):
        m = seen[:, g]
        if not np.any(m):
            continue
        term = 1.0 - p[m, None, None] * model.s[None, :, None] * model.A_g[g][None, :, :]
        logL[m] += np.log(np.clip(term, 1e-300, None))
    if np.any(nofall):
        term = 1.0 - p[nofall, None, None] * model.s[None, :, None] * np.ones((1, K, S))
        logL[nofall] = np.log(np.clip(term, 1e-300, None))
    L = np.exp(logL)
    prior = model.pi_k[:, None] * model.pi_phi[None, :]
    Mi = np.sum(prior[None] * L, axis=(1, 2))
    p0 = snapshot.prior_density_per_ha * snapshot.resolution ** 2 / 1e4
    q = p0 * Mi / (1.0 - p0 + p0 * Mi)
    with np.errstate(invalid="ignore", divide="ignore"):
        w = prior[None] * L / Mi[:, None, None]
    w = np.nan_to_num(w)
    qw = q[:, None, None] * w
    index = {int(c): i for i, c in enumerate(flat)}
    # MATH §9.2: Belief's p_background against its closed form p0 Q / (1 - p0 + p0 Q)
    rel = None
    if snapshot.p_background is not None and len(flat):
        pb = snapshot.p_background.ravel()[flat]
        closed = p0 * Q / (1.0 - p0 + p0 * Q)
        ok = np.isfinite(pb) & (pb > 0)
        if np.any(ok):
            rel = float(np.median(np.abs(closed[ok] - pb[ok]) / pb[ok]))
    return Background(flat, qw, seen, q, int(clamped.sum()), int(nofall.sum()), index, source,
                      rel)


def background_value(bg: Background, model: SensorModel, rows, g, pd) -> float:
    """Expected new detections E = sum_i q_i sum w_i [1 - prod_new (1 - pd s_k A_k(beta_g))]
    over unique (row, g) looks (rows index into bg; looks already seen must be removed by the
    caller)."""
    if len(rows) == 0:
        return 0.0
    rows = np.asarray(rows)
    g = np.asarray(g)
    pd = np.asarray(pd, float)
    order = np.argsort(rows, kind="stable")
    rows, g, pd = rows[order], g[order], pd[order]
    logt = np.log(np.clip(1.0 - pd[:, None, None] * model.s[None, :, None] * model.A_g[g],
                          1e-300, None))
    starts = np.flatnonzero(np.r_[True, rows[1:] != rows[:-1]])
    sums = np.add.reduceat(logt, starts, axis=0)
    ur = rows[starts]
    return float(np.sum(bg.qw[ur] * (1.0 - np.exp(sums))))


# =========================================================================== contacts
@dataclass
class ContactState:
    uid: int
    p_tilde: float
    lam: np.ndarray            # (K, S)
    seen_g: frozenset          # folded sectors detected or missed (grid)
    detected_g: frozenset
    confirmed: bool
    cell: int
    x: float
    y: float
    cov: np.ndarray
    fallbacks: tuple
    q0: float = float("nan")           # first_confidence as read (raw while uncalibrated)
    missed: tuple = ()                 # ((folded sector, strength), ...) used in the posterior
    mapped_mass: float = 1.0           # class probability carried by classes with a family


def _p_k(model: SensorModel, g: int, pd: float) -> np.ndarray:
    return pd * model.s[:, None] * model.A_g[g]


def cell_misses(snapshot, model: SensorModel, cell: int):
    """{folded sector: recorded strength} of the sectors a detector searched at `cell`
    (MATH §7): strength = max covered PD = 1 - exp(aspect_log_miss). None when the grid has no
    per-sector miss record."""
    if snapshot.miss_pd is None:
        return None
    row = snapshot.miss_pd.reshape(-1, snapshot.sector_count)[cell]
    out = {}
    for k in np.flatnonzero(row > 0.0):
        g = int(model.fold(int(k)))
        out[g] = max(out.get(g, 0.0), float(row[k]))
    return out


def contact_posterior(c, model: SensorModel, cfg, cell: int, cell_miss=None) -> ContactState:
    """MATH §7: p~_c and lambda_c(k, phi) from the contact's look history. Misses are the
    sectors searched at the contact's cell (`cell_miss`, at their recorded strength) and the
    contact's own `sectors_missed` (at the plateau when the cell record does not hold them),
    minus the detected sectors."""
    fold = model.fold
    D = sorted({int(fold(k)) for k in c.sectors_detected})
    pbar = model.p
    miss = {int(fold(k)): pbar for k in c.sectors_missed}
    fallbacks = list(c.fallbacks)
    if cell_miss is None:
        fallbacks.append("misses_from_sector_sets")
    else:
        miss.update({int(g): float(v) for g, v in cell_miss.items()})
    for g in D:
        miss.pop(g, None)
    Mi = sorted(miss)
    prior = (np.asarray(c.class_probabilities)[:, None] * model.pi_phi[None, :])
    f = cfg.clutter_f

    def post_given_first(g1):
        w1 = prior * _p_k(model, g1, pbar)
        if w1.sum() <= 0:
            w1 = prior.copy()
        w1 = w1 / w1.sum()
        lik = w1.copy()
        rest = [g for g in D if g != g1]
        for g in rest:
            lik = lik * _p_k(model, g, pbar)
        L_clut = f ** len(rest)
        for g in Mi:
            lik = lik * (1.0 - _p_k(model, g, miss[g]))
            # the same range factor for clutter: 1 - f on the plateau (MATH §7)
            L_clut *= 1.0 - f * (min(1.0, miss[g] / pbar) if pbar > 0 else 0.0)
        return lik, float(lik.sum()), L_clut

    if c.first_sector != 255 and int(fold(c.first_sector)) in D:
        firsts = [int(fold(c.first_sector))]
    else:
        firsts = D if D else []
        if c.first_sector != 255:
            fallbacks.append("first_sector_not_in_detected")
    if not firsts:
        lam = prior / prior.sum()
        L_real, L_clut = 1.0, 1.0
    else:
        lams, Lr, Lc = [], [], []
        for g1 in firsts:
            lik, lr, lc = post_given_first(g1)
            lams.append(lik)
            Lr.append(lr)
            Lc.append(lc)
        lam = np.mean(lams, axis=0)
        L_real, L_clut = float(np.mean(Lr)), float(np.mean(Lc))
        lam = lam / lam.sum() if lam.sum() > 0 else prior / prior.sum()
    q0 = c.first_confidence
    if math.isnan(q0):
        p_t = float(c.p_exists)
    else:
        num = q0 * L_real
        den = num + (1.0 - q0) * L_clut
        p_t = float(num / den) if den > 0 else float(q0)
    mapped = float(np.sum(np.asarray(c.class_probabilities)[model.mapped]))
    return ContactState(c.uid, p_t, lam, frozenset(D) | frozenset(Mi), frozenset(D),
                        bool(c.confirmed), cell, c.x, c.y, c.cov, tuple(fallbacks),
                        float(q0), tuple((g, miss[g]) for g in Mi), mapped)


def contact_value(cs: ContactState, model: SensorModel, cfg, looks: dict) -> float:
    """MATH §8: V_c(N) for new folded sectors {g: pd}. Zero if confirmed or N empty."""
    if cs.confirmed or not looks:
        return 0.0
    prod = np.ones_like(cs.lam)
    for g, pd in looks.items():
        prod = prod * (1.0 - _p_k(model, g, pd))
    p_real = float(np.sum(cs.lam * (1.0 - prod)))
    p_clut = 1.0 - (1.0 - cfg.clutter_f) ** len(looks)
    return cs.p_tilde * p_real - cfg.kappa * (1.0 - cs.p_tilde) * p_clut


def detection_probability(cs: ContactState, model: SensorModel, cfg, looks: dict) -> float:
    if cs.confirmed or not looks:
        return 0.0
    prod = np.ones_like(cs.lam)
    for g, pd in looks.items():
        prod = prod * (1.0 - _p_k(model, g, pd))
    p_real = float(np.sum(cs.lam * (1.0 - prod)))
    p_clut = 1.0 - (1.0 - cfg.clutter_f) ** len(looks)
    return cs.p_tilde * p_real + (1.0 - cs.p_tilde) * p_clut


def after_miss(cs: ContactState, model: SensorModel, cfg, looks: dict) -> ContactState:
    """Posterior after a miss on `looks` (MATH §11.1)."""
    prod = np.ones_like(cs.lam)
    for g, pd in looks.items():
        prod = prod * (1.0 - _p_k(model, g, pd))
    pm_real = float(np.sum(cs.lam * prod))
    pm_clut = (1.0 - cfg.clutter_f) ** len(looks)
    den = cs.p_tilde * pm_real + (1.0 - cs.p_tilde) * pm_clut
    p_t = cs.p_tilde * pm_real / den if den > 0 else cs.p_tilde
    lam = cs.lam * prod
    lam = lam / lam.sum() if lam.sum() > 0 else cs.lam
    return ContactState(cs.uid, float(p_t), lam, cs.seen_g | frozenset(looks), cs.detected_g,
                        cs.confirmed, cs.cell, cs.x, cs.y, cs.cov, cs.fallbacks, cs.q0,
                        cs.missed, cs.mapped_mass)

