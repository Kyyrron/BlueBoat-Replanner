"""Value of executed sequences over the remaining budget (MATH.md §10-§11).

J(a) = lam * N(background looks) + sum_c V_c(contact looks) over the UNION of new looks along
the executed tail D_a ⊕ K, with K truncated at the budget (segment boundaries). Looks already
seen in the grid or pending (mirror over [tau - t_lat, c]) carry no value (MATH §11.3).

Exact and efficient (MATH §10.4): K's background value is precomputed as a step function of the
truncation time; only cells touched by a detour are recomputed.
"""

from __future__ import annotations

import math

import numpy as np

from .looks import Looks, plan_looks, union
from .model import KEY_STRIDE, SensorModel
from .plan import Plan
from .spacing import altitude
from .value import (after_miss, background_value, build_background, cell_misses,
                    contact_posterior, contact_value, detection_probability)


class Evaluator:
    def __init__(self, cfg, snapshot, area, K_plan: Plan | None, pending: Looks, B_rem: float,
                 pending_contacts: Looks | None = None):
        """`pending`: looks not yet in the grid (MATH §11.3, grid cells). `pending_contacts`:
        the same over the wider contact window (default: `pending`)."""
        self.cfg = cfg
        self.snap = snapshot
        h = altitude(cfg.planning_depth_m, cfg.sensor_depth_m)
        self.model = SensorModel(cfg, snapshot.class_names, snapshot.sector_count,
                                 snapshot.sector_origin_deg, h, snapshot.model)
        H, W = snapshot.coverage.shape
        cx, cy = snapshot.cell_centre(*np.mgrid[0:H, 0:W])
        inside = area.contains(np.column_stack([cx.ravel(), cy.ravel()])).reshape(H, W)
        elig = inside & np.isfinite(snapshot.coverage)
        # contacts: exclusion zones (Mahalanobis <= sigma) and states
        self.contacts = []
        for c in snapshot.contacts:
            try:
                inv = np.linalg.inv(c.cov + 1e-9 * np.eye(2))
            except np.linalg.LinAlgError:
                inv = np.eye(2)
            dx, dy = cx - c.x, cy - c.y
            m2 = inv[0, 0] * dx * dx + 2 * inv[0, 1] * dx * dy + inv[1, 1] * dy * dy
            elig &= m2 > cfg.contact_exclusion_sigma ** 2
            row, col = snapshot.cell_of(c.x, c.y)
            if 0 <= row < H and 0 <= col < W:
                cell = int(row * W + col)
                cs = contact_posterior(c, self.model, cfg, cell,
                                       cell_misses(snapshot, self.model, cell))
                self.contacts.append(cs)
        self.bg = build_background(snapshot, self.model, elig, cfg)
        self.pending = pending
        self.pending_keys = pending.key if len(pending) else np.zeros(0, np.int64)
        # contact seen sets include pending looks at their cell (the contact window)
        pc = pending if pending_contacts is None else pending_contacts
        self.pending_contacts = pc
        self.contact_pending = {}
        for cs in self.contacts:
            m = pc.cell == cs.cell if len(pc) else np.zeros(0, bool)
            self.contact_pending[cs.uid] = frozenset(int(g) for g in pc.g[m]) if len(
                pc) else frozenset()
        # K: the default continuation (relative time 0 = its first line's start); tails K[j:]
        self.B_rem = float(B_rem)
        self.K = K_plan
        self._tails = {}

    # ---------------------------------------------------------------- filters
    def _bg_new(self, looks: Looks):
        """(rows, g, pd, t) of looks that are background-eligible and not seen or pending."""
        if len(looks) == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0), np.zeros(0)
        rows = np.array([self.bg.index.get(int(c), -1) for c in looks.cell], dtype=int)
        m = rows >= 0
        if np.any(m):
            m[m] &= ~self.bg.seen[rows[m], looks.g[m]]
        if len(self.pending_keys):
            m &= ~np.isin(looks.key, self.pending_keys)
        return rows[m], looks.g[m], looks.pd[m], looks.t[m]

    def _contact_looks(self, cs, looks: Looks, exclude=frozenset()) -> dict:
        if len(looks) == 0:
            return {}
        m = looks.cell == cs.cell
        out = {}
        blocked = cs.seen_g | self.contact_pending[cs.uid] | exclude
        for g, pd in zip(looks.g[m], looks.pd[m]):
            g = int(g)
            if g in blocked:
                continue
            out[g] = max(out.get(g, 0.0), float(pd))
        return out

    # ---------------------------------------------------------------- K tails
    def tail(self, j: int) -> dict:
        """K[j:] retimed to 0: its looks and the background step function (MATH §10.4)."""
        if j in self._tails:
            return self._tails[j]
        if self.K is None or j >= len(self.K.segments):
            t = {"plan": None, "looks": Looks.empty(), "t": np.zeros(0), "inc": np.zeros(0),
                 "rows": np.zeros(0, int), "cum": np.zeros(0)}
            self._tails[j] = t
            return t
        plan = Plan(self.K.segments[j:], 0.0, self.K.speed)
        looks = plan_looks(plan, 0.0, plan.t_end, self.snap, self.model)
        rows, g, pd, t = self._bg_new(looks)
        out = {"plan": plan, "looks": looks}
        if len(rows) == 0:
            out.update(t=np.zeros(0), inc=np.zeros(0), rows=np.zeros(0, int), cum=np.zeros(0))
        else:
            order = np.lexsort((t, rows))
            rows, g, pd, t = rows[order], g[order], pd[order], t[order]
            logt = np.log(np.clip(1.0 - pd[:, None, None] * self.model.s[None, :, None]
                                  * self.model.A_g[g], 1e-300, None))
            starts = np.flatnonzero(np.r_[True, rows[1:] != rows[:-1]])
            cum = np.cumsum(logt, axis=0)
            grp = np.repeat(np.arange(len(starts)), np.diff(np.r_[starts, len(rows)]))
            prefix = cum - logt - (cum[starts] - logt[starts])[grp]
            inc = np.sum(self.bg.qw[rows] * np.exp(prefix) * (1.0 - np.exp(logt)), axis=(1, 2))
            o2 = np.argsort(t, kind="stable")
            out.update(t=t[o2], inc=inc[o2], rows=rows[o2], cum=np.cumsum(inc[o2]))
        self._tails[j] = out
        return out

    @staticmethod
    def cut_of(plan, limit: float) -> float:
        """Last segment boundary <= limit after stripping trailing non-productive segments
        (the execution truncation rule, MATH §4)."""
        if plan is None or not plan.segments:
            return -math.inf
        tr = plan.truncated(limit)
        return tr.t_end if tr.segments else -math.inf

    # ---------------------------------------------------------------- values
    def sequence_value(self, prefix: Looks, T_prefix: float, tail: int = 0,
                       contact_override=None, exclude_contact=None, only_contact=None) -> dict:
        """Value of `prefix` (looks of the detour pieces, times relative to c) followed by
        K[tail:] starting at T_prefix and truncated at B_rem (MATH §10.3-10.4).
        contact_override: {uid: ContactState or None} (branch posteriors; None = confirmed).
        exclude_contact: uid left out. only_contact: compute that contact's value only."""
        tl = self.tail(tail)
        cut = self.cut_of(tl["plan"], self.B_rem - T_prefix)
        Kl = tl["looks"]
        if len(Kl) and cut > -math.inf:
            m = Kl.t <= cut + 1e-9
            K_in = Looks(Kl.cell[m], Kl.g[m], Kl.pd[m], Kl.t[m] + T_prefix)
        else:
            K_in = Looks.empty()
        allk = union(prefix, K_in)
        if only_contact is not None:
            st = (contact_override or {}).get(only_contact, self.contact(only_contact))
            if st is None:
                return {"J": 0.0}
            return {"J": contact_value(st, self.model, self.cfg, self._contact_looks(st, allk))}
        rows_p, g_p, pd_p, _ = self._bg_new(prefix)
        touched = np.unique(rows_p)
        if len(touched):
            rows_k, g_k, pd_k, _ = self._bg_new(K_in)
            mk = np.isin(rows_k, touched)
            rr = np.r_[rows_p, rows_k[mk]]
            gg = np.r_[g_p, g_k[mk]]
            pp = np.r_[pd_p, pd_k[mk]]
            key = rr.astype(np.int64) * KEY_STRIDE + gg
            order = np.lexsort((-pp, key))
            key, rr, gg, pp = key[order], rr[order], gg[order], pp[order]
            first = np.r_[True, key[1:] != key[:-1]]
            bg_touched = background_value(self.bg, self.model, rr[first], gg[first], pp[first])
        else:
            bg_touched = 0.0
        if len(tl["t"]) and cut > -math.inf:
            i = int(np.searchsorted(tl["t"], cut + 1e-9, side="right"))
            bgK = float(tl["cum"][i - 1]) if i > 0 else 0.0
            if len(touched):
                mm = (tl["t"] <= cut + 1e-9) & np.isin(tl["rows"], touched)
                bgK -= float(tl["inc"][mm].sum())
        else:
            bgK = 0.0
        bg = bg_touched + bgK
        cval = 0.0
        per = {}
        for cs in self.contacts:
            if exclude_contact is not None and cs.uid == exclude_contact:
                continue
            st = cs
            if contact_override and cs.uid in contact_override:
                st = contact_override[cs.uid]
                if st is None:
                    continue
            v = contact_value(st, self.model, self.cfg, self._contact_looks(st, allk))
            per[cs.uid] = v
            cval += v
        return {"J": self.cfg.lam * bg + cval, "background": bg, "contacts": cval, "per": per,
                "cut": cut}

    def union_value(self, looks: Looks) -> float:
        """Direct (unoptimised) value of a look set — reference for tests (MATH §10.4)."""
        rows, g, pd, _ = self._bg_new(looks)
        key = rows.astype(np.int64) * KEY_STRIDE + g
        order = np.lexsort((-pd, key))
        key, rows, g, pd = key[order], rows[order], g[order], pd[order]
        first = np.r_[True, key[1:] != key[:-1]] if len(key) else np.zeros(0, bool)
        bg = background_value(self.bg, self.model, rows[first], g[first], pd[first])
        cval = sum(contact_value(cs, self.model, self.cfg, self._contact_looks(cs, looks))
                   for cs in self.contacts)
        return self.cfg.lam * bg + cval

    def piece_looks(self, piece: Plan, offset: float) -> Looks:
        lk = plan_looks(piece, piece.t0, piece.t_end, self.snap, self.model)
        if len(lk) == 0:
            return lk
        return Looks(lk.cell, lk.g, lk.pd, lk.t - piece.t0 + offset)

    def contact(self, uid):
        for cs in self.contacts:
            if cs.uid == uid:
                return cs
        return None

    # ---------------------------------------------------------------- branches
    def branch_terms(self, uid, a1_looks: Looks):
        """(V_c1(N1), P(det), miss posterior, N1) for the revisited contact (MATH §11.4)."""
        cs = self.contact(uid)
        if cs is None:
            return 0.0, 0.0, None, {}
        n1 = self._contact_looks(cs, a1_looks)
        v1 = contact_value(cs, self.model, self.cfg, n1)
        pdet = detection_probability(cs, self.model, self.cfg, n1)
        miss = after_miss(cs, self.model, self.cfg, n1) if n1 else cs
        return v1, pdet, miss, n1
