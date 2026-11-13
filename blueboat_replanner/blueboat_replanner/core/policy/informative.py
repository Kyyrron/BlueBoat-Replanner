"""C4 family — aspect-aware informative replanner (MATH.md §10-§12).

At each decision point (end of a coverage leg or a revisit body) the policy compares
CONTINUE with REVISIT(contact, folded sector) detours by the value of the whole executed tail
over the remaining budget, with a two-step lookahead whose second step is chosen by the same
combined value. Only the first action is sent. Variants (configuration, not code):
  C4        default
  C4_iso    aspect-agnostic planner (A = 1)
  C4_rand   bearings from the pre-drawn table (eval/), C4's own choice of whether and whom
  C4_depth1 one-step
  C4_degraded  degraded planner block (set by eval/ through configuration)
  C4_live   may_interrupt (not part of E4; decision points as C4 in this version)
RECOVER (out-of-order coverage legs) is not implemented in this version: K already contains
pass 2; see MATH §10.2.
"""

from __future__ import annotations

import math


from ..candidates import revisit_legs
from ..dubins import dubins_candidates
from ..looks import Looks, plan_looks, union
from ..plan import Plan, Segment
from ..routing import connect
from ..sequence import Evaluator
from ..value import contact_value
from .base import CONTINUE, REPLACE, STOP, Decision, Policy, horizon_of

VARIANTS = ("C4", "C4_iso", "C4_rand", "C4_depth1", "C4_degraded", "C4_live")


def informative_policy(condition: str) -> "InformativePolicy":
    if condition not in VARIANTS:
        raise ValueError(f"unknown C4 variant {condition!r}")
    return InformativePolicy(condition)


class InformativePolicy(Policy):
    def __init__(self, variant: str = "C4"):
        self.name = variant
        self.variant = variant
        self.may_interrupt = variant == "C4_live"

    def planner_cfg(self, cfg):
        over = {}
        if self.variant == "C4_iso":
            over["aspect_agnostic"] = True
        if self.variant == "C4_depth1":
            over["lookahead_depth"] = 1
        return cfg.with_overrides(over) if over else cfg

    def initial_plan(self, coverage):
        return coverage.c2

    def decision_points(self, plan) -> list:
        return [(plan.times[i][1], i) for i, s in enumerate(plan.segments)
                if s.kind in ("line", "body")]

    # ------------------------------------------------------------------ helpers
    def _detour(self, ev, cfg, area, pose, cs, g, k_start, snapshot):
        """Cheapest feasible REVISIT(c, g) detour from `pose`: (transit, body leg, rejoin)."""
        depth = snapshot.depth_near(cs.x, cs.y, cfg.revisit_depth_radius_m)
        depth = cfg.planning_depth_m if depth is None else depth

        class _C:  # contact geometry for revisit_legs
            pass
        cc = _C()
        cc.x, cc.y, cc.cov, cc.uid = cs.x, cs.y, cs.cov, cs.uid
        legs = revisit_legs(cc, ev.model.beta(g), cfg, depth, area)
        ranked = []
        for leg in legs:
            c = dubins_candidates(tuple(pose), leg.start_pose, cfg.r_turn_m)
            L = (c[0][0] if c else math.inf) + leg.body.length
            ranked.append((round(L, 9), round(leg.bearing, 12), -leg.side, leg))
        ranked.sort(key=lambda r: r[:3])
        for _, _, _, leg in ranked:
            tr = connect(tuple(pose), leg.start_pose, area, cfg)
            if tr is None:
                continue
            rj = connect(leg.end_pose, k_start, area, cfg) if k_start is not None else []
            if rj is None:
                continue
            return tr, leg, rj
        return None

    @staticmethod
    def _plan(segs, speed):
        return Plan([s for s in segs if s is not None], 0.0, speed)

    def _revisit_candidate(self, ev, cfg, area, pose, cs, g, k_start, snapshot, offset):
        d = self._detour(ev, cfg, area, pose, cs, g, k_start, snapshot)
        if d is None:
            return None
        tr, leg, rj = d
        segs = []
        if tr:
            segs.append(Segment.make("transit", tr, uid=cs.uid))
        segs.append(Segment.make("body", [leg.body], uid=cs.uid, bearing=round(leg.bearing, 12),
                                 side=leg.side, g=int(g)))
        a_plan = self._plan(segs, cfg.speed_mps)
        r_plan = self._plan([Segment.make("rejoin", rj)] if rj else [], cfg.speed_mps)
        a_looks = ev.piece_looks(a_plan, offset) if a_plan.segments else Looks.empty()
        r_looks = (ev.piece_looks(r_plan, offset + a_plan.duration) if r_plan.segments
                   else Looks.empty())
        return {"uid": cs.uid, "g": int(g), "segs": segs, "rejoin": r_plan.segments,
                "T_a": a_plan.duration, "T_r": r_plan.duration,
                "T": a_plan.duration + r_plan.duration, "a_looks": a_looks,
                "r_looks": r_looks, "bearing": leg.bearing, "side": leg.side,
                "end_pose": leg.end_pose}

    # ------------------------------------------------------------------ epoch
    def on_epoch(self, ctx) -> Decision:
        cfg = self.planner_cfg(ctx.cfg)
        U = cfg.speed_mps
        if ctx.belief is None:
            return Decision(CONTINUE, ctx.c, info={"reason": "stale_belief"})
        snap, plan, area = ctx.belief, ctx.plan, ctx.area
        c = ctx.c
        B_rem = ctx.budget.t_limit - c
        k_idx = plan.next_line(ctx.dp_index)
        K = Plan(plan.segments[k_idx:], 0.0, U) if k_idx is not None else None
        k_start = plan.segments[k_idx].start_pose() if k_idx is not None else None
        cont_plan = Plan(plan.segments[ctx.dp_index + 1:k_idx] if k_idx is not None else [],
                         0.0, U)
        T_cont = cont_plan.duration
        tau = ctx.state.get("tau", c)
        # ---- pending looks: what the grid does not hold yet (MATH §11.3)
        tau_h, h_src = horizon_of(ctx, cfg, tau)
        guard = cfg.contact_guard_m / U
        if c > plan.t0:
            m0 = _model_for(cfg, snap)
            pend = plan_looks(plan, max(plan.t0, tau_h), c, snap, m0)
            pend_c = plan_looks(plan, max(plan.t0, tau_h - guard), c, snap, m0)
        else:
            pend = pend_c = Looks.empty()
        ev = Evaluator(cfg, snap, area, K, pend, B_rem, pend_c)
        cont_looks = ev.piece_looks(cont_plan, 0.0) if cont_plan.segments else Looks.empty()
        J_cont = ev.sequence_value(cont_looks, T_cont)["J"]
        state = ctx.state
        nrev = state.setdefault("n_rev", {})
        hist = state.setdefault("rev_hist", {})
        rules = []
        resolved, abandoned = set(), set()
        # ---- abandonment bookkeeping (MATH §12): p_after once the horizon covers the body
        for uid, entries in hist.items():
            cs = ev.contact(uid)
            for e in entries:
                if e.get("p_after") is None and cs is not None and \
                        tau_h - guard >= e["t_end"]:
                    e["p_after"] = cs.p_tilde
        # ---- candidate contacts (resolution, abandonment, pending)
        cands = []
        elapsed = c - ctx.budget.t_start
        cap_left = ctx.budget.revisit_cap(cfg.rho_rev) - ctx.budget.revisit_used
        for cs in ev.contacts:
            if cs.confirmed:
                rules.append({"rule": "resolution", "uid": cs.uid, "why": "confirmed"})
                resolved.add(cs.uid)
                continue
            if cs.p_tilde < cfg.p_res:
                rules.append({"rule": "resolution", "uid": cs.uid, "why": "p_tilde<p_res"})
                resolved.add(cs.uid)
                continue
            if nrev.get(cs.uid, 0) >= cfg.n_max:
                rules.append({"rule": "abandonment", "uid": cs.uid, "why": "n_max"})
                abandoned.add(cs.uid)
                continue
            h = [e for e in hist.get(cs.uid, []) if e.get("p_after") is not None]
            if len(h) >= 2 and all(abs(e["p_after"] - e["p_before"]) < cfg.delta for e in h[-2:]):
                rules.append({"rule": "abandonment", "uid": cs.uid, "why": "no_progress"})
                abandoned.add(cs.uid)
                continue
            if ev.contact_pending.get(cs.uid):
                continue
            avail = [g for g in range(ev.model.G) if g not in cs.seen_g]
            if not avail:
                rules.append({"rule": "resolution", "uid": cs.uid, "why": "all_sectors_seen"})
                resolved.add(cs.uid)
                continue
            # prefilter: the k best sectors by the body look alone (MATH §11.2, top-K proxy)
            r_star = cfg.r_star_m
            pdr = float(ev.model.pd_r(r_star))
            prox = sorted(((contact_value(cs, ev.model, cfg, {g: pdr}), g) for g in avail),
                          key=lambda t: (-t[0], t[1]))
            if prox[0][0] <= 0:
                rules.append({"rule": "resolution", "uid": cs.uid, "why": "no_positive_bearing"})
                resolved.add(cs.uid)
                continue
            cands += [(cs, g) for v, g in prox[:cfg.sector_prefilter_k] if v > 0]
        pose_c = plan.pose_at(c)
        built = []
        for cs, g in cands:
            cand = self._revisit_candidate(ev, cfg, area, pose_c, cs, g, k_start, snap, 0.0)
            if cand is None:
                continue
            dT = cand["T"] - T_cont
            if ctx.budget.admission(elapsed, cand["T"], dT, cap_left):
                continue
            res = ev.sequence_value(union(cand["a_looks"], cand["r_looks"]), cand["T"])
            cand.update(J=res["J"], dT=dT)
            built.append(cand)
        built.sort(key=lambda d: (-d["J"], d["uid"], d["g"]))
        scored = [{"id": "CONT", "kind": "continue", "J": J_cont, "T_detour": T_cont, "dT": 0.0}]
        top = built[:cfg.k1]
        # ---- two-step lookahead (MATH §11.4)
        if cfg.lookahead_depth >= 2:
            J2_cont = self._j2_continue(ev, cfg, area, plan, ctx, K, k_idx, cont_plan,
                                        cont_looks, T_cont, snap, cap_left)
            for cand in top:
                cand["J2"] = self._j2_revisit(ev, cfg, area, cand, k_start, snap, ctx,
                                              cap_left - cand["dT"])
        else:
            J2_cont = J_cont
            for cand in top:
                cand["J2"] = cand["J"]
        scored[0]["J2"] = J2_cont
        for cand in built:
            scored.append({"id": f"REV:{cand['uid']}:g{cand['g']}", "kind": "revisit",
                           "uid": cand["uid"], "folded_sector": cand["g"],
                           "bearing_deg": math.degrees(cand["bearing"]),
                           "side": "port" if cand["side"] > 0 else "starboard",
                           "T_detour": cand["T"], "dT": cand["dT"], "J": cand["J"],
                           "J2": cand.get("J2")})
        best = max(top, key=lambda d: (d["J2"], -d["uid"], -d["g"]), default=None)
        info = {"J2_continue": J2_cont, "epsilon_J": cfg.epsilon_j,
                "calibration_clamped": ev.bg.clamped, "pending_keys": int(len(pend)),
                "n_candidates": len(built),
                "calibration": ev.bg.info(),
                "pending": {"tau_horizon": tau_h if math.isfinite(tau_h) else None,
                            "source": h_src, "keys": int(len(pend)),
                            "contact_keys": int(len(pend_c)), "contact_guard_s": guard},
                "unmapped_classes": list(ev.model.unmapped),
                "aspect_inert": not bool(ev.model.mapped.any()),
                "contacts": [{"uid": cs.uid, "p_tilde": cs.p_tilde,
                              "q0": None if math.isnan(cs.q0) else cs.q0,
                              "state": 1 if cs.confirmed else 0,
                              "n_rev": nrev.get(cs.uid, 0),
                              "resolved": cs.uid in resolved, "abandoned": cs.uid in abandoned,
                              "pending": bool(ev.contact_pending.get(cs.uid)),
                              "detected_g": sorted(cs.detected_g),
                              "missed_g": [[g, round(v, 6)] for g, v in cs.missed],
                              "fallbacks": list(cs.fallbacks)} for cs in ev.contacts]}
        # ---- mission stop (MATH §12): the best action's value rate is below the floor
        J_best = J2_cont if best is None else max(J2_cont, best["J2"])
        if cfg.rho_floor > 0 and B_rem > 0 and J_best / B_rem < cfg.rho_floor:
            rules.append({"rule": "mission_stop", "why": "rho_floor",
                          "rate": J_best / B_rem, "rho_floor": cfg.rho_floor})
            info["chosen"] = "STOP"
            return Decision(STOP, c, rules_fired=tuple(rules), scored=tuple(scored), info=info)
        if best is None or best["J2"] < J2_cont + cfg.epsilon_j:
            return Decision(CONTINUE, c, rules_fired=tuple(rules), scored=tuple(scored),
                            info=info)
        # ---- C4_rand: same contact, pre-drawn bearing (DESIGN decision 7)
        if self.variant == "C4_rand" and ctx.cfg.bearing_table:
            n = state.get("rand_n", 0)
            cs = ev.contact(best["uid"])
            avail = [g for g in range(ev.model.G) if g not in cs.seen_g]
            chosen = None
            for k in range(len(avail)):
                g = avail[(ctx.cfg.bearing_table[n % len(ctx.cfg.bearing_table)] + k) %
                          len(avail)]
                cand = self._revisit_candidate(ev, cfg, area, pose_c, cs, g, k_start, snap, 0.0)
                if cand is None:
                    continue
                dT = cand["T"] - T_cont
                if ctx.budget.admission(elapsed, cand["T"], dT, cap_left):
                    continue
                cand.update(dT=dT, J=None, J2=None)
                chosen = cand
                break
            info["rand_index"] = n
            if chosen is not None:
                best = chosen
        tail = list(best["segs"]) + list(best["rejoin"])
        if k_idx is not None:
            tail += plan.segments[k_idx:]
        tail_plan = Plan(tail, c, U).truncated(ctx.budget.t_limit)
        body_end = c + best["T_a"]
        info.update(chosen=f"REV:{best['uid']}:g{best['g']}", dT=best["dT"],
                    revisit={"uid": best["uid"], "g": best["g"], "t_end": body_end,
                             "p_before": ev.contact(best["uid"]).p_tilde})
        return Decision(REPLACE, c, tail=tail_plan, rules_fired=tuple(rules),
                        scored=tuple(scored), info=info)

    # ------------------------------------------------------------------ lookahead
    def _a2_candidates(self, ev, cfg, area, pose, k_start, snap, offset, cap_left, exclude=None,
                       elapsed_extra=0.0, budget=None):
        """Top-K2 second actions from `pose` by a cheap proxy (contact value of the body look per
        unit unconstrained time), then built exactly (MATH §11.4)."""
        pdr = float(ev.model.pd_r(cfg.r_star_m))
        prox = []
        for cs in ev.contacts:
            if cs.confirmed or cs.p_tilde < cfg.p_res or ev.contact_pending.get(cs.uid):
                continue
            for g in range(ev.model.G):
                if g in cs.seen_g or (exclude and (cs.uid, g) in exclude):
                    continue
                v = contact_value(cs, ev.model, cfg, {g: pdr})
                if v <= 0:
                    continue
                d = math.hypot(cs.x - pose[0], cs.y - pose[1]) / cfg.speed_mps + 1.0
                prox.append((v / d, cs, g))
        prox.sort(key=lambda t: (-t[0], t[1].uid, t[2]))
        out = []
        for _, cs, g in prox[:cfg.k2]:
            cand = self._revisit_candidate(ev, cfg, area, pose, cs, g, k_start, snap, offset)
            if cand is None:
                continue
            out.append(cand)
        return out

    def _j2_revisit(self, ev, cfg, area, a1, k_start, snap, ctx, cap_left):
        uid = a1["uid"]
        v1, pdet, miss, _ = ev.branch_terms(uid, a1["a_looks"])
        T1 = a1["T_a"]
        elapsed = ctx.c - ctx.budget.t_start
        options = [(union(a1["a_looks"], a1["r_looks"]), a1["T"])]          # a2 = CONTINUE
        for a2 in self._a2_candidates(ev, cfg, area, a1["end_pose"], k_start, snap, T1,
                                      cap_left, exclude={(uid, a1["g"])}):
            T = T1 + a2["T"]
            dT2 = a2["T"] - a1["T_r"]
            if ctx.budget.admission(elapsed, T, dT2, cap_left):
                continue
            options.append((union(a1["a_looks"], a2["a_looks"], a2["r_looks"]), T))
        best_det = best_miss = -math.inf
        for looks, T in options:
            R = ev.sequence_value(looks, T, exclude_contact=uid)["J"]
            m = ev.sequence_value(looks, T, contact_override={uid: miss},
                                  only_contact=uid)["J"] if miss is not None else 0.0
            best_det = max(best_det, R)
            best_miss = max(best_miss, R + m)
        return v1 + pdet * best_det + (1.0 - pdet) * best_miss

    def _j2_continue(self, ev, cfg, area, plan, ctx, K, k_idx, cont_plan, cont_looks, T_cont,
                     snap, cap_left):
        """CONTINUE's two-step value: next leg of K, then CONTINUE or a detour (MATH §11.4)."""
        J = ev.sequence_value(cont_looks, T_cont)["J"]
        if K is None or not K.segments:
            return J
        j_next = K.next_line(0)
        leg0 = Plan(K.segments[:1], 0.0, cfg.speed_mps)
        T0 = T_cont + leg0.duration
        leg_looks = ev.piece_looks(leg0, T_cont)
        pose = K.segments[0].end_pose()
        k2_start = K.segments[j_next].start_pose() if j_next is not None else None
        cont2 = Plan(K.segments[1:j_next] if j_next is not None else K.segments[1:], 0.0,
                     cfg.speed_mps).duration
        elapsed = ctx.c - ctx.budget.t_start
        best = J
        for a2 in self._a2_candidates(ev, cfg, area, pose, k2_start, snap, T0, cap_left):
            T = T0 + a2["T"]
            dT2 = a2["T"] - cont2
            if ctx.budget.admission(elapsed, T, dT2, cap_left):
                continue
            looks = union(cont_looks, leg_looks, a2["a_looks"], a2["r_looks"])
            val = ev.sequence_value(looks, T, tail=j_next if j_next is not None else
                                    len(K.segments))["J"]
            best = max(best, val)
        return best

    # ------------------------------------------------------------------ bookkeeping
    @staticmethod
    def on_accepted(decision: Decision, state: dict) -> None:
        """Record an accepted REVISIT (counters for abandonment, C4_rand index)."""
        rv = decision.info.get("revisit")
        if not rv:
            return
        state.setdefault("n_rev", {})
        state["n_rev"][rv["uid"]] = state["n_rev"].get(rv["uid"], 0) + 1
        state.setdefault("rev_hist", {}).setdefault(rv["uid"], []).append(
            {"t_end": rv["t_end"], "p_before": rv["p_before"], "p_after": None})
        if "rand_index" in decision.info:
            state["rand_n"] = state.get("rand_n", 0) + 1


def _model_for(cfg, snap):
    from ..model import SensorModel
    from ..spacing import altitude
    return SensorModel(cfg, snap.class_names, snap.sector_count, snap.sector_origin_deg,
                       altitude(cfg.planning_depth_m, cfg.sensor_depth_m))
