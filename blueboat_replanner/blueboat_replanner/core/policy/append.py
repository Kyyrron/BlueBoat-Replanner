"""C3 — append-on-completion (MATH.md §13; user decision 2026-11-04).

Initial plan: the C2 plan. One decision at the end of pass 1's last leg: one revisit leg per
candidate contact (state 0), perpendicular to its first detection, nearest-neighbour order by
route length, admitted while the budget rule holds (including the revisit cap), then the rejoin
to pass 2's first leg and pass 2, truncated at the budget. One request; deterministic given the
grid. If the decision is missed, the run simply continues as C2.
"""

from __future__ import annotations

import math

from ..candidates import revisit_legs
from ..plan import Plan, Segment
from ..routing import connect, route_length
from .base import CONTINUE, REPLACE, Decision, Policy


def last_pass1_line(plan) -> int | None:
    idx = None
    for i, s in enumerate(plan.segments):
        if s.kind == "line" and s.info.get("pass_no") == 1:
            idx = i
    return idx


class AppendOnCompletionPolicy(Policy):
    name = "C3"

    def initial_plan(self, coverage):
        return coverage.c2

    def decision_points(self, plan) -> list:
        i = last_pass1_line(plan)
        return [] if i is None else [(plan.times[i][1], i)]

    def on_epoch(self, ctx) -> Decision:
        cfg, U = ctx.cfg, ctx.cfg.speed_mps
        if ctx.belief is None:
            return Decision(CONTINUE, ctx.c, info={"reason": "stale_belief"})
        belief = ctx.belief
        pose_c = ctx.plan.pose_at(ctx.c)
        k_idx = ctx.plan.next_line(ctx.dp_index)
        k_start = ctx.plan.segments[k_idx].start_pose() if k_idx is not None else None
        if k_start is not None:
            d_cont = connect(pose_c, k_start, ctx.area, cfg)
            t_cont = route_length(d_cont) / U
        else:
            t_cont = 0.0
        elapsed = ctx.c - ctx.budget.t_start
        cap = ctx.budget.revisit_cap(cfg.rho_rev) - ctx.budget.revisit_used
        theta1 = getattr(ctx.coverage, "theta1", 0.0) if ctx.coverage is not None else 0.0

        remaining = sorted((c for c in belief.contacts if c.state == 0), key=lambda c: c.uid)
        scored = []
        chain = []             # segments of the inserted detours
        chain_len = 0.0
        pose = pose_c
        admitted = []
        while remaining:
            best = None
            for c in remaining:
                if c.first_sector != 255:
                    b = belief.sector_centre(c.first_sector) + math.pi / 2.0
                    fb = False
                else:
                    b = theta1          # perpendicular to the pass-1 look bearings theta1 ± pi/2
                    fb = True
                depth = belief.depth_near(c.x, c.y, cfg.revisit_depth_radius_m)
                if depth is None:
                    depth = cfg.planning_depth_m
                legs = revisit_legs(c, b, cfg, depth, ctx.area)
                opt = None
                for leg in legs:
                    tr = connect(pose, leg.start_pose, ctx.area, cfg)
                    if tr is None:
                        continue
                    L = route_length(tr) + leg.body.length
                    key = (round(L, 9), round(leg.bearing, 12), -leg.side)
                    if opt is None or key < opt[0]:
                        opt = (key, leg, tr)
                if opt is None:
                    scored.append({"uid": c.uid, "admitted": False, "reason": "no_feasible_leg",
                                   "fallback_bearing": fb})
                    continue
                nn_key = (round(route_length(opt[2]), 9), c.uid)
                if best is None or nn_key < best[0]:
                    best = (nn_key, c, opt[1], opt[2], fb)
            if best is None:
                break
            _, c, leg, tr, fb = best
            remaining = [r for r in remaining if r.uid != c.uid]
            rejoin = connect(leg.end_pose, k_start, ctx.area, cfg) if k_start is not None else []
            if rejoin is None:
                scored.append({"uid": c.uid, "admitted": False, "reason": "no_rejoin"})
                continue
            new_len = chain_len + route_length(tr) + leg.body.length
            t_d = (new_len + route_length(rejoin)) / U
            dT = t_d - t_cont
            entry = {"uid": c.uid, "bearing_deg": math.degrees(leg.bearing), "side": leg.side,
                     "r_star": leg.r_star, "transit_m": route_length(tr),
                     "body_m": leg.body.length, "T_detour": t_d, "dT": dT,
                     "fallback_bearing": fb, "r_star_adjusted": leg.r_star_adjusted}
            why = ctx.budget.admission(elapsed, t_d, dT, cap)
            if why:
                entry.update(admitted=False, reason=why)
                scored.append(entry)
                continue
            entry.update(admitted=True, reason="")
            scored.append(entry)
            if tr:
                chain.append(Segment.make("transit", tr, uid=c.uid))
            chain.append(Segment.make("body", [leg.body], uid=c.uid,
                                      bearing=round(leg.bearing, 12), side=leg.side))
            chain_len = new_len
            pose = leg.end_pose
            admitted.append((c.uid, dT))
        if not admitted:
            return Decision(CONTINUE, ctx.c, scored=tuple(scored),
                            info={"reason": "no_admissible_revisit"})
        tail = list(chain)
        rejoin_len = 0.0
        if k_start is not None:
            rejoin = connect(pose, k_start, ctx.area, cfg) or []
            rejoin_len = route_length(rejoin)
            if rejoin:
                tail.append(Segment.make("rejoin", rejoin))
            tail.extend(ctx.plan.segments[k_idx:])
        plan = Plan(tail, ctx.c, U).truncated(ctx.budget.t_limit)
        dT_total = (chain_len + rejoin_len) / U - t_cont
        return Decision(REPLACE, ctx.c, tail=plan,
                        rules_fired=tuple({"rule": "append_on_completion", "uid": u}
                                          for u, _ in admitted),
                        scored=tuple(scored),
                        info={"admitted": [u for u, _ in admitted], "dT_revisit": dT_total,
                              "t_cont": t_cont})
