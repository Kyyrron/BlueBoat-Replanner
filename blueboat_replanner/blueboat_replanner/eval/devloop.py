"""ROS-free development loop (brief §10; prompt §B.2). DEVELOPMENT EVIDENCE ONLY.

world -> looks along the executed path -> reference belief (or the real core, in-machine)
-> replanner policy -> request -> offline Control stub -> executed path.
The boat flies the accepted path exactly at nominal speed (tau = time). Every condition goes
through the same execution path. Results are "reference-belief-conditional, preliminary" and
never E4 (synthesis §6.2 ⚠: the world's detection model is a placeholder, not fitted from
Gazebo).
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from ..core.config import Config
from ..core.evidence import validate_grid
from ..core.geo import local_en_to_latlon
from ..core.looks import plan_poses
from ..core.mission_plan import setup_mission
from ..core.plan import Plan
from ..core.policy.base import EpochContext
from ..core.spacing import r_geo
from .belief_port import ReferencePort
from .belief_reference import ReferenceBelief
from .control_stub import ControlStub
from .world import Calibrator, draw_detection, make_world

DATUM = (34.5, 133.5)
LABEL = "reference-belief-conditional, preliminary"


@dataclass
class Scenario:
    width_m: float = 100.0
    height_m: float = 60.0
    depth_m: float = 2.0
    hold: tuple = (12.0, 12.0, 0.0)
    resolution: float = 0.5
    world: dict = field(default_factory=dict)

    def polygon_local(self):
        return [(0.0, 0.0), (self.width_m, 0.0), (self.width_m, self.height_m),
                (0.0, self.height_m)]

    def areas_latlon(self):
        return [[local_en_to_latlon(x, y, *DATUM) for x, y in self.polygon_local()]]


def degraded_overrides(cfg: Config) -> dict:
    """C4_degraded: planner windows x2, PD -20 % (brief §8.1) — planner block only."""
    classes = {}
    for name, cm in cfg.classes.items():
        w = math.degrees(cm.w_rad) * 2.0
        if cm.m > 0:
            w = min(w, 180.0 / cm.m)
        classes[name] = {"m": cm.m, "w_deg": w, "a": cm.a, "s": cm.s}
    return {"pd_plateau": cfg.pd_plateau * 0.8, "classes": classes}


def match(contacts, world, gate: float = 2.0, confirmed_only=True):
    """Greedy one-to-one nearest matching of contacts to true targets within `gate` m."""
    cs = [c for c in contacts if (c.state == 1 or not confirmed_only)]
    tg = world.targets
    pairs = []
    for i, c in enumerate(cs):
        for j, t in enumerate(tg):
            d = math.hypot(c.x - t.x, c.y - t.y)
            if d <= gate:
                pairs.append((d, i, j))
    pairs.sort()
    used_c, used_t, m = set(), set(), {}
    for d, i, j in pairs:
        if i in used_c or j in used_t:
            continue
        used_c.add(i)
        used_t.add(j)
        m[i] = j
    true_ids = {tg[j].oid for j in m.values()}
    false = len(cs) - len(m)
    return true_ids, false


def metrics_now(belief: ReferenceBelief, world, kappa: float) -> dict:
    conf_true, false_conf = match(belief.contacts, world, confirmed_only=True)
    det_true, _ = match(belief.contacts, world, confirmed_only=False)
    return {"confirmed_true": len(conf_true), "false_confirmations": false_conf,
            "score": len(conf_true) - kappa * false_conf, "recall_detected": len(det_true),
            "confirmed_ids": sorted(conf_true), "n_contacts": len(belief.contacts)}


def run_mission(seed: int, condition: str, scenario: Scenario | None = None,
                overrides: dict | None = None, planner_overrides: dict | None = None,
                calibrator: Calibrator | None = None, budget_fraction: float = 1.0,
                step_s: float = 1.0, belief_backend: str = "reference",
                fractions=(0.25, 0.5, 0.75, 1.0)) -> dict:
    sc = scenario or Scenario()
    base = {"planning_depth_m": sc.depth_m}
    base.update(overrides or {})
    cfg = Config.from_dict(base)
    pcfg = cfg.with_overrides(planner_overrides) if planner_overrides else cfg
    calibrator = calibrator or Calibrator()
    hold = sc.hold
    setup = setup_mission(cfg, condition, sc.areas_latlon(), DATUM[0], DATUM[1], hold, t0=0.0)
    area, U = setup.area, cfg.speed_mps
    B_full = setup.budget.B
    budget = setup.budget
    budget.B = budget_fraction * B_full
    plan = setup.initial_plan.truncated(budget.t_limit)
    world = make_world(seed, area, sc.world)
    if belief_backend != "reference":
        raise NotImplementedError("belief_backend 'real': in-machine session (L-B7)")
    names = sorted(world.params["classes"])
    density = world.params["n_targets"] / (area.area / 1e4)
    rb = ReferenceBelief(area, sc.resolution, DATUM, names, 12, 0.0, density, sc.depth_m,
                         pd_plateau=world.params["pd_plateau"], pd_r_lo=world.params["pd_r_lo_m"],
                         pd_r_hi=world.params["pd_r_hi_m"], range_m=cfg.range_m,
                         sensor_depth_m=cfg.sensor_depth_m, f_ref=world.params["clutter_pd"])
    port = ReferencePort(rb)
    stub = ControlStub(hold)
    policy = setup.policy
    state = {}
    h = max(0.0, sc.depth_m - cfg.sensor_depth_m)
    rgeo = r_geo(cfg.range_m, h)

    # ---- first path (same for every condition)
    t, x, y, w = plan.sample(cfg.sample_ds_m, cfg.max_samples_per_request)
    ans = stub.handle("r1-0000", float("nan"), t, x, y, w, 0.0)
    if not ans["accepted"]:
        raise RuntimeError(f"first path refused: {ans}")
    n_req, n_acc, refusals, missed = 1, 1, [], 0
    decisions = []
    dps = [d for d in policy.decision_points(plan)]
    done_dp = set()
    tau = 0.0
    snaps = deque()
    pass_ids = {}
    obj_cells = {o.oid: rb.cell_of(o.x, o.y) for o in world.objects}
    drawn = set()
    marks = sorted(f * budget.B for f in fractions)
    anytime = []
    dec_times = []
    raw_scores = []

    def take_snapshot(now):
        snaps.append((now, port.snapshot(now)))
        while len(snaps) > 2 and snaps[1][0] <= now - cfg.t_lat_s:
            snaps.popleft()

    take_snapshot(0.0)
    t_end = plan.t_end
    while tau < t_end - 1e-9:
        # ---- decision trigger (DESIGN §5.2)
        for (t_dp, idx) in list(dps):
            key = (round(t_dp, 6), plan.segments[idx].kind, idx)
            if key in done_dp or t_dp <= tau:
                continue
            lead = stub.lead_min(tau)
            trig = t_dp - (lead + cfg.t_decide_s + cfg.latency_margin_s)
            if tau < trig:
                break
            done_dp.add(key)
            snap = None
            for st, sd in snaps:
                if st <= tau - cfg.t_lat_s + 1e-9:
                    snap = sd
            belief = validate_grid(snap) if snap is not None else None
            state["tau"] = tau
            ctx = EpochContext(pcfg, area, plan, t_dp, idx, budget, belief=belief,
                               coverage=setup.coverage, state=state)
            w0 = time.perf_counter()
            dec = policy.on_epoch(ctx)
            dec_times.append(time.perf_counter() - w0)
            rec = {"tau": tau, "c": t_dp, "action": dec.action,
                   "info": {k: v for k, v in dec.info.items() if k != "revisit"}}
            if dec.action == "REPLACE" and dec.tail is not None and dec.tail.segments:
                mcp = stub.progress(tau)["min_change_point"]
                if mcp is not None and t_dp < mcp + cfg.latency_margin_s:
                    missed += 1
                    rec["outcome"] = "missed_lead"
                else:
                    tt, xx, yy, ww = dec.tail.sample(cfg.sample_ds_m, cfg.max_samples_per_request)
                    rid = f"r1-{n_req:04d}"
                    n_req += 1
                    a = stub.handle(rid, t_dp, tt, xx, yy, ww, tau)
                    rec["outcome"] = a["reason"]
                    if a["accepted"]:
                        n_acc += 1
                        plan = Plan(plan.segments[:idx + 1] + dec.tail.segments, plan.t0, U)
                        budget.revisit_used += float(dec.info.get("dT", dec.info.get(
                            "dT_revisit", 0.0)))
                        if hasattr(policy, "on_accepted"):
                            policy.on_accepted(dec, state)
                        dps = policy.decision_points(plan)
                        t_end = plan.t_end
                    else:
                        refusals.append(a["reason"])
            decisions.append(rec)
            break
        # ---- fly one step
        t_next = min(tau + step_s, t_end)
        pt, px, py, pp = plan_poses(plan, tau, t_next, sc.resolution / 2.0)
        if len(pt):
            ends = np.array([b for _, b in plan.times])
            segi = np.searchsorted(ends, pt - 1e-9, side="left")
            segi = np.minimum(segi, len(plan.segments) - 1)
            pids = []
            for si in segi:
                key = (id(plan.segments[si]), round(plan.times[si][0], 6))
                pids.append(pass_ids.setdefault(key, len(pass_ids)))
            events = port.look_events(pt, px, py, pp, pids)
            # emulator: one draw per object per pass when its cell is looked at
            first_ev = {}
            for ev in events:
                first_ev.setdefault((ev[0], ev[1], ev[5]), ev)
            dets = []
            for o in world.objects:
                rc = obj_cells[o.oid]
                for pid in set(pids):
                    evk = first_ev.get((rc[0], rc[1], pid))
                    if evk is None or (o.oid, pid) in drawn:
                        continue
                    drawn.add((o.oid, pid))
                    world.aspects.setdefault(o.oid, set()).add(int(evk[2]) % 6)
                    ok, dx, dy, q, raw = draw_detection(world, o, evk[3], evk[4], rgeo, calibrator)
                    raw_scores.append((raw, o.kind == "target", ok))
                    if ok:
                        dets.append((dx, dy, q, int(evk[2]), pid))
            port.feed_detections(dets)
            port.feed_swath(events)
            port.end_passes(pids[-1])
        tau = t_next
        take_snapshot(tau)
        while marks and tau >= marks[0] - 1e-9:
            m = metrics_now(rb, world, cfg.kappa)
            m["t"] = marks.pop(0)
            anytime.append(m)
    port.end_passes(-1)
    while marks:                       # plan ended before the budget (C1)
        m = metrics_now(rb, world, cfg.kappa)
        m["t"] = marks.pop(0)
        anytime.append(m)
    final = metrics_now(rb, world, cfg.kappa)
    aspects = [len(world.aspects.get(o.oid, ())) for o in world.targets]
    return {"seed": seed, "condition": condition, "budget_fraction": budget_fraction,
            "B": budget.B, "B_full": B_full, "duration": plan.t_end - plan.t0,
            "final": final, "anytime": anytime, "requests": n_req, "accepted": n_acc,
            "refusals": refusals, "missed_lead": missed, "decisions": len(decisions),
            "revisit_used": budget.revisit_used, "n_rev": dict(state.get("n_rev", {})),
            "decision_time_mean_s": float(np.mean(dec_times)) if dec_times else 0.0,
            "decision_time_max_s": float(np.max(dec_times)) if dec_times else 0.0,
            "target_aspects_mean": float(np.mean(aspects)) if aspects else 0.0,
            "raw_scores": raw_scores, "label": LABEL,
            "stub_log": [{k: v for k, v in e.items() if k != "message"} for e in stub.log]}
