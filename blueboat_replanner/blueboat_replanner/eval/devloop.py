"""ROS-free development loop (brief §10). DEVELOPMENT EVIDENCE ONLY.

world -> pings along the executed path -> BlueBoat-Belief's core (`eval/belief_real.py`)
-> the node's executor (`ros/executor.py`: trigger, policy, pre-send checks, request, splice,
budget) -> offline Control stub -> executed path.

- The belief map is Belief's own core, driven through its documented offline interface.
- The detector is emulated (`eval/emulator.py`): one draw per look at a world object.
- The boat flies the accepted path exactly at nominal speed (tau = time).
- Every condition goes through the same execution path: the executor that runs in the node.

Results are "real-Belief dev loop, preliminary": offline, with assumed PD, rho and aspect
windows, an emulated detector and perfect tracking. They are never E4 (synthesis §6.2 ⚠: the
world's detection model is a placeholder, not fitted from Gazebo).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from ..core.config import Config
from ..core.evidence import validate_grid
from ..core.geo import local_en_to_latlon
from ..core.looks import plan_poses_k
from ..core.model import folded_count
from ..core.mission_plan import setup_mission
from ..core.spacing import r_geo
from ..io.mission import Mission
from ..ros.adapters import grid_to_dict
from ..ros.executor import Executor
from .belief_real import T0, EmDet, RealBelief
from .control_stub import ControlStub, StubClient
from .emulator import LookTracker, TurnRate, reported_class
from .world import Calibrator, draw_detection, make_world, world_pd

DATUM = (34.5, 133.5)
LABEL = "real-Belief dev loop, preliminary"
PING_HZ = 10.0          # emulated ping rate: 0.05 m of track at 0.5 m/s (the sonar pings faster)
CLASS_MODES = ("none", "true")


@dataclass
class Scenario:
    width_m: float = 100.0
    height_m: float = 60.0
    depth_m: float = 2.0
    hold: tuple = (12.0, 12.0, 0.0)
    world: dict = field(default_factory=dict)

    def polygon_local(self):
        return [(0.0, 0.0), (self.width_m, 0.0), (self.width_m, self.height_m),
                (0.0, self.height_m)]

    def areas_latlon(self):
        return [[local_en_to_latlon(x, y, *DATUM) for x, y in self.polygon_local()]]


def rand_table(seed: int, G: int, n: int = 64) -> list:
    """C4_rand's pre-drawn bearing table: n folded sectors in [0, G), G from Belief's grid."""
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), 3]))
    return [int(v) for v in rng.integers(0, G, n)]


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


def contacts_of(grid: dict) -> list:
    """The contacts of a `belief_grid/0` dict as (x, y, state) records (scoring only)."""
    return [SimpleNamespace(x=float(c["x"]), y=float(c["y"]), state=int(c["state"]),
                            uid=int(c["uid"])) for c in grid["contacts"]]


def metrics_now(contacts, world, kappa: float) -> dict:
    conf_true, false_conf = match(contacts, world, confirmed_only=True)
    det_true, _ = match(contacts, world, confirmed_only=False)
    return {"confirmed_true": len(conf_true), "false_confirmations": false_conf,
            "score": len(conf_true) - kappa * false_conf, "recall_detected": len(det_true),
            "confirmed_ids": sorted(conf_true), "n_contacts": len(contacts)}


def run_mission(seed: int, condition: str, scenario: Scenario | None = None,
                overrides: dict | None = None, planner_overrides: dict | None = None,
                calibrator: Calibrator | None = None, budget_fraction: float = 1.0,
                step_s: float = 1.0, miss_credit: str = "records", class_info: str = "none",
                base_rate: float = 0.5, belief_overrides: dict | None = None,
                fractions=(0.25, 0.5, 0.75, 1.0), rand_seed: int | None = None) -> dict:
    """One mission on one world, executed by `ros/executor.Executor` against the Control stub.

    miss_credit: how Belief credits a miss ("records": as live; "ungated": at once).
    class_info:  what class the emulated detector reports ("none" / "true",
                 `eval/emulator.reported_class`).
    base_rate:   the fraction of real targets among the calibration detections; given to
                 Belief with the calibration flag when `calibrator` is a fitted one.
    rand_seed:   C4_rand: the seed of its bearing table, drawn over the grid's folded sectors.

    The loop's clock is the path time (the boat flies the accepted path exactly). Each step it
    feeds the executor Control's progress and the odometry (the mirror pose), and Belief's grid
    when the executor will read it (`Executor.decision_due`), ticks it once, then flies the
    step. A policy computes in zero
    path time here (the world stands still meanwhile), so the executor's compute-time ageing is
    0; the wall time of each deciding tick is measured outside it (DESIGN §5.2, DEVLOOP §2).
    """
    if class_info not in CLASS_MODES:
        raise ValueError(f"class_info {class_info!r} not in {CLASS_MODES}")
    sc = scenario or Scenario()
    # perfect tracking: no lag between the path parameter and the boat (MATH §11.3)
    base = {"planning_depth_m": sc.depth_m, "tracking_lag_m": 0.0}
    base.update(overrides or {})
    cfg = Config.from_dict(base)
    calibrator = calibrator or Calibrator()
    calibrated = calibrator.fitted_on != "identity"
    hold = sc.hold
    # the world needs the area before Belief and the executor exist: the same set-up as the
    # executor's own (same configuration, datum and hold), used here for the area only
    world_area = setup_mission(cfg, condition, sc.areas_latlon(), DATUM[0], DATUM[1], hold,
                               t0=0.0).area
    world = make_world(seed, world_area, sc.world)
    names = sorted(world.params["classes"])
    rb = RealBelief(0.0, 0.0, sc.width_m, sc.height_m, DATUM, sc.depth_m, cfg, names,
                    mode=miss_credit, calibrated=calibrated,
                    base_rate=base_rate if calibrated else 0.5,
                    det_sigma_m=world.params["det_sigma_m"], overrides=belief_overrides)
    g0 = validate_grid(grid_to_dict(rb.grid()))
    tracker = LookTracker(world.objects, g0.origin_x, g0.origin_y, g0.resolution, g0.width,
                          g0.height, g0.sector_count, g0.sector_origin_deg, rb.offsets,
                          rb.lever_arm_m, rb.look_timeout_s, TurnRate(**rb.turn_params()))
    G = folded_count(g0.sector_count)
    if rand_seed is not None:
        cfg = cfg.with_overrides({"c4_rand": {"bearing_table": rand_table(rand_seed, G)}})
    pcfg = cfg.with_overrides(planner_overrides) if planner_overrides else None
    U = cfg.speed_mps
    h = max(0.0, sc.depth_m - cfg.sensor_depth_m)
    rgeo = r_geo(cfg.range_m, h)
    ping_ds = U / PING_HZ

    # ---- the executor, against the stub, on the loop's clock
    stub = ControlStub(hold)
    clk = {"tau": 0.0}
    ex = Executor(cfg, StubClient(stub, lambda: (clk["tau"], clk["tau"])),
                  timer=lambda: clk["tau"])
    mission = Mission(f"devloop-{seed}", condition,
                      tuple(tuple(p) for p in sc.areas_latlon()), cfg, {},
                      planner_config=pcfg, budget_fraction=budget_fraction)
    ok, why = ex.start(mission, 0.0)
    if not ok:
        raise RuntimeError(f"executor start refused: {why}")
    dec_times = []
    lags = []

    def step_inputs(tau):
        clk["tau"] = tau
        ex.on_progress(stub.progress(tau, ros_time=T0 + tau), tau)
        pose = stub.mirror.pose(tau) if stub.mirror else hold
        ex.on_odom(float(pose[0]), float(pose[1]), float(pose[2]), tau)
        # Belief's grid where the executor reads one (the start, a decision): the grid it
        # decides on is Belief's state at tau, as fresh as possible. Feeding it every step costs
        # a third of the loop's time and changes nothing the executor does in between
        if ex.state == "waiting" or ex.decision_due():
            ex.on_grid(grid_to_dict(rb.grid()), tau)

    def tick(tau):
        n0 = len(ex.records)
        w0 = time.perf_counter()
        ex.tick(tau)
        if len(ex.records) > n0:
            dec_times.append(time.perf_counter() - w0)
            tau_h, src = ex.last_horizon
            if src == "stamp" and tau_h is not None:
                lags.append(tau - tau_h)
        if ex.state == "failed":
            raise RuntimeError(f"executor failed at tau={tau:.2f}: {ex.reason}")

    step_inputs(0.0)
    tick(0.0)                          # the first path (same for every condition)
    if ex.state != "running":
        raise RuntimeError(f"first path not accepted: {ex.state} {ex.reason}")
    if not np.array_equal(ex.setup.area.vertices, world_area.vertices):
        raise RuntimeError("the executor's area differs from the world's")
    budget = ex.setup.budget
    B_full = ex.setup.coverage.c2.duration
    tau = 0.0
    marks = sorted(f * budget.B for f in fractions)
    anytime = []
    raw_scores = []
    draw_times = {}                    # oid -> time of its last draw (close draws are reported)
    close_draws = 0
    last_ping_t = -math.inf

    while tau < ex.plan.t_end - 1e-9:
        if tau > 0.0:
            step_inputs(tau)
        tick(tau)                      # decision trigger, pre-send checks, splice (DESIGN §5.2)
        plan = ex.plan
        t_end = plan.t_end
    # ---- fly one step: pings, looks at world objects, emulated detections, Belief
        t_next = min(tau + step_s, t_end)
        pt, px, py, pp, _ = plan_poses_k(plan, tau, t_next, ping_ds)
        keep = np.zeros(len(pt), dtype=bool)
        for i in range(len(pt)):
            if pt[i] > last_ping_t + 1e-9:
                keep[i] = True
                last_ping_t = float(pt[i])
        pt, px, py, pp = pt[keep], px[keep], py[keep], pp[keep]
        dets = []
        for i in range(len(pt)):
            for oi, hit in tracker.observe(float(pt[i]), float(px[i]), float(py[i]),
                                           float(pp[i])):
                o = world.objects[oi]
                pd = world_pd(world, o, hit.r, hit.bearing, rgeo)
                if pd <= 0.0:
                    continue                       # out of the world's band: no opportunity
                world.aspects.setdefault(o.oid, set()).add(hit.look.sector % G)
                if hit.look.drawn:
                    continue
                hit.look.drawn = True              # one draw per look
                if o.oid in draw_times and hit.t - draw_times[o.oid] <= rb.look_timeout_s:
                    close_draws += 1               # two looks in one sweep (reported)
                draw_times[o.oid] = hit.t
                ok_d, dx, dy, q, raw = draw_detection(world, o, hit.r, hit.bearing, rgeo,
                                                      calibrator)
                raw_scores.append((raw, o.kind == "target", ok_d))
                if ok_d:
                    dets.append(EmDet(t=hit.t, x=dx, y=dy, score=q,
                                      class_name=reported_class(world, o, class_info),
                                      psi=float(pp[i]), uid=f"{o.oid}-{world.looks[o.oid]}"))
        rb.feed(pt, px, py, pp, dets)
        tau = t_next
        while marks and tau >= marks[0] - 1e-9:
            m = metrics_now(contacts_of(rb.grid()), world, cfg.kappa)
            m["t"] = marks.pop(0)
            anytime.append(m)
    step_inputs(tau)                   # the end: Control reports end_reached
    tick(tau)
    rb.finish()
    health = rb.run_health()
    final_grid = rb.grid()
    final_contacts = contacts_of(final_grid)
    while marks:                       # plan ended before the budget (C1, a mission stop)
        m = metrics_now(final_contacts, world, cfg.kappa)
        m["t"] = marks.pop(0)
        anytime.append(m)
    final = metrics_now(final_contacts, world, cfg.kappa)
    # self-check on published output only: the emulator's looks at each object against
    # Belief's `aspect_counts` at the object's cell
    ac = np.asarray(final_grid["aspect_counts"]).reshape(-1, g0.sector_count)
    mism = int(np.sum(tracker.counts[tracker.in_grid] !=
                      ac[tracker.cell[tracker.in_grid]].astype(np.int64)))
    snap_f = validate_grid(grid_to_dict(final_grid))
    aspects = [len(world.aspects.get(o.oid, ())) for o in world.targets]
    plan = ex.plan
    labels_all = sorted({lab for r in ex.records for lab in r["labels"]})
    refusals = [r["answer"]["reason"] for r in ex.requests
                if "answer" in r and not r["answer"]["accepted"]]
    return {"seed": seed, "condition": condition, "budget_fraction": budget_fraction,
            "B": budget.B, "B_full": B_full, "duration": plan.t_end - plan.t0,
            "final": final, "anytime": anytime, "requests": ex.counts["requests"],
            "accepted": ex.counts["accepted"], "refusals": refusals,
            "missed_lead": sum(1 for r in ex.records if r["epoch"].get("outcome") ==
                               "missed_lead"),
            "decisions": ex.counts["decisions"],
            "mission_stop": ex.stop_id is not None and
            ex.stop_reason.startswith("mission_stop"),
            "revisit_used": budget.revisit_used,
            "n_rev": dict(ex.policy_state.get("n_rev", {})),
            "decision_time_mean_s": float(np.mean(dec_times)) if dec_times else 0.0,
            "decision_time_max_s": float(np.max(dec_times)) if dec_times else 0.0,
            "target_aspects_mean": float(np.mean(aspects)) if aspects else 0.0,
            "raw_scores": raw_scores, "label": LABEL,
            "labels": [lab for lab in labels_all if lab in ("aspect-inert", "score-conditional")],
            "record_labels": labels_all,
            "end_state": ex.state, "end_reason": ex.reason,
            "compute_ageing_s": max((r["timing"]["compute_s"] or 0.0 for r in ex.records),
                                    default=0.0),
            "belief": {**rb.settings(), "class_info": class_info,
                       "model": dict(snap_f.model), "pings": rb.n_pings,
                       "images": rb.n_images, "detections": rb.n_dets,
                       "horizon_lag_mean_s": float(np.mean(lags)) if lags else None,
                       "horizon_lag_max_s": float(np.max(lags)) if lags else None,
                       "health": health,
                       "look_mismatch": mism,
                       "looks_tracked": int(tracker.counts.sum()),
                       "draws_within_look_timeout": close_draws,
                       "searched_cell_fraction": float(np.mean(
                           snap_f.searched.any(axis=2)[np.isfinite(snap_f.coverage)]))},
            "stub_log": [{k: v for k, v in e.items() if k != "message"} for e in stub.log]}
