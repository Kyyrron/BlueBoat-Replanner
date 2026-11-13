"""The executor — the ONLY code that knows Control's live path update (DESIGN.md §5, §7).

ROS-free and clock-injected: the node feeds it progress, events, belief snapshots, odometry and
service answers with the arrival wall time `now`, and calls `tick(now)`. The executor sends
requests through a `client`: `client.send(request_dict, on_answer)`, where the client calls
`on_answer(answer_dict, now)` with the answer's arrival time; an optional `client.ready()` says
whether the service is there (checked before the first request). The offline gate and the dev
loop (`eval/devloop.py`) drive it against `eval.control_stub` with a fake clock.

Control facts used (verified, E-01..E-19): service `/blueboat/update_path` (request id,
change_point, frame "world", t, x, y, yaw), refusal codes, progress `blueboat_path_progress/1`
(state, tau, min_change_point, hold, active_id, end_reached, last_answer), events
`blueboat_path_update_event/1` (cleared, activated, end_reached, ...).
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from ..core.areas import MissionError
from ..core.evidence import (ContractError, geometry_signature, mission_area_check,
                             validate_grid)
from ..core.mission_plan import setup_mission
from ..core.plan import ControlPathMirror, Plan, hold_segment, transform_segments
from ..core.config import CONTROL_RULES
from ..core.policy.base import EpochContext
from ..core.validate import continuity_check, plan_in_area, request_check

STATE_CODES = ("not_ready", "no_run", "mission_owned", "operator_override")
GEOMETRY_CODES = ("lead_too_short", "discontinuous_position", "discontinuous_heading")
STATES = ("idle", "waiting", "starting", "running", "deciding", "requesting", "disarmed",
          "stop_requested", "stop_accepted", "stopped_holding", "stop_rejected", "done",
          "run_ended", "failed")
STOP_NOT_IN_EFFECT = ("STOP NOT IN EFFECT - the boat continues the previous path; use E-STOP, "
                      "manual target or run stop")
TERMINAL = ("done", "failed", "run_ended", "stopped_holding")
HIST_MAX = 3000                 # progress (ros_time, tau) pairs kept: 5 min at 10 Hz
BASE_LABELS = ("model-conditional", "planner-placeholders")


@dataclass
class Pending:
    req_id: str
    kind: str                  # first | replace | stop
    change_point: float
    sent: float
    t: list
    x: list
    y: list
    yaw: list
    decision: object = None
    dp_index: int = -1
    tail_m: object = None      # Plan (M) of a REPLACE / stop tail


@dataclass
class Executor:
    cfg: object
    client: object
    mission: object = None                 # io.mission.Mission
    state: str = "idle"
    reason: str = ""
    progress: dict | None = None
    progress_at: float = -math.inf
    grid: object = None                    # validated BeliefSnapshot
    grid_at: float = -math.inf
    grid_sig: tuple | None = None
    odom: tuple | None = None
    odom_at: float = -math.inf
    setup: object = None
    pcfg: object = None                    # the policy's configuration (the mission's planner
                                           # block; = cfg except for the dev loop's ablations)
    plan: Plan | None = None               # in M, Control's t axis
    plan_frame: object = None              # Frame2D the plan is expressed with
    mirror: ControlPathMirror = field(default_factory=ControlPathMirror)
    pending: Pending | None = None
    counter: int = 0
    run: int = 0
    stop_id: str | None = None
    stop_tries: int = 0
    stop_wait: bool = False
    policy_state: dict = field(default_factory=dict)
    done_dp: set = field(default_factory=set)
    records: list = field(default_factory=list)       # decision records (topics.md §5.1)
    requests: list = field(default_factory=list)      # request/answer log
    counts: dict = field(default_factory=lambda: {"decisions": 0, "requests": 0,
                                                  "accepted": 0, "refused": 0,
                                                  "belief_restarts": 0, "transform_jumps": 0})
    last_request: dict | None = None
    seq: int = 0
    last_seq: int | None = None            # grid_seq of the last grid received (MATH §11.3)
    hist: list = field(default_factory=list)          # (ros_time, tau) pairs of the progress
    hist_run: object = None
    events: list = field(default_factory=list)        # belief_restart, transform_jump (logged)
    stop_reason: str = ""
    last_horizon: tuple = (None, "")
    grid_note: str = ""                    # why the last grid received is not usable ("" = it is)
    # a monotonic clock in seconds, given by the node (`time.perf_counter`). The executor has no
    # clock of its own: without one the decision time is not measured (None in the record).
    timer: object = None
    decision_times: list = field(default_factory=list)   # seconds per decision, this run

    # ------------------------------------------------------------------ inputs
    def on_progress(self, d: dict, now: float):
        self.progress, self.progress_at = d, now
        # history of (node clock, path parameter) for the evidence horizon (MATH §11.3)
        rt, tau, run = d.get("ros_time"), d.get("tau"), d.get("run")
        if isinstance(rt, (int, float)) and isinstance(tau, (int, float)) and \
                not isinstance(rt, bool) and not isinstance(tau, bool):
            if self.hist and (rt < self.hist[-1][0] or tau < self.hist[-1][1] or
                              run != self.hist_run):
                self.hist = []
            self.hist_run = run
            if not self.hist or rt > self.hist[-1][0]:
                self.hist.append((float(rt), float(tau)))
                if len(self.hist) > HIST_MAX:
                    del self.hist[:len(self.hist) - HIST_MAX]

    def on_event(self, d: dict, now: float):
        ev = d.get("event")
        if ev == "cleared":
            self.hist = []
        if ev == "cleared" and self.state not in ("idle", "waiting", "done", "failed",
                                                  "run_ended"):
            self._set("run_ended", "run_cleared")

    def on_grid(self, grid_dict: dict, now: float):
        seq = grid_dict.get("grid_seq")
        try:
            seq = int(seq)
        except (TypeError, ValueError):
            seq = None
        if seq is not None and self.last_seq is not None:
            if seq == self.last_seq:
                return                               # a repeat of the grid already held
            if seq < self.last_seq:
                self._belief_restart(seq, now)       # never dropped silently (MATH §11.3)
        if seq is not None:
            self.last_seq = seq
        try:
            snap = validate_grid(grid_dict)
        except ContractError as exc:
            if exc.field == "odom_to_map_valid":
                self.grid = None                     # transient: no plan, no request
                self.grid_note = ("odom_to_map_valid is false: Belief's alignment has not "
                                  "converged yet")
                return
            if self.state in ("running", "deciding", "requesting"):
                self._set("failed", f"contract_error:{exc.field}")
            self.reason = f"contract_error:{exc.field}"
            self.grid_note = f"contract_error:{exc.field}"
            return
        self.grid_note = ""
        sig = geometry_signature(snap)
        if self.grid_sig is not None and sig != self.grid_sig and self.plan is not None:
            self._set("failed", "contract_error:geometry_changed")
            return
        self.grid, self.grid_at, self.grid_sig = snap, now, sig

    def on_odom(self, x: float, y: float, psi: float, now: float):
        self.odom, self.odom_at = (x, y, psi), now

    def _belief_restart(self, seq: int, now: float):
        """`grid_seq` went down: Belief restarted (Belief topics.md §1). Its map holds no
        earlier evidence and its contact uids start again at 1: the per-contact memory of the
        policy is forgotten, the restart is counted and logged, later records are labelled."""
        self.counts["belief_restarts"] += 1
        for key in ("n_rev", "rev_hist"):
            self.policy_state.pop(key, None)
        self.grid = None
        self.events.append({"schema": "blueboat_replanner_event/1", "event": "belief_restart",
                            "wall_time": now, "grid_seq_before": self.last_seq,
                            "grid_seq": seq, "tau": self._tau() if self.progress else None,
                            "run": self.run})
        self.reason = f"belief_restart: grid_seq {self.last_seq} -> {seq}"

    def _horizon(self, belief, tau: float) -> tuple:
        """(tau_h, source): the path time up to which `belief` is complete (MATH §11.3)."""
        lag = self.cfg.tracking_lag_m / self.cfg.speed_mps
        st = belief.stamp
        if st is None or not math.isfinite(st) or st <= 0.0:
            return tau - self.cfg.t_lat_s, "t_lat:no_stamp"
        if not self.hist:
            return tau - self.cfg.t_lat_s, "t_lat:no_clock"
        t0, t1 = self.hist[0][0], self.hist[-1][0]
        if st > t1 + self.cfg.clock_tolerance_s or st < t1 - self.cfg.horizon_max_lag_s:
            return tau - self.cfg.t_lat_s, "t_lat:clock_mismatch"
        if st < t0:
            return self.hist[0][1] - lag, "stamp_before_history"
        if st >= t1:
            return self.hist[-1][1] - lag, "stamp"
        i = bisect.bisect_right(self.hist, (st, math.inf))
        (ta, tau_a), (tb, tau_b) = self.hist[i - 1], self.hist[i]
        u = 0.0 if tb <= ta else (st - ta) / (tb - ta)
        return tau_a + u * (tau_b - tau_a) - lag, "stamp"

    def _labels(self, belief, info: dict) -> list:
        """Labels of a decision record (topics.md §5.1): what the numbers are conditional on."""
        out = list(BASE_LABELS)
        if belief is None:
            return out
        if belief.model.get("detector_calibrated") is not True:
            out.append("score-conditional")
        if belief.model.get("negative_evidence") == "ungated":
            out.append("belief-ungated")
        if belief.model.get("fold_opposite") is False:
            out.append("fold-mismatch")
        if "searched_from_aspect_counts" in belief.fallbacks:
            out.append("searched-from-aspect-counts")
        if info.get("aspect_inert"):
            out.append("aspect-inert")
        if self.counts["belief_restarts"]:
            out.append("belief-restarted")
        return out

    # ------------------------------------------------------------------ helpers
    def _set(self, state, reason=""):
        assert state in STATES, state
        self.state, self.reason = state, reason

    def _fresh_progress(self, now, age=None):
        age = self.cfg.progress_max_age_s if age is None else age
        return self.progress is not None and now - self.progress_at <= age

    def _fresh_grid(self, now):
        return self.grid is not None and now - self.grid_at <= self.cfg.belief_max_age_s

    def _tau(self):
        return float(self.progress.get("tau") or 0.0) if self.progress else 0.0

    def _next_id(self):
        rid = f"r{self.run}-{self.counter:04d}"
        self.counter += 1
        return rid

    def _to_world(self, plan_m: Plan):
        t, x, y, w = plan_m.sample(self.cfg.sample_ds_m, self.cfg.max_samples_per_request)
        xw, yw, ww = self.plan_frame.map_to_world(x, y, w)
        return list(map(float, t)), list(map(float, xw)), list(map(float, yw)), \
            list(map(float, ww))

    def _send(self, kind, c, plan_m, now, decision=None, dp_index=-1):
        t, x, y, w = self._to_world(plan_m)
        rid = self._next_id()
        code, msg = request_check(rid, t, x, y, w)
        if code != "ok":                     # a defect here, never sent
            self._set("failed", f"request_invalid:{code}")
            return False
        # the area (MATH §3.6): every sample in F_(margin - slack). Plans are built inside
        # F_margin; the slack absorbs a re-expression in a moved map frame (DESIGN §4)
        margin = max(0.0, self.cfg.margin_m - self.cfg.area_check_slack_m)
        ok, worst = plan_in_area(plan_m, self.setup.area, margin, self.cfg.geom_step_m)
        if not ok:
            self._set("failed", "request_invalid:outside_area "
                                f"({worst[0]:.2f}, {worst[1]:.2f}, {worst[2]}; margin {margin:g} m)")
            return False
        # the junction at c against the mirror (E-07), as Control will check it
        if kind != "first":
            code, msg = continuity_check(self.mirror, c, t, x, y, w, CONTROL_RULES["gov_h"])
            if code != "ok":
                self._set("failed", f"request_invalid:{code} ({msg})")
                return False
        self.pending = Pending(rid, kind, c, now, t, x, y, w, decision, dp_index, plan_m)
        req = {"id": rid, "change_point": c, "frame": "world", "t": t, "x": x, "y": y, "yaw": w}
        self.counts["requests"] += 1
        self.requests.append({"id": rid, "kind": kind, "change_point": c, "sent": now,
                              "n": len(t)})
        self.client.send(req, lambda ans, at, rid=rid: self.on_answer(rid, ans, at))
        return True

    # ------------------------------------------------------------------ operator
    def start(self, mission, now: float):
        """~/start: arm a new mission from idle, or re-arm from disarmed (DESIGN §7)."""
        if self.state == "disarmed":
            ok, why = self._rearm_ok(now)
            if not ok:
                return False, f"re-arm refused: {why}"
            self._set("running", "re-armed")
            return True, "re-armed"
        if self.state not in ("idle", "done", "failed", "run_ended", "stopped_holding"):
            return False, f"busy ({self.state})"
        self.mission = mission
        self.plan = None
        self.mirror = ControlPathMirror()
        self.pending = None
        self.policy_state = {}
        self.done_dp = set()
        self.stop_id = None
        self.stop_tries = 0
        self.stop_reason = ""
        self.decision_times = []
        self.run += 1
        self.counter = 0
        self.start_at = now
        self._set("waiting", "")
        return True, "waiting for awaiting_path, hold and a valid belief grid"

    def _rearm_ok(self, now):
        if not self._fresh_progress(now):
            return False, "progress stale"
        if self.progress.get("state") != "following":
            return False, f"progress state {self.progress.get('state')}"
        if self.progress.get("active_id") != self.mirror.active_id:
            return False, "active_id differs from the mirror"
        if not self._fresh_grid(now):
            return False, "belief stale"
        if self.stop_id is not None:
            return False, "stop pending"
        return True, ""

    def stop(self, now: float):
        """~/stop: truncation to a hold at the earliest feasible change point (DESIGN §5.3)."""
        if self.state in ("idle", "done", "failed", "run_ended", "stopped_holding"):
            return False, f"nothing to stop ({self.state})"
        if not self.mirror:
            self.pending = None
            self._set("stopped_holding", "operator_stop before the first path: Control holds")
            return True, self.reason
        self.stop_tries = 0
        self.stop_reason = ""
        self._set("stop_requested", "operator_stop")
        self._try_stop(now)
        return True, "stop requested"

    def _try_stop(self, now):
        if self.pending is not None:
            self.stop_wait = True
            return
        if not self._fresh_progress(now) or self.progress.get("refusal_now"):
            self.stop_wait = True
            return
        self.stop_wait = False
        mcp = self.progress.get("min_change_point")
        tau = self._tau()
        c = (mcp if mcp is not None else tau + 0.2) + self.cfg.latency_margin_s
        c = min(c, self.mirror.t_end) if self.mirror.t_end is not None else c
        xw, yw, ww = self.mirror.pose(c)
        xm, ym, wm = self.plan_frame.world_to_map(xw, yw, ww)
        tail = Plan([hold_segment((float(xm), float(ym), float(wm)), self.cfg.stop_hold_s)], c,
                    self.cfg.speed_mps)
        self.stop_tries += 1
        self._send("stop", c, tail, now)

    # ------------------------------------------------------------------ answers
    def on_answer(self, rid: str, ans: dict, now: float):
        p = self.pending
        if p is None or p.req_id != rid:
            return                                  # late or duplicate answer: ignored
        self.pending = None
        self.last_request = {"id": rid, "accepted": bool(ans.get("accepted")),
                             "reason": ans.get("reason", ""), "message": ans.get("message", "")}
        self.requests[-1].update(answer=self.last_request, answered=now)
        if ans.get("accepted"):
            self.counts["accepted"] += 1
            self._accepted(p)
        else:
            self.counts["refused"] += 1
            self._refused(p, ans.get("reason", ""), now)

    def _accepted(self, p: Pending):
        self.mirror.splice(p.change_point if p.kind != "first" else float("nan"),
                           p.t, p.x, p.y, p.yaw, p.req_id)
        if p.kind == "first":
            self._set("running", "first path accepted")
        elif p.kind == "replace":
            self.plan = Plan(self.plan.segments[:p.dp_index + 1] + p.tail_m.segments,
                             self.plan.t0, self.plan.speed)
            d = p.decision
            self.setup.budget.revisit_used += float(d.info.get("dT", d.info.get("dT_revisit",
                                                                                  0.0)))
            if hasattr(self.setup.policy, "on_accepted"):
                self.setup.policy.on_accepted(d, self.policy_state)
            self._set("running", "update accepted")
        elif p.kind == "stop":
            self.stop_id = p.req_id
            if p.decision is not None and p.dp_index >= 0 and self.plan is not None:
                # a mission stop decided by the policy: the plan ends with the hold
                self.plan = Plan(self.plan.segments[:p.dp_index + 1] + p.tail_m.segments,
                                 self.plan.t0, self.plan.speed)
            pre = f"{self.stop_reason}: " if self.stop_reason else ""
            self._set("stop_accepted", f"{pre}holds from t={p.change_point:.2f}")

    def _refused(self, p: Pending, code: str, now: float):
        if p.kind == "stop":
            if self.stop_tries >= self.cfg.stop_retries:
                self._set("stop_rejected", f"request_refused:{code}; {STOP_NOT_IN_EFFECT}")
                return
            self._set("stop_requested", f"retry after {code}")
            if code in STATE_CODES:
                self.stop_wait = True
            else:
                self._try_stop(now)
            return
        if p.kind == "first":
            if code in STATE_CODES and code != "mission_owned":
                self._set("waiting", f"first path refused ({code}); waiting")
            else:
                self._set("failed", f"request_refused:{code}")
            return
        if code in ("operator_override", "not_ready"):
            self._set("disarmed", "operator_override" if code == "operator_override" else "estop")
        elif code == "no_run":
            self._set("run_ended", "run_cleared")
        elif code == "mission_owned":
            self._set("failed", "request_refused:mission_owned")
        elif code == "lead_too_short":
            self._set("running", "lead_too_short: re-decide at the next decision point")
        elif code in ("discontinuous_position", "discontinuous_heading"):
            n = self.policy_state.get("discontinuities", 0) + 1
            self.policy_state["discontinuities"] = n
            self._set("running" if n < 2 else "failed",
                      f"request_refused:{code}" + ("" if n < 2 else " (twice)"))
        else:
            self._set("failed", f"request_refused:{code}")

    # ------------------------------------------------------------------ the loop
    def tick(self, now: float):
        # pending answer timeout -> reconcile from progress (DESIGN §5.2)
        if self.pending is not None:
            age = now - self.pending.sent
            if age > self.cfg.answer_timeout_s and self.progress is not None:
                la = self.progress.get("last_answer") or {}
                if self.progress.get("active_id") == self.pending.req_id:
                    self.on_answer(self.pending.req_id, {"accepted": True, "reason": "ok",
                                                         "message": "reconciled"}, now)
                elif la.get("id") == self.pending.req_id and la.get("accepted") is False:
                    self.on_answer(self.pending.req_id, {"accepted": False,
                                                         "reason": la.get("reason", ""),
                                                         "message": "reconciled"}, now)
                elif age > self.cfg.answer_timeout_s + self.cfg.reconcile_timeout_s:
                    self.pending = None
                    self._set("failed", "answer_lost")
            return
        st = self.state
        if st == "waiting":
            return self._try_first(now)
        if self.progress is None:
            return
        ps = self.progress.get("state")
        if st in ("running", "deciding", "requesting", "stop_accepted", "stop_requested",
                  "stop_rejected"):
            act = self.progress.get("active_id")
            known = {s.id for s in self.mirror.segments}
            if act is not None and self.mirror and act not in known:
                return self._set("failed", "foreign_update")
            if ps == "no_run":
                return self._set("run_ended", "run_cleared")
        if st == "running":
            if ps in ("override", "not_ready"):
                return self._set("disarmed", "operator_override" if ps == "override" else "estop")
            if self.progress.get("end_reached") and self.progress.get("active_id") == \
                    self.mirror.active_id:
                return self._set("done", "plan exhausted")
            return self._maybe_decide(now)
        if st == "stop_requested" and self.stop_wait and ps == "following":
            return self._try_stop(now)
        if st == "stop_accepted" and self.progress.get("end_reached") and \
                self.progress.get("active_id") == self.stop_id:
            pre = f"{self.stop_reason}: " if self.stop_reason else ""
            return self._set("stopped_holding", f"{pre}holding at the stop pose")

    def _try_first(self, now):
        if now - getattr(self, "start_at", now) > self.cfg.start_timeout_s:
            return self._set("failed", "start_timeout: " + self._missing(now))
        if self._missing(now):
            return
        g = self.grid
        hold = self.progress["hold"]
        self.plan_frame = g.frame
        psi0 = hold["yaw"]
        if self.odom is not None and now - self.odom_at <= self.cfg.odom_max_age_s:
            psi0 = self.odom[2]
        xm, ym, pm = g.frame.world_to_map(hold["x"], hold["y"], psi0)
        m = self.mission
        try:
            self.setup = setup_mission(m.config, m.condition, m.areas, g.datum_lat, g.datum_lon,
                                       (float(xm), float(ym), float(pm)), t0=0.0)
        except MissionError as exc:
            return self._set("failed", f"mission_error: {exc}")
        # the mission polygon must lie in Belief's area: outside it there is no evidence
        ok, detail = mission_area_check(self.setup.area, g, m.config.area_tolerance_m)
        if not ok:
            return self._set("failed", f"mission_outside_belief_area: {detail}")
        self.cfg = m.config
        self.pcfg = getattr(m, "planner_config", None) or m.config
        # a mission cut short (dev loop only; a mission file cannot set it): B = fraction * B_C2
        self.setup.budget.B = getattr(m, "budget_fraction", 1.0) * self.setup.budget.B
        self.plan = self.setup.initial_plan.truncated(self.setup.budget.t_limit)
        self._set("starting", "first path sent")       # before sending: the answer may be sync
        self._send("first", float("nan"), self.plan, now)

    def _missing(self, now) -> str:
        miss = []
        if not self._fresh_progress(now):
            miss.append("progress")
        elif self.progress.get("state") != "awaiting_path" or not self.progress.get("hold"):
            miss.append(f"awaiting_path (state {self.progress.get('state')})")
        if not self._fresh_grid(now):
            miss.append("belief grid (valid, fresh)")
        ready = getattr(self.client, "ready", None)
        if ready is not None and not ready():
            miss.append("update_path service")
        return ", ".join(miss)

    def _due(self):
        """(t_dp, idx) of the next decision point if its trigger has fired (DESIGN §5.2 rule 1),
        else None."""
        lead = self.progress.get("lead_min")
        if lead is None:
            return None
        tau = self._tau()
        for t_dp, idx in self.setup.policy.decision_points(self.plan):
            if t_dp <= tau or (round(t_dp, 6), idx) in self.done_dp:
                continue
            trig = t_dp - (lead + self.cfg.t_decide_s + self.cfg.latency_margin_s)
            return (t_dp, idx) if tau >= trig else None
        return None

    def decision_due(self) -> bool:
        """True when the next `tick` decides: running, no request pending, a trigger fired. Lets
        an offline driver hand over a fresh grid only when one is read (eval/devloop.py)."""
        return (self.state == "running" and self.pending is None and self.progress is not None
                and self.setup is not None and self.plan is not None and self._due() is not None)

    def _maybe_decide(self, now):
        due = self._due()
        if due is None:
            return
        t_dp, idx = due
        self.done_dp.add((round(t_dp, 6), idx))
        return self._decide(t_dp, idx, now)

    def _decide(self, c, idx, now):
        self._set("deciding")
        self.counts["decisions"] += 1
        tau = self._tau()
        belief = self.grid if self._fresh_grid(now) else None
        # re-express the plan in the current map frame (DESIGN §4); a jump is logged
        jump = 0.0
        if belief is not None and self.plan_frame is not None and belief.frame != self.plan_frame:
            jump = belief.frame.distance_to(self.plan_frame)
            if jump > self.cfg.transform_jump_m:
                self.counts["transform_jumps"] += 1
                self.events.append({"schema": "blueboat_replanner_event/1",
                                    "event": "transform_jump", "wall_time": now, "tau": tau,
                                    "jump_m": jump, "limit_m": self.cfg.transform_jump_m,
                                    "run": self.run})
            self.plan = Plan(transform_segments(self.plan.segments, self.plan_frame,
                                                belief.frame), self.plan.t0, self.plan.speed)
            self.plan_frame = belief.frame
        self.policy_state["tau"] = tau
        tau_h, h_src = self._horizon(belief, tau) if belief is not None else (None, "")
        self.last_horizon = (tau_h, h_src)
        ctx = EpochContext(self.pcfg or self.cfg, self.setup.area, self.plan, c, idx,
                           self.setup.budget,
                           belief=belief, coverage=self.setup.coverage, state=self.policy_state,
                           tau_horizon=tau_h, horizon_source=h_src)
        t0 = self.timer() if self.timer is not None else None
        dec = self.setup.policy.on_epoch(ctx)
        compute_s = float(self.timer() - t0) if t0 is not None else None
        if compute_s is not None:
            self.decision_times.append(compute_s)
        # The pre-send check (DESIGN §5.2 rule 2). The node computes inside its callback, so the
        # progress held here dates from BEFORE the computation. The path parameter advances by
        # at most 1 path-s per second (Control's governor, E-14), so `min_change_point` is aged
        # by the compute time: the conservative reading of "the latest progress".
        aged = compute_s if compute_s is not None else 0.0
        mcp0 = self.progress.get("min_change_point")
        lead_left = (c - (mcp0 + aged)) if isinstance(mcp0, (int, float)) else None
        odom_ok = self.odom is not None and now - self.odom_at <= self.cfg.odom_max_age_s
        pose = self.odom if odom_ok else (self.mirror.pose(tau) if self.mirror else None)
        bud = self.setup.budget
        rec = {"schema": "blueboat_replanner_decision/1", "seq": self.seq,
               "mission": self.mission.name, "condition": self.mission.condition,
               "run": self.run, "tau": tau, "wall_time": now,
               "ros_time": self.progress.get("ros_time"),
               "epoch": {"source": "decision_point", "segment_index": idx, "t_dp": c,
                         "change_point": c,
                         "min_change_point_at_send": self.progress.get("min_change_point")},
               "inputs": {"grid_seq": belief.grid_seq if belief else None,
                          "grid_age_s": now - self.grid_at if belief else None,
                          "grid_stamp": belief.stamp if belief else None,
                          "progress_age_s": now - self.progress_at,
                          "pose_source": "odom" if odom_ok else "mirror",
                          "odom_age_s": (now - self.odom_at) if self.odom else None,
                          "pose_xy_psi": [float(v) for v in pose] if pose else None,
                          "odom_to_map": ([belief.frame.tx, belief.frame.ty, belief.frame.yaw]
                                          if belief else None),
                          "odom_to_map_sigma_m": (belief.transform_sigma_m if belief and
                                                  math.isfinite(belief.transform_sigma_m)
                                                  else None),
                          "transform_jump_m": jump,
                          "belief_model": dict(belief.model) if belief else None,
                          "belief_fallbacks": list(belief.fallbacks) if belief else None,
                          "belief_restarts": self.counts["belief_restarts"],
                          "unmapped_classes": dec.info.get("unmapped_classes")},
               "budget": {"B": bud.B, "elapsed": tau - bud.t_start,
                          "remaining": bud.t_limit - tau,
                          "revisit_used": bud.revisit_used,
                          "revisit_cap": bud.revisit_cap(self.cfg.rho_rev)},
               "calibration": dec.info.get("calibration"),
               "pending": dec.info.get("pending") or ({"tau_horizon": tau_h, "source": h_src}
                                                      if belief else None),
               "contacts": dec.info.get("contacts"),
               "chosen": dec.info.get("chosen", dec.action), "action": dec.action,
               "candidates": list(dec.scored), "rules_fired": list(dec.rules_fired),
               # how long the policy took, against the time the trigger reserves for it
               "timing": {"compute_s": compute_s, "allowance_s": self.cfg.t_decide_s,
                          "over_allowance": (None if compute_s is None
                                             else bool(compute_s > self.cfg.t_decide_s)),
                          # path seconds between Control's earliest change point, aged by the
                          # compute time, and c; a request needs at least latency_margin_s
                          "lead_left_s": lead_left},
               "request": None, "labels": self._labels(belief, dec.info)}
        self.seq += 1
        if belief is None:
            rec["epoch"]["outcome"] = "stale_belief"
        if dec.action == "STOP":
            # mission stop decided by the policy (MATH §12): a truncation to a hold at c
            mcp = self.progress.get("min_change_point")
            if not self._fresh_progress(now, self.cfg.progress_send_max_age_s) or mcp is None \
                    or c < mcp + aged + self.cfg.latency_margin_s:
                rec["epoch"]["outcome"] = "missed_lead"
                self.records.append(rec)
                return self._set("running", "missed_lead")
            tail = Plan([hold_segment(self.plan.pose_at(c), self.cfg.stop_hold_s)], c,
                        self.cfg.speed_mps)
            self.stop_reason = "mission_stop:rho_floor"
            self.stop_tries = 1
            self._set("stop_requested", self.stop_reason)
            rid_next = f"r{self.run}-{self.counter:04d}"
            if self._send("stop", c, tail, now, decision=dec, dp_index=idx):
                rec["request"] = {"id": rid_next, "change_point": c,
                                  "n_samples": self.requests[-1]["n"]}
                rec["epoch"]["outcome"] = "sent"
            else:
                rec["epoch"]["outcome"] = "request_invalid"
        elif dec.action == "REPLACE" and dec.tail is not None and dec.tail.segments:
            # pre-send check against the freshest progress (DESIGN §5.2)
            mcp = self.progress.get("min_change_point")
            if not self._fresh_progress(now, self.cfg.progress_send_max_age_s) or mcp is None \
                    or c < mcp + aged + self.cfg.latency_margin_s:
                rec["epoch"]["outcome"] = "missed_lead"
                self.records.append(rec)
                return self._set("running", "missed_lead")
            self._set("requesting")                   # before sending: the answer may be sync
            rid_next = f"r{self.run}-{self.counter:04d}"
            if self._send("replace", c, dec.tail, now, decision=dec, dp_index=idx):
                rec["request"] = {"id": rid_next, "change_point": c,
                                  "n_samples": self.requests[-1]["n"]}
                rec["epoch"]["outcome"] = "sent"
            else:
                rec["epoch"]["outcome"] = "request_invalid"
        else:
            rec["epoch"].setdefault("outcome", "continue")
            self._set("running")
        self.records.append(rec)

    # ------------------------------------------------------------------ status
    def status(self, now: float) -> dict:
        s = {"schema": "blueboat_replanner_status/1", "state": self.state, "reason": self.reason,
             "mission": getattr(self.mission, "name", None),
             "condition": getattr(self.mission, "condition", None), "run": self.run,
             "tau": self._tau() if self.progress else None,
             "active_id": self.mirror.active_id,
             "mirror_ids": [sg.id for sg in self.mirror.segments],
             "last_request": self.last_request, "counts": dict(self.counts),
             "inputs": {"grid_ok": self.grid is not None,
                        "grid_seq_received": self.last_seq,
                        "grid_note": self.grid_note or None,
                        "grid_age_s": (now - self.grid_at) if self.grid else None,
                        "progress_age_s": (now - self.progress_at) if self.progress else None,
                        "odom_age_s": (now - self.odom_at) if self.odom else None,
                        "horizon_source": self.last_horizon[1] or None,
                        "detector_calibrated": (self.grid.model.get("detector_calibrated")
                                                if self.grid else None)},
             "budget": ({"B": self.setup.budget.B, "revisit_used": self.setup.budget.revisit_used}
                        if self.setup else None),
             "wall_time": now}
        if self.state == "stop_rejected":
            s["stop_in_effect"] = False
            s["message"] = STOP_NOT_IN_EFFECT
        if self.state == "disarmed":
            s["rearm_required"] = True
        return s

    def end_record(self, now: float) -> dict:
        """`end.json` of a run (topics.md §6): the final state, why, and the counts."""
        dt = self.decision_times
        timing = None if not dt else {
            "n": len(dt), "mean": sum(dt) / len(dt), "max": max(dt),
            "allowance_s": self.cfg.t_decide_s,
            "over_allowance": sum(1 for v in dt if v > self.cfg.t_decide_s)}
        return {"schema": "blueboat_replanner_end/1", "state": self.state,
                "decision_time_s": timing,
                "reason": self.reason, "terminal": self.state in TERMINAL,
                "mission": getattr(self.mission, "name", None),
                "condition": getattr(self.mission, "condition", None), "run": self.run,
                "tau": self._tau() if self.progress else None, "counts": dict(self.counts),
                "budget": ({"B": self.setup.budget.B,
                            "revisit_used": self.setup.budget.revisit_used}
                           if self.setup else None),
                "active_id": self.mirror.active_id, "wall_time": now}
