"""Gate section [23]: executor against the Control stub (fake clock; DESIGN §5.2, §5.3).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [23]
print("[23] executor against the Control stub (fake clock; DESIGN §5.2, §5.3)")
MISSION = {"mission": {"name": "t1", "condition": "C2", "areas": [[[34.5, 133.5], [34.5, 133.5013], [34.50054, 133.5013], [34.50054, 133.5]]], "config": {"planning_depth_m": 2.0}}}
import py_compile  # noqa: E402
from blueboat_replanner.ros.executor import Executor  # noqa: E402


class StubClient:
    def __init__(self, stub_, clock):
        self.stub, self.clock, self.auto, self.held = stub_, clock, True, []

    def send(self, req, cb):
        if not self.auto:
            self.held.append((req, cb))
            return
        cb(self.stub.handle(req["id"], req["change_point"], req["t"], req["x"], req["y"],
                            req["yaw"], self.clock["tau"]), self.clock.get("now", 0.0))


def progress_of(stub_, tau, override=None):
    d = stub_.progress(tau)
    d["refusal_now"] = "" if override is None else override
    if override == "operator_override":
        d["state"] = "override"
    if override == "not_ready":
        d["state"] = "not_ready"
    d["lead_min"] = stub_.lead_min(tau)
    d["ros_time"] = 500.0 + tau          # Control's node clock (E-11); the path starts at 500 s
    d["run"] = 1
    return d


def new_exec(cond, contacts_=()):
    mission = parse_mission({"mission": {**MISSION["mission"], "condition": cond}})
    clock = {"tau": 0.0}
    st_ = ControlStub((10.0, 10.0, 0.0))
    ex = Executor(mission.config, StubClient(st_, clock))
    ok, _ = ex.start(mission, 0.0)
    gr = make_grid(list(contacts_))
    ex.on_grid(gr, 0.0)
    ex.on_progress(progress_of(st_, 0.0), 0.0)
    ex.tick(0.0)
    return ex, st_, clock, gr


def fly(ex, st_, clock, gr, t_to, dt=2.0, override_at=None):
    now = clock.get("now", 0.0)
    while clock["tau"] < t_to:
        clock["tau"] += dt
        now += dt
        clock["now"] = now                  # the arrival time of a synchronous answer
        gr["grid_seq"] += 1
        ex.on_grid(gr, now)
        ov = override_at(clock["tau"]) if override_at else None
        ex.on_progress(progress_of(st_, clock["tau"], ov), now)
        ex.tick(now)
        if ex.state in ("done", "failed", "disarmed", "stopped_holding", "stop_rejected"):
            break
    clock["now"] = now
    return now


ex, st_, clock, gr = new_exec("C2")
check("first path sent and accepted -> running", ex.state == "running" and
      st_.mirror.active_id == "r1-0000" and ex.mirror.active_id == "r1-0000")
fly(ex, st_, clock, gr, ex.plan.t_end + 5)
check("C2: one request, runs to done", ex.state == "done" and ex.counts["requests"] == 1)

ex, st_, clock, gr = new_exec("C3", [contact(1, 30, 30), contact(2, 90, 40, first_sector=3)])
fly(ex, st_, clock, gr, ex.plan.t_end + 5, dt=1.0)
check("C3 through the executor: one decision, update accepted by the stub",
      ex.counts["decisions"] == 1 and ex.counts["accepted"] == 2 and ex.state == "done",
      f"{ex.counts} {ex.state} {ex.reason}")

ex, st_, clock, gr = new_exec("C2")
fly(ex, st_, clock, gr, 100.0)
now = clock["now"]
ok, _ = ex.stop(now)
check("D-1: stop accepted -> stop_accepted (boat still moving)", ok and ex.state ==
      "stop_accepted" and ex.status(now).get("stop_in_effect") is None)
fly(ex, st_, clock, gr, ex.mirror.t_end + 5)
check("D-1: stopped_holding only after end_reached of the stop id",
      ex.state == "stopped_holding" and st_.mirror.active_id == ex.stop_id)


class RefusingStub(ControlStub):
    def handle(self, req_id, c, t_, x_, y_, w_, tau):
        if not self.awaiting:
            return self._refuse("discontinuous_position", "forced", tau)
        return super().handle(req_id, c, t_, x_, y_, w_, tau)


mission = parse_mission(MISSION)
clock = {"tau": 0.0}
rs = RefusingStub((10.0, 10.0, 0.0))
ex = Executor(mission.config, StubClient(rs, clock))
ex.start(mission, 0.0)
gr = make_grid([])
ex.on_grid(gr, 0.0)
ex.on_progress(progress_of(rs, 0.0), 0.0)
ex.tick(0.0)
fly(ex, rs, clock, gr, 50.0)
before = list(rs.mirror.segments)
ex.stop(clock["now"])
st_now = ex.status(clock["now"])
check("D-2: stop refused 3 times -> stop_rejected, 'not in effect', old path unchanged",
      ex.state == "stop_rejected" and st_now["stop_in_effect"] is False and
      rs.mirror.segments == before and ex.stop_tries == 3)

ex, st_, clock, gr = new_exec("C2")
fly(ex, st_, clock, gr, 300.0, override_at=lambda t: "operator_override" if t > 120 else None)
check("D-3: manual override during running -> disarmed", ex.state == "disarmed" and
      ex.status(clock["now"])["rearm_required"])
n_req = ex.counts["requests"]
st_.state = "following"
fly(ex, st_, clock, gr, 160.0)
check("D-3: no request after release until ~/start", ex.state == "disarmed" and
      ex.counts["requests"] == n_req)
ok, why = ex.start(None, clock["now"])
check("D-3: ~/start re-arms when the checks pass", ok and ex.state == "running", why)
ex2, st2, clock2, gr2 = new_exec("C2")
fly(ex2, st2, clock2, gr2, 300.0, override_at=lambda t: "not_ready" if t > 80 else None)
check("D-4: E-STOP -> disarmed (reason estop)", ex2.state == "disarmed" and ex2.reason == "estop")
st2.state = "following"
pr = progress_of(st2, clock2["tau"])
pr["active_id"] = "someone-else"
ex2.on_progress(pr, clock2["now"])
ok, why = ex2.start(None, clock2["now"])
check("D-5: re-arm refused while active_id differs from the mirror", not ok and
      "active_id" in why)

ex, st_, clock, gr = new_exec("C3", [contact(1, 30, 30)])
dpi = ex.setup.policy.decision_points(ex.plan)[0]
clock["tau"] = dpi[0] - 1.0
gr["grid_seq"] += 1
ex.on_grid(gr, 10.0)
ex.on_progress(progress_of(st_, clock["tau"]), 10.0)
ex._decide(dpi[0], dpi[1], 10.0)
check("G-1: a decision under a missed lead is never sent",
      ex.records[-1]["epoch"]["outcome"] == "missed_lead" and ex.counts["requests"] == 1,
      f"{ex.records[-1]['epoch']}")
st_stale = ex.records[-1]
ex.on_progress(progress_of(st_, clock["tau"]), 30.0)
ex._decide(dpi[0], dpi[1], 30.0)
check("stale belief -> no request (outcome stale_belief)",
      ex.records[-1]["epoch"].get("outcome") in ("stale_belief", "continue") and
      ex.counts["requests"] == 1)

mission = parse_mission(MISSION)
clock = {"tau": 0.0}
st3 = ControlStub((10.0, 10.0, 0.0))
cl3 = StubClient(st3, clock)
cl3.auto = False
ex = Executor(mission.config, cl3)
ex.start(mission, 0.0)
gr = make_grid([])
ex.on_grid(gr, 0.0)
ex.on_progress(progress_of(st3, 0.0), 0.0)
ex.tick(0.0)
req, cb = cl3.held[0]
st3.handle(req["id"], req["change_point"], req["t"], req["x"], req["y"], req["yaw"], 0.0)
ex.on_progress(progress_of(st3, 0.0), 6.0)
ex.tick(6.0)
check("answer lost -> reconciled from progress active_id", ex.state == "running" and
      ex.mirror.active_id == req["id"])
pr = progress_of(st3, 10.0)
pr["active_id"] = "intruder-1"
ex.on_progress(pr, 8.0)
ex.tick(8.0)
check("foreign update -> failed", ex.state == "failed" and ex.reason == "foreign_update")
# ---- reading of the real Belief (MATH 11.3, 12): horizon, restart, area, mission stop, records
ex, st_, clock, gr = new_exec("C3", [contact(1, 30, 30)])
fly(ex, st_, clock, gr, 60.0, dt=1.0)
tauX = clock["tau"]
lagX = ex.cfg.tracking_lag_m / ex.cfg.speed_mps
hz = {k_: ex._horizon(validate_grid(dict(gr, stamp=v_)), tauX) for k_, v_ in
      {"in": 500.0 + tauX - 6.0, "none": None, "zero": 0.0, "ahead": 500.0 + tauX + 5.0,
       "wall": 1.79e9, "old": 470.0}.items()}
hist_saved, ex.hist = ex.hist, []
hz["noclock"] = ex._horizon(validate_grid(dict(gr, stamp=500.0 + tauX - 6.0)), tauX)
ex.hist = hist_saved
check("H-3: the grid stamp is converted to path time through Control's progress clock, minus "
      "the tracking margin", hz["in"][1] == "stamp" and
      abs(hz["in"][0] - (tauX - 6.0 - lagX)) < 1e-6, f"{hz['in']}")
check("H-3: each fallback names itself (no stamp, no clock, clock mismatch, before history)",
      hz["none"] == (tauX - 10.0, "t_lat:no_stamp") and hz["zero"][1] == "t_lat:no_stamp" and
      hz["ahead"] == (tauX - 10.0, "t_lat:clock_mismatch") and
      hz["wall"][1] == "t_lat:clock_mismatch" and hz["noclock"][1] == "t_lat:no_clock" and
      hz["old"] == (0.0 - lagX, "stamp_before_history"), f"{hz}")
ex.policy_state["n_rev"] = {1: 2}
ex.policy_state["rev_hist"] = {1: []}
seq_before = gr["grid_seq"]
gr["grid_seq"] = 1
ex.on_grid(gr, clock["now"] + 0.5)
check("H-4: grid_seq going down = a Belief restart: accepted, counted, logged, the per-contact "
      "memory reset", ex.counts["belief_restarts"] == 1 and ex.grid is not None and
      ex.grid.grid_seq == 1 and "n_rev" not in ex.policy_state and
      ex.events[-1]["event"] == "belief_restart" and
      ex.events[-1]["grid_seq_before"] == seq_before)
ex.on_grid(gr, clock["now"] + 0.6)
check("a repeated grid_seq is a repeat, not a restart", ex.counts["belief_restarts"] == 1)
gr["grid_seq"] += 1
ex.on_grid(dict(gr, odom_to_map_valid=False), clock["now"] + 0.7)
stN = ex.status(clock["now"] + 0.7)["inputs"]
check("a grid on an unaligned map is received, not usable, and the status says why",
      stN["grid_ok"] is False and stN["grid_seq_received"] == gr["grid_seq"] and
      "odom_to_map_valid" in stN["grid_note"])
gr["grid_seq"] += 1
ex.on_grid(gr, clock["now"] + 0.8)
check("the next valid grid is usable again", ex.status(clock["now"] + 0.8)["inputs"]["grid_ok"]
      and ex.status(clock["now"] + 0.8)["inputs"]["grid_note"] is None)
fly(ex, st_, clock, gr, ex.plan.t_end + 5, dt=1.0)
recX = ex.records[-1]
check("decision record: labels, Belief's model flags, transform, pose, pending source, budget",
      {"belief-restarted", "score-conditional", "model-conditional"} <= set(recX["labels"]) and
      recX["inputs"]["odom_to_map"] == [0.0, 0.0, 0.0] and
      recX["inputs"]["transform_jump_m"] == 0.0 and
      recX["inputs"]["belief_model"]["detector_calibrated"] is None and
      recX["inputs"]["pose_source"] in ("odom", "mirror") and
      len(recX["inputs"]["pose_xy_psi"]) == 3 and
      recX["pending"]["source"] == "t_lat:no_stamp" and
      abs(recX["budget"]["remaining"] + recX["budget"]["elapsed"] - recX["budget"]["B"]) < 1e-9
      and recX["ros_time"] == 500.0 + recX["tau"], f"{recX['labels']} {recX['pending']}")
endX = ex.end_record(clock["now"])
check("end record: final state, reason, counts", endX["schema"] == "blueboat_replanner_end/1"
      and endX["state"] == ex.state == "done" and endX["terminal"] and
      endX["counts"]["belief_restarts"] == 1)
check("no clock given to the executor: the decision time is not measured, and the records say "
      "so (None, never 0)", recX["timing"]["compute_s"] is None and
      recX["timing"]["allowance_s"] == 2.0 and recX["timing"]["over_allowance"] is None and
      endX["decision_time_s"] is None, f"{recX['timing']} {endX['decision_time_s']}")
mA_ = parse_mission(MISSION)
clockA = {"tau": 0.0}
stA = ControlStub((10.0, 10.0, 0.0))
exA = Executor(mA_.config, StubClient(stA, clockA))
exA.start(mA_, 0.0)
grA = make_grid([])
grA["area_x"], grA["area_y"] = [0.0, 60.0, 60.0, 0.0], [0.0, 0.0, 60.0, 60.0]
exA.on_grid(grA, 0.0)
exA.on_progress(progress_of(stA, 0.0), 0.0)
exA.tick(0.0)
check("H-5: a mission polygon outside Belief's area stops the mission before the first path",
      exA.state == "failed" and exA.reason.startswith("mission_outside_belief_area") and
      exA.counts["requests"] == 0, f"{exA.state} {exA.reason}")
mS_ = parse_mission({"mission": {**MISSION["mission"], "condition": "C4",
                                 "config": {"planning_depth_m": 2.0, "rho_floor": 10.0}}})
clockS = {"tau": 0.0}
stS = ControlStub((10.0, 10.0, 0.0))
tickS = {"t": 0.0}


def fake_timer():                    # each reading is 0.5 s after the last: a decision "takes" 0.5 s
    tickS["t"] += 0.5
    return tickS["t"]


exS = Executor(mS_.config, StubClient(stS, clockS), timer=fake_timer)
exS.start(mS_, 0.0)
grS = make_grid([contact(1, 30, 30)])
exS.on_grid(grS, 0.0)
exS.on_progress(progress_of(stS, 0.0), 0.0)
exS.tick(0.0)
fly(exS, stS, clockS, grS, exS.plan.t_end + 5, dt=1.0)
recS = exS.records[-1]
check("H-8: mission stop (rho_floor > 0): STOP decided, a hold sent at the change point, the "
      "run ends as stopped_holding with its reason", exS.state == "stopped_holding" and
      exS.reason.startswith("mission_stop:rho_floor") and recS["action"] == "STOP" and
      recS["chosen"] == "STOP" and exS.plan.segments[-1].kind == "hold" and
      exS.counts["accepted"] == 2 and stS.mirror.active_id == exS.stop_id,
      f"{exS.state} {exS.reason} {recS['action']}")
check("C4 decision record: calibration, pending and contacts blocks (topics.md 5.1)",
      recS["calibration"]["cells"] > 1000 and recS["calibration"]["searched_source"] ==
      "aspect_log_miss" and recS["pending"]["source"] == "t_lat:no_stamp" and
      recS["pending"]["keys"] > 0 and recS["contacts"][0]["uid"] == 1 and
      "p_tilde" in recS["contacts"][0] and recS["inputs"]["unmapped_classes"] == [],
      f"{recS['calibration']} {recS['pending']}")
endS = exS.end_record(0.0)
check("decision time: measured with the node's clock, written to the record with the allowance "
      "(t_decide_s) and the lead left at sending, and to the end record",
      all(r["timing"]["compute_s"] == 0.5 and r["timing"]["allowance_s"] == 2.0 and
          r["timing"]["over_allowance"] is False for r in exS.records) and
      recS["timing"]["lead_left_s"] >= mS_.config.latency_margin_s and
      endS["decision_time_s"] == {"n": len(exS.records), "mean": 0.5, "max": 0.5,
                                  "allowance_s": 2.0, "over_allowance": 0} and
      len(exS.records) >= 1, f"{recS['timing']} {endS['decision_time_s']}")
# G-1 with the clock: the node computes inside its callback, so the progress it holds dates from
# before the computation. A decision that took 2.5 s has used up the lead: never sent.
exL, stL, clockL, grL = new_exec("C3", [contact(1, 30, 30), contact(2, 90, 40, first_sector=3)])
tickL = {"t": 0.0}


def slow_timer():
    tickL["t"] += 2.5
    return tickL["t"]


exL.timer = slow_timer
fly(exL, stL, clockL, grL, exL.plan.t_end + 5, dt=1.0)
recL = exL.records[-1]
endL = exL.end_record(0.0)
check("G-1 under the node's clock: a decision whose computation used up the lead (2.5 s against "
      "the 2.0 s allowance) is not sent, is recorded as missed_lead and flagged; the mission "
      "goes on to done on the path it had", exL.counts["decisions"] == 1 and
      recL["epoch"]["outcome"] == "missed_lead" and recL["request"] is None and
      exL.counts["requests"] == 1 and exL.counts["refused"] == 0 and exL.state == "done" and
      recL["timing"]["over_allowance"] is True and
      recL["timing"]["lead_left_s"] < exL.cfg.latency_margin_s and
      endL["decision_time_s"]["over_allowance"] == 1,
      f"{recL['epoch']} {recL['timing']} {exL.counts} {exL.state}")
compiled = True
for f in sorted((pk / "ros").glob("*.py")) + sorted((PKG_ROOT / "launch").glob("*.py")) + \
        [pk / "eval" / "emulator_node.py"]:
    if not f.exists():
        continue
    try:
        with tempfile.TemporaryDirectory() as tdc:
            py_compile.compile(str(f), doraise=True, cfile=os.path.join(tdc, "x.pyc"))
    except py_compile.PyCompileError as e:
        compiled = False
        print(e)
check("ROS and launch files compile (syntax only; the runtime check is test/ros_graph_check.py, "
      "run by hand on an isolated ROS domain)", compiled)

# ---- pre-send checks (DESIGN §5.2 rule 2a; MATH §3.6), readiness, arrival time of an answer
from blueboat_replanner.core.geo import Frame2D  # noqa: E402
from blueboat_replanner.core.plan import transform_segments  # noqa: E402


def shifted_tail(ex_, d):
    """The executor's plan moved by d m along x (as a re-expression in a moved frame would)."""
    return Plan(transform_segments(ex_.plan.segments, Frame2D(), Frame2D(tx=d)), ex_.plan.t0,
                ex_.plan.speed)


exP, stP, clockP, grP = new_exec("C2")
fly(exP, stP, clockP, grP, 20.0, dt=1.0)
cP = clockP["tau"] + 15.0
nP = exP.counts["requests"]
exP._set("requesting")
sentP = exP._send("replace", cP, shifted_tail(exP, 3.0), clockP["now"])
check("a tail more than area_check_slack_m into the margin band is never sent -> failed",
      sentP is False and exP.state == "failed" and
      exP.reason.startswith("request_invalid:outside_area") and exP.counts["requests"] == nP,
      exP.reason)
exQ, stQ, clockQ, grQ = new_exec("C2")
fly(exQ, stQ, clockQ, grQ, 20.0, dt=1.0)
nQ = exQ.counts["requests"]
exQ._set("requesting")
sentQ = exQ._send("replace", clockQ["tau"] + 15.0, shifted_tail(exQ, 0.3), clockQ["now"])
check("0.3 m into the band passes the area check; the 0.3 m jump at c fails the mirror's "
      "continuity check, unsent", sentQ is False and exQ.state == "failed" and
      exQ.reason.startswith("request_invalid:discontinuous_position") and
      exQ.counts["requests"] == nQ, exQ.reason)
check("the plans the executor sends lie in F_margin itself (gate tolerance 0)",
      plan_in_area(exQ.plan, exQ.setup.area, exQ.cfg.margin_m, 0.05)[0])


class NotReadyClient(StubClient):
    def ready(self):
        return False


mR = parse_mission(MISSION)
stR = ControlStub((10.0, 10.0, 0.0))
exR = Executor(mR.config, NotReadyClient(stR, {"tau": 0.0}))
exR.start(mR, 0.0)
grR = make_grid([])
for nowR in (0.0, 30.0, 61.0):
    grR["grid_seq"] += 1
    exR.on_grid(grR, nowR)
    exR.on_progress(progress_of(stR, 0.0), nowR)
    exR.tick(nowR)
    if nowR == 30.0:
        stR_wait = exR.state
check("no update_path server: no first request; start_timeout names the service",
      stR_wait == "waiting" and exR.state == "failed" and "update_path service" in exR.reason
      and exR.counts["requests"] == 0, exR.reason)
stH = ControlStub((10.0, 10.0, 0.0))
clH = StubClient(stH, {"tau": 0.0})
clH.auto = False
mH = parse_mission(MISSION)
exH = Executor(mH.config, clH)
exH.start(mH, 0.0)
grH = make_grid([])
exH.on_grid(grH, 0.0)
exH.on_progress(progress_of(stH, 0.0), 0.0)
exH.tick(0.0)
reqH, cbH = clH.held[0]
cbH(stH.handle(reqH["id"], reqH["change_point"], reqH["t"], reqH["x"], reqH["y"], reqH["yaw"],
               0.0), 0.75)
check("an answer is handled at its arrival time (logged as `answered`), not the send time",
      exH.state == "running" and exH.requests[-1]["sent"] == 0.0 and
      exH.requests[-1]["answered"] == 0.75)

