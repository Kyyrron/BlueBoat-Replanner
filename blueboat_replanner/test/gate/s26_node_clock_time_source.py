"""Gate section [26]: the node's clock, parameter `time_source` (wall | ros; MATH §11.3; RQ-1).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier
sections are visible (the fake rclpy of [24], the Control stub and `progress_of` of [21] / [23], the grid
fixtures of [11])."""
# flake8: noqa

# =========================================================================== [26]
print("[26] node clock: time_source wall (= the node of 2f84a46, byte for byte) | ros (simulated seconds)")
import importlib.util  # noqa: E402
import json  # noqa: E402
import subprocess  # noqa: E402
import time  # noqa: E402

from blueboat_replanner.ros.executor import Executor  # noqa: E402

REF26 = "2f84a46"      # the last node before `time_source` (2026-11-13): wall time everywhere
OUT26 = {}


def _ref_node_module():
    """replanner_node.py at REF26 as a module of this package (relative imports resolve here), or None."""
    r = subprocess.run(["git", "-C", str(PKG_ROOT.parent), "show",
                        f"{REF26}:blueboat_replanner/blueboat_replanner/ros/replanner_node.py"],
                       capture_output=True, text=True)
    if r.returncode != 0 or not r.stdout:
        return None
    spec = importlib.util.spec_from_loader("blueboat_replanner.ros._replanner_node_ref", loader=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = "blueboat_replanner.ros"
    # the same file location, so default_c2_freeze() resolves the same c2_tuned.yaml
    mod.__file__ = str(PKG_ROOT / "blueboat_replanner" / "ros" / "replanner_node.py")
    exec(compile(r.stdout, f"<replanner_node.py@{REF26}>", "exec"), mod.__dict__)
    return mod


def _node_run(mod, td, overrides, wall0=1000.0, dwall=1.0, sim0=None, dsim=1.0):
    """[24]'s C3 run through `mod`'s ReplannerNode on fake rclpy, the wall clock, the decision timer and
    the folder stamp fixed. `sim0` set: the fake node clock starts there and advances `dsim` per step
    (the wall clock `dwall`). Returns (node, run folder, {relative file: text}, published payloads,
    {"sim_feed": node-clock seconds of the last input fed})."""
    import yaml  # noqa: E402 - io/ dependency, used here only to write the mission file
    mfile = os.path.join(td, "mission.yaml")
    with open(mfile, "w") as fh:
        yaml.safe_dump({"mission": {**MISSION["mission"], "name": "node26", "condition": "C3"}}, fh)
    _FakeNode.OVERRIDES = {"mission_file": mfile, "data_dir": td, **overrides}
    wall, perf = {"t": wall0}, {"t": 0.0}
    _FakeNode.CLOCK["t"] = sim0 if sim0 is not None else 0.0

    def _perf():
        perf["t"] += 0.001
        return perf["t"]

    mod.time = _NS(time=lambda: wall["t"], strftime=lambda *a_, **k_: "2026_01_01-00_00_00",
                   perf_counter=_perf)
    node = mod.ReplannerNode()
    stub26 = ControlStub((10.0, 10.0, 0.0))
    cli26 = node.clients["/blueboat/update_path"]
    tau26 = {"v": 0.0}
    gr26 = make_grid([contact(1, 30, 30), contact(2, 90, 40, first_sector=3)])
    tick_cb = [cb for p, cb in node.timers if p < 1.0][0]
    status_cb = [cb for p, cb in node.timers if p == 1.0][0]
    info = {"sim_feed": None}

    def feed():
        gr26["grid_seq"] += 1
        node.subs["/belief_map/belief_grid"][1](gr26)
        node.subs["/blueboat/path_progress"][1](
            _FakeString(data=json.dumps(progress_of(stub26, tau26["v"]), default=float)))
        od = _FakeOdom()
        x_, y_, w_ = stub26.mirror.pose(tau26["v"]) if stub26.mirror else stub26.hold
        od.pose.pose.position.x, od.pose.pose.position.y = x_, y_
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(w_ / 2), math.cos(w_ / 2)
        node.subs["/blueboat/odom"][1](od)
        info["sim_feed"] = _FakeNode.CLOCK["t"]

    def answer_calls():
        while cli26.calls:
            req, fut = cli26.calls.pop(0)
            a_ = stub26.handle(req.id, req.change_point, list(req.t), list(req.x), list(req.y),
                               list(req.yaw), tau26["v"])
            fut.set_result(_NS(accepted=a_["accepted"], reason=a_.get("reason", "ok"),
                               message=a_.get("message", ""), active_id=a_.get("active_id", ""),
                               tau=a_.get("tau", tau26["v"]),
                               min_change_point=a_.get("min_change_point", float("nan"))))
            ev = {"schema": "blueboat_path_update_event/1", "id": req.id,
                  "accepted": a_["accepted"], "reason": a_.get("reason", "ok")}
            node.subs["/blueboat/path_update_event"][1](_FakeString(data=json.dumps(ev)))

    feed()
    node.srvs["/replanner/start"][1](_FakeTrigger.Request(), _FakeTrigger.Response())
    for step in range(20000):
        feed()
        tick_cb()
        answer_calls()
        if step % 5 == 0:
            status_cb()
        if node.ex.state in ("done", "failed", "disarmed"):
            break
        tau26["v"] += 1.0 if node.ex.state != "waiting" else 0.0
        wall["t"] += dwall
        _FakeNode.CLOCK["t"] += dsim if sim0 is not None else 0.0
    tick_cb()
    status_cb()
    run_dir = node.rec.dir
    node.rec.close()
    files = {}
    for root, _dirs, names in os.walk(run_dir):
        for n in sorted(names):
            p = os.path.join(root, n)
            with open(p) as fh:
                files[os.path.relpath(p, run_dir)] = fh.read().replace(td, "<ROOT>")
    pubs = {"decision": [m.data for m in node.pubs["/replanner/decision"].sent],
            "status": [m.data.replace(td, "<ROOT>") for m in node.pubs["/replanner/status"].sent],
            "plan": [repr([(p.pose.position.x, p.pose.position.y, p.header.stamp.sec,
                            p.header.stamp.nanosec, p.pose.orientation.z, p.pose.orientation.w)
                           for p in m.poses]) for m in node.pubs["/replanner/plan"].sent]}
    return node, run_dir, files, pubs, info


try:
    rn26 = _install_fake_ros()
    ref26 = _ref_node_module()
    with tempfile.TemporaryDirectory() as td_ref, tempfile.TemporaryDirectory() as td_w, \
            tempfile.TemporaryDirectory() as td_d, tempfile.TemporaryDirectory() as td_r:
        # ---- 1. wall = the node of REF26, byte for byte (records and published payloads)
        new_w = _node_run(rn26, td_w, {"time_source": "wall"})
        new_d = _node_run(rn26, td_d, {})
        if ref26 is not None:
            old = _node_run(ref26, td_ref, {})
            diff = sorted(k for k in set(old[2]) | set(new_w[2]) if old[2].get(k) != new_w[2].get(k))
            OUT26["wall_files"] = (not diff and len(old[2]) >= 9, f"differ: {diff[:4]}")
            OUT26["wall_pubs"] = (old[3] == new_w[3], "published payloads differ")
            OUT26["wall_run"] = (old[0].ex.state == "done" and old[0].ex.counts["decisions"] == 1 and
                                 old[0].ex.counts["accepted"] == 2, f"{old[0].ex.counts}")
        else:
            OUT26["wall_skip"] = True
        OUT26["default_is_wall"] = (new_d[2] == new_w[2] and new_d[3] == new_w[3] and
                                    new_d[0].time_source == "wall", "default differs from wall")
        OUT26["wall_no_key"] = (not any('"time_source"' in t for t in new_w[2].values()),
                                "a wall record names a time_source")

        # ---- 2. ros: the executor is fed the node clock; every record names it
        sim_start = 50.0
        node_r, dir_r, files_r, pubs_r, info_r = _node_run(
            rn26, td_r, {"time_source": "ros", "use_sim_time": True}, wall0=1000.0, dwall=10.0,
            sim0=sim_start, dsim=1.0)
        ex_r = node_r.ex
        sim_end = _FakeNode.CLOCK["t"]
        reqs_r = [json.loads(ln) for ln in files_r["requests.jsonl"].splitlines()]
        man_r = json.loads(files_r["manifest.json"])
        OUT26["ros_run"] = (ex_r.state == "done" and ex_r.counts["decisions"] == 1 and
                            ex_r.counts["accepted"] == 2, f"{ex_r.state} {ex_r.reason} {ex_r.counts}")
        OUT26["ros_arrivals"] = (
            abs(ex_r.progress_at - info_r["sim_feed"]) < 1e-9 and
            abs(ex_r.grid_at - info_r["sim_feed"]) < 1e-9 and
            abs(ex_r.odom_at - info_r["sim_feed"]) < 1e-9 and
            all(sim_start <= r["sent"] <= sim_end and sim_start <= r["answered"] <= sim_end
                for r in reqs_r), f"progress_at {ex_r.progress_at} vs {info_r['sim_feed']}; "
                                  f"requests {[(r['sent'], r.get('answered')) for r in reqs_r]}")
        OUT26["ros_manifest"] = (man_r.get("time_source") == "ros" and man_r.get("started_wall") == 1000.0
                                 and man_r.get("started_clock") == sim_start and
                                 man_r.get("use_sim_time") is True, f"{ {k: man_r.get(k) for k in ('time_source', 'started_wall', 'started_clock', 'use_sim_time')} }")
        lines_r = [json.loads(ln) for k in ("decisions.jsonl", "requests.jsonl", "status.jsonl")
                   for ln in files_r[k].splitlines()] + [json.loads(files_r["end.json"])]
        pub_r = [json.loads(s) for s in pubs_r["decision"] + pubs_r["status"]]
        OUT26["ros_tagged"] = (all(d.get("time_source") == "ros" for d in lines_r + pub_r) and
                               len(lines_r) > 5, f"{len(lines_r)} lines")
        st_r = json.loads(files_r["end.json"])
        OUT26["ros_times"] = (sim_start <= st_r["wall_time"] <= sim_end + 1.0,
                              f"end.json wall_time {st_r['wall_time']} not on the node clock")
        _FakeNode.OVERRIDES = {"time_source": "sim"}
        OUT26["bad_value"] = raises(lambda: rn26.ReplannerNode(), SystemExit, "time_source")
except Exception as exc:  # noqa: BLE001 - reported as a failure, with the traceback
    import traceback
    traceback.print_exc()
    OUT26["exception"] = (False, repr(exc))
finally:
    _restore_ros()
    _FakeNode.OVERRIDES = {}
    _FakeNode.CLOCK["t"] = 0.0


# ---- 3. the executor under a simulated clock: 0.1x real time and a 16 s render hold
def _next_trigger(ex):
    """The path time at which the next decision point's trigger fires (`Executor._due`, without the
    test against tau)."""
    lead = ex.progress.get("lead_min") if ex.progress else None
    if lead is None or ex.setup is None or ex.plan is None:
        return None
    tau = ex._tau()
    for t_dp, idx in ex.setup.policy.decision_points(ex.plan):
        if t_dp <= tau or (round(t_dp, 6), idx) in ex.done_dp:
            continue
        return t_dp - (lead + ex.cfg.t_decide_s + ex.cfg.latency_margin_s)
    return None


def _hold_run(clock, hold_s=16.0, rate=0.1):
    """A C3 mission through the executor, fed as in Gazebo with the ray-cast sonar:
    - Control's progress every 0.1 simulated s;
    - the node's tick on its timer, which under use_sim_time runs on /clock: every 0.2 simulated s,
      half a progress period after the progress;
    - Belief's grid once a wall second (its steady clock).
    The run is at `rate` x real time around the start and the decision, at real time in between.
    When Control's progress has just crossed the decision trigger, Gazebo is paused for `hold_s`
    wall seconds (a slow ping renders): no progress, no tick, Belief repeats its grid (same
    grid_seq). The executor is given `clock` ('sim' or 'wall') seconds. Returns (executor, info)."""
    mission = parse_mission({"mission": {**MISSION["mission"], "condition": "C3"}})
    stub = ControlStub((10.0, 10.0, 0.0))
    clk = {"tau": 0.0, "now": 0.0}
    ex = Executor(mission.config, StubClient(stub, clk))
    gr = make_grid([contact(1, 30, 30), contact(2, 90, 40, first_sector=3)])
    w = {"wall": 0.0, "sim": 0.0, "grid_wall": -1e9}
    info = {"held": False, "hold_fresh_sim": None, "start_timeout": False}

    def now():
        return w["sim"] if clock == "sim" else w["wall"]

    def grid(new=True):
        if new:
            gr["grid_seq"] += 1
            w["grid_wall"] = w["wall"]
        ex.on_grid(gr, now())

    def progress():
        clk["tau"] = w["sim"]
        ex.on_progress(progress_of(stub, clk["tau"]), now())

    def advance(dsim, r):
        w["sim"] += dsim
        w["wall"] += dsim / r
        if w["wall"] - w["grid_wall"] >= 1.0:          # Belief: a new grid every wall second
            grid()

    grid()
    progress()
    ex.start(mission, now())
    for _ in range(20000):
        if ex.state in ("done", "failed", "disarmed"):
            break
        trig = _next_trigger(ex) if ex.state == "running" else None
        decided = ex.counts["decisions"] >= 1 and ex.pending is None
        slow = (not decided) and (w["sim"] < 10.0 or (trig is not None and w["sim"] >= trig - 3.0))
        r, dt = (rate, 0.1) if slow else (1.0, 1.0)
        advance(dt, r)
        progress()                                     # Control: its 10 Hz progress (sim time)
        if slow and not info["held"] and ex.decision_due():
            for _k in range(int(hold_s)):              # the render hold: wall time only
                w["wall"] += 1.0
                grid(new=False)                        # Belief repeats its last grid (same seq)
            info["held"] = True
            info["hold_fresh_sim"] = (ex._fresh_progress(now(), ex.cfg.progress_send_max_age_s),
                                      ex._fresh_grid(now()))
        else:
            advance(dt / 2.0, r)                       # the tick half a progress period later
        clk["now"] = now()
        ex.tick(now())
        if ex.reason.startswith("start_timeout"):
            info["start_timeout"] = True
    return ex, info


ex_s, inf_s = _hold_run("sim")
ex_w, inf_w = _hold_run("wall")
out_s = [r["epoch"].get("outcome") for r in ex_s.records]
out_w = [r["epoch"].get("outcome") for r in ex_w.records]
ages_s = [(r["inputs"]["progress_age_s"], r["inputs"]["grid_age_s"]) for r in ex_s.records]
cfg26 = ex_s.cfg

# ---- 4. and a simulated clock that runs on without messages makes the inputs stale
t_p, t_g = ex_s.progress_at, ex_s.grid_at
stale = (ex_s._fresh_progress(t_p + 0.9) and not ex_s._fresh_progress(t_p + 1.5) and
         ex_s._fresh_grid(t_g + 4.9) and not ex_s._fresh_grid(t_g + 5.5) and
         "progress" in ex_s._missing(t_p + 1.5) and "belief grid" in ex_s._missing(t_g + 5.5) and
         ex_s.status(t_p + 1.5)["inputs"]["progress_age_s"] > cfg26.progress_max_age_s)


def _ok(v):
    return v[0] if isinstance(v, tuple) else bool(v)


def _why(v):
    return v[1] if isinstance(v, tuple) else ""


if OUT26.get("wall_skip"):
    skip("time_source wall: records and payloads byte-identical to the node of " + REF26,
         f"git show {REF26} not available")
else:
    for key, label in [("wall_files", f"time_source wall: every record file byte-identical to the node of "
                                      f"{REF26} (manifest, mission, decisions, requests, status, events, "
                                      f"belief, end, plans) [fake rclpy]"),
                       ("wall_pubs", f"time_source wall: /replanner/decision, /status, /plan payloads "
                                     f"identical to {REF26} [fake rclpy]"),
                       ("wall_run", f"the node of {REF26} ran the C3 scenario (1 decision, 2 accepted)")]:
        check(label, _ok(OUT26.get(key, False)), _why(OUT26.get(key, "")))
for key, label in [("default_is_wall", "time_source default = wall (same records, same payloads)"),
                   ("wall_no_key", "time_source wall: no record carries a time_source key"),
                   ("ros_run", "time_source ros: C3 run through the node, 1 decision, 2 accepted, done"),
                   ("ros_arrivals", "time_source ros: arrival times of grid, progress, odometry and "
                                    "answers are the node clock's seconds, not wall seconds"),
                   ("ros_manifest", "time_source ros: manifest names time_source and use_sim_time, "
                                    "started_clock = node clock, started_wall = wall"),
                   ("ros_tagged", "time_source ros: every decision, request, status and end record, "
                                  "written and published, carries time_source ros"),
                   ("ros_times", "time_source ros: end.json times are node-clock seconds"),
                   ("bad_value", "an unknown time_source is refused at start-up")]:
    check(f"{label}", _ok(OUT26.get(key, False)), _why(OUT26.get(key, "")))
if "exception" in OUT26:
    check("section [26] node runs raised", False, _why(OUT26["exception"]))
check("fake modules removed afterwards [26]", all(
    sys.modules.get(n) is _saved_mods[n] for n in _FAKE_NAMES))
check("sim clock, 0.1x and a 16 s render hold at the decision: progress and belief fresh, "
      "decision sent, C3 done",
      inf_s["held"] and inf_s["hold_fresh_sim"] == (True, True) and not inf_s["start_timeout"] and
      ex_s.state == "done" and ex_s.counts["accepted"] == 2 and
      not any(o in ("stale_belief", "missed_lead") for o in out_s) and "sent" in out_s,
      f"{ex_s.state} {ex_s.reason} {ex_s.counts} {out_s} {inf_s}")
check("sim clock: every decision's progress age within the pre-send limit and grid age within "
      "the belief limit (simulated s)",
      bool(ages_s) and all(p <= cfg26.progress_send_max_age_s and g is not None and
                           g <= cfg26.belief_max_age_s for p, g in ages_s), f"{ages_s}")
check("the same schedule on the wall clock declares the inputs stale (the check discriminates)",
      inf_w["held"] and inf_w["hold_fresh_sim"] != (True, True) and
      any(o in ("stale_belief", "missed_lead") for o in out_w) and ex_w.counts["accepted"] < 2,
      f"{out_w} {ex_w.counts} {inf_w}")
check("sim clock advancing without messages: progress stale after 1.5 s, belief after 5.5 s",
      stale)
