"""ROS runtime check of the Replanner node beside BlueBoat-Belief's node, without a simulator
(dev-only, not installed). It must only ever run on an ISOLATED ROS domain.

What is real: Belief's installed `belief_node`; this package's installed `replanner_node`; DDS;
every message and service type (`BeliefGrid`, `UpdatePath`, `ProcessedSSSPing`, ...).

What is made up, by this script:
- Belief's inputs: sonar pings, odometry, GPS fixes and two detections, as real messages with
  the upstream publishers' QoS (in [R6]-[R7] the detections come from this package's emulator
  node instead, which looks at a made-up world file);
- Control: `/blueboat/path_progress` and the `/blueboat/update_path` service, answered by the
  dev loop's Control test double (`eval/control_stub.ControlStub`) behind the real service type.
  The path parameter runs faster than real time (`--speedup`).
So this is NOT Control running, NOT a simulation and NOT a sonar: it shows that the node, its
QoS, its parameters, its service client, its records and its reading of Belief's live grid
work under rclpy.

Checks:
  [R1] both nodes run; the Replanner's endpoints and QoS; its BeliefGrid subscription matches
       Belief's publisher (RELIABLE + TRANSIENT_LOCAL on both ends, CM-4)
  [R2] the first grids arrive while Belief's alignment is not valid: no plan; once it is valid
       the grid is usable; the transform is not the identity
  [R3] a C3 mission: Belief's area made from the mission file by Belief's own tool; the first
       path through the real service type; two contacts; one decision, a REPLACE accepted; the
       evidence horizon read from the grid stamp; latched plan and status; done; run records
  [R4] Belief restarted: grid_seq starts again at 1; counted, logged, never dropped silently
  [R5] SIGINT: exit code 0, end.json written
  [R6] the detection emulator node (`eval/emulator_node.py`) beside Belief, on a second
       bring-up where this package's two nodes are started by their LAUNCH FILES and Belief
       runs with its DEFAULT coverage rule (a ping is searched only where a detector
       reported on it): one array per seabed image, empty ones included; Belief credits the
       search between them; its contacts lie on the made-up objects (so the frames agree)
  [R7] a C4 mission on that gated grid, at REAL speed (`--c4-speedup 1`, about 25 minutes):
       decisions under rclpy, none refused, their labels and their measured decision time (a
       small area and no simulator running: not the timing of a full mission)

Run it from `blueboat_replanner/` with the system python, in a shell where `ROS_DOMAIN_ID` is
set and is not 0 and where Belief's and this package's install spaces are sourced:

    /usr/bin/python3 -B -m test.ros_graph_check [--work DIR] [--speedup 10] [--c4-speedup 1]

Exit codes: 0 pass, 1 a check failed, 3 the environment is not isolated.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

N_CHECKS = 0
FAILED = []
# the required keys of a decision record (docs/topics.md §5.1)
DECISION_KEYS = {"schema", "seq", "condition", "tau", "epoch", "inputs", "budget", "chosen",
                 "request"}
# a 70 x 45 m area about (34.5, 133.5): small, so that the mission is short
AREA = [[34.5, 133.5], [34.5, 133.500763], [34.500404, 133.500763], [34.500404, 133.5]]
W_OFFSET = (5.0, -3.0)          # map position of the world (odom) origin: a non-identity transform
# the made-up world of [R6]: (object type, x, y, yaw) in Belief's map frame. Four types have an
# aspect family in the emulator's world model; "rock" has none and is clutter.
OBJECTS_M = [("concrete_block", 15.0, 12.0, 0.3), ("pipe_pvc", 30.0, 30.0, 1.1),
             ("mesh_crate", 45.0, 15.0, 0.7), ("tire_car", 55.0, 32.0, 0.0),
             ("concrete_block", 22.0, 34.0, 2.0), ("tire_car", 38.0, 10.0, 0.0),
             ("rock", 50.0, 22.0, 0.0), ("rock", 12.0, 26.0, 0.0)]


def check(name: str, cond, detail: str = "") -> bool:
    global N_CHECKS
    N_CHECKS += 1
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""),
          flush=True)
    if not cond:
        FAILED.append(name)
    return bool(cond)


def state_sequence(run_dir: Path) -> list:
    """The states of `status.jsonl`, repeats removed."""
    seq = []
    f = run_dir / "status.jsonl"
    for ln in (f.read_text().splitlines() if f.is_file() else []):
        st = json.loads(ln).get("state")
        if not seq or seq[-1] != st:
            seq.append(st)
    return seq


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--work", default="", help="work directory (default: a temporary one)")
    ap.add_argument("--speedup", type=float, default=10.0,
                    help="path seconds per wall second of the made-up Control in [R3]")
    ap.add_argument("--c4-speedup", type=float, default=1.0,
                    help="the same for the C4 mission of [R7]. Keep 1: the node reserves "
                         "t_decide_s REAL seconds for a decision, so a faster path clock makes "
                         "Control refuse every decision that takes longer than about 4 s / "
                         "speed-up (lead_too_short); such a run only shows the wiring")
    args = ap.parse_args(argv)
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not domain or domain == "0":
        print("refused: ROS_DOMAIN_ID must be set and must not be 0 (never on a live graph)")
        return 3

    import rclpy
    import yaml
    from ament_index_python.packages import get_package_prefix
    from blueboat_belief_interfaces.msg import BeliefGrid
    from blueboat_interfaces.msg import ProcessedSSSPing
    from blueboat_interfaces.srv import UpdatePath
    from nav_msgs.msg import Odometry, Path as PathMsg
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    from blueboat_replanner.core.geo import local_en_to_latlon
    from blueboat_replanner.eval.control_stub import ControlStub
    from blueboat_replanner.eval.emulator import side_offsets

    work = Path(args.work) if args.work else Path(tempfile.mkdtemp(prefix="replanner_ros_check_"))
    work.mkdir(parents=True, exist_ok=True)
    print(f"ROS runtime check: domain {domain}, work dir {work}, speed-up {args.speedup:g}")

    # ------------------------------------------------------------------ files
    mission_file = work / "mission.yaml"
    mission_file.write_text(yaml.safe_dump({"mission": {
        "name": "roscheck", "condition": "C3", "areas": [AREA],
        "config": {"planning_depth_m": 2.0}}}))
    bel_prefix = Path(get_package_prefix("blueboat_belief"))
    rep_prefix = Path(get_package_prefix("blueboat_replanner"))
    area_file = work / "area.yaml"
    if not area_file.exists():
        mk = subprocess.run([str(bel_prefix / "lib" / "blueboat_belief" /
                                 "blueboat-belief-make-area"), "--out", str(area_file),
                             "--from-mission", str(mission_file)], capture_output=True, text=True)
        if mk.returncode != 0:
            print(f"Belief's make-area tool failed: {mk.stdout}{mk.stderr}")
            return 1
    area_d = yaml.safe_load(area_file.read_text())
    lat0, lon0 = float(area_d["geo_anchor"]["lat0"]), float(area_d["geo_anchor"]["lon0"])
    belief_params = work / "belief_params.yaml"
    belief_params.write_text(yaml.safe_dump({"belief_map": {"ros__parameters": {
        "area.file": str(area_file), "io.data_dir": str(work), "snapshot.period_s": 3600.0,
        # every ping counts as searched at once: no detector runs here (labelled "ungated")
        "coverage.require_detector": False}}}))

    # ------------------------------------------------------------------ the graph
    rclpy.init()
    node = rclpy.create_node("replanner_ros_check")

    def spin(dt=0.0):
        rclpy.spin_once(node, timeout_sec=dt)

    def spin_until(cond, timeout: float, tick=None) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if tick is not None:
                tick()
            spin(0.02)
            if cond():
                return True
        return bool(cond())

    spin_until(lambda: False, 1.0)
    others = [n for n in node.get_node_names() if n != "replanner_ros_check"]
    if others or node.get_publishers_info_by_topic("/clock"):
        print(f"refused: this graph is not empty ({others or '/clock has a publisher'})")
        rclpy.shutdown()
        return 3

    def start(exe: Path, log_name: str, *ros_args):
        log = open(work / log_name, "a", encoding="utf-8")
        return subprocess.Popen([str(exe), "--ros-args", *ros_args], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)

    ros2_exe = shutil.which("ros2")

    def start_launch(log_name: str, launch_file: str, *launch_args):
        """`ros2 launch blueboat_replanner <file> k:=v ...`: the installed launch file."""
        log = open(work / log_name, "a", encoding="utf-8")
        return subprocess.Popen([ros2_exe, "launch", "blueboat_replanner", launch_file,
                                 *launch_args], stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)

    def stop_launch(proc, log_name: str, node_exe: str) -> tuple:
        """SIGINT to the launch process only (it passes it on to its node once); returns
        (clean, detail): the node "finished cleanly" by launch's own account, no traceback."""
        os.kill(proc.pid, signal.SIGINT)
        try:
            code = proc.wait(timeout=30.0)
        except subprocess.TimeoutExpired:
            code = None
        log = (work / log_name).read_text()
        clean = any("process has finished cleanly" in ln and node_exe in ln
                    for ln in log.splitlines())
        return (code is not None and clean and "Traceback" not in log and
                "process has died" not in log), f"launch exit code {code}, node clean: {clean}"

    def start_belief():
        return start(bel_prefix / "lib" / "blueboat_belief" / "belief_node", "belief_node.log",
                     "--params-file", str(belief_params), "-p", "use_sim_time:=false")

    def now_s() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def stamp_of(t: float, st) -> None:
        st.sec = int(t)
        st.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000

    # ---- what Belief reads (upstream QoS: Belief topics.md §1)
    be10 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,
                      depth=10)
    pub_ping = node.create_publisher(ProcessedSSSPing, "/sss_processor/processed", be10)
    pub_det = node.create_publisher(Detection2DArray, "/sss_ai/detections", 10)
    pub_odom = node.create_publisher(Odometry, "/blueboat/odom", 10)
    pub_fix = node.create_publisher(NavSatFix, "/mavros/global_position/global",
                                    qos_profile_sensor_data)
    # ---- Control, made up: progress + the real service type
    pub_prog = node.create_publisher(String, "/blueboat/path_progress", 10)
    hold = (10.0 - W_OFFSET[0], 10.0 - W_OFFSET[1], 0.3)          # world frame
    stub = ControlStub(hold)
    ctl = {"tau": 0.0, "wall": time.monotonic(), "calls": 0, "accepted": 0, "ids": [],
           "speedup": args.speedup}

    def on_update(req, resp):
        a = stub.handle(req.id, req.change_point, np.asarray(req.t, float),
                        np.asarray(req.x, float), np.asarray(req.y, float),
                        np.asarray(req.yaw, float), ctl["tau"])
        ctl["calls"] += 1
        ctl["accepted"] += bool(a["accepted"])
        ctl["ids"].append((req.id, a["reason"], len(req.t), req.frame))
        resp.accepted, resp.reason = bool(a["accepted"]), str(a.get("reason", ""))
        resp.message = str(a.get("message", ""))
        resp.active_id = str(a.get("active_id") or "")
        resp.tau = float(a.get("tau", ctl["tau"]))
        mcp = a.get("min_change_point")
        resp.min_change_point = float("nan") if mcp is None else float(mcp)
        return resp

    node.create_service(UpdatePath, "/blueboat/update_path", on_update)

    # ---- what the Replanner publishes
    latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
    got = {"status": [], "decisions": [], "plans": 0, "grids": [], "grid_msg": None,
           "arrays": []}
    node.create_subscription(String, "/replanner/status",
                             lambda m: got["status"].append(json.loads(m.data)), latched)
    node.create_subscription(String, "/replanner/decision",
                             lambda m: got["decisions"].append(json.loads(m.data)),
                             QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                        history=HistoryPolicy.KEEP_LAST, depth=50))
    node.create_subscription(PathMsg, "/replanner/plan",
                             lambda m: got.__setitem__("plans", got["plans"] + 1), latched)
    def on_grid(m):
        got["grid_msg"] = m
        got["grids"].append((int(m.grid_seq), bool(m.odom_to_map_valid), float(m.odom_to_map_x),
                             float(m.odom_to_map_y), len(m.contacts)))

    node.create_subscription(BeliefGrid, "/belief_map/belief_grid", on_grid, latched)
    node.create_subscription(
        Detection2DArray, "/sss_ai/detections",
        lambda m: got["arrays"].append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                                        [(d.bbox.center.position.x, d.bbox.center.position.y,
                                          d.results[0].hypothesis.class_id, d.id)
                                         for d in m.detections])),
        QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                   depth=500))

    off = side_offsets(1.78, 20.0, 0.285)[::8].astype(np.float32)        # a thinned swath
    feed = {"fixes": False, "n_ping": 0, "last": 0.0, "dets": [], "dist": 0.0, "xy": None}

    def boat():
        if stub.mirror:
            return stub.mirror.pose(ctl["tau"])
        return hold

    def tick():
        """10 Hz: advance the made-up Control and publish Belief's inputs and the progress."""
        t_wall = time.monotonic()
        if t_wall - feed["last"] < 0.1:
            return
        dt = t_wall - feed["last"] if feed["last"] else 0.1
        feed["last"] = t_wall
        if stub.mirror and not stub.awaiting and stub.state == "following":
            ctl["tau"] = min(ctl["tau"] + ctl["speedup"] * dt, stub.mirror.t_end + 5.0)
        x, y, psi = boat()
        t = now_s()
        od = Odometry()
        stamp_of(t, od.header.stamp)
        od.header.frame_id = "world"
        od.pose.pose.position.x, od.pose.pose.position.y = float(x), float(y)
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(psi / 2), math.cos(psi / 2)
        pub_odom.publish(od)
        if feed["fixes"]:
            lat, lon = local_en_to_latlon(x + W_OFFSET[0], y + W_OFFSET[1], lat0, lon0)
            fx = NavSatFix()
            stamp_of(t, fx.header.stamp)
            fx.latitude, fx.longitude, fx.status.status = float(lat), float(lon), 0
            pub_fix.publish(fx)
            feed["n_ping"] += 1
            if feed["xy"] is not None:
                feed["dist"] += math.hypot(x - feed["xy"][0], y - feed["xy"][1])
            feed["xy"] = (x, y)
            pg = ProcessedSSSPing()
            stamp_of(t, pg.port_stamp)
            stamp_of(t, pg.starboard_stamp)
            pg.port_ping_number = pg.starboard_ping_number = feed["n_ping"]
            pg.robot_x, pg.robot_y = float(x), float(y)
            pg.robot_orientation.z, pg.robot_orientation.w = math.sin(psi / 2), math.cos(psi / 2)
            pg.water_depth = 2.0
            pg.port_y = off.tolist()
            pg.starboard_y = (-off).tolist()
            pg.port_intensity_db = [-20.0] * len(off)
            pg.starboard_intensity_db = [-20.0] * len(off)
            pub_ping.publish(pg)
            for d in feed["dets"]:
                if not d["sent"] and ctl["tau"] >= d["tau"]:
                    d["sent"] = True
                    arr = Detection2DArray()
                    arr.header.frame_id = "odom"
                    stamp_of(t, arr.header.stamp)
                    det = Detection2D()
                    stamp_of(t, det.header.stamp)
                    det.id = d["id"]
                    px, py = x + 9.0 * math.cos(psi + math.pi / 2), y + 9.0 * math.sin(psi + math.pi / 2)
                    det.bbox.center.position.x, det.bbox.center.position.y = float(px), float(py)
                    det.bbox.center.theta = float(psi)
                    det.bbox.size_x = det.bbox.size_y = 1.0
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id, hyp.hypothesis.score = "unclassified", 0.7
                    hyp.pose.pose.position.x, hyp.pose.pose.position.y = float(px), float(py)
                    cov = [0.0] * 36
                    cov[0] = cov[7] = 1.25
                    hyp.pose.covariance = cov
                    det.results.append(hyp)
                    arr.detections.append(det)
                    pub_det.publish(arr)
                    d["xy_world"] = (px, py)
        pr = stub.progress(ctl["tau"])
        pr.update(ros_time=t, wall_time=time.time(), run=1, refusal_now="",
                  path_source="replanner", lead_min=stub.lead_min(ctl["tau"]))
        pub_prog.publish(String(data=json.dumps(pr, default=float)))

    def status():
        return got["status"][-1] if got["status"] else {}

    procs = {"belief": start_belief(),
             "replanner": start(rep_prefix / "lib" / "blueboat_replanner" / "replanner_node",
                                "replanner_node.log", "-p", f"mission_file:={mission_file}",
                                "-p", f"data_dir:={work}", "-p", "use_sim_time:=false")}
    rc = 1
    try:
        # ============================================================== [R1]
        print("[R1] both nodes, endpoints and QoS")
        up = spin_until(lambda: {"belief_map", "replanner"} <= set(node.get_node_names()) and
                        bool(status()), 30.0, tick)
        alive = all(p.poll() is None for p in procs.values())
        check("belief_map and replanner are running", up and alive,
              f"nodes {sorted(node.get_node_names())}; exit codes "
              f"{ {k: p.poll() for k, p in procs.items()} }; logs in {work}")
        if not (up and alive):
            return 1
        sub = [i for i in node.get_subscriptions_info_by_topic("/belief_map/belief_grid")
               if i.node_name == "replanner"]
        pub = [i for i in node.get_publishers_info_by_topic("/belief_map/belief_grid")
               if i.node_name == "belief_map"]
        check("BeliefGrid: RELIABLE + TRANSIENT_LOCAL on both ends (replanner subscribes, "
              "belief_map publishes)", len(sub) == 1 and len(pub) == 1 and all(
                  i.qos_profile.reliability == ReliabilityPolicy.RELIABLE and
                  i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                  for i in sub + pub))
        names = dict(node.get_topic_names_and_types())
        for t, typ in (("/replanner/plan", "nav_msgs/msg/Path"),
                       ("/replanner/decision", "std_msgs/msg/String"),
                       ("/replanner/status", "std_msgs/msg/String")):
            pi = [i for i in node.get_publishers_info_by_topic(t) if i.node_name == "replanner"]
            check(f"{t} [{typ}] published by replanner", names.get(t) == [typ] and len(pi) == 1,
                  str(names.get(t)))
        for t in ("/blueboat/path_progress", "/blueboat/path_update_event", "/blueboat/odom"):
            si = [i for i in node.get_subscriptions_info_by_topic(t) if i.node_name == "replanner"]
            check(f"replanner subscribes {t}", len(si) == 1)
        svc = dict(node.get_service_names_and_types())
        check("/replanner/start and /replanner/stop [std_srvs/srv/Trigger]",
              svc.get("/replanner/start") == ["std_srvs/srv/Trigger"] and
              svc.get("/replanner/stop") == ["std_srvs/srv/Trigger"])
        check("our progress publisher is matched by the replanner (QoS compatible, CM-4)",
              pub_prog.get_subscription_count() >= 1)
        check("status is latched: a subscriber gets it at once; state idle, the grid not usable",
              status().get("schema") == "blueboat_replanner_status/1" and
              status().get("state") == "idle", str(status().get("state")))

        # ============================================================== [R2]
        print("[R2] the first grids: alignment not valid, then valid")
        spin_until(lambda: len(got["grids"]) >= 3 and
                   status().get("inputs", {}).get("grid_seq_received") is not None, 15.0, tick)
        st = status()
        check("grids arrive while Belief's alignment is not valid: received, not usable, and "
              "the status says why", bool(got["grids"]) and not got["grids"][-1][1] and
              st["inputs"]["grid_ok"] is False and st["inputs"]["grid_seq_received"] >= 1 and
              "odom_to_map_valid" in (st["inputs"].get("grid_note") or ""),
              f"{got['grids'][-1:]} {st.get('inputs')}")
        start_cli = node.create_client(Trigger, "/replanner/start")
        stop_cli = node.create_client(Trigger, "/replanner/stop")
        spin_until(lambda: start_cli.service_is_ready(), 10.0, tick)
        feed["fixes"] = True
        ok = spin_until(lambda: bool(got["grids"]) and got["grids"][-1][1] and
                        status().get("inputs", {}).get("grid_ok") is True, 30.0, tick)
        g = got["grids"][-1]
        check("after 10 GPS / odometry pairs the alignment is valid and the grid is usable", ok,
              f"{g} {status().get('inputs')}")
        check("the transform Belief found is the made-up offset (not the identity)",
              abs(g[2] - W_OFFSET[0]) < 0.05 and abs(g[3] - W_OFFSET[1]) < 0.05, f"{g[2:4]}")
        seqs = [s for s, *_ in got["grids"]]
        check("grid_seq advances by exactly 1", all(b - a == 1 for a, b in zip(seqs, seqs[1:])),
              str(seqs[:8]))

        # ============================================================== [R3]
        print("[R3] a C3 mission through the real service type")
        feed["dets"] = [{"tau": 60.0, "id": "roscheck-1", "sent": False},
                        {"tau": 150.0, "id": "roscheck-2", "sent": False}]
        fut = start_cli.call_async(Trigger.Request())
        spin_until(fut.done, 10.0, tick)
        check("~/start accepted", fut.done() and fut.result().success, str(
            fut.result().message if fut.done() else "no answer"))
        ok = spin_until(lambda: status().get("state") == "running", 20.0, tick)
        first = ctl["ids"][0] if ctl["ids"] else None
        check("the first path went through /blueboat/update_path and was accepted",
              ok and first is not None and first[1] == "ok" and first[3] == "world" and
              first[2] > 100, f"{status().get('state')} {status().get('reason')} {first}")
        ok = spin_until(lambda: status().get("state") in ("done", "failed", "disarmed"),
                        600.0, tick)
        st = status()
        check("the mission ran to done", ok and st.get("state") == "done",
              f"{st.get('state')} {st.get('reason')} tau {ctl['tau']:.0f}")
        dec = got["decisions"][-1] if got["decisions"] else {}
        check("one decision, published with its schema; a REPLACE with two contacts was sent "
              "and accepted (two accepted requests in all)", len(got["decisions"]) == 1 and
              dec.get("schema") == "blueboat_replanner_decision/1" and
              dec.get("action") == "REPLACE" and st["counts"]["accepted"] == 2 and
              ctl["accepted"] == 2 and st["counts"]["refused"] == 0,
              f"{dec.get('action')} {dec.get('epoch')} {st.get('counts')} {ctl['ids']}")
        pend = dec.get("pending") or {}
        check("the decision read the evidence horizon from the grid stamp (Control's clock and "
              "the sonar stamps are one clock here)", pend.get("source") == "stamp" and
              pend.get("tau_horizon") is not None and
              0.0 <= dec["tau"] - pend["tau_horizon"] <= 60.0, f"{pend} tau {dec.get('tau')}")
        check("the decision record carries Belief's model flags and its labels",
              (dec.get("inputs") or {}).get("belief_model", {}).get("negative_evidence") ==
              "ungated" and "belief-ungated" in dec.get("labels", []) and
              "score-conditional" in dec.get("labels", []) and
              abs(dec["inputs"]["odom_to_map"][0] - W_OFFSET[0]) < 0.05, f"{dec.get('labels')}")
        tm = dec.get("timing") or {}
        check("the decision record carries its measured decision time and the allowance",
              isinstance(tm.get("compute_s"), float) and tm["compute_s"] > 0.0 and
              tm.get("allowance_s") == 2.0 and tm.get("over_allowance") is False, f"{tm}")
        check("/replanner/plan published once per accepted path", got["plans"] >= 2,
              str(got["plans"]))
        run_dir = Path(st.get("output_dir", ""))
        files = sorted(p.name for p in run_dir.iterdir()) if run_dir.is_dir() else []
        check("run folder: manifest, mission, belief.json, plans, logs, end.json",
              {"manifest.json", "mission.yaml", "belief.json", "plans", "decisions.jsonl",
               "requests.jsonl", "status.jsonl", "end.json"} <= set(files) and
              json.loads((run_dir / "end.json").read_text())["state"] == "done" and
              len(list((run_dir / "plans").iterdir())) == 2, f"{files}")
        seq3 = state_sequence(run_dir)
        check("the state sequence of the run (DESIGN §7) and the decision record's required "
              "keys (topics.md §5.1)", seq3 == ["starting", "running", "requesting", "running",
                                               "done"] and DECISION_KEYS <= set(dec),
              f"{' > '.join(map(str, seq3))}; missing keys {sorted(DECISION_KEYS - set(dec))}")
        fut = stop_cli.call_async(Trigger.Request())
        spin_until(fut.done, 10.0, tick)
        check("~/stop after done is refused (nothing to stop)",
              fut.done() and not fut.result().success)

        # ============================================================== [R4]
        print("[R4] Belief restarted")
        seq_before = got["grids"][-1][0]
        os.killpg(procs["belief"].pid, signal.SIGINT)
        try:
            procs["belief"].wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(procs["belief"].pid, signal.SIGKILL)
        n_before = len(got["grids"])
        procs["belief"] = start_belief()
        ok = spin_until(lambda: status().get("counts", {}).get("belief_restarts", 0) >= 1 and
                        status().get("inputs", {}).get("grid_ok") is True, 60.0, tick)
        new = got["grids"][n_before:]
        check("the restarted Belief starts again at grid_seq 1", bool(new) and new[0][0] == 1 and
              seq_before > 1, f"before {seq_before}, after {[s for s, *_ in new[:4]]}")
        check("the replanner counts the restart and uses the new grid (not dropped)", ok and
              status()["counts"]["belief_restarts"] == 1, f"{status().get('counts')} "
              f"{status().get('inputs')}")
        ev = [json.loads(ln) for ln in (run_dir / "events.jsonl").read_text().splitlines()] \
            if (run_dir / "events.jsonl").is_file() else []
        check("the restart is written to events.jsonl",
              any(e.get("event") == "belief_restart" and e.get("grid_seq") == 1 for e in ev),
              f"{[e.get('event') for e in ev]}")

        # ============================================================== [R5]
        print("[R5] shutdown")
        os.killpg(procs["replanner"].pid, signal.SIGINT)
        try:
            code = procs["replanner"].wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            code = None
        log = (work / "replanner_node.log").read_text()
        check("SIGINT -> the replanner exits with code 0 and no traceback", code == 0 and
              "Traceback" not in log, f"exit code {code}")

        # ============================================================== [R6]
        print("[R6] the detection emulator node beside Belief (Belief's default, gated coverage)")
        os.killpg(procs["belief"].pid, signal.SIGINT)
        try:
            procs["belief"].wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            os.killpg(procs["belief"].pid, signal.SIGKILL)
        node.destroy_publisher(pub_det)          # the emulator is the only detection publisher
        work2 = work / "emulator_phase"
        (work2 / "world").mkdir(parents=True, exist_ok=True)
        objs_w = [(typ, xm - W_OFFSET[0], ym - W_OFFSET[1], yaw) for typ, xm, ym, yaw in OBJECTS_M]
        loc = area_d["limits"]["local"]              # Belief's area, in its map frame
        (work2 / "world" / "metadata.yaml").write_text(yaml.safe_dump({
            "format": "blueboat_world_meta/1",
            # Belief's area, in the world (odom) frame: the emulator's cells then are Belief's
            "limits": {"local": {"x_min": float(loc["x_min"]) - W_OFFSET[0],
                                 "x_max": float(loc["x_max"]) - W_OFFSET[0],
                                 "y_min": float(loc["y_min"]) - W_OFFSET[1],
                                 "y_max": float(loc["y_max"]) - W_OFFSET[1]}},
            "objects": [{"object_id": i + 1, "type": typ, "x": float(xw), "y": float(yw),
                         "yaw": float(yaw)} for i, (typ, xw, yw, yaw) in enumerate(objs_w)]}))
        mission2 = work2 / "mission.yaml"
        mission2.write_text(yaml.safe_dump({"mission": {
            "name": "roscheck_emu", "condition": "C4", "areas": [AREA],
            "config": {"planning_depth_m": 2.0}}}))
        params2 = work2 / "belief_params.yaml"
        params2.write_text(yaml.safe_dump({"belief_map": {"ros__parameters": {
            "area.file": str(area_file), "io.data_dir": str(work2),
            "snapshot.period_s": 3600.0}}}))        # coverage.require_detector: Belief's default
        stub = ControlStub(hold)
        ctl.update(tau=0.0, calls=0, accepted=0, ids=[], speedup=args.c4_speedup)
        feed.update(dets=[], dist=0.0, xy=None)
        got.update(status=[], decisions=[], plans=0, grids=[], grid_msg=None, arrays=[])
        procs["belief"] = start(bel_prefix / "lib" / "blueboat_belief" / "belief_node",
                                "belief_node_emu.log", "--params-file", str(params2),
                                "-p", "use_sim_time:=false")
        # both nodes of this package through their installed launch files
        procs["emulator"] = start_launch("emulator_launch.log", "emulator_launch.py",
                                         f"world_dir:={work2 / 'world'}", "seed:=7",
                                         "class_info:=none", "range_m:=20.0")
        procs["replanner"] = start_launch("replanner_launch.log", "replanner_launch.py",
                                          f"mission_file:={mission2}", f"data_dir:={work2}")
        up = spin_until(lambda: {"belief_map", "replanner", "replanner_emulator"} <=
                        set(node.get_node_names()) and bool(status()) and
                        status().get("inputs", {}).get("grid_ok") is True, 60.0, tick)
        check("belief_map is running; replanner_emulator and replanner were started by their "
              "launch files and are running; the grid is usable",
              up and all(p.poll() is None for p in procs.values()),
              f"nodes {sorted(node.get_node_names())}; exit codes "
              f"{ {k: p.poll() for k, p in procs.items()} }")
        if not up:
            return 1
        dp = node.get_publishers_info_by_topic("/sss_ai/detections")
        ps = [i for i in node.get_subscriptions_info_by_topic("/sss_processor/processed")
              if i.node_name == "replanner_emulator"]
        check("the emulator is the one publisher of /sss_ai/detections and reads the pings "
              "BEST_EFFORT (matched with our publisher)", [i.node_name for i in dp] ==
              ["replanner_emulator"] and len(ps) == 1 and ps[0].qos_profile.reliability ==
              ReliabilityPolicy.BEST_EFFORT and pub_ping.get_subscription_count() >= 2,
              f"{[i.node_name for i in dp]} subs {pub_ping.get_subscription_count()}")

        # ============================================================== [R7]
        print(f"[R7] a C4 mission on the gated grid (path clock x{args.c4_speedup:g}; "
              f"about {1400.0 / args.c4_speedup / 60.0:.0f} min)", flush=True)
        feed.update(dist=0.0, xy=None)
        got["arrays"] = []
        spin_until(lambda: start_cli.service_is_ready(), 10.0, tick)
        fut = start_cli.call_async(Trigger.Request())
        spin_until(fut.done, 10.0, tick)
        check("~/start accepted", fut.done() and fut.result().success, str(
            fut.result().message if fut.done() else "no answer"))
        ok = spin_until(lambda: status().get("state") in ("done", "failed", "disarmed",
                                                          "stopped_holding"),
                        1800.0 / args.c4_speedup + 120.0, tick)
        st = status()
        check("the C4 mission ran to done", ok and st.get("state") == "done",
              f"{st.get('state')} {st.get('reason')} tau {ctl['tau']:.0f} {st.get('counts')}")
        spin_until(lambda: False, 2.0, tick)        # the last arrays and grids
        from blueboat_replanner.eval.emulator import image_stride_m
        n_exp = feed["dist"] / image_stride_m(20.0)
        arrays = got["arrays"]
        n_empty = sum(1 for _, d in arrays if not d)
        dets = [d for _, ds in arrays for d in ds]
        check("one detection array per seabed image, empty ones included: as many arrays as "
              "image strides were sailed (10 %), stamps increasing",
              arrays and abs(len(arrays) - n_exp) <= max(3.0, 0.1 * n_exp) and n_empty > 0 and
              all(b[0] > a[0] for a, b in zip(arrays, arrays[1:])),
              f"{len(arrays)} arrays ({n_empty} empty) for {feed['dist']:.0f} m sailed = "
              f"{n_exp:.1f} strides; {len(dets)} detections")
        check("the emulator detected something, reported no class, and its ids are unique",
              len(dets) >= 1 and {d[2] for d in dets} == {"unclassified"} and
              len({d[3] for d in dets}) == len(dets), f"{len(dets)} detections")
        gm = got["grid_msg"]
        model = json.loads(gm.model_params_json) if gm is not None and gm.model_params_json else {}
        alm = np.asarray(gm.aspect_log_miss, float) if gm is not None else np.zeros(0)
        n_cells = int((alm.reshape(-1, int(gm.aspect_sector_count)) < 0).any(axis=1).sum()) \
            if alm.size else 0
        check("Belief credits the search between the emulator's arrays: its negative evidence "
              "is gated (not the development mode) and cells are marked searched",
              model.get("negative_evidence") not in (None, "ungated") and n_cells > 1000,
              f"negative_evidence {model.get('negative_evidence')}, coverage_source "
              f"{model.get('coverage_source')}, {n_cells} of {int(gm.width) * int(gm.height)} "
              f"cells searched from at least one sector")
        cts = [(float(c.x), float(c.y), int(c.state)) for c in gm.contacts]
        far = [c for c in cts if min(math.hypot(c[0] - xm, c[1] - ym)
                                     for _, xm, ym, _ in OBJECTS_M) > 3.0]
        check("Belief holds contacts, each within 3 m of a made-up object (the emulator's world "
              "frame and Belief's map frame agree through the transform)",
              len(cts) >= 1 and not far, f"{len(cts)} contacts "
              f"({sum(1 for c in cts if c[2] == 1)} confirmed), {len(far)} far from every object")
        decs = got["decisions"]
        labs = sorted({lb for d in decs for lb in d.get("labels", [])})
        srcs = sorted({(d.get("pending") or {}).get("source") for d in decs})
        real_time = abs(args.c4_speedup - 1.0) < 1e-9
        check("C4 decisions under rclpy: published, none labelled belief-ungated, all "
              "score-conditional, the horizon read from the grid stamp" +
              ("; no request refused" if real_time else
               " [path clock faster than real time: refusals are expected and not judged]"),
              len(decs) >= 1 and "belief-ungated" not in labs and all(
                  "score-conditional" in d.get("labels", []) and DECISION_KEYS <= set(d)
                  for d in decs) and
              srcs == ["stamp"] and (st["counts"]["refused"] == 0 or not real_time),
              f"{len(decs)} decisions, actions "
              f"{sorted({d.get('action') for d in decs})}, outcomes "
              f"{ {o: sum(1 for d in decs if d['epoch'].get('outcome') == o) for o in sorted({str(d['epoch'].get('outcome')) for d in decs})} }, "
              f"labels {labs}, sources {srcs}, counts {st.get('counts')}")
        run2 = Path(st.get("output_dir", ""))
        end2 = json.loads((run2 / "end.json").read_text()) if (run2 / "end.json").is_file() else {}
        dts = end2.get("decision_time_s") or {}
        check("every decision's time was measured and written (decision record and end.json)",
              dts.get("n") == len(decs) and all(
                  isinstance((d.get("timing") or {}).get("compute_s"), float) for d in decs),
              f"n {dts.get('n')}, mean {dts.get('mean', float('nan')):.3f} s, max "
              f"{dts.get('max', float('nan')):.3f} s, smallest lead left "
              f"{min([(d.get('timing') or {}).get('lead_left_s') or float('inf') for d in decs] or [float('nan')]):.2f} "
              f"path s, over the {dts.get('allowance_s')} s "
              f"allowance: {dts.get('over_allowance')}  [small area, made-up Control, no "
              f"simulator running: not the timing of a full mission]")
        ok, detail = stop_launch(procs["emulator"], "emulator_launch.log",
                                 "replanner_emulator_node")
        check("SIGINT to its launch -> the emulator finishes cleanly, no traceback", ok, detail)
        ok, detail = stop_launch(procs["replanner"], "replanner_launch.log", "replanner_node")
        check("SIGINT to its launch -> the replanner finishes cleanly, no traceback", ok, detail)
        rc = 0 if not FAILED else 1
    finally:
        for p in procs.values():
            if p.poll() is None:
                try:
                    os.killpg(p.pid, signal.SIGINT)
                    p.wait(timeout=10.0)
                except Exception:  # noqa: BLE001 - shutting down
                    try:
                        os.killpg(p.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        node.destroy_node()
        rclpy.try_shutdown()
    print(("ALL ROS CHECKS PASSED" if not FAILED else f"FAILED: {FAILED}") +
          f" ({N_CHECKS} checks, {len(FAILED)} failed); work dir {work}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
