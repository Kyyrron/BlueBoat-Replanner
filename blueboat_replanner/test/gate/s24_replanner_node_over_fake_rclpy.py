"""Gate section [24]: replanner_node over FAKE rclpy modules (wiring only; NOT ROS integration).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# ---------------------------------------------------------------------------------------------
print("[24] replanner_node over FAKE rclpy modules (wiring only; NOT ROS integration)")
# The node's callbacks, services, timers and records are driven with fake rclpy / message
# modules and the Control stub as the service. This catches attribute and wiring errors in the
# one rclpy file; it says nothing about DDS, QoS matching, executors or the real messages
# (LOCAL_CHECKLIST L-R1…L-R6). The fake modules are removed again afterwards.
import importlib  # noqa: E402
import json  # noqa: E402
import time  # noqa: E402

_FAKE_NAMES = ["rclpy", "rclpy.node", "rclpy.qos", "geometry_msgs", "geometry_msgs.msg",
               "nav_msgs", "nav_msgs.msg", "std_msgs", "std_msgs.msg", "std_srvs",
               "std_srvs.srv", "blueboat_interfaces", "blueboat_interfaces.srv",
               "blueboat_belief_interfaces", "blueboat_belief_interfaces.msg",
               "blueboat_interfaces.msg", "vision_msgs", "vision_msgs.msg",
               "launch", "launch.actions", "launch.substitutions", "launch_ros",
               "launch_ros.actions", "launch_ros.parameter_descriptions"]
_saved_mods = {n: sys.modules.get(n) for n in _FAKE_NAMES +
               ["blueboat_replanner.ros.replanner_node",
                "blueboat_replanner.eval.emulator_node"]}


class _NS(types.SimpleNamespace):
    pass


def _msg_class(name, **defaults):
    def init(self, **kw):
        for k, v in defaults.items():
            setattr(self, k, v() if callable(v) else v)
        for k, v in kw.items():
            setattr(self, k, v)
    return type(name, (), {"__init__": init})


def _stamp_ns():
    return _NS(sec=0, nanosec=0)


_Header = _msg_class("Header", frame_id="", stamp=_stamp_ns)
_FakeString = _msg_class("String", data="")
_FakePath = _msg_class("Path", header=_Header, poses=list)
_FakePose = _msg_class("PoseStamped", header=_Header, pose=lambda: _NS(
    position=_NS(x=0.0, y=0.0, z=0.0), orientation=_NS(x=0.0, y=0.0, z=0.0, w=1.0)))
_FakeOdom = _msg_class("Odometry", header=_Header, pose=lambda: _NS(pose=_NS(
    position=_NS(x=0.0, y=0.0, z=0.0), orientation=_NS(x=0.0, y=0.0, z=0.0, w=1.0))))


class _FakeTrigger:
    Request = _msg_class("TriggerRequest")
    Response = _msg_class("TriggerResponse", success=False, message="")


class _FakeUpdatePath:
    Request = _msg_class("UpdatePathRequest", id="", change_point=0.0, frame="", t=list,
                         x=list, y=list, yaw=list)


class _FakeFuture:
    def __init__(self):
        self._cbs, self._res, self.done = [], None, False

    def add_done_callback(self, cb):
        self._cbs.append(cb)
        if self.done:
            cb(self)

    def result(self):
        return self._res

    def set_result(self, r):
        self._res, self.done = r, True
        for cb in list(self._cbs):
            cb(self)


class _FakeClient:
    def __init__(self, name):
        self.name, self.calls = name, []

    def service_is_ready(self):
        return True

    def call_async(self, req):
        fut = _FakeFuture()
        self.calls.append((req, fut))
        return fut


class _FakePub:
    def __init__(self, topic, qos):
        self.topic, self.qos, self.sent = topic, qos, []

    def publish(self, msg):
        self.sent.append(msg)


class _FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("info", m))

    def warn(self, m):
        self.lines.append(("warn", m))

    warning = warn

    def error(self, m):
        self.lines.append(("error", m))


class _FakeNode:
    OVERRIDES: dict = {}

    def __init__(self, name):
        self.node_name, self._params, self._log = name, {}, _FakeLogger()
        self.subs, self.pubs, self.srvs, self.timers, self.clients = {}, {}, {}, [], {}

    def declare_parameter(self, n, v):
        self._params[n] = self.OVERRIDES.get(n, v)

    def get_parameter(self, n):
        return _NS(value=self._params[n])

    def create_subscription(self, typ, topic, cb, qos):
        self.subs[topic] = (typ, cb, qos)

    def create_publisher(self, typ, topic, qos):
        self.pubs[topic] = _FakePub(topic, qos)
        return self.pubs[topic]

    def create_service(self, typ, name, cb):
        self.srvs[name.replace("~", "/" + self.node_name)] = (typ, cb)

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def create_client(self, typ, name):
        self.clients[name] = _FakeClient(name)
        return self.clients[name]

    def get_logger(self):
        return self._log

    def get_clock(self):
        return _NS(now=lambda: _NS(to_msg=_stamp_ns))

    def destroy_node(self):
        pass


def _install_fake_ros():
    m = {n: types.ModuleType(n) for n in _FAKE_NAMES}
    m["rclpy.node"].Node = _FakeNode
    q = m["rclpy.qos"]
    q.QoSProfile = lambda **kw: _NS(**kw)
    q.ReliabilityPolicy = _NS(RELIABLE="reliable", BEST_EFFORT="best_effort")
    q.DurabilityPolicy = _NS(TRANSIENT_LOCAL="transient_local", VOLATILE="volatile")
    q.HistoryPolicy = _NS(KEEP_LAST="keep_last")
    m["geometry_msgs.msg"].PoseStamped = _FakePose
    m["nav_msgs.msg"].Odometry, m["nav_msgs.msg"].Path = _FakeOdom, _FakePath
    m["std_msgs.msg"].String = _FakeString
    m["std_srvs.srv"].Trigger = _FakeTrigger
    m["blueboat_interfaces.srv"].UpdatePath = _FakeUpdatePath
    m["blueboat_belief_interfaces.msg"].BeliefGrid = _msg_class("BeliefGrid")
    m["blueboat_interfaces.msg"].ProcessedSSSPing = _msg_class("ProcessedSSSPing")
    vm = m["vision_msgs.msg"]
    vm.Detection2DArray = _msg_class("Detection2DArray", header=_Header, detections=list)
    vm.Detection2D = _msg_class("Detection2D", header=_Header, id="", results=list,
                                bbox=lambda: _NS(center=_NS(position=_NS(x=0.0, y=0.0),
                                                            theta=0.0), size_x=0.0, size_y=0.0))
    vm.ObjectHypothesisWithPose = _msg_class(
        "ObjectHypothesisWithPose", hypothesis=lambda: _NS(class_id="", score=0.0),
        pose=lambda: _NS(pose=_NS(position=_NS(x=0.0, y=0.0, z=0.0)), covariance=[0.0] * 36))
    m["launch"].LaunchDescription = lambda items: _NS(items=items)
    m["launch.actions"].DeclareLaunchArgument = lambda name, default_value="": _NS(
        name=name, default=default_value)
    m["launch.substitutions"].LaunchConfiguration = lambda name: _NS(conf=name)
    m["launch_ros.actions"].Node = lambda **kw: _NS(**kw)
    m["launch_ros.parameter_descriptions"].ParameterValue = lambda v, value_type=None: _NS(
        value=v, value_type=value_type)
    for n, mod in m.items():
        sys.modules[n] = mod
    sys.modules.pop("blueboat_replanner.ros.replanner_node", None)
    sys.modules.pop("blueboat_replanner.eval.emulator_node", None)
    return importlib.import_module("blueboat_replanner.ros.replanner_node")


def _restore_ros():
    for n, mod in _saved_mods.items():
        if mod is None:
            sys.modules.pop(n, None)
        else:
            sys.modules[n] = mod


node_ok = {}
try:
    rn = _install_fake_ros()
    with tempfile.TemporaryDirectory() as td24:
        import yaml  # noqa: E402 - io/ dependency, used here only to write the mission file
        mfile = os.path.join(td24, "mission.yaml")
        with open(mfile, "w") as fh:
            yaml.safe_dump({"mission": {**MISSION["mission"], "name": "node24",
                                        "condition": "C3"}}, fh)
        _FakeNode.OVERRIDES = {"mission_file": mfile, "data_dir": td24}
        fake_now = {"t": 1000.0}
        rn.time = _NS(time=lambda: fake_now["t"], strftime=time.strftime,
                      perf_counter=time.perf_counter)
        node = rn.ReplannerNode()
        node_ok["graph"] = (
            set(node.subs) == {"/belief_map/belief_grid", "/blueboat/path_progress",
                               "/blueboat/path_update_event", "/blueboat/odom"} and
            set(node.pubs) == {"/replanner/plan", "/replanner/decision", "/replanner/status"} and
            set(node.srvs) == {"/replanner/start", "/replanner/stop"} and
            set(node.clients) == {"/blueboat/update_path"} and
            node.subs["/belief_map/belief_grid"][2].durability == "transient_local" and
            node.subs["/belief_map/belief_grid"][2].depth == 1)
        stub24 = ControlStub((10.0, 10.0, 0.0))
        cli24 = node.clients["/blueboat/update_path"]
        tau24 = {"v": 0.0}
        gr24 = make_grid([contact(1, 30, 30), contact(2, 90, 40, first_sector=3)])
        tick_cb = [cb for p, cb in node.timers if p < 1.0][0]
        status_cb = [cb for p, cb in node.timers if p == 1.0][0]

        def feed24():
            gr24["grid_seq"] += 1
            node.subs["/belief_map/belief_grid"][1](gr24)
            pr24 = progress_of(stub24, tau24["v"])
            node.subs["/blueboat/path_progress"][1](
                _FakeString(data=json.dumps(pr24, default=float)))
            od = _FakeOdom()
            x24, y24, w24 = stub24.mirror.pose(tau24["v"]) if stub24.mirror else stub24.hold
            od.pose.pose.position.x, od.pose.pose.position.y = x24, y24
            od.pose.pose.orientation.z, od.pose.pose.orientation.w = (math.sin(w24 / 2),
                                                                      math.cos(w24 / 2))
            node.subs["/blueboat/odom"][1](od)

        def answer_calls():
            while cli24.calls:
                req, fut = cli24.calls.pop(0)
                a = stub24.handle(req.id, req.change_point, list(req.t), list(req.x),
                                  list(req.y), list(req.yaw), tau24["v"])
                fut.set_result(_NS(accepted=a["accepted"], reason=a.get("reason", "ok"),
                                   message=a.get("message", ""),
                                   active_id=a.get("active_id", ""),
                                   tau=a.get("tau", tau24["v"]),
                                   min_change_point=a.get("min_change_point", float("nan"))))
                ev = {"schema": "blueboat_path_update_event/1", "id": req.id,
                      "accepted": a["accepted"], "reason": a.get("reason", "ok")}
                node.subs["/blueboat/path_update_event"][1](_FakeString(data=json.dumps(ev)))

        feed24()
        resp = node.srvs["/replanner/start"][1](_FakeTrigger.Request(), _FakeTrigger.Response())
        node_ok["start"] = resp.success
        steps = 0
        while steps < 20000:
            feed24()
            tick_cb()
            answer_calls()
            if steps % 5 == 0:
                status_cb()
            if node.ex.state in ("done", "failed", "disarmed"):
                break
            tau24["v"] += 1.0 if node.ex.state != "waiting" else 0.0
            fake_now["t"] += 1.0
            steps += 1
        tick_cb()
        status_cb()
        run_dir = node.rec.dir
        node.rec.close()
        node_ok["run"] = (node.ex.state == "done" and node.ex.counts["decisions"] == 1 and
                          node.ex.counts["accepted"] == 2, f"{node.ex.state} {node.ex.reason} "
                          f"{node.ex.counts}")
        plans24 = node.pubs["/replanner/plan"].sent
        node_ok["plan"] = (len(plans24) == 2 and plans24[-1].header.frame_id == "world" and
                           len(plans24[-1].poses) > 100 and all(
                               plans24[-1].poses[i].header.stamp.sec <=
                               plans24[-1].poses[i + 1].header.stamp.sec
                               for i in range(len(plans24[-1].poses) - 1)))
        sts = [json.loads(m.data) for m in node.pubs["/replanner/status"].sent]
        decs = [json.loads(m.data) for m in node.pubs["/replanner/decision"].sent]
        node_ok["topics"] = (sts[-1].get("schema") == "blueboat_replanner_status/1" and
                             sts[-1].get("state") == "done" and len(decs) == 1 and
                             decs[0].get("schema") == "blueboat_replanner_decision/1")
        files24 = sorted(os.listdir(run_dir))
        with open(os.path.join(run_dir, "requests.jsonl")) as fh:
            reqs24 = [json.loads(ln) for ln in fh]
        node_ok["records"] = (
            {"manifest.json", "mission.yaml", "decisions.jsonl", "requests.jsonl",
             "status.jsonl", "events.jsonl", "plans", "belief.json", "end.json"} <=
            set(files24) and
            json.load(open(os.path.join(run_dir, "end.json")))["state"] == "done" and
            json.load(open(os.path.join(run_dir, "belief.json")))["contract"] ==
            "belief_grid/0" and
            len(os.listdir(os.path.join(run_dir, "plans"))) == 2 and
            len(reqs24) == 2 and all(r.get("answer", {}).get("accepted") for r in reqs24) and
            all(isinstance(r.get("answered"), float) for r in reqs24) and
            json.load(open(os.path.join(run_dir, "manifest.json")))["c2_tuned"]["settings"] ==
            _yaml.safe_load((PKG_ROOT / "config" / "c2_tuned.yaml").read_text())["c2_tuned"] and
            run_dir.startswith(os.path.join(td24, "data", "Replanner_data")), files24)
        resp = node.srvs["/replanner/stop"][1](_FakeTrigger.Request(), _FakeTrigger.Response())
        node_ok["stop_when_done"] = not resp.success

        # ---- the detection emulator node (eval/ only), same fake modules
        em = importlib.import_module("blueboat_replanner.eval.emulator_node")
        wdir = os.path.join(td24, "world")
        os.mkdir(wdir)
        objs24 = [{"object_id": 0, "type": "concrete_block", "x": 20.0, "y": 5.0, "yaw": 0.3},
                  {"object_id": 1, "type": "bottle_plastic", "x": 40.0, "y": -6.0,
                   "yaw": 1.0}]
        with open(os.path.join(wdir, "metadata.yaml"), "w") as fh:
            yaml.safe_dump({"format": "blueboat_world_meta/1", "objects": objs24}, fh)
        _FakeNode.OVERRIDES = {"world_dir": wdir, "seed": 7, "calib_a": 2.0, "calib_b": 0.5}
        emn = em.EmulatorNode()
        sub_t, sub_cb, sub_q = emn.subs["/sss_processor/processed"]
        node_ok["em_graph"] = (set(emn.pubs) == {"/sss_ai/detections"} and
                               sub_q.reliability == "best_effort" and sub_q.depth == 200)
        n_pass = 4
        for k in range(n_pass):                 # east, west, east, west along y = 0 at 0.5 m/s
            psi = 0.0 if k % 2 == 0 else math.pi
            xs = np.arange(0.0, 60.0, 0.025)
            for i, xx in enumerate(xs if k % 2 == 0 else xs[::-1]):
                tk = 1000.0 + 150.0 * k + 0.05 * i
                pg = _NS(robot_x=float(xx), robot_y=0.0, water_depth=2.22,
                         robot_orientation=_NS(x=0.0, y=0.0, z=math.sin(psi / 2),
                                               w=math.cos(psi / 2)),
                         port_stamp=_NS(sec=0, nanosec=0),
                         starboard_stamp=_NS(sec=int(tk), nanosec=int(round((tk - int(tk)) *
                                                                            1e9))))
                sub_cb(pg)
        emn.flush()
        sent24 = emn.pubs["/sss_ai/detections"].sent
        dets24 = [d for a in sent24 for d in a.detections]
        near = all(min(math.hypot(d.bbox.center.position.x - o["x"],
                                  d.bbox.center.position.y - o["y"]) for o in objs24) < 3.0
                   for d in dets24)
        ids24 = [d.id for d in dets24]

        def _sec(st):
            return st.sec + st.nanosec * 1e-9

        node_ok["em_draws"] = (emn.world.looks == {0: n_pass, 1: n_pass},
                               f"looks {emn.world.looks}")
        n_img = int(4 * 60.0 / emn.cutter.D)
        node_ok["em_arrays"] = (
            abs(len(sent24) - n_img) <= 2 and sum(1 for a in sent24 if not a.detections) > 50
            and all(_sec(sent24[i].header.stamp) < _sec(sent24[i + 1].header.stamp)
                    for i in range(len(sent24) - 1)) and
            all(_sec(d.header.stamp) <= _sec(a.header.stamp) + 1e-9
                for a in sent24 for d in a.detections),
            f"{len(sent24)} arrays for about {n_img} images")
        node_ok["em_dets"] = (len(dets24) >= 1 and near and len(set(ids24)) == len(ids24) and
                              all(a.header.frame_id == "odom" and a.header.stamp.sec >= 1000
                                  for a in sent24) and
                              all(0.0 < d.results[0].hypothesis.score < 1.0 and
                                  d.results[0].hypothesis.class_id == "unclassified" and
                                  abs(d.results[0].pose.covariance[0] - 1.25) < 1e-12 and
                                  d.results[0].pose.pose.position.x ==
                                  d.bbox.center.position.x for d in dets24),
                              f"{len(dets24)} detections")
        _FakeNode.OVERRIDES = {"world_dir": wdir, "class_info": "sometimes"}
        node_ok["em_class"] = raises(lambda: em.EmulatorNode(), SystemExit, "class_info")
        _FakeNode.OVERRIDES = {"world_dir": wdir, "seed": 7, "class_info": "true"}
        emT = em.EmulatorNode()
        node_ok["em_true"] = (em.reported_class(emT.world, emT.world.objects[0], "true") ==
                              "concrete_block" and
                              em.reported_class(emT.world, emT.world.objects[1], "true") ==
                              "bottle_plastic")

        # ---- launch files: every argument is a declared node parameter of the same type
        import importlib.util  # noqa: E402
        bad = []
        for lf, exe, nd in [("replanner_launch.py", "replanner_node", node),
                            ("emulator_launch.py", "replanner_emulator_node", emn)]:
            spec = importlib.util.spec_from_file_location("_l24", PKG_ROOT / "launch" / lf)
            lmod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(lmod)
            ld = lmod.generate_launch_description()
            nodes = [a for a in ld.items if hasattr(a, "executable")]
            decl = {a.name: a.default for a in ld.items if hasattr(a, "default")}
            params = {k: v for d in nodes[0].parameters for k, v in d.items()}
            if len(nodes) != 1 or nodes[0].executable != exe or set(params) != set(decl):
                bad.append(f"{lf}: graph")
            for k, pv in params.items():
                if k not in nd._params:
                    bad.append(f"{lf}: {k} not declared by the node")
                elif pv.value_type is not type(nd._params[k]):
                    bad.append(f"{lf}: {k} {pv.value_type} vs {type(nd._params[k])}")
                else:
                    try:                                    # the launch default parses
                        pv.value_type(decl[k])
                    except ValueError:
                        bad.append(f"{lf}: {k} default {decl[k]!r}")
        node_ok["launch"] = (not bad, "; ".join(bad))
except Exception as exc:  # noqa: BLE001 - reported as a failure, with the traceback
    import traceback
    traceback.print_exc()
    node_ok["exception"] = (False, repr(exc))
finally:
    _restore_ros()
    _FakeNode.OVERRIDES = {}


def _ok(v):
    return v[0] if isinstance(v, tuple) else bool(v)


def _why(v):
    return v[1] if isinstance(v, tuple) else ""


for key, label in [("graph", "node graph: 4 subscriptions, 3 publishers, 2 services, 1 client; "
                             "BeliefGrid TRANSIENT_LOCAL depth 1"),
                   ("start", "~/start with a mission file -> success"),
                   ("run", "C3 run through the node: 1 decision, 2 accepted requests, done"),
                   ("plan", "/replanner/plan published once per accepted path, frame world, "
                            "stamps non-decreasing"),
                   ("topics", "/replanner/status and /replanner/decision carry their schemas"),
                   ("records", "run folder under <data_dir>/data/Replanner_data: manifest, "
                               "mission, jsonl logs, 2 plans; one answered line per request"),
                   ("stop_when_done", "~/stop after done -> refused (nothing to stop)"),
                   ("em_graph", "emulator: one publisher /sss_ai/detections; processed pings "
                                "BEST_EFFORT depth 200"),
                   ("em_draws", "emulator: one draw per object per look (4 passes more than "
                                "the look time-out apart, target and clutter)"),
                   ("em_arrays", "emulator: one array per seabed image, empty ones included, "
                                 "stamps increasing, each detection stamped with its row"),
                   ("em_dets", "emulator: detections near true objects, unique ids, frame odom, "
                               "stamp from the present side, Perception's fill convention"),
                   ("em_class", "emulator: an unknown class_info is refused"),
                   ("em_true", "emulator: class_info true reports the object's type in the "
                               "world file"),
                   ("launch", "launch files: one node each, every argument a declared node "
                              "parameter with the same type")]:
    check(f"{label} [fake rclpy]", _ok(node_ok.get(key, False)), _why(node_ok.get(key, "")))
if "exception" in node_ok:
    check("replanner_node raised under the fake rclpy", False, _why(node_ok["exception"]))
check("fake modules removed afterwards", all(
    sys.modules.get(n) is _saved_mods[n] for n in _FAKE_NAMES))

