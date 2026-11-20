"""Thin rclpy node (DESIGN.md §5, §10; topics.md). The only rclpy code of the module.

[Wiring checked through fake rclpy modules in gate [24]. Run under ROS on 2026-11-12 on an
isolated graph beside BlueBoat-Belief's node, with made-up sonar, navigation and Control
(test/ros_graph_check.py; LOCAL_CHECKLIST.md L-X). In Gazebo once, on 2026-11-18, beside Control's
node in a built world (a C2 development run, LOCAL_CHECKLIST.md L-G1). Never on the boat.]

Inputs: BeliefGrid (contract C-2 QoS), /blueboat/path_progress, /blueboat/path_update_event,
/blueboat/odom (arrival-time freshness). Client: /blueboat/update_path. Outputs:
/replanner/plan, /replanner/decision, /replanner/status; services ~/start, ~/stop. All logic
lives in `ros/executor.py` (ROS-free).

Clocks (MATH §11.3). Arrival times, ticks, start, stop, status and `end.json` are on the node's
time source, parameter `time_source`: `wall` (default, `time.time()`) or `ros` (the node clock,
`self.get_clock().now()`: `/clock` under `use_sim_time`, so a simulation slowed below real time or
paused while a slow ping renders keeps every age and timeout in simulated seconds; without
`use_sim_time` it is the system time, the same as `wall`). Records of a `ros` run carry
`"time_source": "ros"`; a record without the key is wall time. The decision time stays on
`time.perf_counter` (a wall measurement by design) and the run folder is named from the wall
clock. The timers are rclpy's, on the node clock in either mode. The grid's header stamp
(Belief's evidence horizon, on the sonar-profile clock) is compared only with `ros_time` of
Control's progress, which is Control's node clock: simulated time in Gazebo, wall time on the
boat.
"""

from __future__ import annotations

import json
import math
import os
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except ImportError:            # offline: the gate only compiles this file
    rclpy = None
    Node = object

from ..core.config import Config
from ..io.mission import load_c2_freeze, load_mission
from ..io.recordlog import RunRecorder, data_root
from ..io.trajectory_yaml import trajectory_doc
from .adapters import BELIEF_TOPIC, grid_to_dict, parse_event, parse_progress
from .executor import TERMINAL, Executor

TIME_SOURCES = ("wall", "ros")


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def default_c2_freeze() -> str:
    """The installed `share/blueboat_replanner/config/c2_tuned.yaml`, else the source tree's."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("blueboat_replanner"), "config",
                            "c2_tuned.yaml")
        if os.path.isfile(path):
            return path
    except Exception:  # noqa: BLE001 - not installed / no ament index: the source tree
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "config",
                        "c2_tuned.yaml")


class _ServiceClient:
    """Executor client over /blueboat/update_path (rclpy async call)."""

    def __init__(self, node, srv_type):
        self.node = node
        self.srv_type = srv_type
        self.cli = node.create_client(srv_type, "/blueboat/update_path")

    def ready(self) -> bool:
        return self.cli.service_is_ready()

    def send(self, req: dict, cb) -> None:
        r = self.srv_type.Request()
        r.id = req["id"]
        r.change_point = float(req["change_point"])
        r.frame = req["frame"]
        r.t, r.x, r.y, r.yaw = req["t"], req["x"], req["y"], req["yaw"]
        fut = self.cli.call_async(r)

        def done(f):
            try:
                a = f.result()
            except Exception as exc:  # noqa: BLE001 - reconciled by the executor's timeout
                self.node.get_logger().error(f"/blueboat/update_path call failed: {exc}")
                return
            # outside the try: an error in the executor's handling is not a failed call
            cb({"accepted": bool(a.accepted), "reason": a.reason, "message": a.message,
                "active_id": a.active_id, "tau": a.tau,
                "min_change_point": a.min_change_point}, self.node._now())
        fut.add_done_callback(done)


class ReplannerNode(Node):
    def __init__(self):
        super().__init__("replanner")
        from blueboat_interfaces.srv import UpdatePath
        try:
            from blueboat_belief_interfaces.msg import BeliefGrid
        except ImportError as exc:
            raise SystemExit("blueboat_belief_interfaces not found (contract belief_grid/0, "
                             f"LOCAL_CHECKLIST L-B1): {exc}") from None
        self.declare_parameter("mission_file", "")
        self.declare_parameter("config_file", "")
        self.declare_parameter("c2_tuned_file", "")
        self.declare_parameter("data_dir", "")
        self.declare_parameter("belief_topic", BELIEF_TOPIC)
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("time_source", "wall")
        self.time_source = str(self.get_parameter("time_source").value)
        if self.time_source not in TIME_SOURCES:
            raise SystemExit(f"time_source {self.time_source!r}: expected one of "
                             f"{', '.join(TIME_SOURCES)} (MATH §11.3)")
        base = {}
        cf = self.get_parameter("config_file").value
        if cf:
            import yaml
            with open(cf) as fh:
                base = (yaml.safe_load(fh) or {}).get("replanner", {})
        self.base_overrides = base
        cfg = Config.from_dict({"planning_depth_m": 1.0, **base})
        self.client = _ServiceClient(self, UpdatePath)
        self.ex = Executor(cfg, self.client, timer=time.perf_counter)
        self.rec = None
        self._last_state = None
        self._n_records = 0
        self._n_requests = 0
        self._n_events = 0
        self._belief_written = False
        self._end_written = False
        self._mirror_ids = []
        qos_belief = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(BeliefGrid, self.get_parameter("belief_topic").value,
                                 self._on_grid, qos_belief)
        self.create_subscription(String, "/blueboat/path_progress", self._on_progress, 10)
        self.create_subscription(String, "/blueboat/path_update_event", self._on_event,
                                 QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                            history=HistoryPolicy.KEEP_LAST, depth=50))
        self.create_subscription(Odometry, "/blueboat/odom", self._on_odom, 10)
        self.pub_plan = self.create_publisher(Path, "/replanner/plan", qos_latched)
        self.pub_dec = self.create_publisher(String, "/replanner/decision",
                                             QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                                        history=HistoryPolicy.KEEP_LAST,
                                                        depth=50))
        self.pub_status = self.create_publisher(String, "/replanner/status", qos_latched)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_timer(1.0 / float(self.get_parameter("tick_hz").value), self._tick)
        self.create_timer(1.0, self._status)
        self.get_logger().info("replanner ready (idle): call ~/start with a mission_file")

    # ------------------------------------------------------------- clock
    def _now(self) -> float:
        """Seconds on the node's time source: `time.time()` (wall, the default) or the node
        clock (`ros`: /clock under use_sim_time). Every time the executor is given comes from
        here; the decision time stays on `time.perf_counter`."""
        if self.time_source == "ros":
            return self.get_clock().now().nanoseconds * 1e-9
        return time.time()

    def _tag(self, rec: dict) -> dict:
        """A record as written and published: with `time_source: ros` a copy that names its
        clock (`"time_source": "ros"`); unchanged otherwise, so wall records stay byte-identical."""
        return rec if self.time_source == "wall" else {**rec, "time_source": self.time_source}

    # ------------------------------------------------------------- inputs
    def _on_grid(self, msg):
        self.ex.on_grid(grid_to_dict(msg), self._now())

    def _on_progress(self, msg):
        d = parse_progress(msg.data)
        if d is not None:
            self.ex.on_progress(d, self._now())

    def _on_event(self, msg):
        d = parse_event(msg.data)
        if d is not None:
            self.ex.on_event(d, self._now())
            if self.rec:
                self.rec.append("events.jsonl", d)

    def _on_odom(self, msg):
        p = msg.pose.pose
        self.ex.on_odom(p.position.x, p.position.y, _yaw_of(p.orientation), self._now())

    # ------------------------------------------------------------- services
    def _srv_start(self, req, resp):
        now = self._now()
        if self.ex.state == "disarmed":
            ok, msg = self.ex.start(None, now)
        else:
            path = self.get_parameter("mission_file").value
            try:
                freeze = load_c2_freeze(self.get_parameter("c2_tuned_file").value or
                                        default_c2_freeze())
                mission = load_mission(path, self.base_overrides, freeze)
            except Exception as exc:  # noqa: BLE001 - reported to the operator
                resp.success, resp.message = False, f"mission_error: {exc}"
                return resp
            ok, msg = self.ex.start(mission, now)
            if ok:
                if self.rec:
                    self._write_end(now)
                    self.rec.close()
                self._belief_written = False
                self._end_written = False
                stamp = time.strftime("%Y_%m_%d-%H_%M_%S")
                self.rec = RunRecorder(data_root(self.get_parameter("data_dir").value or None),
                                       stamp, mission.name, mission.condition)
                with open(path) as fh:
                    self.rec.write_once("mission.yaml", fh.read())
                manifest = {
                    "schema": "blueboat_replanner_manifest/1", "mission": mission.name,
                    "condition": mission.condition, "config": mission.config.as_dict(),
                    "c2_tuned": mission.freeze,
                    "contract": "belief_grid/0", "control_schemas":
                        ["blueboat_path_progress/1", "blueboat_path_update_event/1"],
                    "started_wall": now}
                if self.time_source != "wall":     # additive: the clock of every time below
                    manifest.update(started_wall=time.time(), started_clock=now,
                                    time_source=self.time_source,
                                    use_sim_time=self._use_sim_time())
                self.rec.write_once("manifest.json", manifest)
        resp.success, resp.message = bool(ok), msg
        return resp

    def _use_sim_time(self):
        try:
            return bool(self.get_parameter("use_sim_time").value)
        except Exception:  # noqa: BLE001 - not declared (a fake node): unknown
            return None

    def _srv_stop(self, req, resp):
        ok, msg = self.ex.stop(self._now())
        resp.success, resp.message = bool(ok), msg
        return resp

    # ------------------------------------------------------------- loop
    def _tick(self):
        now = self._now()
        self.ex.tick(now)
        while self._n_records < len(self.ex.records):
            r = self._tag(self.ex.records[self._n_records])
            self._n_records += 1
            self.pub_dec.publish(String(data=json.dumps(r, default=str)))
            if self.rec:
                self.rec.append("decisions.jsonl", r)
        self._log_requests()
        self._log_events()
        self._write_belief()
        ids = [s.id for s in self.ex.mirror.segments]
        if ids != self._mirror_ids:
            self._mirror_ids = ids
            self._publish_plan()
            if self.rec and self.ex.plan is not None and self.ex.grid is not None:
                t, x, y, w = self.ex.plan.sample(self.ex.cfg.sample_ds_m,
                                                 self.ex.cfg.max_samples_per_request)
                self.rec.write_once(f"plans/{len(ids):03d}-{ids[-1]}.yaml",
                                    json.dumps(trajectory_doc(ids[-1], t, x, y, w,
                                                              self.ex.grid.datum_lat,
                                                              self.ex.grid.datum_lon)))
        if self.ex.state != self._last_state:
            self._last_state = self.ex.state
            self._status()
            if self.ex.state in TERMINAL:
                self._write_end(now)

    def _log_events(self):
        """events.jsonl: the executor's own events (a Belief restart, a transform jump)."""
        while self._n_events < len(self.ex.events):
            e = self._tag(self.ex.events[self._n_events])
            self._n_events += 1
            self.get_logger().warn(f"replanner event: {json.dumps(e, default=str)}")
            if self.rec:
                self.rec.append("events.jsonl", e)

    def _write_belief(self):
        """belief.json, once per run: the grid the first path was planned on (datum,
        transform, the model flags Belief published). MATH §7, §11.3."""
        if self._belief_written or not self.rec or self.ex.plan is None or self.ex.grid is None:
            return
        g = self.ex.grid
        self._belief_written = True
        self.rec.write_once("belief.json", {
            "schema": "blueboat_replanner_belief/1", "contract": "belief_grid/0",
            "grid_seq": g.grid_seq, "stamp": g.stamp, "datum": [g.datum_lat, g.datum_lon],
            "odom_to_map": [g.frame.tx, g.frame.ty, g.frame.yaw],
            "resolution": g.resolution, "size": [g.width, g.height],
            "sectors": [g.sector_count, g.sector_origin_deg],
            "class_names": list(g.class_names),
            "prior_density_per_ha": g.prior_density_per_ha,
            "area": None if g.area_xy is None else g.area_xy.tolist(),
            "model": dict(g.model), "fallbacks": list(g.fallbacks)})

    def _write_end(self, now):
        """end.json, once per run: the final state, its reason and the counts."""
        if self._end_written or not self.rec:
            return
        self._end_written = True
        self.rec.write_once("end.json", self._tag(self.ex.end_record(now)))

    def _log_requests(self):
        """requests.jsonl: one line per request, written once its answer is known (the
        executor adds `answer` to the entry on reply or reconciliation)."""
        while self._n_requests < len(self.ex.requests):
            r = self.ex.requests[self._n_requests]
            p = self.ex.pending
            if "answer" not in r and p is not None and p.req_id == r["id"]:
                break                       # still in flight
            self._n_requests += 1
            if self.rec:
                self.rec.append("requests.jsonl", self._tag(r))

    def _publish_plan(self):
        msg = Path()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        segs = self.ex.mirror.segments
        for i, seg in enumerate(segs):
            # what Control follows: each segment from its start up to the next one's start
            t_next = segs[i + 1].start if i + 1 < len(segs) else math.inf
            for t, x, y, w in zip(seg.t, seg.x, seg.y, seg.yaw):
                if t < seg.start or t >= t_next:
                    continue
                ps = PoseStamped()
                ps.header.frame_id = "world"
                ps.header.stamp.sec = int(t)
                ps.header.stamp.nanosec = int((t - int(t)) * 1e9)
                ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                ps.pose.orientation.z = math.sin(w / 2.0)
                ps.pose.orientation.w = math.cos(w / 2.0)
                msg.poses.append(ps)
        self.pub_plan.publish(msg)

    def _status(self):
        s = self._tag(self.ex.status(self._now()))
        if self.rec:
            s["output_dir"] = self.rec.dir
            self.rec.append("status.jsonl", s)
        self.pub_status.publish(String(data=json.dumps(s, default=str)))


def main(args=None):
    if rclpy is None:
        raise SystemExit("rclpy is not available: source a ROS 2 Jazzy workspace")
    rclpy.init(args=args)
    node = ReplannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.rec:
            node._write_end(node._now())
            node.rec.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
