"""Detection emulator node (eval/ only; stands in for D3 in Gazebo; brief §10, decision 15).

[Wiring checked through fake rclpy modules in gate [24]. Run under ROS on 2026-11-12 on an
isolated graph beside BlueBoat-Belief's node, on made-up pings and a made-up world file
(test/ros_graph_check.py [R6]-[R7]; LOCAL_CHECKLIST.md L-X, L-E). Never in Gazebo, never on a
built world.]

Subscribes `/sss_processor/processed` (ProcessedSSSPing: robot_x, robot_y, robot_orientation,
water_depth, port_y / starboard_y, the side stamps). Reads the true objects from the world
folder's `metadata.yaml` (a FILE, allowed in eval/ only; world (0, 0) = spawn = geo_anchor =
Control's `world` origin in simulation, E-37). Run INSTEAD of Perception (one publisher per
topic), never beside it.

What it publishes, and why (Belief `docs/topics.md` §5 and §11 note 9):
- `/sss_ai/detections`, `vision_msgs/Detection2DArray`, in Perception's fill convention;
- **one array per seabed image, empty ones included**, stamped with the image's newest row.
  Belief credits a ping as searched only between two such arrays, so an emulator that
  published only when it detected something would leave Belief with almost no search record;
- one detection draw per look at an object, the look being defined as Belief defines it
  (`eval/emulator.py`), with the same world model and confidence calibration as the dev loop;
- the class name: `class_info:=none` -> "unclassified" (Belief keeps it as `other` and counts
  it); `class_info:=true` -> the object's type in the world file, which is what a detector
  trained on those worlds would report. Mapping types to aspect families is the planner's
  `class_families` configuration.

Image records (`/sss_ai/image_records`) are not published: Belief works without them.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
except ImportError:
    rclpy = None
    Node = object

from .emulator import (ImageCutter, LookTracker, TurnRate, image_stride_m, reported_class,
                       side_offsets)
from .world import WORLD_DEFAULTS, Calibrator, World, WorldObject, draw_detection, world_pd

# The world model's aspect family of a built world's object type (DESIGN decision 17,
# [ASSUMPTION]): a type listed here is a target with that family's response; every other type
# is clutter. This is the emulator's world, not the planner's `class_families`.
CLASS_MAP = {"concrete_block": "block", "pipe_pvc": "pipe", "mesh_crate": "cage",
             "tire_car": "tyre"}
NAV_SIGMA_M = 1.0                # Perception's assumed navigation sigma, part of its covariance


def load_world(world_dir: str) -> tuple:
    """(objects, (x_min, y_min, x_max, y_max)) of a built world."""
    import yaml
    with open(f"{world_dir}/metadata.yaml") as fh:
        meta = yaml.safe_load(fh)
    if meta.get("format") != "blueboat_world_meta/1":
        raise SystemExit("metadata.yaml: format is not blueboat_world_meta/1")
    objs = []
    for o in meta.get("objects", []):
        cls = CLASS_MAP.get(o["type"])
        kind = "target" if cls else "clutter"
        objs.append(WorldObject(int(o["object_id"]), kind, cls or "clutter", float(o["x"]),
                                float(o["y"]), float(o["yaw"]), label=str(o["type"])))
    loc = (meta.get("limits") or {}).get("local")
    if loc:
        rect = (float(loc["x_min"]), float(loc["y_min"]), float(loc["x_max"]),
                float(loc["y_max"]))
    elif objs:                      # no limits in the file: a box around the objects
        xs, ys = [o.x for o in objs], [o.y for o in objs]
        rect = (min(xs) - 30.0, min(ys) - 30.0, max(xs) + 30.0, max(ys) + 30.0)
    else:
        rect = (-100.0, -100.0, 100.0, 100.0)
    return objs, rect


def load_world_objects(world_dir: str) -> list:
    return load_world(world_dir)[0]


class EmulatorNode(Node):
    def __init__(self):
        super().__init__("replanner_emulator")
        from blueboat_interfaces.msg import ProcessedSSSPing
        self.declare_parameter("world_dir", "")
        self.declare_parameter("seed", 1)
        self.declare_parameter("class_info", "none")
        self.declare_parameter("calib_a", 1.0)
        self.declare_parameter("calib_b", 0.0)
        self.declare_parameter("range_m", 20.0)
        # Belief's grid and look parameters (its configuration; defaults = Belief's defaults)
        self.declare_parameter("resolution_m", 0.5)
        self.declare_parameter("sector_count", 12)
        self.declare_parameter("sector_origin_deg", 0.0)
        self.declare_parameter("look_timeout_s", 20.0)
        # the transducer below the waterline (root CLAUDE.md §4.3) and the processor's lateral
        # transducer offset (Belief topics.md §4 item 2): the defaults of the real stack
        self.declare_parameter("sensor_depth_m", 0.22)
        self.declare_parameter("lever_arm_m", 0.285)
        self.sensor_depth_m = float(self.get_parameter("sensor_depth_m").value)
        self.lever_arm_m = float(self.get_parameter("lever_arm_m").value)
        self.class_info = str(self.get_parameter("class_info").value)
        if self.class_info not in ("none", "true"):
            raise SystemExit(f"class_info must be 'none' or 'true', got {self.class_info!r}")
        from vision_msgs.msg import Detection2DArray
        self.Det = Detection2DArray
        objs, rect = load_world(self.get_parameter("world_dir").value)
        self.world = World(int(self.get_parameter("seed").value), objs, dict(WORLD_DEFAULTS))
        self.cal = Calibrator(float(self.get_parameter("calib_a").value),
                              float(self.get_parameter("calib_b").value), "parameter")
        self.R = float(self.get_parameter("range_m").value)
        res = float(self.get_parameter("resolution_m").value)
        # Belief's grid over an area starts at the area's lower-left corner (topics.md §3)
        width = max(1, int(math.ceil((rect[2] - rect[0]) / res - 1e-9)))
        height = max(1, int(math.ceil((rect[3] - rect[1]) / res - 1e-9)))
        self.tracker = LookTracker(
            objs, rect[0], rect[1], res, width, height,
            int(self.get_parameter("sector_count").value),
            float(self.get_parameter("sector_origin_deg").value), np.zeros(0), self.lever_arm_m,
            float(self.get_parameter("look_timeout_s").value), TurnRate())
        self.cutter = ImageCutter(image_stride_m(self.R))
        self.queue = []                       # detections not yet reported: (t, stamp, x, y, ...)
        self.stamps = deque(maxlen=8)         # (t, stamp message) of the latest pings
        self.last_t = -math.inf
        self.n_dets = 0
        self.n_arrays = 0
        self.pub = self.create_publisher(Detection2DArray, "/sss_ai/detections", 10)
        self.create_subscription(ProcessedSSSPing, "/sss_processor/processed", self._on_ping,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=200))

    def _on_ping(self, m):
        # a one-sided row carries a zeroed stamp on its absent side (ProcessedSSSPing)
        stamp = m.port_stamp if (m.port_stamp.sec or m.port_stamp.nanosec) else m.starboard_stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if t <= self.last_t:
            return
        self.last_t = t
        self.stamps.append((t, stamp))
        q = m.robot_orientation
        psi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        h = max(0.0, float(m.water_depth) - self.sensor_depth_m)
        r_geo = math.sqrt(max(0.0, self.R ** 2 - h * h))
        off = getattr(m, "port_y", None)
        if off is None or len(off) == 0:
            off = getattr(m, "starboard_y", None)
        if off is None or len(off) == 0:
            off = side_offsets(h, self.R, self.lever_arm_m)
        off = np.sort(np.abs(np.asarray(off, float)))
        x, y = float(m.robot_x), float(m.robot_y)
        for oi, hit in self.tracker.observe(t, x, y, psi, off):
            o = self.world.objects[oi]
            if hit.look.drawn or world_pd(self.world, o, hit.r, hit.bearing, r_geo) <= 0.0:
                continue
            hit.look.drawn = True
            ok, dx, dy, conf, _ = draw_detection(self.world, o, hit.r, hit.bearing, r_geo,
                                                 self.cal)
            if ok:
                self.queue.append((t, stamp, dx, dy, conf, psi,
                                   reported_class(self.world, o, self.class_info)))
        for img in self.cutter.add_ping(t, x, y):
            self._publish(img)

    def _stamp_of(self, t):
        for ts, st in reversed(self.stamps):
            if abs(ts - t) < 1e-9:
                return st
        return self.stamps[-1][1]

    def _publish(self, img):
        """One array per image, empty ones included, stamped with the image's newest row."""
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        mine = [d for d in self.queue if d[0] <= img.t_max + 1e-9]
        self.queue = [d for d in self.queue if d[0] > img.t_max + 1e-9]
        self.n_arrays += 1
        out = self.Det()
        out.header.frame_id = "odom"
        out.header.stamp = self._stamp_of(img.t_max)
        for (_, stamp, x, y, conf, psi, cls) in mine:
            d = Detection2D()
            d.header.stamp = stamp              # the row the detection lies on
            # unique per run, outside Perception's ranges (2e6 + 16 i + k, 1e9 + 1000 i + k)
            self.n_dets += 1
            d.id = str(3_000_000_000 + self.n_dets)
            d.bbox.center.position.x = float(x)
            d.bbox.center.position.y = float(y)
            d.bbox.center.theta = float(psi)
            d.bbox.size_x = d.bbox.size_y = 1.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls
            hyp.hypothesis.score = float(conf)
            hyp.pose.pose.position.x, hyp.pose.pose.position.y = float(x), float(y)
            cov = [0.0] * 36
            cov[0] = cov[7] = self.world.params["det_sigma_m"] ** 2 + NAV_SIGMA_M ** 2
            hyp.pose.covariance = cov
            d.results.append(hyp)
            out.detections.append(d)
        self.pub.publish(out)

    def flush(self):
        """The last, shorter image (at shutdown)."""
        img = self.cutter.flush()
        if img is not None:
            self._publish(img)


def main(args=None):
    if rclpy is None:
        raise SystemExit("rclpy is not available")
    rclpy.init(args=args)
    node = EmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.flush()
        except Exception:  # noqa: BLE001 - shutting down
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
