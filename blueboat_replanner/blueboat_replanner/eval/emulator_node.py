"""Detection emulator node (eval/ only; stands in for D3 in Gazebo; brief §10, decision 15).

[NOT RUN — written in the cloud session; LOCAL_CHECKLIST.md L-E*.]

Subscribes `/sss_processor/processed` (ProcessedSSSPing: robot_x, robot_y, robot_orientation,
water_depth, port_y/starboard_y). Reads the true objects from the world folder's
`metadata.yaml` (a FILE, allowed in eval/ only; world (0, 0) = spawn = geo_anchor = Control's
`world` origin in simulation, E-37). One draw per object per pass when the object is abeam and
in the detection band, with the same `eval.world` model and confidence calibration as the dev
loop. Run INSTEAD of Perception (one publisher per topic).

Output (`output` parameter):
  detections     -> /sss_ai/detections, vision_msgs/Detection2DArray in Perception's fill
                    convention (BlueBoat-Perception docs/topics.md §3)
  image_records  -> NOT IMPLEMENTED: whether Belief consumes /sss_ai/image_records or
                    /sss_ai/detections is [OPEN] (Q-9, REQUESTS RQ-3, LOCAL_CHECKLIST L-B2)
"""

from __future__ import annotations

import math

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
except ImportError:
    rclpy = None
    Node = object

from .world import WORLD_DEFAULTS, Calibrator, World, WorldObject, draw_detection

CLASS_MAP = {"concrete_block": "block", "pipe_pvc": "pipe", "mesh_crate": "cage",
             "tire_car": "tyre"}                       # DESIGN decision 17 ([ASSUMPTION])


def load_world_objects(world_dir: str) -> list:
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
                                float(o["y"]), float(o["yaw"])))
    return objs


class EmulatorNode(Node):
    def __init__(self):
        super().__init__("replanner_emulator")
        from blueboat_interfaces.msg import ProcessedSSSPing
        self.declare_parameter("world_dir", "")
        self.declare_parameter("seed", 1)
        self.declare_parameter("output", "detections")
        self.declare_parameter("calib_a", 1.0)
        self.declare_parameter("calib_b", 0.0)
        self.declare_parameter("range_m", 20.0)
        out = self.get_parameter("output").value
        if out != "detections":
            raise SystemExit("output 'image_records' is not implemented: [OPEN] Q-9 / L-B2")
        from vision_msgs.msg import Detection2DArray
        self.Det = Detection2DArray
        self.world = World(int(self.get_parameter("seed").value),
                           load_world_objects(self.get_parameter("world_dir").value),
                           dict(WORLD_DEFAULTS))
        self.cal = Calibrator(float(self.get_parameter("calib_a").value),
                              float(self.get_parameter("calib_b").value), "parameter")
        self.R = float(self.get_parameter("range_m").value)
        self.pass_id = 0
        self.last_heading = None
        self.drawn = set()
        self.n_msgs = 0
        self.pub = self.create_publisher(Detection2DArray, "/sss_ai/detections", 10)
        self.create_subscription(ProcessedSSSPing, "/sss_processor/processed", self._on_ping,
                                 QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                            history=HistoryPolicy.KEEP_LAST, depth=200))

    def _on_ping(self, m):
        q = m.robot_orientation
        psi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # a new pass when the heading changes by more than 30 deg (turns separate passes)
        if self.last_heading is None or abs(math.atan2(math.sin(psi - self.last_heading),
                                                       math.cos(psi - self.last_heading))) > \
                math.radians(30):
            self.pass_id += 1
            self.last_heading = psi
        h = max(0.0, float(m.water_depth) - 0.22)
        r_geo = math.sqrt(max(0.0, self.R ** 2 - h * h))
        u = (math.cos(psi), math.sin(psi))
        dets = []
        for o in self.world.objects:
            dx, dy = o.x - m.robot_x, o.y - m.robot_y
            along = dx * u[0] + dy * u[1]
            if abs(along) > 0.25 or (o.oid, self.pass_id) in self.drawn:
                continue
            cross = -dx * u[1] + dy * u[0]
            r = abs(cross)
            side = 1 if cross > 0 else -1
            b = psi + side * math.pi / 2
            self.drawn.add((o.oid, self.pass_id))
            ok, x, y, conf, _ = draw_detection(self.world, o, r, b, r_geo, self.cal)
            if ok:
                dets.append((x, y, conf, psi))
        self._publish(m, dets)

    def _publish(self, m, dets):
        if not dets:
            return
        from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose
        # a one-sided row carries a zeroed stamp on its absent side (ProcessedSSSPing)
        stamp = m.port_stamp if (m.port_stamp.sec or m.port_stamp.nanosec) else m.starboard_stamp
        self.n_msgs += 1
        out = self.Det()
        out.header.frame_id = "odom"
        out.header.stamp = stamp
        for k, (x, y, conf, psi) in enumerate(dets):
            d = Detection2D()
            d.header.stamp = stamp
            # unique per run, outside Perception's ranges (2e6 + 16 i + k, 1e9 + 1000 i + k)
            d.id = str(3_000_000_000 + 1000 * self.n_msgs + k)
            d.bbox.center.position.x = float(x)
            d.bbox.center.position.y = float(y)
            d.bbox.center.theta = float(psi)
            d.bbox.size_x = d.bbox.size_y = 1.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "emulated"
            hyp.hypothesis.score = float(conf)
            hyp.pose.pose.position.x, hyp.pose.pose.position.y = float(x), float(y)
            cov = [0.0] * 36
            cov[0] = cov[7] = self.world.params["det_sigma_m"] ** 2
            hyp.pose.covariance = cov
            d.results.append(hyp)
            out.detections.append(d)
        self.pub.publish(out)


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
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
