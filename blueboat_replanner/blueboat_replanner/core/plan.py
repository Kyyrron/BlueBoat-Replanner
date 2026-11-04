"""Plans, sampling and the mirror of Control's path (MATH.md §4; DESIGN.md §4-§5).

A `Plan` is a chain of `Segment`s in one frame, flown at constant speed U from parameter t0.
Its time axis is Control's path-parameter axis (path seconds). `sample()` produces exactly the
[t, x, y, yaw] request Control receives, so planned and charged durations agree by
construction (MATH §4, closure B).

`ControlPathMirror` holds what Control holds (accepted samples, spliced at change points) and
evaluates it with Control's rule (PATH_UPDATE_INTERFACE.md §3; E-10).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .dubins import Hold, Straight

KINDS = ("initial", "line", "connection", "transit", "body", "rejoin", "hold")
PRODUCTIVE = ("line", "body", "hold")


@dataclass(frozen=True)
class Segment:
    kind: str
    prims: tuple
    meta: tuple = ()           # sorted (key, value) pairs, hashable and deterministic

    def __post_init__(self):
        if self.kind not in KINDS:
            raise ValueError(f"segment kind {self.kind!r} not in {KINDS}")

    @staticmethod
    def make(kind, prims, **meta) -> "Segment":
        return Segment(kind, tuple(prims), tuple(sorted(meta.items())))

    @property
    def info(self) -> dict:
        return dict(self.meta)

    @property
    def length(self) -> float:
        return float(sum(p.length for p in self.prims))

    def duration(self, speed: float) -> float:
        if self.kind == "hold":
            return float(sum(p.duration for p in self.prims))
        return self.length / speed

    def start_pose(self):
        p = self.prims[0]
        return (p.x, p.y, p.th)

    def end_pose(self):
        return self.prims[-1].end()

    def pose_at_s(self, s: float):
        acc = 0.0
        for p in self.prims:
            if s <= acc + p.length or p is self.prims[-1]:
                x, y, th = p.pose(min(max(s - acc, 0.0), p.length))
                return float(x), float(y), float(th)
            acc += p.length
        return self.end_pose()


class Plan:
    """Segments flown from parameter t0 at speed U."""

    def __init__(self, segments, t0: float, speed: float):
        self.segments = list(segments)
        self.t0 = float(t0)
        self.speed = float(speed)
        times = []
        t = self.t0
        for seg in self.segments:
            d = seg.duration(self.speed)
            times.append((t, t + d))
            t += d
        self.times = times

    # ------------------------------------------------------------- queries
    @property
    def t_end(self) -> float:
        return self.times[-1][1] if self.times else self.t0

    @property
    def duration(self) -> float:
        return self.t_end - self.t0

    @property
    def length(self) -> float:
        return float(sum(s.length for s in self.segments))

    def index_at(self, t: float) -> int:
        for i, (a, b) in enumerate(self.times):
            if t < b:
                return i
        return len(self.segments) - 1

    def pose_at(self, t: float):
        if not self.segments:
            raise ValueError("empty plan")
        if t <= self.t0:
            return self.segments[0].start_pose()
        i = self.index_at(t)
        a, b = self.times[i]
        seg = self.segments[i]
        if seg.kind == "hold":
            return seg.start_pose()
        s = (min(t, b) - a) * self.speed
        return seg.pose_at_s(s)

    def boundaries(self) -> list:
        return [b for (_, b) in self.times]

    def decision_points(self, kinds=("line", "body")) -> list:
        """(t_end, segment index) of the default decision points (DESIGN §6)."""
        return [(self.times[i][1], i) for i, s in enumerate(self.segments) if s.kind in kinds]

    # ------------------------------------------------------------- edits
    def retimed(self, t0: float) -> "Plan":
        return Plan(self.segments, t0, self.speed)

    def from_index(self, i: int, t0: float | None = None) -> "Plan":
        t0 = self.times[i][0] if t0 is None else t0
        return Plan(self.segments[i:], t0, self.speed)

    def concat(self, other_segments) -> "Plan":
        return Plan(self.segments + list(other_segments), self.t0, self.speed)

    def truncated(self, t_limit: float) -> "Plan":
        """Keep whole segments ending at or before t_limit, then strip trailing
        non-productive segments (connection, transit, rejoin, initial) (MATH §4)."""
        keep = [s for s, (_, b) in zip(self.segments, self.times) if b <= t_limit + 1e-9]
        while keep and keep[-1].kind not in PRODUCTIVE:
            keep.pop()
        return Plan(keep, self.t0, self.speed)

    # ------------------------------------------------------------- sampling
    def sample(self, ds: float, max_samples: int = 19000):
        """[t, x, y, yaw] samples: every <= ds of arc length, segment ends included, yaw =
        analytic tangent, t strictly increasing in >= 2 ms steps (Control: >= 1 ms)."""
        total = self.length
        n_est = total / ds + 2 * len(self.segments) + 2
        if n_est > max_samples:
            ds = total / max(1.0, (max_samples - 2 * len(self.segments) - 2))
        ts, xs, ys, ws = [], [], [], []
        for seg, (a, b) in zip(self.segments, self.times):
            if seg.kind == "hold":
                x, y, th = seg.start_pose()
                st = np.array([a, b])
                sx, sy, sw = np.array([x, x]), np.array([y, y]), np.array([th, th])
            else:
                parts_t, parts_x, parts_y, parts_w = [], [], [], []
                acc = 0.0
                for p in seg.prims:
                    if p.length <= 0:
                        continue
                    n = max(1, int(math.ceil(p.length / ds - 1e-9)))
                    s = np.linspace(0.0, p.length, n + 1)
                    px, py, pw = p.pose(s)
                    parts_t.append(a + (acc + s) / self.speed)
                    parts_x.append(np.atleast_1d(px))
                    parts_y.append(np.atleast_1d(py))
                    parts_w.append(np.atleast_1d(pw))
                    acc += p.length
                if not parts_t:
                    continue
                st, sx = np.concatenate(parts_t), np.concatenate(parts_x)
                sy, sw = np.concatenate(parts_y), np.concatenate(parts_w)
            ts.append(st)
            xs.append(sx)
            ys.append(sy)
            ws.append(sw)
        if not ts:
            return np.zeros(0), np.zeros(0), np.zeros(0), np.zeros(0)
        t = np.concatenate(ts)
        x = np.concatenate(xs)
        y = np.concatenate(ys)
        w = np.concatenate(ws)
        # drop samples closer than 2 ms to the previous kept one (segment joins duplicate)
        keep = np.ones(len(t), dtype=bool)
        last = t[0]
        for i in range(1, len(t)):
            if t[i] - last < 2e-3:
                keep[i] = False
            else:
                last = t[i]
        if not keep[-1]:
            # keep the true end: replace the last kept sample by the final one
            j = np.flatnonzero(keep)[-1]
            if j > 0:
                keep[j] = False
            keep[-1] = True
        t, x, y, w = t[keep], x[keep], y[keep], w[keep]
        w = (w + np.pi) % (2.0 * np.pi) - np.pi
        return t, x, y, w


def hold_segment(pose, duration: float, **meta) -> Segment:
    return Segment.make("hold", [Hold(float(pose[0]), float(pose[1]), float(pose[2]),
                                      float(duration))], **meta)


def line_segment(p0, p1, **meta) -> Segment:
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return Segment.make("line", [Straight(float(p0[0]), float(p0[1]), math.atan2(dy, dx),
                                          math.hypot(dx, dy))], **meta)


# ---------------------------------------------------------------------------
# Control's evaluation rule and the mirror
# ---------------------------------------------------------------------------
def control_eval(t, x, y, yaw, s: float):
    """Pose of sampled path at parameter s, exactly as Control evaluates it
    (PATH_UPDATE_INTERFACE.md §3): clamp outside, linear x/y, short-way yaw."""
    t = np.asarray(t, float)
    if s <= t[0]:
        return float(x[0]), float(y[0]), float(yaw[0])
    if s >= t[-1]:
        return float(x[-1]), float(y[-1]), float(yaw[-1])
    i = int(np.searchsorted(t, s, side="right"))
    span = t[i] - t[i - 1]
    u = 0.0 if span <= 0 else (s - t[i - 1]) / span
    xx = x[i - 1] + u * (x[i] - x[i - 1])
    yy = y[i - 1] + u * (y[i] - y[i - 1])
    dyaw = math.atan2(math.sin(yaw[i] - yaw[i - 1]), math.cos(yaw[i] - yaw[i - 1]))
    th = yaw[i - 1] + u * dyaw
    return float(xx), float(yy), math.atan2(math.sin(th), math.cos(th))


@dataclass
class MirrorSegment:
    start: float
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray
    id: str


@dataclass
class ControlPathMirror:
    """This module's copy of the path Control holds (frame W). Changes only on acceptance."""

    segments: list = field(default_factory=list)

    def __bool__(self):
        return bool(self.segments)

    def segment_at(self, s: float) -> MirrorSegment:
        for seg in reversed(self.segments):
            if seg.start <= s:
                return seg
        return self.segments[0]

    def pose(self, s: float):
        seg = self.segment_at(s)
        return control_eval(seg.t, seg.x, seg.y, seg.yaw, s)

    def speed(self, s: float, h: float) -> float:
        if h <= 0:
            return 0.0
        x0, y0, _ = self.pose(s)
        x1, y1, _ = self.pose(s + h)
        return math.hypot(x1 - x0, y1 - y0) / h

    @property
    def t_start(self):
        return self.segments[0].start if self.segments else None

    @property
    def t_end(self):
        return float(self.segments[-1].t[-1]) if self.segments else None

    @property
    def active_id(self):
        return self.segments[-1].id if self.segments else None

    def splice(self, change_point: float, t, x, y, yaw, seg_id: str) -> None:
        start = float(t[0]) if not self.segments else float(change_point)
        new = MirrorSegment(start, np.asarray(t, float), np.asarray(x, float),
                            np.asarray(y, float), np.asarray(yaw, float), seg_id)
        self.segments = [s for s in self.segments if s.start < new.start] + [new]


def transform_segments(segments, frame_from, frame_to):
    """Re-express segments planned in map frame `frame_from` (Frame2D: x_M = R x_W + t) in the
    map frame `frame_to` — a rigid motion of every primitive (DESIGN §4: the executor
    re-expresses its plan whenever Belief refines odom_to_map, so junctions stay exact in W)."""
    from .dubins import Arc, Hold, Straight
    dyaw = frame_to.yaw - frame_from.yaw

    def pose(x, y, th):
        xw, yw, tw = frame_from.map_to_world(x, y, th)
        xm, ym, tm = frame_to.world_to_map(float(xw), float(yw), float(tw))
        return float(xm), float(ym), float(tm)

    out = []
    for seg in segments:
        prims = []
        for p in seg.prims:
            x, y, th = pose(p.x, p.y, p.th)
            if isinstance(p, Straight):
                prims.append(Straight(x, y, th, p.length))
            elif isinstance(p, Arc):
                prims.append(Arc(x, y, th, p.radius, p.d, p.length))
            elif isinstance(p, Hold):
                prims.append(Hold(x, y, th, p.duration))
            else:
                raise TypeError(type(p))
        out.append(Segment(seg.kind, tuple(prims), seg.meta))
    _ = dyaw
    return out
