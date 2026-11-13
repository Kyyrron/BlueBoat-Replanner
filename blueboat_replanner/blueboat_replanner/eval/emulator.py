"""Detection emulator, ROS-free part (eval/ only): WHEN a world object gets one detection draw,
and HOW detections are grouped into seabed images. Used by the dev loop and by the ROS
emulator node, so both follow one rule.

A **look** at a world object is defined as BlueBoat-Belief's contract defines a look at a grid
cell (Belief `docs/topics.md` §4 items 1-4 and 6a), from the ping poses only:
- a ping side hits the object when one of its ground samples falls in the object's grid cell;
- a look is a run of hits from one directed aspect sector with gaps <= `look_timeout_s`; a
  sector change or a longer gap opens a new look;
- in a turn, inner-side samples beyond r* = v/|omega| are not counted (omega and v from the
  ping poses over `window_s`).
The emulator draws once per look (the caller draws at the first hit of the look where the world
gives a detection probability above zero). Nothing here reads Belief's state: it is the world
side's own bookkeeping, written from the contract text. The dev loop compares it with Belief's
published `aspect_counts` after every mission (`look_mismatch` in the result).

**Seabed images** (BlueBoat-SSS GCS / Perception): 256 rows at a stride of 128 rows; a row is
range/1200 m of track (square pixels), so an image spans 2 strides of along-track distance.
`ImageCutter` cuts the ping stream at these distances; the emulator reports one detection array
per image, empty ones included, as Perception does (Belief `docs/topics.md` §5).
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

IMAGE_STRIDE_ROWS = 128
IMAGE_ROWS = 256
RANGE_BINS = 1200


def image_stride_m(range_m: float) -> float:
    """Along-track distance between two seabed images (128 rows of range/1200 m)."""
    return IMAGE_STRIDE_ROWS * float(range_m) / RANGE_BINS


def side_offsets(altitude_m: float, range_m: float, lever_arm_m: float) -> np.ndarray:
    """Ground offsets of one side's samples from the boat's centre line, near to far, when a
    ping message carries none: slant bins of range/1199 m, the water column dropped, ground =
    sqrt(slant^2 - altitude^2), plus the lateral lever arm (the processor's geometry)."""
    slant = np.arange(RANGE_BINS) / (RANGE_BINS - 1) * float(range_m)
    slant = slant[slant > altitude_m]
    return lever_arm_m + np.sqrt(np.maximum(slant ** 2 - altitude_m ** 2, 0.0))


def sector_of(bearing: float, sector_count: int, origin_deg: float) -> int:
    """Directed sector of a look bearing [rad] (Belief topics.md §4 item 3)."""
    a = round(math.degrees(bearing) - origin_deg, 9) % 360.0
    return min(sector_count - 1, int(math.floor(a / (360.0 / sector_count))))


class TurnRate:
    """Yaw rate and speed from the ping poses over `window_s` (Belief topics.md §4 item 6a):
    r* = v/|omega| when |omega| >= min_yaw_rate (0 when v < min_speed); the inner side is port
    for omega > 0. A ping gap longer than the window restarts the estimate."""

    def __init__(self, window_s: float = 1.0, min_yaw_rate: float = 0.01,
                 min_speed: float = 0.05, enabled: bool = True):
        self.window, self.min_rate, self.min_speed = window_s, min_yaw_rate, min_speed
        self.enabled = enabled
        self.h = deque()                 # (t, x, y, unwrapped yaw)
        self.last_yaw = None
        self.unwrapped = 0.0

    def update(self, t: float, x: float, y: float, yaw: float):
        """-> (r_star, inner): inner = +1 port, -1 starboard, 0 not turning (r_star = inf)."""
        if not self.enabled:
            return math.inf, 0
        if self.h and t < self.h[-1][0]:
            return self._estimate()
        if self.h and t - self.h[-1][0] > self.window:
            self.h.clear()
            self.last_yaw = None
        if self.last_yaw is None:
            self.unwrapped = yaw
        else:
            d = yaw - self.last_yaw
            self.unwrapped += (d + math.pi) % (2.0 * math.pi) - math.pi
        self.last_yaw = yaw
        self.h.append((t, x, y, self.unwrapped))
        while len(self.h) > 2 and self.h[1][0] <= t - self.window:
            self.h.popleft()
        return self._estimate()

    def _estimate(self):
        if len(self.h) < 2:
            return math.inf, 0
        t0, x0, y0, a0 = self.h[0]
        t1, x1, y1, a1 = self.h[-1]
        dt = t1 - t0
        if dt <= 0.0:
            return math.inf, 0
        omega = (a1 - a0) / dt
        if abs(omega) < self.min_rate:
            return math.inf, 0
        v = math.hypot(x1 - x0, y1 - y0) / dt
        return (v / abs(omega) if v >= self.min_speed else 0.0), (1 if omega > 0 else -1)


@dataclass
class Look:
    oid: int
    sector: int
    t_first: float
    t_last: float
    number: int                 # this object's n-th look (1-based)
    drawn: bool = False         # the emulator made its draw for this look


@dataclass
class Hit:
    look: Look
    t: float
    r: float                    # ground range of the object from the transducer [m]
    bearing: float              # directed look bearing [rad]
    opened: bool                # this ping opened the look


class LookTracker:
    """Looks at world objects, from ping poses (see the module text)."""

    def __init__(self, objects, origin_x: float, origin_y: float, resolution: float,
                 width: int, height: int, sector_count: int, sector_origin_deg: float,
                 offsets, lever_arm_m: float, look_timeout_s: float = 20.0,
                 turn: TurnRate | None = None):
        self.objects = list(objects)
        self.ox, self.oy, self.res = float(origin_x), float(origin_y), float(resolution)
        self.W, self.H = int(width), int(height)
        self.S, self.o = int(sector_count), float(sector_origin_deg)
        self.off = np.sort(np.abs(np.asarray(offsets, float)))   # from the centre line
        self.lever = float(lever_arm_m)
        self.timeout = float(look_timeout_s)
        self.turn = turn or TurnRate()
        n = len(self.objects)
        self.pos = np.array([[o.x, o.y] for o in self.objects], float).reshape(n, 2)
        col = np.floor((self.pos[:, 0] - self.ox) / self.res)
        row = np.floor((self.pos[:, 1] - self.oy) / self.res)
        self.in_grid = (col >= 0) & (col < self.W) & (row >= 0) & (row < self.H)
        self.cell = (row * self.W + col).astype(np.int64)
        self.x0 = self.ox + col * self.res               # the object's cell box
        self.y0 = self.oy + row * self.res
        self.current = [None] * n                        # the open look of each object
        self.counts = np.zeros((n, self.S), dtype=np.int64)   # looks opened per sector
        self.n_looks = [0] * n
        self.reach = float(self.off[-1]) + 2.0 * self.res if len(self.off) else 0.0

    def observe(self, t: float, x: float, y: float, psi: float, offsets=None) -> list:
        """One ping at pose (x, y, psi), time t: the hits on world objects, port side first.
        `offsets`: this ping's own sample offsets from the centre line (sorted), when they
        differ from the ones given at construction (a live ping message)."""
        r_star, inner = self.turn.update(t, x, y, psi)
        hits = []
        if offsets is not None:
            self.off = np.asarray(offsets, float)
            self.reach = float(self.off[-1]) + 2.0 * self.res if len(self.off) else 0.0
        if not len(self.off):
            return hits
        near = self.in_grid & (np.abs(self.pos[:, 0] - x) <= self.reach) & \
            (np.abs(self.pos[:, 1] - y) <= self.reach)
        idx = np.flatnonzero(near)
        if not len(idx):
            return hits
        for side in (1, -1):
            b = psi + side * (math.pi / 2.0)
            cb, sb = math.cos(b), math.sin(b)
            hi_cap = r_star if (inner == side) else math.inf
            k = sector_of(b, self.S, self.o)
            for i in idx:
                # the stretch of the ray inside the object's cell box [x0, x0+res) x [y0, y0+res)
                lo, hi = _ray_box(x, y, cb, sb, self.x0[i], self.y0[i], self.res)
                if hi <= lo:
                    continue
                hi = min(hi, hi_cap)
                a = int(np.searchsorted(self.off, lo, side="left"))
                e = int(np.searchsorted(self.off, hi, side="right" if hi == hi_cap else "left"))
                if e <= a:
                    continue                         # no ground sample of this ping in the cell
                cur = self.current[i]
                opened = cur is None or cur.sector != k or t - cur.t_last > self.timeout
                if opened:
                    self.n_looks[i] += 1
                    cur = Look(self.objects[i].oid, k, t, t, self.n_looks[i])
                    self.current[i] = cur
                    self.counts[i, k] += 1
                cur.t_last = max(cur.t_last, t)
                along = (self.pos[i, 0] - x) * cb + (self.pos[i, 1] - y) * sb
                hits.append((int(i), Hit(cur, t, max(0.0, along - self.lever), b, opened)))
        return hits


def _ray_box(x, y, cb, sb, x0, y0, res):
    """[lo, hi) of the distance along the ray (x, y) + d (cb, sb), d >= 0, inside the box."""
    lo, hi = 0.0, math.inf
    for p, d, a in ((x, cb, x0), (y, sb, y0)):
        if abs(d) < 1e-12:
            if not (a <= p < a + res):
                return 0.0, 0.0
            continue
        d1, d2 = (a - p) / d, (a + res - p) / d
        if d1 > d2:
            d1, d2 = d2, d1
        lo, hi = max(lo, d1), min(hi, d2)
    return lo, hi


@dataclass
class Image:
    index: int
    t_min: float                # oldest row time
    t_max: float                # newest row time = the stamp of the image's detection array


class ImageCutter:
    """Cuts a ping stream into seabed images by along-track distance: image m holds the pings
    whose travelled distance s is in [m D, (m + 2) D), D = `stride_m`."""

    def __init__(self, stride_m: float):
        if stride_m <= 0:
            raise ValueError("stride_m must be > 0")
        self.D = float(stride_m)
        self.s = 0.0
        self.last = None            # (t, x, y) of the previous ping
        self.t_first = {}           # boundary k -> time of the first ping with s >= k D
        self.t_before = {}          # boundary k -> time of the last ping with s < k D
        self.k = -1                 # highest boundary reached
        self.done = False

    def add_ping(self, t: float, x: float, y: float) -> list:
        """-> the images this ping completes (in order)."""
        out = []
        if self.last is not None:
            self.s += math.hypot(x - self.last[1], y - self.last[2])
        k_now = int(math.floor(self.s / self.D + 1e-12))
        while self.k < k_now:
            self.k += 1
            self.t_first[self.k] = t
            self.t_before[self.k] = self.last[0] if self.last is not None else t
            if self.k >= 2:
                m = self.k - 2
                out.append(Image(m, self.t_first[m], self.t_before[self.k]))
        self.last = (t, x, y)
        return out

    def flush(self):
        """The last, shorter image: every ping not yet inside a completed image."""
        if self.done or self.last is None:
            return None
        self.done = True
        m = max(0, self.k - 1)
        return Image(m, self.t_first.get(m, self.last[0]), self.last[0])


def world_range_ok(r: float, r_lo: float, r_hi: float) -> bool:
    return r_lo <= r <= r_hi


def reported_class(world, obj, mode: str) -> str:
    """The class name the emulated detector gives a detection of `obj` (a world-model choice,
    recorded in every manifest):
    - "none": no class information. Every detection is "unclassified"; with Belief's class list
      set to the planner's families and no `other`, the class probabilities stay uniform;
    - "true": a perfect classifier. A target reports its label; a clutter object reports one
      fixed label drawn for it, so the class tells nothing about real against clutter."""
    if mode == "none":
        return "unclassified"
    if mode != "true":
        raise ValueError(f"class information mode {mode!r} is not 'none' or 'true'")
    if obj.label:
        return obj.label
    if obj.kind == "target":
        return obj.cls
    names = sorted(world.params["classes"])
    rng = np.random.default_rng(np.random.SeedSequence([int(world.seed), 4, int(obj.oid)]))
    return names[int(rng.integers(len(names)))]
