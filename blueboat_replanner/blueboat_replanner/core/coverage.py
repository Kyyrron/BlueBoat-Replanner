"""One coverage generator for every condition (MATH.md §3, §4).

C1 plan = initial manoeuvre + pass 1. C2 plan = initial manoeuvre + pass 1 + pass 2 (C2, C3 and
the C4 family all start from it; closure B). The initial manoeuvre starts at the hold position
with the boat's heading (closure E): no in-place turn is assumed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .areas import Area, MissionError
from .dubins import dubins_candidates
from .plan import Plan, Segment, line_segment
from .routing import connect
from .spacing import CoverageBand, spacing_for_width


@dataclass(frozen=True)
class Leg:
    pass_no: int
    line: int
    piece: int
    p0: tuple
    p1: tuple

    @property
    def heading(self) -> float:
        return math.atan2(self.p1[1] - self.p0[1], self.p1[0] - self.p0[0])

    @property
    def length(self) -> float:
        return math.hypot(self.p1[0] - self.p0[0], self.p1[1] - self.p0[1])

    def reversed(self) -> "Leg":
        return Leg(self.pass_no, self.line, self.piece, self.p1, self.p0)

    @property
    def leg_id(self) -> str:
        return f"P{self.pass_no}-L{self.line:03d}-{self.piece}"

    def start_pose(self):
        return (self.p0[0], self.p0[1], self.heading)

    def end_pose(self):
        return (self.p1[0], self.p1[1], self.heading)


@dataclass(frozen=True)
class PassLayout:
    pass_no: int
    theta: float
    spacing: object          # SpacingResult
    lines: tuple             # tuple of tuples of Leg (per line index, along +u)
    interlace: int


def pass_layout(area: Area, theta: float, band: CoverageBand, cfg, pass_no: int) -> PassLayout:
    """Lines parallel to `theta`, offsets from MATH §2.2, legs = runs inside F_{margin+r_turn},
    each shortened by l_in = U * t_settle at both ends (MATH §3.2)."""
    e = cfg.margin_m + cfg.r_turn_m
    cmin, cmax = area.cross_track_extent(theta)
    amin, amax = area.along_track_extent(theta)
    width = cmax - cmin
    if width < 2.0 * e:
        raise MissionError(f"area: cross-track width {width:.2f} m < 2(margin + r_turn) = "
                           f"{2 * e:.2f} m for pass {pass_no}")
    sp = spacing_for_width(width, band, e, cfg)
    u = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])
    step = cfg.geom_step_m
    xs = np.arange(amin, amax + step, step)
    l_in = cfg.speed_mps * cfg.t_settle_s
    lines = []
    for j, off in enumerate(sp.offsets):
        c = cmin + float(off)
        pts = c * n[None, :] + xs[:, None] * u[None, :]
        ok = area.feasible(pts, e)
        legs = []
        k = 0
        i = 0
        while i < len(ok):
            if not ok[i]:
                i += 1
                continue
            a = i
            while i < len(ok) and ok[i]:
                i += 1
            b = i - 1
            xa, xb = xs[a] + l_in, xs[b] - l_in
            if xb - xa >= cfg.min_leg_m:
                p0 = tuple(c * n + xa * u)
                p1 = tuple(c * n + xb * u)
                legs.append(Leg(pass_no, j, k, (float(p0[0]), float(p0[1])),
                                (float(p1[0]), float(p1[1]))))
                k += 1
        lines.append(tuple(legs))
    s = sp.s
    if cfg.interlace == "auto" and s < 2.0 * cfg.r_turn_m:
        inter = int(math.ceil(2.0 * cfg.r_turn_m / s - 1e-12))
    else:
        inter = 1
    return PassLayout(pass_no, theta, sp, tuple(lines), inter)


def line_visit_order(n_lines: int, k: int) -> list:
    """Interlaced boustrophedon order: residue groups mod k, alternating direction."""
    order = []
    for r in range(k):
        grp = list(range(r, n_lines, k))
        order.extend(grp if r % 2 == 0 else grp[::-1])
    return order


def _ordered_legs(layout: PassLayout, reverse_lines: bool, first_forward: bool) -> list:
    nonempty = [j for j in range(len(layout.lines)) if layout.lines[j]]
    order = [nonempty[i] for i in line_visit_order(len(nonempty), layout.interlace)]
    if reverse_lines:
        order = order[::-1]
    legs = []
    forward = first_forward
    for j in order:
        pieces = list(layout.lines[j])
        if forward:
            legs.extend(pieces)
        else:
            legs.extend(p.reversed() for p in reversed(pieces))
        forward = not forward
    return legs


def ordered_pass(layout: PassLayout, start_pose, cfg) -> list:
    """Start-corner rule: of the four variants (line order × first direction), the one whose
    first leg start has the shortest unconstrained Dubins length from start_pose."""
    best = None
    for vi, (rev, fwd) in enumerate([(False, True), (False, False), (True, True), (True, False)]):
        legs = _ordered_legs(layout, rev, fwd)
        if not legs:
            continue
        cands = dubins_candidates(tuple(start_pose), legs[0].start_pose(), cfg.r_turn_m)
        L = cands[0][0] if cands else math.inf
        key = (round(L, 9), vi)
        if best is None or key < best[0]:
            best = (key, legs)
    if best is None:
        raise MissionError(f"pass {layout.pass_no}: no leg inside F_(margin+r_turn)")
    return best[1]


@dataclass
class AssemblyReport:
    dropped_legs: list
    shortened_m: float


def assemble(start_pose, legs: list, area: Area, cfg, first_kind: str = "initial",
             first_meta: dict | None = None):
    """Segments: route(start_pose -> legs[0]) + legs joined by feasible connections (MATH §3.2).
    A connection that cannot be built shortens the next leg's start by connection_shorten_m
    (up to connection_retries times); a leg that still cannot be reached is dropped and
    reported. Returns (segments, report)."""
    segs = []
    dropped = []
    shortened = 0.0
    pose = tuple(start_pose)
    first = True
    i = 0
    while i < len(legs):
        leg = legs[i]
        route = None
        cur = leg
        for attempt in range(cfg.connection_retries + 1):
            route = connect(pose, cur.start_pose(), area, cfg)
            if route is not None or cur.length <= cfg.min_leg_m:
                break
            u = np.array([math.cos(cur.heading), math.sin(cur.heading)])
            new_p0 = tuple(np.asarray(cur.p0) + cfg.connection_shorten_m * u)
            cur = Leg(cur.pass_no, cur.line, cur.piece, (float(new_p0[0]), float(new_p0[1])),
                      cur.p1)
            shortened += cfg.connection_shorten_m
            route = None
        if route is None:
            if first and first_kind == "initial":
                raise MissionError("initial manoeuvre: no feasible route from the hold pose "
                                   "to the first coverage leg inside the area (closure E)")
            dropped.append(leg.leg_id)
            i += 1
            continue
        kind = first_kind if first else "connection"
        meta = dict(first_meta or {}) if first else {}
        meta.setdefault("to_leg", cur.leg_id)
        if route:
            segs.append(Segment.make(kind, route, **meta))
        segs.append(line_segment(cur.p0, cur.p1, leg=cur.leg_id, pass_no=cur.pass_no,
                                 line=cur.line))
        pose = cur.end_pose()
        first = False
        i += 1
    return segs, AssemblyReport(dropped, shortened)


@dataclass
class CoveragePlans:
    theta1: float
    band: CoverageBand
    pass1: PassLayout
    pass2: PassLayout
    c1: Plan
    c2: Plan
    reports: dict


def build_coverage(area: Area, start_pose, band: CoverageBand, cfg, t0: float = 0.0) -> CoveragePlans:
    """The C1 and C2 plans from the hold pose (MATH §3.2, §3.5)."""
    if not area.all_feasible(np.asarray([start_pose[:2]]), cfg.margin_m):
        raise MissionError("the hold position is outside the area's feasible set F_margin: "
                           "move the boat inside the mission area before ~/start (closure E)")
    theta1 = area.long_axis_direction()
    if cfg.pass1_direction == "long_axis_plus_90":
        theta1 = (theta1 + math.pi / 2.0) % math.pi
    lay1 = pass_layout(area, theta1, band, cfg, 1)
    lay2 = pass_layout(area, (theta1 + math.pi / 2.0) % math.pi, band, cfg, 2)
    legs1 = ordered_pass(lay1, start_pose, cfg)
    segs1, rep1 = assemble(start_pose, legs1, area, cfg)
    if not segs1:
        raise MissionError("pass 1 is empty")
    end1 = segs1[-1].end_pose()
    legs2 = ordered_pass(lay2, end1, cfg)
    segs2, rep2 = assemble(end1, legs2, area, cfg, first_kind="connection",
                           first_meta={"between_passes": True})
    c1 = Plan(segs1, t0, cfg.speed_mps)
    c2 = Plan(segs1 + segs2, t0, cfg.speed_mps)
    return CoveragePlans(theta1, band, lay1, lay2, c1, c2,
                         {"pass1": rep1, "pass2": rep2})


def coverage_fraction(plan: Plan, area: Area, band: CoverageBand, res: float,
                      pass_no: int | None = None) -> float:
    """Fraction of area cells (centre inside the area) with >= 1 look in the coverage band
    (MATH §3.4), using the contract look geometry b = psi + s*pi/2."""
    vx = area.vertices
    x0, y0 = vx.min(axis=0)
    x1, y1 = vx.max(axis=0)
    nx = int(math.ceil((x1 - x0) / res))
    ny = int(math.ceil((y1 - y0) / res))
    hit = np.zeros((ny, nx), dtype=bool)
    rs = np.arange(band.r_near, band.r_far + 1e-9, res / 2.0)
    for seg in plan.segments:
        if seg.kind != "line":
            continue
        if pass_no is not None and seg.info.get("pass_no") != pass_no:
            continue
        L = seg.length
        n = max(1, int(math.ceil(L / res)))
        for s in np.linspace(0.0, L, n + 1):
            x, y, psi = seg.pose_at_s(s)
            for side in (1, -1):
                b = psi + side * math.pi / 2.0
                gx = x + rs * math.cos(b)
                gy = y + rs * math.sin(b)
                ci = np.floor((gx - x0) / res).astype(int)
                cj = np.floor((gy - y0) / res).astype(int)
                m = (ci >= 0) & (ci < nx) & (cj >= 0) & (cj < ny)
                hit[cj[m], ci[m]] = True
    cx = x0 + (np.arange(nx) + 0.5) * res
    cy = y0 + (np.arange(ny) + 0.5) * res
    gx, gy = np.meshgrid(cx, cy)
    inside = area.contains(np.column_stack([gx.ravel(), gy.ravel()])).reshape(ny, nx)
    if inside.sum() == 0:
        return 0.0
    return float((hit & inside).sum() / inside.sum())
