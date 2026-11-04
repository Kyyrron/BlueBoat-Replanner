"""Offline gate (no ROS): `python3 -m test.smoke_test` from the package root.

Linear script, numbered sections, one line per check, exit status 1 on any failure (pattern of
BlueBoat-SSS-Sim's smoke test). Checks that need a neighbour module (MCS geo, Control's
path_update) SKIP visibly when it is absent. Foreign modules are imported with bytecode
writing disabled so that no file is created in another module's tree.
"""

from __future__ import annotations

import ast
import inspect
import math
import os
import sys
import tempfile
import types
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from blueboat_replanner.core import geo as geo_mod  # noqa: E402
from blueboat_replanner.core.areas import Area, MissionError  # noqa: E402
from blueboat_replanner.core.config import (CONTROL_RULES, PARAMS, Config,  # noqa: E402
                                            ConfigError)
from blueboat_replanner.core.coverage import (build_coverage,  # noqa: E402
                                              coverage_fraction, line_visit_order)
from blueboat_replanner.core.dubins import (Arc, dubins_candidates,  # noqa: E402
                                            sample_prims)
from blueboat_replanner.core.evidence import (ContractError,  # noqa: E402
                                              validate_grid)
from blueboat_replanner.core.geo import Frame2D  # noqa: E402
from blueboat_replanner.core.mission_plan import policy_for, setup_mission  # noqa: E402
from blueboat_replanner.core.plan import (ControlPathMirror, Plan,  # noqa: E402
                                          control_eval, hold_segment)
from blueboat_replanner.core.policy.append import last_pass1_line  # noqa: E402
from blueboat_replanner.core.policy.base import Budget, EpochContext  # noqa: E402
from blueboat_replanner.core.routing import connect, prims_feasible  # noqa: E402
from blueboat_replanner.core.spacing import (coverage_band, multiplicity,  # noqa: E402
                                             nominal_spacing, spacing_for_width)
from blueboat_replanner.core.validate import (continuity_check,  # noqa: E402
                                              plan_in_area, request_check)
from blueboat_replanner.io.mission import parse_mission  # noqa: E402
from blueboat_replanner.io.trajectory_yaml import (trajectory_doc,  # noqa: E402
                                                   write_exclusive)
from blueboat_replanner.ros.adapters import grid_to_dict  # noqa: E402

PASS = FAIL = SKIP = 0


def check(name: str, cond, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok    {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def skip(name: str, why: str) -> None:
    global SKIP
    SKIP += 1
    print(f"  SKIP  {name}  ({why})")


def raises(fn, exc, field=None) -> bool:
    try:
        fn()
    except exc as e:
        return field is None or field in str(e) or getattr(e, "field", None) == field
    return False


def sibling(name: str):
    """A neighbour repository: env override, sibling of this repo's git root, or the
    superproject layout (sibling submodule)."""
    env = os.environ.get("BLUEBOAT_" + name.upper().replace("-", "_") + "_DIR")
    cands = [Path(env)] if env else []
    root = PKG_ROOT.parent
    cands += [root.parent / name, root.parent.parent / name]
    for c in cands:
        if c.is_dir() and any(c.iterdir()):
            return c
    return None


CFG = Config.from_dict({"planning_depth_m": 2.0})
RECT = Area([(0, 0), (120, 0), (120, 60), (0, 60)])
HOLD = (10.0, 10.0, 0.3)

# =========================================================================== [1]
print("[1] configuration")
check("defaults load", CFG.speed_mps == 0.5 and CFG.range_m == 20.0 and CFG.grazing_floor is False)
check("every parameter has a category and unit",
      all(s.category in "URLPD" and len(s.category) == 1 and s.unit for s in PARAMS.values()))
check("unknown key raises, naming it",
      raises(lambda: Config.from_dict({"speed": 1.0}), ConfigError, "speed"))
check("wrong type raises", raises(lambda: Config.from_dict({"n_max": 2.5}), ConfigError, "n_max"))
check("bool is not an int", raises(lambda: Config.from_dict({"k1": True}), ConfigError, "k1"))
check("out of range raises", raises(lambda: Config.from_dict({"spacing_factor": 1.5}),
                                    ConfigError, "spacing_factor"))
check("non-finite raises", raises(lambda: Config.from_dict({"margin_m": float("nan")}),
                                  ConfigError, "margin_m"))
check("choices enforced", raises(lambda: Config.from_dict({"interlace": "x"}), ConfigError))
check("class s must be in (0, 1]",
      raises(lambda: Config.from_dict({"classes": {"x": {"m": 2, "w_deg": 10, "a": 0.1,
                                                         "s": 1.2}}}), ConfigError, "classes.x"))
check("class table override", Config.from_dict(
    {"classes": {"x": {"m": 0, "s": 0.5}}}).classes["x"].s == 0.5)
check("c4_rand table parsed", Config.from_dict({"c4_rand": {"bearing_table": [1, 0, 5]}}
                                               ).bearing_table == (1, 0, 5))
check("immutable", raises(lambda: setattr(CFG, "speed_mps", 1.0), AttributeError))
check("as_dict round trip", Config.from_dict({k: v for k, v in CFG.as_dict().items()}).as_dict()
      == CFG.as_dict())

# =========================================================================== [2]
print("[2] geodesy and frames")
mcs = sibling("BlueBoat-MCS")
if mcs is not None and (mcs / "mcs/core/geo.py").is_file():
    src = (mcs / "mcs/core/geo.py").read_text()
    tree = ast.parse(src)
    ns = {"math": math}
    bodies = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name in ("latlon_to_local_en",
                                                               "local_en_to_latlon"):
            bodies[node.name] = ast.dump(node)
            exec(compile(ast.Module([node], []), "mcs_geo", "exec"), ns)
        if (isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") ==
                "EARTH_RADIUS_M"):
            exec(compile(ast.Module([node], []), "mcs_geo", "exec"), ns)
    ours = ast.parse(inspect.getsource(geo_mod))
    our_bodies = {n.name: ast.dump(n) for n in ours.body if isinstance(n, ast.FunctionDef)}
    check("MCS function bodies identical (AST)",
          all(bodies.get(k) == our_bodies.get(k) for k in ("latlon_to_local_en",
                                                           "local_en_to_latlon")))
    check("MCS EARTH_RADIUS_M identical", ns.get("EARTH_RADIUS_M") == geo_mod.EARTH_RADIUS_M)
    rng = np.random.default_rng(1)
    same = True
    for _ in range(2000):
        lat0, lon0 = rng.uniform(-60, 60), rng.uniform(-170, 170)
        lat, lon = lat0 + rng.uniform(-0.01, 0.01), lon0 + rng.uniform(-0.01, 0.01)
        a = ns["latlon_to_local_en"](lat, lon, lat0, lon0)
        b = geo_mod.latlon_to_local_en(lat, lon, lat0, lon0)
        c = ns["local_en_to_latlon"](a[0], a[1], lat0, lon0)
        d = geo_mod.local_en_to_latlon(a[0], a[1], lat0, lon0)
        same &= (a == b) and (c == d)
    check("MCS values bit-identical (2000 random points)", same)
else:
    skip("MCS geo bit-identity", "BlueBoat-MCS not found beside this repository")
rng = np.random.default_rng(2)
err = 0.0
for _ in range(500):
    lat0, lon0 = rng.uniform(-60, 60), rng.uniform(-170, 170)
    e, n = rng.uniform(-500, 500, 2)
    lat, lon = geo_mod.local_en_to_latlon(e, n, lat0, lon0)
    e2, n2 = geo_mod.latlon_to_local_en(lat, lon, lat0, lon0)
    err = max(err, abs(e2 - e), abs(n2 - n))
check("WGS-84 <-> local round trip < 1e-6 m", err < 1e-6, f"{err}")
err = 0.0
for _ in range(500):
    f = Frame2D(*rng.uniform(-100, 100, 2), rng.uniform(-math.pi, math.pi))
    x, y, p = rng.uniform(-200, 200, 3)
    xm, ym, pm = f.world_to_map(x, y, p)
    xw, yw, pw = f.map_to_world(xm, ym, pm)
    err = max(err, abs(xw - x), abs(yw - y), abs(math.atan2(math.sin(pw - p), math.cos(pw - p))))
check("frame W -> M -> W round trip < 1e-9 (rotation included)", err < 1e-9, f"{err}")
f = Frame2D(3.0, -1.0, math.pi / 2)
xm, ym, pm = f.world_to_map(1.0, 0.0, 0.0)
check("frame convention x_M = R(yaw) x_W + t",
      abs(xm - 3.0) < 1e-12 and abs(ym - 0.0) < 1e-12 and abs(pm - math.pi / 2) < 1e-12)

# =========================================================================== [3]
print("[3] areas")
cw = Area([(0, 0), (0, 10), (10, 10), (10, 0)])
check("clockwise input normalised to CCW", geo_mod.wrap_scalar(0) == 0 and
      cw.vertices[1][0] == 10.0 and abs(cw.area - 100.0) < 1e-9)
check("closing duplicate removed", len(Area([(0, 0), (10, 0), (10, 10), (0, 0)]).vertices) == 3)
check("self-intersecting raises", raises(lambda: Area([(0, 0), (10, 10), (10, 0), (0, 10)]),
                                         MissionError, "not simple"))
check("< 3 vertices raises", raises(lambda: Area([(0, 0), (1, 1)]), MissionError))
check("zero area raises", raises(lambda: Area([(0, 0), (1, 1), (2, 2)]), MissionError))
check("min area enforced", raises(lambda: Area([(0, 0), (5, 0), (5, 5), (0, 5)], min_area=100),
                                  MissionError, "minimum"))
pts = np.array([[60, 30], [-1, 30], [60, 61], [119.9, 59.9]])
check("contains", list(RECT.contains(pts)) == [True, False, False, True])
check("boundary distance exact", np.allclose(RECT.boundary_distance([[60, 30], [5, 30],
                                                                     [3, 4]]), [30, 5, 3]))
check("F_d threshold", list(RECT.feasible([[2.0, 30], [1.999, 30]], 2.0)) == [True, False])
check("long axis of 120x60 rectangle = 0", abs(RECT.long_axis_direction()) < 1e-12)
rot = math.radians(30)
R2 = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
rr = Area((R2 @ RECT.vertices.T).T)
check("long axis follows a 30 deg rotation", abs(rr.long_axis_direction() - rot) < 1e-9)
L_AREA = Area([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])
check("L-shape has one reflex vertex", L_AREA.reflex_vertices() == [3])
off = L_AREA.reflex_offset_points(5.0)
check("reflex offset point at distance >= d", len(off) == 1 and
      L_AREA.boundary_distance(off)[0] >= 5.0 - 1e-9 and bool(L_AREA.contains(off)[0]))

# =========================================================================== [4]
print("[4] spacing (MATH §2)")
e = CFG.margin_m + CFG.r_turn_m
table = {0.9: (19.98, 11.18, 11.21, 4.17), 1.0: (19.97, 11.18, 12.46, 5.17),
         1.5: (19.94, 11.15, 18.69, 10.15), 2.0: (19.90, 11.12, 19.90, 11.12),
         3.0: (19.77, 11.02, 19.77, 11.02)}
ok_tab = True
for h, (rf, s, rfg, sg) in table.items():
    depth = h + CFG.sensor_depth_m
    b0 = coverage_band(CFG, depth)
    b1 = coverage_band(CFG.with_overrides({"grazing_floor": True}), depth)
    s0 = nominal_spacing(b0, e, CFG).s
    s1 = nominal_spacing(b1, e, CFG.with_overrides({"grazing_floor": True})).s
    ok_tab &= (round(b0.r_far, 2) == rf and round(s0, 2) == s and round(b1.r_far, 2) == rfg
               and round(s1, 2) == sg)
check("MATH §2.4 table reproduced (both modes)", ok_tab)
rng = np.random.default_rng(3)
ok_rand = True
for _ in range(60):
    R = rng.uniform(5, 50)
    h = rng.uniform(0.5, 10)
    cfg = CFG.with_overrides({"range_m": R})
    band = coverage_band(cfg, h + cfg.sensor_depth_m)
    if band.width <= 1.0:
        continue
    width = rng.uniform(2 * band.r_far, 2 * band.r_far + 200)
    res = spacing_for_width(width, band, e, cfg)
    m = multiplicity(res.offsets, width, band, 0.01)
    ok_rand &= bool(m.min() >= 1) and res.s <= res.s0 + 1e-12
check("random (R, h, width): verified spacing covers every point (brute force, 0.01 m)", ok_rand)
check("area narrower than the nadir gap -> mission error (no spacing can cover it)",
      raises(lambda: spacing_for_width(2 * e + 0.3, bnd0 := coverage_band(CFG, 2.0), e, CFG),
             MissionError, "single coverage"))
check("no usable band raises", raises(lambda: spacing_for_width(
    50, coverage_band(CFG.with_overrides({"range_m": 5.0}), 10.0), e, CFG), MissionError))
bnd = coverage_band(CFG, 2.0)
check("F-2: spacing ignores ranges below r_near",
      multiplicity(np.array([0.0]), 3.0, bnd, 0.5).sum() == 0)

# =========================================================================== [5]
print("[5] Dubins paths")
rng = np.random.default_rng(4)
ok_end = ok_len = ok_g1 = True
for _ in range(1500):
    s = (*rng.uniform(-30, 30, 2), rng.uniform(-math.pi, math.pi))
    g = (*rng.uniform(-30, 30, 2), rng.uniform(-math.pi, math.pi))
    L, w, prims = dubins_candidates(s, g, 3.0)[0]
    ex, ey, eth = prims[-1].end()
    ok_end &= math.hypot(ex - g[0], ey - g[1]) < 1e-6 and abs(geo_mod.wrap_scalar(eth - g[2])) < 1e-6
    ok_len &= L >= math.hypot(g[0] - s[0], g[1] - s[1]) - 1e-9
    for a, b in zip(prims[:-1], prims[1:]):
        ax, ay, ath = a.end()
        ok_g1 &= math.hypot(ax - b.x, ay - b.y) < 1e-9 and abs(geo_mod.wrap_scalar(ath - b.th)) < 1e-9
check("shortest path reaches the goal pose (1500 random)", ok_end)
check("length >= Euclidean distance", ok_len)
check("G1 continuity at primitive joins", ok_g1)
check("straight case", abs(dubins_candidates((0, 0, 0), (10, 0, 0), 3.0)[0][0] - 10.0) < 1e-12)
check("U-turn = pi r", abs(dubins_candidates((0, 0, 0), (0, 6, math.pi), 3.0)[0][0]
                           - 3 * math.pi) < 1e-9)
arc = Arc(0.0, 0.0, 0.0, 3.0, 1, 3.0 * math.pi / 2)
x, y, th = arc.end()
check("left arc geometry (quarter turn: (3, 3), heading north)",
      abs(x - 3.0) < 1e-12 and abs(y - 3.0) < 1e-12 and abs(th - math.pi / 2) < 1e-12)

# =========================================================================== [6]
print("[6] routing inside the area")
r = connect((10, 10, 0.0), (100, 50, math.pi), RECT, CFG)
check("direct connection feasible in a rectangle", r is not None and
      prims_feasible(r, RECT, CFG.margin_m, 0.05))
st, gl = (80.0, 20.0, math.pi), (20.0, 80.0, math.pi / 2)
direct_ok = any(prims_feasible(p, L_AREA, CFG.margin_m, 0.05)
                for _, _, p in dubins_candidates(st, gl, CFG.r_turn_m))
r = connect(st, gl, L_AREA, CFG)
check("L-shape: no direct Dubins, routed around the reflex corner",
      (not direct_ok) and r is not None and prims_feasible(r, L_AREA, CFG.margin_m, 0.05))
check("goal outside the area -> None", connect((10, 10, 0), (200, 10, 0), RECT, CFG) is None)

# =========================================================================== [7]
print("[7] plans, sampling, Control's request rules")
cov = build_coverage(RECT, HOLD, coverage_band(CFG, 2.0), CFG)
t, x, y, w = cov.c2.sample(CFG.sample_ds_m, CFG.max_samples_per_request)
check("C2 request passes Control's rules (offline mirror of validate_samples)",
      request_check("r1-0000", t, x, y, w)[0] == "ok")
check("t strictly increasing >= 1 ms", bool(np.all(np.diff(t) >= 1e-3)))
spd = np.hypot(np.diff(x), np.diff(y)) / np.diff(t)
check("speed <= U", float(spd.max()) <= CFG.speed_mps * (1 + 1e-9), f"{spd.max()}")
check("B-1: sampled duration == planned duration", abs((t[-1] - t[0]) - cov.c2.duration) < 1e-9)
rng = np.random.default_rng(5)
errp = 0.0
for tt in rng.uniform(t[0], t[-1], 200):
    px, py, pth = cov.c2.pose_at(tt)
    cx, cy, cth = control_eval(t, x, y, w, tt)
    errp = max(errp, math.hypot(px - cx, py - cy))
check("samples follow the plan (Control interpolation within 1 cm)", errp < 1e-2, f"{errp}")
check("<= max samples", len(t) <= CFG.max_samples_per_request)
hp = Plan([hold_segment((5.0, 5.0, 1.0), 1.0)], 100.0, CFG.speed_mps)
th_, xh, yh, wh = hp.sample(CFG.sample_ds_m)
check("hold segment = two stationary samples, request ok",
      len(th_) == 2 and request_check("stop-1", th_, xh, yh, wh)[0] == "ok")
tr = cov.c2.truncated(cov.c2.t0 + 0.6 * cov.c2.duration)
check("truncation ends on a productive segment within the limit",
      tr.segments[-1].kind in ("line", "body", "hold") and
      tr.t_end <= cov.c2.t0 + 0.6 * cov.c2.duration + 1e-9)
bad = w.copy()
bad[10:20] += math.pi / 2       # Control takes the min over both ends: one sample is tolerated
check("heading_incoherent detected", request_check("x", t, x, y, bad)[0] == "heading_incoherent")
check("too_fast detected", request_check("x", t * 0.1, x, y, w)[0] == "too_fast")
check("bad id detected", request_check("a b", t, x, y, w)[0] == "bad_request")

# =========================================================================== [8]
print("[8] mirror of Control's path and continuity")
tt = np.array([0.0, 10.0, 20.0])
check("eval clamps before and after", control_eval(tt, [0, 5, 5], [0, 0, 5], [0, 0, 1.5], -1)
      == (0.0, 0.0, 0.0) and control_eval(tt, [0, 5, 5], [0, 0, 5], [0, 0, 1.5], 30)[:2]
      == (5.0, 5.0))
ex, ey, eth = control_eval([0, 10], [0, 0], [0, 0], [3.0, -3.0], 5.0)
check("short-way yaw across +-pi", abs(abs(eth) - math.pi) < 0.05)
mir = ControlPathMirror()
mir.splice(float("nan"), t, x, y, w, "r1-0000")
cp = 0.5 * cov.c2.duration
tail = cov.c2.from_index(cov.c2.index_at(cp) + 1)
tail = tail.retimed(cov.c2.times[cov.c2.index_at(cp)][1])
c = tail.t0
t2, x2, y2, w2 = tail.sample(CFG.sample_ds_m)
check("tail from a segment end is continuous with the mirror",
      continuity_check(mir, c, t2, x2, y2, w2, 0.05)[0] == "ok")
check("0.5 m offset -> discontinuous_position",
      continuity_check(mir, c, t2, x2 + 0.5, y2, w2, 0.05)[0] == "discontinuous_position")
check("20 deg -> discontinuous_heading",
      continuity_check(mir, c, t2, x2, y2, w2 + math.radians(20), 0.05)[0] ==
      "discontinuous_heading")
mir.splice(c, t2, x2, y2, w2, "r1-0001")
check("splice keeps the old path before c", mir.pose(c - 5.0) == control_eval(t, x, y, w, c - 5.0)
      and mir.active_id == "r1-0001")
ctrl = sibling("BlueBoat-Control")
lib = ctrl / "blueboat_control/src/_custom_libraries" if ctrl else None
if lib is not None and (lib / "path_update.py").is_file():
    sys.path.insert(0, str(lib))
    try:
        import path_update as pu  # Control's ROS-free core (read-only, not modified)
        upd = pu.PathUpdater()
        ctx = pu.Context(ready=True, path_source="replanner", tau=0.0)
        upd.reset(ctx, "launch")
        upd.set_hold((float(x[0]), float(y[0]), 0.0))
        ans, _, _ = upd.handle(pu.Request("r1-0000", float("nan"), "world", t, x, y, w), ctx)
        check("Control PathUpdater accepts the C2 first path", ans["accepted"], ans["message"])
        ctx2 = pu.Context(ready=True, path_source="replanner", tau=c - 20.0)
        ans2, _, _ = upd.handle(pu.Request("r1-0001", c, "world", t2, x2, y2, w2), ctx2)
        check("Control PathUpdater accepts our tail at a segment end", ans2["accepted"],
              ans2["message"])
        ans3, _, _ = upd.handle(pu.Request("r1-0002", c + 30.0, "world", t2, x2 + 0.5, y2, w2),
                                ctx2)
        check("Control refuses a 0.5 m offset tail like our check",
              ans3["reason"] == "discontinuous_position")
        agree = True
        rng = np.random.default_rng(6)
        for k in range(40):
            tb, xb, yb, wb = t[:200].copy(), x[:200].copy(), y[:200].copy(), w[:200].copy()
            mode = k % 4
            if mode == 1:
                wb[50] += 1.2
            elif mode == 2:
                tb = tb * 0.2
            elif mode == 3:
                tb[60] = tb[59]
            ours = request_check("q1", tb, xb, yb, wb)[0]
            theirs = pu.validate_samples(pu.Request("q1", float("nan"), "world", tb, xb, yb, wb),
                                         pu.Config())[0]
            agree &= ours == theirs
        check("our request check agrees with Control's validate_samples (40 cases)", agree)
    except Exception as exc:  # noqa: BLE001 - reported as a failure
        check("Control path_update cross-check ran", False, repr(exc))
else:
    skip("Control PathUpdater cross-check", "BlueBoat-Control not found beside this repository")

# =========================================================================== [9]
print("[9] coverage (C1, C2)")
check("C2 plan inside F_margin", plan_in_area(cov.c2, RECT, CFG.margin_m, 0.05)[0])
h1 = {round(s.start_pose()[2] % math.pi, 9) for s in cov.c2.segments
      if s.kind == "line" and s.info["pass_no"] == 1}
h2 = {round(s.start_pose()[2] % math.pi, 9) for s in cov.c2.segments
      if s.kind == "line" and s.info["pass_no"] == 2}
check("pass 2 orthogonal to pass 1", len(h1) == 1 and len(h2) == 1 and
      abs(abs(next(iter(h1)) - next(iter(h2))) - math.pi / 2) < 1e-6)
fu = coverage_fraction(cov.c2, RECT, cov.band, 0.5)
check("C2 covers >= 97 % of the area cells in the coverage band", fu >= 0.97, f"{fu:.3f}")
check("C1 is a prefix of C2", cov.c1.segments == cov.c2.segments[:len(cov.c1.segments)])
s0 = cov.c2.segments[0]
check("E-1: initial manoeuvre starts at hold with the boat's heading",
      s0.kind == "initial" and s0.start_pose() == HOLD)
check("hold outside the area -> mission error",
      raises(lambda: build_coverage(RECT, (-5.0, 10.0, 0.0), cov.band, CFG), MissionError,
             "outside"))
covL = build_coverage(L_AREA, (10.0, 10.0, 0.0), coverage_band(CFG, 2.0), CFG)
check("concave L-shape: C2 inside F_margin", plan_in_area(covL.c2, L_AREA, CFG.margin_m, 0.05)[0])
tL = covL.c2.sample(CFG.sample_ds_m)
check("concave L-shape: request ok", request_check("r1-0000", *tL)[0] == "ok")
check("interlace order visits every line once", sorted(line_visit_order(9, 3)) == list(range(9))
      and line_visit_order(5, 2) == [0, 2, 4, 3, 1])
cov_b = build_coverage(RECT, HOLD, coverage_band(CFG, 2.0), CFG)
tb = cov_b.c2.sample(CFG.sample_ds_m)
check("deterministic: identical samples on rebuild", all(np.array_equal(a, b) for a, b in
                                                          zip(tb, (t, x, y, w))))

# =========================================================================== [10]
print("[10] budget")
setup = setup_mission(CFG, "C2", [[(34.5, 133.5), (34.5, 133.5013), (34.50054, 133.5013),
                                  (34.50054, 133.5)]], 34.5, 133.5, (10.0, 10.0, 0.0))
check("B = C2 plan duration (incl. the initial manoeuvre)",
      setup.budget.B == setup.coverage.c2.duration and setup.coverage.c2.segments[0].kind ==
      "initial")
check("C1 shorter than B", setup.coverage.c1.duration < setup.budget.B)
check("area built from WGS-84 through the datum", abs(setup.area.area - 120.3 * 60.0) /
      (120 * 60) < 0.01, f"{setup.area.area}")


# =========================================================================== [11]
def make_grid(contacts=(), S=12, W=260, H=140, res=0.5, ox=-5.0, oy=-5.0, cov_val=0.3):
    n = W * H
    cv = np.full(n, cov_val)
    ac = np.zeros(n * S, dtype=np.int64)
    return {
        "contract_version": "belief_grid/0", "grid_seq": 7, "width": W, "height": H,
        "resolution": res, "origin_x": ox, "origin_y": oy, "datum_lat": 34.5,
        "datum_lon": 133.5, "odom_to_map_x": 0.0, "odom_to_map_y": 0.0,
        "odom_to_map_yaw": 0.0, "odom_to_map_sigma_m": 0.5, "odom_to_map_valid": True,
        "aspect_sector_count": S, "aspect_sector_origin_deg": 0.0,
        "class_names": ["block", "pipe", "cage", "tyre", "plate"],
        "prior_density_per_ha": 20.0, "model_params_json": '{"schema": 1}',
        "p_background": np.full(n, 0.001), "coverage": cv, "aspect_counts": ac,
        "depth_m": np.full(n, 2.0), "contacts": list(contacts)}


def contact(uid, x, y, first_sector=0, state=0, fc=0.6):
    return {"uid": uid, "x": x, "y": y, "covariance": [1.0, 0.0, 0.0, 1.0], "p_exists": 0.5,
            "state": state, "class_probabilities": [0.2] * 5, "sectors_detected": [first_sector],
            "sectors_missed": [], "n_looks_detected": 1, "first_confidence": fc,
            "first_sector": first_sector, "max_confidence": fc, "shadow_known": False,
            "has_shadow": False}


print("[11] Belief contract validation (belief_grid/0)")
G = make_grid([contact(1, 40, 30), contact(2, 80, 20, first_sector=3)])
snap = validate_grid(G)
check("valid grid passes", len(snap.contacts) == 2 and snap.coverage.shape == (140, 260))
muts = [("contract_version", lambda g: g.update(contract_version="belief_grid/1")),
        ("coverage", lambda g: g.update(coverage=g["coverage"][:-1])),
        ("aspect_counts", lambda g: g.update(aspect_counts=g["aspect_counts"][:-3])),
        ("coverage", lambda g: g["coverage"].__setitem__(5, 1.5)),
        ("odom_to_map_valid", lambda g: g.update(odom_to_map_valid=False)),
        ("p_background", lambda g: g["p_background"].__setitem__(3, np.nan)),
        ("contacts", lambda g: g.update(contacts=[contact(1, 0, 0), contact(1, 1, 1)])),
        ("covariance", lambda g: g["contacts"][0].update(covariance=[1, 0, 0, -1])),
        ("sectors_detected", lambda g: g["contacts"][0].update(sectors_detected=[3, 1])),
        ("first_sector", lambda g: g["contacts"][0].update(first_sector=12)),
        ("class_probabilities", lambda g: g["contacts"][0].update(class_probabilities=[1, 0])),
        ("datum_lat", lambda g: g.pop("datum_lat")),
        ("class_names", lambda g: g.update(class_names=["a", "a"]))]
for field, mut in muts:
    g = make_grid([contact(1, 40, 30)])
    mut(g)
    check(f"ContractError names '{field}'", raises(lambda: validate_grid(g), ContractError, field))
g = make_grid([contact(1, 40, 30, fc=float("nan"))])
g["contacts"][0]["first_sector"] = 255
g.pop("p_background")
sn = validate_grid(g)
check("degradable fields -> recorded fallbacks", "first_confidence_missing" in
      sn.contacts[0].fallbacks and "first_sector_unknown" in sn.contacts[0].fallbacks and
      "p_background_missing" in sn.fallbacks)
obj = types.SimpleNamespace(**{k: v for k, v in G.items() if k != "contacts"})
obj.contacts = [types.SimpleNamespace(**c) for c in G["contacts"]]
obj.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=12, nanosec=5e8))
sa = validate_grid(grid_to_dict(obj))
sb = validate_grid(grid_to_dict(G))
check("adapter: attribute object == dict", sa.contacts[1].uid == sb.contacts[1].uid and
      np.array_equal(sa.coverage, sb.coverage) and sa.stamp == 12.5)
check("sectors: 29.9 -> 0, 30.1 -> 1, -0.1 -> 11, centre(0) = 15 deg",
      list(snap.sector_of(np.radians([29.9, 30.1, -0.1]))) == [0, 1, 11] and
      abs(math.degrees(snap.sector_centre(0)) - 15.0) < 1e-12)

# =========================================================================== [12]
print("[12] C1 / C2 policies")
for cond in ("C1", "C2"):
    p = policy_for(cond)
    pl = p.initial_plan(cov)
    check(f"{cond}: no decision points (one request, never changed)", p.decision_points(pl) == [])
check("C2 initial plan is the two-pass plan", policy_for("C2").initial_plan(cov) is cov.c2)

# =========================================================================== [13]
print("[13] C3 append-on-completion")
pol = policy_for("C3")
plan = pol.initial_plan(cov)
dps = pol.decision_points(plan)
i1 = last_pass1_line(plan)
check("C3 starts from the C2 plan; one decision point at pass 1's end",
      plan is cov.c2 and len(dps) == 1 and dps[0][1] == i1)
cpt, di = dps[0]
B = Budget(plan.t0, plan.duration)
contacts = [contact(1, 30, 30, first_sector=0), contact(2, 90, 40, first_sector=3),
            contact(3, 60, 15, first_sector=1, state=1), contact(4, 100, 25, first_sector=0)]
snapC = validate_grid(make_grid(contacts))
ctx = EpochContext(CFG, RECT, plan, cpt, di, B, belief=snapC, coverage=cov)
dec = pol.on_epoch(ctx)
check("C3 replaces the remainder", dec.action == "REPLACE" and dec.tail is not None)
adm = dec.info.get("admitted", [])
check("confirmed contacts are not revisited", 3 not in adm and len(adm) >= 1, f"{adm}")
kinds = [s.kind for s in dec.tail.segments]
check("B-4: bodies, then a rejoin, then pass-2 lines",
      "body" in kinds and "rejoin" in kinds and
      kinds.index("rejoin") > max(i for i, k in enumerate(kinds) if k == "body") and
      all(s.info.get("pass_no") == 2 for s in dec.tail.segments if s.kind == "line"))
check("tail starts at c with the plan's pose", dec.tail.t0 == cpt and
      max(abs(a - b) for a, b in zip(dec.tail.segments[0].start_pose(),
                                       plan.pose_at(cpt))) < 1e-9)
check("tail ends within the budget", dec.tail.t_end <= B.t_limit + 1e-9)
check("tail inside F_margin", plan_in_area(dec.tail, RECT, CFG.margin_m, 0.05)[0])
tt3 = dec.tail.sample(CFG.sample_ds_m)
check("tail request ok", request_check("r1-0001", *tt3)[0] == "ok")
m0 = ControlPathMirror()
m0.splice(float("nan"), t, x, y, w, "r1-0000")
check("tail continuous with the mirror at c", continuity_check(m0, cpt, *tt3, 0.05)[0] == "ok")
ok_geom = True
for seg in dec.tail.segments:
    if seg.kind != "body":
        continue
    info = seg.info
    cobj = next(cc for cc in snapC.contacts if cc.uid == info["uid"])
    b = info["bearing"]
    side = info["side"]
    perp = geo_mod.wrap_scalar((b - (snapC.sector_centre(cobj.first_sector) + math.pi / 2)))
    ok_geom &= min(abs(perp), abs(abs(perp) - math.pi)) < 1e-9
    bx, by, bth = seg.start_pose()
    u = np.array([math.cos(bth), math.sin(bth)])
    rel = np.array([cobj.x - bx, cobj.y - by])
    dist = abs(u[0] * rel[1] - u[1] * rel[0])
    cross = u[0] * rel[1] - u[1] * rel[0]
    ok_geom &= abs(dist - CFG.r_star_m) < 1e-9 and (np.sign(cross) == side)
    ok_geom &= abs(geo_mod.wrap_scalar(bth + side * math.pi / 2 - b)) < 1e-9
check("bodies look perpendicular to the first detection, at r*, on the stated side", ok_geom)
dec2 = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=snapC, coverage=cov))
check("deterministic decision", dec2.info == dec.info and
      all(np.array_equal(a, b) for a, b in zip(dec2.tail.sample(0.25), tt3)))
dec_cap = pol.on_epoch(EpochContext(CFG.with_overrides({"rho_rev": 0.0}), RECT, plan, cpt, di,
                                    B, belief=snapC, coverage=cov))
check("revisit cap 0 -> CONTINUE", dec_cap.action == "CONTINUE")
dec_b = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, Budget(plan.t0, cpt - plan.t0 + 1.0),
                                  belief=snapC, coverage=cov))
check("no budget left -> CONTINUE", dec_b.action == "CONTINUE")
check("stale belief -> CONTINUE (run continues as C2)",
      pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=None,
                                coverage=cov)).action == "CONTINUE")
g255 = make_grid([contact(5, 40, 30, first_sector=0)])
g255["contacts"][0]["first_sector"] = 255
d255 = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=validate_grid(g255),
                                 coverage=cov))
bod = [s for s in d255.tail.segments if s.kind == "body"] if d255.tail else []
check("first_sector 255 -> fallback bearing along the pass-1 lines",
      len(bod) == 1 and min(abs(geo_mod.wrap_scalar(bod[0].info["bearing"] - cov.theta1)),
                            abs(abs(geo_mod.wrap_scalar(bod[0].info["bearing"] - cov.theta1))
                                - math.pi)) < 1e-9)
used = [e["dT"] for e in dec.scored if e.get("admitted")]
check("revisit time within rho_rev * B", sum(used) <= CFG.rho_rev * B.B + 1e-9 and
      abs(dec.info["dT_revisit"] - used[-1]) < 1e-6 if used else False)

# =========================================================================== [14]
print("[14] purity sweep")
pk = PKG_ROOT / "blueboat_replanner"
bad_imports = []
for f in list((pk / "core").rglob("*.py")) + list((pk / "io").rglob("*.py")) + \
        [pk / "ros" / "adapters.py"]:
    txt = f.read_text()
    for pat in ("import rclpy", "from rclpy", "numpy.random", "np.random", "import random",
                "import time", "from time", "datetime"):
        if pat in txt:
            bad_imports.append(f"{f.name}:{pat}")
    if f.parent.name in ("core", "policy") and ("import yaml" in txt):
        bad_imports.append(f"{f.name}:yaml")
check("core/io/adapters: no rclpy, randomness, clock; core: no yaml", not bad_imports,
      f"{bad_imports}")
forbidden = ("/sss_ai", "ground_truth", "/sim/", "/side_scan_sonar", "/sss_processor")
hits = []
for f in list((pk / "core").rglob("*.py")) + list((pk / "ros").rglob("*.py")):
    txt = f.read_text()
    hits += [f"{f.name}:{p}" for p in forbidden if p in txt]
check("online code names no detection, sonar or simulation topic (CM-9)", not hits, f"{hits}")
check("importing the core loaded no rclpy", "rclpy" not in sys.modules)

# =========================================================================== [15]
print("[15] mission file and trajectory archive")
M = {"mission": {"name": "t1", "condition": "C3",
                 "areas": [[[34.5, 133.5], [34.5, 133.5013], [34.50054, 133.5013],
                            [34.50054, 133.5]]], "config": {"planning_depth_m": 2.0}}}
check("valid mission parses", parse_mission(M).condition == "C3")


def mut_mission(fn):
    import copy
    m = copy.deepcopy(M)
    fn(m["mission"])
    return lambda: parse_mission(m)


check("unknown key raises", raises(mut_mission(lambda m: m.update(extra=1)), MissionError))
check("two areas -> [OPEN] O-8", raises(mut_mission(lambda m: m["areas"].append(
    m["areas"][0])), MissionError, "O-8"))
check("missing planning depth raises", raises(mut_mission(lambda m: m.update(config={})),
                                              MissionError, "planning_depth_m"))
check("bad condition raises", raises(mut_mission(lambda m: m.update(condition="C9")),
                                     MissionError))
check("C4_rand without table raises", raises(mut_mission(lambda m: m.update(condition="C4_rand")),
                                             MissionError, "bearing_table"))
check("unknown config key raises", raises(mut_mission(lambda m: m["config"].update(foo=1)),
                                          MissionError, "foo"))
with tempfile.TemporaryDirectory() as td:
    doc = trajectory_doc("p", t, x, y, w, 34.5, 133.5)
    p1 = write_exclusive(os.path.join(td, "plan.yaml"), doc)
    p2 = write_exclusive(os.path.join(td, "plan.yaml"), doc)
    import yaml
    back = yaml.safe_load(open(p1))
    pts = np.asarray(back["points"])
    check("archive: never overwrites (-2 suffix)", p2.endswith("plan-2.yaml"))
    check("archive: Control loader's format (format key, N x 4, increasing t)",
          back["format"] == "blueboat_trajectory/1" and pts.shape[1] == 4 and
          bool(np.all(np.diff(pts[:, 0]) > 0)))


# =========================================================================== [16]
print("[16] background calibration (closure A)")
from blueboat_replanner.core.value import (calibrate_p, build_background,  # noqa: E402
                                           background_value, contact_posterior,
                                           contact_value, detection_probability, after_miss)
from blueboat_replanner.core.model import SensorModel  # noqa: E402

s5 = np.array([1.0, 1.0, 0.9, 0.6, 0.3])
pi5 = np.full(5, 0.2)
rng = np.random.default_rng(16)
worst = 0.0
for _ in range(400):
    s_ = rng.uniform(0.05, 1.0, 5)
    pi_ = rng.dirichlet(np.ones(5))
    n_ = int(rng.integers(1, 7))
    ptrue = rng.uniform(0.0, 1.0)
    Q = float(np.sum(pi_ * (1 - ptrue * s_) ** n_))
    p, cl = calibrate_p(np.array([Q]), np.array([n_]), s_, pi_)
    M = float(np.sum(pi_ * (1 - p[0] * s_) ** n_))
    worst = max(worst, abs(M - Q))
check("A-1: heterogeneous s, reachable Q -> mixture residual = Q within 1e-12", worst <= 1e-12,
      f"{worst}")
Qs = np.array([0.25, 0.5, 0.09])
ns = np.array([2.0, 3.0, 1.0])
p1, _ = calibrate_p(Qs, ns, np.ones(5), pi5)
Qr, nr = np.array([0.25, 0.5]), np.array([2.0, 3.0])          # reachable for s_bar = 0.6
pb, _ = calibrate_p(Qr, nr, np.full(5, 0.6), pi5)
pbc, pbcl = calibrate_p(np.array([0.09]), np.array([1.0]), np.full(5, 0.6), pi5)
pn, _ = calibrate_p(np.array([0.3]), np.array([1.0]), s5, pi5)
check("A-2: closed forms (s = 1: brief formula; s = s_bar; n = 1)",
      np.allclose(p1, 1 - Qs ** (1 / ns), atol=1e-12) and
      np.allclose(pb, (1 - Qr ** (1 / nr)) / 0.6, atol=1e-12) and bool(pbcl[0]) and
      pbc[0] == 1.0 and
      abs(pn[0] - 0.7 / float(np.sum(pi5 * s5))) < 1e-12)
pc, clc = calibrate_p(np.array([0.09]), np.array([2.0]), s5, pi5)
check("A-3: unreachable Q -> p = 1, clamped, M = Q_min",
      bool(clc[0]) and pc[0] == 1.0 and abs(float(np.sum(pi5 * (1 - s5) ** 2)) - 0.132) < 1e-12)
pd25, _ = calibrate_p(np.array([0.25]), np.array([2.0]), s5, pi5)
brief = float(np.sum(pi5 * (1 - 0.5 * s5) ** 2))
check("closure A numbers: Q = 0.25, n = 2 -> brief M = 0.403, calibrated p = 0.709",
      abs(brief - 0.403) < 5e-4 and abs(pd25[0] - 0.709) < 5e-4)
gA = make_grid([], W=40, H=30, cov_val=0.75)
gA["aspect_counts"].reshape(30, 40, 12)[:, :, [0, 3]] = 1
snapA = validate_grid(gA)
cfg_iso = CFG.with_overrides({"aspect_agnostic": True})
mA = SensorModel(cfg_iso, snapA.class_names, 12, 0.0, 1.78)
bgA = build_background(snapA, mA, np.ones((30, 40), bool), cfg_iso)
p0 = 20.0 * 0.25 / 1e4
check("A-4: q = p0 Q / (1 - p0 + p0 Q) with heterogeneous s and A = 1",
      np.allclose(bgA.q, p0 * 0.25 / (1 - p0 + p0 * 0.25), rtol=1e-10))
wk = bgA.qw[0].sum(axis=1)
check("A-5: residual weight increases as detectability s decreases",
      wk[snapA.class_names.index("plate")] > wk[snapA.class_names.index("tyre")] >
      wk[snapA.class_names.index("cage")] > wk[snapA.class_names.index("block")] - 1e-15)

# =========================================================================== [17]
print("[17] sensor model, contact posterior and value (MATH §6-§8)")
mod = SensorModel(CFG, ["block", "pipe", "cage", "tyre", "plate"], 12, 0.0, 1.78)


def one_contact(cls_probs, det=(0,), mis=(), fs=0, q0=0.6):
    from blueboat_replanner.core.evidence import Contact
    return Contact(1, 50.0, 30.0, np.eye(2), 0.5, 0, np.asarray(cls_probs, float), tuple(det),
                   tuple(mis), q0, fs, q0)


def sector_values(cs):
    return [contact_value(cs, mod, CFG, {g: 0.7}) if g not in cs.seen_g else None
            for g in range(6)]


cs_iso = contact_posterior(one_contact([0, 0, 0, 1, 0]), mod, CFG, 0)
vi = [v for v in sector_values(cs_iso) if v is not None]
check("1: isotropic contact values every unseen sector equally", max(vi) - min(vi) < 1e-12)
cs_cage = contact_posterior(one_contact([0, 0, 1, 0, 0]), mod, CFG, 0)
vc = sector_values(cs_cage)
check("2: cage seen at sector 0 -> best next folded sector 3 (90 deg), above 1 and 2",
      int(np.nanargmax([v if v is not None else -9 for v in vc])) == 3 and vc[3] > vc[1] and
      vc[3] > vc[2])
cs_pipe = contact_posterior(one_contact([0, 1, 0, 0, 0]), mod, CFG, 0)
vp = [v for v in sector_values(cs_pipe) if v is not None]
check("3: pipe (narrow window) seen at 0 -> all unseen sectors equal (floor)",
      max(vp) - min(vp) < 1e-12)
check("5: a seen sector adds no value", contact_value(cs_cage, mod, CFG, {}) == 0.0)
rng = np.random.default_rng(17)
okb = okm = True
for _ in range(200):
    cp = rng.dirichlet(np.ones(5))
    det = tuple(sorted(set(int(v) for v in rng.integers(0, 12, rng.integers(1, 3)))))
    cs_r = contact_posterior(one_contact(cp, det=det, fs=det[0], q0=float(rng.uniform(.05, .95))),
                             mod, CFG, 0)
    free = [g for g in range(6) if g not in cs_r.seen_g]
    v = contact_value(cs_r, mod, CFG, {g: 0.7 for g in free})
    okb &= -CFG.kappa - 1e-12 <= v <= 1 + 1e-12 and abs(cs_r.lam.sum() - 1) < 1e-12 and \
        0 <= cs_r.p_tilde <= 1
    if len(free) >= 2:
        a = detection_probability(cs_r, mod, CFG, {free[0]: 0.7})
        b = detection_probability(cs_r, mod, CFG, {free[0]: 0.7, free[1]: 0.7})
        okm &= b >= a - 1e-12
check("13: V in [-kappa, 1], sum(lambda) = 1, p~ in [0, 1] (200 random contacts)", okb)
check("14: detection probability non-decreasing as sectors are added", okm)
cs_nan = contact_posterior(one_contact([0.2] * 5, q0=float("nan")), mod, CFG, 0)
check("fallback: q0 NaN -> p~ = p_exists", cs_nan.p_tilde == 0.5)
qw_iso = bgA.qw[0]
lk = background_value(bgA, mA, [0], [1], [0.7])
closed = float(np.sum(qw_iso * (1 - (1 - 0.7 * mA.s[:, None] * np.ones((1, 12))))))
check("4: A = 1 -> background value equals the aspect-agnostic closed form",
      abs(lk - closed) < 1e-15)

# =========================================================================== [18]
print("[18] union = sequential identity (closure C) and the worked example")
rng = np.random.default_rng(18)
wi = 0.0
for _ in range(100):
    cp = rng.dirichlet(np.ones(5))
    cs_r = contact_posterior(one_contact(cp, det=(int(rng.integers(0, 12)),), fs=None or 0,
                                         q0=float(rng.uniform(.05, .95))), mod, CFG, 0)
    free = [g for g in range(6) if g not in cs_r.seen_g]
    if len(free) < 3:
        continue
    N1 = {free[0]: float(rng.uniform(.2, .7))}
    N2 = {free[1]: float(rng.uniform(.2, .7)), free[2]: float(rng.uniform(.2, .7))}
    U = contact_value(cs_r, mod, CFG, {**N1, **N2})
    pm = 1 - detection_probability(cs_r, mod, CFG, N1)
    Sq = contact_value(cs_r, mod, CFG, N1) + pm * contact_value(after_miss(cs_r, mod, CFG, N1),
                                                                 mod, CFG, N2)
    wi = max(wi, abs(U - Sq))
check("C-1: V(N1 u N2) = V(N1) + P(miss) V'(N2) (contacts, 100 random)", wi < 1e-12, f"{wi}")
B_, r_ = 100.0, 0.02
J = lambda v, t: v + r_ * (B_ - t)   # noqa: E731
rateA, rateB = 1.0 / 10, 0.85 / 10
own_A = J(1.0 + 0.5, 20)
own_B = max([(0.3 / 1, J(0.85 + 0.3, 11)), (1.2 / 5, J(0.85 + 1.2, 15))])[1]
comb_B = max(J(0.85 + 0.3, 11), J(0.85 + 1.2, 15))
check("C-2: worked example: individual rate -> A; a2 by own rate -> A; combined -> B",
      rateA > rateB and own_A > own_B and comb_B > own_A and abs(comb_B - 3.75) < 1e-12 and
      abs(own_A - 3.1) < 1e-12 and abs(own_B - 2.93) < 1e-12)

# =========================================================================== [19]
print("[19] sequence evaluator: decomposition = direct union value (closure B)")
from blueboat_replanner.core.sequence import Evaluator  # noqa: E402
from blueboat_replanner.core.looks import (Looks, plan_looks, union,  # noqa: E402
                                           looks_from_poses, plan_poses)
gS = make_grid([contact(1, 40, 30, first_sector=0), contact(2, 80, 40, first_sector=4)])
rngS = np.random.default_rng(19)
gS["coverage"] = np.clip(rngS.uniform(0, 0.9, gS["coverage"].size), 0, 1)
acS = gS["aspect_counts"].reshape(-1, 12)
acS[rngS.uniform(size=acS.shape) < 0.15] = 1
gS["p_background"] = np.full(gS["coverage"].size, 0.001)
snapS = validate_grid(gS)
idx_dp = last_pass1_line(cov.c2)
kS = Plan(cov.c2.segments[idx_dp + 1:], 0.0, CFG.speed_mps)
Brem = kS.duration * 0.8
evS = Evaluator(CFG, snapS, RECT, kS, Looks.empty(), Brem)
worst = 0.0
for k in range(6):
    st = cov.c2.segments[idx_dp].end_pose()
    gl = (float(rngS.uniform(20, 100)), float(rngS.uniform(15, 45)), float(rngS.uniform(-3, 3)))
    pr = connect(st, gl, RECT, CFG)
    if not pr:
        continue
    from blueboat_replanner.core.plan import Segment as _Seg  # noqa: E402
    dp_ = Plan([_Seg.make("transit", pr)], 0.0, CFG.speed_mps)
    pl = evS.piece_looks(dp_, 0.0)
    Tp = dp_.duration
    fast = evS.sequence_value(pl, Tp)["J"]
    cut = evS.cut_of(evS.tail(0)["plan"], Brem - Tp)
    Kl = evS.tail(0)["looks"]
    m = Kl.t <= cut + 1e-9
    direct = evS.union_value(union(pl, Looks(Kl.cell[m], Kl.g[m], Kl.pd[m], Kl.t[m] + Tp)))
    worst = max(worst, abs(fast - direct))
check("J by decomposition = direct union value (6 random detours)", worst < 1e-9, f"{worst}")
cont0 = evS.sequence_value(Looks.empty(), 0.0)["J"]
Kl = evS.tail(0)["looks"]
cut0 = evS.cut_of(evS.tail(0)["plan"], Brem)
m0 = Kl.t <= cut0 + 1e-9
check("B-3: CONTINUE with no detour = value of K truncated alone",
      abs(cont0 - evS.union_value(Looks(Kl.cell[m0], Kl.g[m0], Kl.pd[m0], Kl.t[m0]))) < 1e-9)
evP = Evaluator(CFG, snapS, RECT, kS, pl, Brem)
check("C-5: looks that are all pending have zero value",
      abs(evP.union_value(pl)) < 1e-15 and evS.union_value(pl) > 0)

# =========================================================================== [20]
print("[20] look predictor vs the contract look geometry (reference)")
from blueboat_replanner.eval.belief_reference import ReferenceBelief  # noqa: E402
rbT = ReferenceBelief(RECT, 0.5, (34.5, 133.5), ["block", "pipe", "cage", "tyre", "plate"], 12,
                      0.0, 20.0, 2.0)
segP = Plan(cov.c2.segments[1:4], 0.0, CFG.speed_mps)
pt, px, py, pp = plan_poses(segP, 0.0, segP.t_end, 0.25)
grid_like = types.SimpleNamespace(width=rbT.width, height=rbT.height, resolution=0.5,
                                  origin_x=rbT.origin_x, origin_y=rbT.origin_y)
modT = SensorModel(CFG, ["block"], 12, 0.0, 1.78)
pred = looks_from_poses(pt, px, py, pp, grid_like, modT)
A_set = set(zip(pred.cell.tolist(), pred.g.tolist()))
ref = rbT.look_events(pt, px, py, pp, 0)
B_set = {(r[0] * rbT.width + r[1], r[2] % 6) for r in ref}
inter = len(A_set & B_set)
check("predictor vs reference > 97 % both ways (cell, folded sector)",
      inter / max(1, len(A_set)) > 0.97 and inter / max(1, len(B_set)) > 0.97,
      f"{inter}/{len(A_set)}/{len(B_set)}")
bel = sibling("BlueBoat-Belief")
skip("predictor vs the REAL Belief core (> 97 % both ways)",
     "BlueBoat-Belief core not importable in this workspace [NOT RUN — needs in-machine session]"
     if bel is None else "binding not written yet (L-B7)")

# =========================================================================== [21]
print("[21] Control stub vs Control's PathUpdater")
from blueboat_replanner.eval.control_stub import ControlStub  # noqa: E402
stub = ControlStub((float(x[0]), float(y[0]), 0.0))
a1 = stub.handle("r1-0000", float("nan"), t, x, y, w, 0.0)
a2 = stub.handle("r1-0001", c, t2, x2, y2, w2, c - 20.0)
a3 = stub.handle("r1-0002", c + 30.0, t2, x2 + 0.5, y2, w2, c - 20.0)
a4 = stub.handle("r1-0003", c + 1.0, t2, x2, y2, w2, c - 1.0)
a5 = stub.handle("r1-0001", c + 40.0, t2, x2, y2, w2, c - 20.0)
check("stub: first path, tail ok; offset -> discontinuous_position; lead_too_short; duplicate",
      a1["accepted"] and a2["accepted"] and a3["reason"] == "discontinuous_position" and
      a4["reason"] == "lead_too_short" and a5["reason"] == "duplicate_id")
mir_before = list(stub.mirror.segments)
stub.handle("r1-0009", c + 30.0, t2, x2 + 0.5, y2, w2, c - 20.0)
check("stub: a refusal leaves the path unchanged", stub.mirror.segments == mir_before)
if lib is not None and (lib / "path_update.py").is_file():
    import path_update as pu  # noqa: E402
    upd = pu.PathUpdater()
    ctx0 = pu.Context(ready=True, path_source="replanner", tau=0.0)
    upd.reset(ctx0, "launch")
    upd.set_hold((float(x[0]), float(y[0]), 0.0))
    reqs = [("r1-0000", float("nan"), (t, x, y, w), 0.0),
            ("r1-0001", c, (t2, x2, y2, w2), c - 20.0),
            ("r1-0002", c + 30.0, (t2, x2 + 0.5, y2, w2), c - 20.0),
            ("r1-0003", c + 1.0, (t2, x2, y2, w2), c - 1.0),
            ("r1-0001", c + 40.0, (t2, x2, y2, w2), c - 20.0)]
    stub2 = ControlStub((float(x[0]), float(y[0]), 0.0))
    agree = True
    for rid, cc, arr, tau_ in reqs:
        theirs, _, _ = upd.handle(pu.Request(rid, cc, "world", *arr),
                                  pu.Context(ready=True, path_source="replanner", tau=tau_))
        ours = stub2.handle(rid, cc, *arr, tau_)
        agree &= theirs["accepted"] == ours["accepted"] and theirs["reason"] == ours["reason"]
    check("stub == Control PathUpdater on the same 5 requests (accept / refusal codes)", agree)
else:
    skip("stub vs Control PathUpdater", "BlueBoat-Control not found")

# =========================================================================== [22]
print("[22] dev loop (small scenario, reference belief)")
from blueboat_replanner.eval.devloop import Scenario, run_mission  # noqa: E402
scS = Scenario(width_m=70.0, height_m=45.0, world={"n_targets": 4, "n_clutter": 3})
rC2 = run_mission(5, "C2", scS)
rC3 = run_mission(5, "C3", scS)
rC4 = run_mission(5, "C4", scS)
rC4b = run_mission(5, "C4", scS)
check("8: C2 sends exactly one request", rC2["requests"] == 1 and rC2["decisions"] == 0)
check("C3: one decision", rC3["decisions"] == 1)
check("C4: decisions taken, no refusal, no missed lead",
      rC4["decisions"] > 0 and not rC4["refusals"] and rC4["missed_lead"] == 0)
check("C3/C4 within the budget", rC3["duration"] <= rC3["B"] + 1e-9 and
      rC4["duration"] <= rC4["B"] + 1e-9)
check("C4 revisit time within rho_rev * B", rC4["revisit_used"] <= CFG.rho_rev * rC4["B"] + 1e-9)
check("7: abandonment bounds revisits per contact",
      all(v <= CFG.n_max for v in rC4["n_rev"].values()))
check("9: C4 deterministic on the same seed", rC4["final"] == rC4b["final"] and
      rC4["requests"] == rC4b["requests"])
check("paired worlds: same seed -> same world for every condition",
      rC2["B_full"] == rC4["B_full"])


# =========================================================================== [23]
print("[23] executor against the Control stub (fake clock; closure D, G)")
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
                            req["yaw"], self.clock["tau"]))


def progress_of(stub_, tau, override=None):
    d = stub_.progress(tau)
    d["refusal_now"] = "" if override is None else override
    if override == "operator_override":
        d["state"] = "override"
    if override == "not_ready":
        d["state"] = "not_ready"
    d["lead_min"] = stub_.lead_min(tau)
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
check("ROS and launch files compile (syntax only) [runtime NOT RUN — needs in-machine session]",
      compiled)

# ---------------------------------------------------------------------------------------------
print("[24] replanner_node over STAND-IN rclpy modules (wiring only; NOT ROS integration)")
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
        rn.time = _NS(time=lambda: fake_now["t"], strftime=time.strftime)
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
             "status.jsonl", "events.jsonl", "plans"} <= set(files24) and
            len(os.listdir(os.path.join(run_dir, "plans"))) == 2 and
            len(reqs24) == 2 and all(r.get("answer", {}).get("accepted") for r in reqs24) and
            run_dir.startswith(os.path.join(td24, "data", "Replanner_data")), files24)
        resp = node.srvs["/replanner/stop"][1](_FakeTrigger.Request(), _FakeTrigger.Response())
        node_ok["stop_when_done"] = not resp.success

        # ---- the detection emulator node (eval/ only), same stand-in modules
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
        for k in range(n_pass):                       # east, west, east, west along y = 0
            psi = 0.0 if k % 2 == 0 else math.pi
            xs = np.arange(0.0, 60.0, 0.025)
            for i, xx in enumerate(xs if k % 2 == 0 else xs[::-1]):
                pg = _NS(robot_x=float(xx), robot_y=0.0, water_depth=2.22,
                         robot_orientation=_NS(x=0.0, y=0.0, z=math.sin(psi / 2),
                                               w=math.cos(psi / 2)),
                         port_stamp=_NS(sec=0, nanosec=0),
                         starboard_stamp=_NS(sec=100 + k, nanosec=i))
                sub_cb(pg)
        sent24 = emn.pubs["/sss_ai/detections"].sent
        dets24 = [d for a in sent24 for d in a.detections]
        near = all(min(math.hypot(d.bbox.center.position.x - o["x"],
                                  d.bbox.center.position.y - o["y"]) for o in objs24) < 3.0
                   for d in dets24)
        ids24 = [d.id for d in dets24]
        node_ok["em_draws"] = (emn.world.looks == {0: n_pass, 1: n_pass},
                               f"looks {emn.world.looks}")
        node_ok["em_dets"] = (len(dets24) >= 1 and near and len(set(ids24)) == len(ids24) and
                              all(a.header.frame_id == "odom" and a.header.stamp.sec >= 100
                                  for a in sent24) and
                              all(0.0 < d.results[0].hypothesis.score < 1.0 and
                                  d.results[0].pose.pose.position.x ==
                                  d.bbox.center.position.x for d in dets24),
                              f"{len(dets24)} detections")
        _FakeNode.OVERRIDES = {"world_dir": wdir, "output": "image_records"}
        node_ok["em_records"] = raises(lambda: em.EmulatorNode(), SystemExit, "image_records")

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
                   ("em_draws", "emulator: one draw per object per pass (4 passes, target and "
                                "clutter)"),
                   ("em_dets", "emulator: detections near true objects, unique ids, frame odom, "
                               "stamp from the present side, Perception's fill convention"),
                   ("em_records", "emulator: output image_records raises [OPEN Q-9]"),
                   ("launch", "launch files: one node each, every argument a declared node "
                              "parameter with the same type")]:
    check(f"{label} [stand-in rclpy]", _ok(node_ok.get(key, False)), _why(node_ok.get(key, "")))
if "exception" in node_ok:
    check("replanner_node raised under the stand-in rclpy", False, _why(node_ok["exception"]))
check("stand-in modules removed afterwards", all(
    sys.modules.get(n) is _saved_mods[n] for n in _FAKE_NAMES))

print(f"\nSMOKE: {PASS} passed, {FAIL} failed, {SKIP} skipped")
sys.exit(1 if FAIL else 0)
