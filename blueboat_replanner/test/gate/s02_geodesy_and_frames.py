"""Gate section [2]: geodesy and frames.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

