"""Gate section [5]: Dubins paths.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

