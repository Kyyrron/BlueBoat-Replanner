"""Gate section [6]: routing inside the area.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

