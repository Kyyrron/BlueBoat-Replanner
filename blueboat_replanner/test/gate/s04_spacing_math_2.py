"""Gate section [4]: spacing (MATH §2).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

