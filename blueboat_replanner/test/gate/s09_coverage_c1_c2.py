"""Gate section [9]: coverage (C1, C2).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

