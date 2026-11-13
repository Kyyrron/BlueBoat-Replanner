"""Gate section [7]: plans, sampling, Control's request rules.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

