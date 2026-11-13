"""Gate section [16]: background calibration (MATH §9.1).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [16]
print("[16] background calibration (MATH §9.1)")
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
check("MATH §9.1 numbers: Q = 0.25, n = 2 -> brief M = 0.403, calibrated p = 0.709",
      abs(brief - 0.403) < 5e-4 and abs(pd25[0] - 0.709) < 5e-4)
gA = make_grid([], W=40, H=30, cov_val=0.75)
set_searched(gA, sectors=[0, 3])
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
gH = make_grid([], W=40, H=30, cov_val=0.0)
gH["aspect_counts"][:] = 3                     # looks everywhere, no detector coverage
snapH = validate_grid(gH)
bgH = build_background(snapH, mA, np.ones((30, 40), bool), cfg_iso)
check("H-1: looks with no detector coverage -> no sector searched, q = p0 everywhere",
      not bgH.seen.any() and np.allclose(bgH.q, p0) and bgH.searched_source ==
      "aspect_log_miss" and bgH.no_sector_fallback == 0)
gH.pop("aspect_log_miss")
bgF = build_background(validate_grid(gH), mA, np.ones((30, 40), bool), cfg_iso)
check("field absent -> S_i from aspect_counts [FALLBACK, named]",
      bool(bgF.seen.all()) and bgF.searched_source == "aspect_counts")
gP = make_grid([], W=40, H=30, cov_val=0.75)
set_searched(gP, sectors=[0, 3])
gP["p_background"] = np.full(1200, p0 * 0.25 / (1 - p0 + p0 * 0.25))
bgP = build_background(validate_grid(gP), mA, np.ones((30, 40), bool), cfg_iso)
check("MATH 9.2: p_background against its closed form -> relative difference reported",
      bgP.pbg_rel_diff is not None and bgP.pbg_rel_diff < 1e-9 and
      bgA.pbg_rel_diff > 0.1 and bgP.info()["cells"] == 1200)

