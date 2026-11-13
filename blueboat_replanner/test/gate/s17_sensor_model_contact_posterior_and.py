"""Gate section [17]: sensor model, contact posterior and value (MATH §6-§8).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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
c_no = contact_posterior(one_contact([0.2] * 5), mod, CFG, 0, {})
c_set = contact_posterior(one_contact([0.2] * 5, mis=(3,)), mod, CFG, 0)
c_cell = contact_posterior(one_contact([0.2] * 5), mod, CFG, 0, {3: 0.7})
c_weak = contact_posterior(one_contact([0.2] * 5), mod, CFG, 0, {3: 0.05})
c_both = contact_posterior(one_contact([0.2] * 5, det=(0, 3), mis=(3, 9)), mod, CFG, 0,
                           {0: 0.7, 3: 0.7})
check("H-2: a miss read from the cell record = the same miss in sectors_missed",
      abs(c_cell.p_tilde - c_set.p_tilde) < 1e-15 and np.allclose(c_cell.lam, c_set.lam) and
      c_cell.seen_g == c_set.seen_g == frozenset({0, 3}) and
      "misses_from_sector_sets" in c_set.fallbacks and
      "misses_from_sector_sets" not in c_cell.fallbacks)
check("H-2: the recorded strength is used (a weak miss lowers p~ less than a plateau miss)",
      c_cell.p_tilde < c_weak.p_tilde < c_no.p_tilde and abs(c_no.p_tilde - 0.6) < 1e-12 and
      c_weak.missed == ((3, 0.05),) and 3 in c_weak.seen_g)
check("H-2: a sector both detected and missed counts as detected",
      c_both.missed == () and c_both.detected_g == frozenset({0, 3}))
from blueboat_replanner.core.value import cell_misses  # noqa: E402
gM = make_grid([contact(1, 40, 30)])
almM = gM["aspect_log_miss"].reshape(140, 260, 12)
almM[70, 90, 3] = math.log(1 - 0.7)
almM[70, 90, 9] = math.log(1 - 0.2)
almM[70, 90, 1] = math.log(1 - 0.1)
cmM = cell_misses(validate_grid(gM), mod, 70 * 260 + 90)
check("cell record: folded sectors with the largest recorded strength",
      set(cmM) == {1, 3} and abs(cmM[3] - 0.7) < 1e-12 and abs(cmM[1] - 0.1) < 1e-12)
cfg_fam = CFG.with_overrides({"class_families": {"pipe_pvc": "pipe"}})
mF = SensorModel(cfg_fam, ["pipe_pvc", "other"], 12, 0.0, 1.78)
mP = SensorModel(CFG, ["pipe", "tyre"], 12, 0.0, 1.78)
check("H-6: a mapped class takes its family's window; an unmapped one is isotropic and named",
      np.array_equal(mF.A_g[:, 0], mP.A_g[:, 0]) and np.all(mF.A_g[:, 1] == 1.0) and
      mF.unmapped == ["other"] and list(mF.mapped) == [True, False] and mP.unmapped == [])
mTG = SensorModel(CFG, ["block"], 12, 0.0, 1.78, {"turn_gating": {"enabled": False,
                                                                 "min_yaw_rate_rad_s": 0.2}})
check("turn gate: Belief's published switch and threshold are used when present",
      mod.turn_gate is True and mod.turn_min_rate == 0.01 and mTG.turn_gate is False and
      mTG.turn_min_rate == 0.2)
qw_iso = bgA.qw[0]
lk = background_value(bgA, mA, [0], [1], [0.7])
closed = float(np.sum(qw_iso * (1 - (1 - 0.7 * mA.s[:, None] * np.ones((1, 12))))))
check("4: A = 1 -> background value equals the aspect-agnostic closed form",
      abs(lk - closed) < 1e-15)

