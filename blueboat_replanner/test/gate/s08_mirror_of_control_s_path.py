"""Gate section [8]: mirror of Control's path and continuity.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

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

