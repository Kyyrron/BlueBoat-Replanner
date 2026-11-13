"""Gate section [20]: the look predictor and the contract reading against the REAL Belief core.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [20]
print("[20] the look predictor and the contract reading against the REAL Belief core")
bel = sibling("BlueBoat-Belief")
BELIEF_OK = bel is not None and (bel / "blueboat_belief" / "blueboat_belief" / "core" /
                                 "belief.py").is_file()
if not BELIEF_OK:
    skip("the real Belief core: predictor, contract reading, evidence horizon",
         "BlueBoat-Belief not found beside this repository")
else:
    from blueboat_replanner.core.looks import plan_poses_k  # noqa: E402
    from blueboat_replanner.eval.belief_real import (EmDet, RealBelief, T0,  # noqa: E402
                                                     belief_info, load_core)
    NAMES5 = ["block", "cage", "pipe", "plate", "tyre"]
    binfo = belief_info()
    check("Belief's core is imported from its checkout, through its offline interface",
          str(load_core().package_dir).startswith(str(bel)) and binfo["core_files"] > 5 and
          len(binfo["core_sha256"]) == 64, f"{binfo['location']}")
    print(f"        Belief revision {binfo['revision']}, uncommitted changes "
          f"{binfo['uncommitted_changes']}, core sha256 {binfo['core_sha256'][:16]}, its gate: "
          f"{binfo['gate']}")

    def real(mode="ungated", x_max=120.0, **kw):
        return RealBelief(0.0, 0.0, x_max, 60.0, (34.5, 133.5), 2.0, CFG, NAMES5, mode=mode,
                          **kw)

    def fed(plan_, mode="ungated", **kw):
        rb_ = real(mode, **kw)
        pt_, px_, py_, pp_, _ = plan_poses_k(plan_, plan_.t0, plan_.t_end, 0.05)
        rb_.feed(pt_, px_, py_, pp_)
        return rb_

    def snap_of(rb_):
        return validate_grid(grid_to_dict(rb_.grid()))

    def pairs_pred(plan_, snap_, model_, gate=True):
        pt_, px_, py_, pp_, pk_ = plan_poses_k(plan_, plan_.t0, plan_.t_end, 0.25)
        lk_ = looks_from_poses(pt_, px_, py_, pp_, snap_, model_, pk_ if gate else None)
        return lk_, set(zip(lk_.cell.tolist(), lk_.g.tolist()))

    def pairs_belief(snap_, above=0.0):
        c_, k_ = np.nonzero(snap_.miss_pd.reshape(-1, 12) > above)
        return set(zip(c_.tolist(), (k_ % 6).tolist()))

    i_line = next(i for i, sg in enumerate(cov.c2.segments) if sg.kind == "line")
    lineP = Plan(cov.c2.segments[i_line:i_line + 1], 0.0, CFG.speed_mps)
    rbL = fed(lineP)
    snapL = snap_of(rbL)
    check("a grid of the real core passes validate_grid with no fallback; its area is read",
          snapL.fallbacks == () and snapL.area_xy is not None and
          snapL.model["rho_ordering"] == "paull_pairwise" and
          snapL.model["fold_opposite"] is True and snapL.model["negative_evidence"] ==
          "ungated", f"{snapL.fallbacks}")
    modB = SensorModel(CFG, NAMES5, 12, 0.0, 1.78, snapL.model)
    lkL, A_ = pairs_pred(lineP, snapL, modB)
    B_all, B_plat = pairs_belief(snapL), pairs_belief(snapL, 0.7 - 1e-6)
    fa, fb = len(A_ & B_all) / len(A_), len(A_ & B_plat) / len(B_plat)
    check("H-7 line: > 97 % of the predicted (cell, folded sector) pairs are searched in Belief",
          fa > 0.97, f"{fa:.4f} of {len(A_)}")
    check("H-7 line: > 97 % of the pairs Belief searched at plateau strength are predicted",
          fb > 0.97, f"{fb:.4f} of {len(B_plat)}")
    print(f"        line: predicted {len(A_)}, searched in Belief {len(B_all)} (of which "
          f"{len(B_all - B_plat)} below plateau strength: Belief's range ramps, MATH 6); "
          f"agreement {fa:.4f} / {fb:.4f}")
    cfg0 = CFG.with_overrides({"lever_arm_m": 0.0})
    _, A0 = pairs_pred(lineP, snapL, SensorModel(cfg0, NAMES5, 12, 0.0, 1.78, snapL.model))
    f0 = len(A0 & B_plat) / len(B_plat)
    check("H-7: the lever arm brings the prediction closer to Belief than neglecting it",
          fb > f0, f"with {fb:.4f}, without {f0:.4f}")
    turnP = Plan(cov.c2.segments[i_line:i_line + 5], 0.0, CFG.speed_mps)
    rbT = fed(turnP)
    snapT = snap_of(rbT)
    _, AT = pairs_pred(turnP, snapT, modB)
    _, AT0 = pairs_pred(turnP, snapT, modB, gate=False)
    BT = pairs_belief(snapT)
    ft, ft0 = len(AT & BT) / len(AT), len(AT0 & BT) / len(AT0)
    check("H-7 turns: > 97 % of the predicted pairs are searched in Belief (lines and turns)",
          ft > 0.97, f"{ft:.4f} of {len(AT)}")
    check("H-7 turns: without the turn gate the predictor credits looks Belief never counts",
          ft0 < ft and len(AT0 - BT) > 10 * len(AT - BT), f"gate {ft:.4f}, no gate {ft0:.4f}")
    print(f"        path with turns ({[sg.kind for sg in turnP.segments]}): predicted "
          f"{len(AT)}, in Belief {ft:.4f}; without the gate {len(AT0)}, in Belief {ft0:.4f}; "
          f"Belief pairs not predicted {1 - len(AT & BT) / len(BT):.4f} (outer side of turns, "
          "range ramps)")
    rbL2 = fed(lineP)
    g1, g2 = rbL.grid(), rbL2.grid()
    check("determinism: the same pings give the same grid",
          all(np.array_equal(np.asarray(g1[k_]), np.asarray(g2[k_]), equal_nan=True)
              for k_ in ("coverage", "aspect_log_miss", "aspect_counts", "p_background")))
    okR, _ = mission_area_check(RECT, snapL)
    okS, whyS = mission_area_check(RECT, snap_of(real(x_max=100.0)))
    check("H-5 on the real core: the mission polygon against Belief's own area",
          okR and not okS and "outside" in whyS, whyS)
    rbU = real(overrides={"alignment": {"origin_lat": float("nan"),
                                        "origin_lon": float("nan")}})
    check("before Belief's alignment is valid the grid is refused (no plan on that map)",
          raises(lambda: validate_grid(grid_to_dict(rbU.grid())), ContractError,
                 "odom_to_map_valid"))
    rbN = fed(lineP, "records", detector_runs=False)
    rbN.finish()
    snapN = snap_of(rbN)
    bgN = build_background(snapN, modB, np.isfinite(snapN.coverage), CFG)
    check("H-1 on the real core: a detector that never ran -> looks counted, nothing searched",
          int(snapN.aspect_counts.sum()) > 1000 and not snapN.searched.any() and
          float(np.nanmax(snapN.coverage)) == 0.0 and not bgN.seen.any() and
          snapN.model["negative_evidence"] == "gated")
    # ---- contacts: what the fields mean
    tE = np.arange(0.0, 160.0, 0.1)
    zE = np.zeros_like(tE)

    def det(t_, score, uid, cls="unclassified"):
        return EmDet(t=float(t_), x=40.0, y=38.0, score=score, class_name=cls, psi=0.0, uid=uid)

    rbC = real()
    rbC.feed(tE, 20 + 0.5 * tE, zE + 30.0, zE, [det(tE[400], 0.8, "a-1")])           # east
    rbC.feed(200 + tE, 100 - 0.5 * tE, zE + 30.0, zE + math.pi)                      # west, miss
    rbC.feed(400 + tE, 20 + 0.5 * tE, zE + 46.0, zE, [det(400 + tE[400], 0.6, "a-2")])
    rbC.finish()
    snapC2 = snap_of(rbC)
    c0 = snapC2.contacts[0]
    check("real contact: first_confidence is the FIRST score, first_sector its sector; two "
          "detected looks confirm", len(snapC2.contacts) == 1 and
          abs(c0.first_confidence - 0.8) < 1e-6 and c0.first_sector == 3 and c0.state == 1 and
          c0.sectors_detected == (3, 9), f"{c0}")
    rowC, colC = snapC2.cell_of(c0.x, c0.y)
    cellC = int(rowC) * snapC2.width + int(colC)
    csC = contact_posterior(c0, modB, CFG, cellC, cell_misses(snapC2, modB, cellC))
    check("real contact: a sector both detected and missed is handled as detected",
          3 in c0.sectors_missed and 3 in c0.sectors_detected and csC.missed == () and
          csC.detected_g == frozenset({3}))
    check("class information 'none': the class probabilities stay uniform",
          np.allclose(c0.class_probabilities, 0.2))
    rbK = real()
    rbK.feed(tE, 20 + 0.5 * tE, zE + 30.0, zE, [det(tE[400], 0.8, "k-1", "pipe")])
    cK = snap_of(rbK).contacts[0]
    check("class information 'true': the reported class gains probability",
          int(np.argmax(cK.class_probabilities)) == NAMES5.index("pipe") and
          abs(cK.class_probabilities[2] - 1.8 / 5.8) < 1e-6)
    rbF = real()
    rbF.feed(tE, 20 + 0.5 * tE, zE + 30.0, zE, [det(tE[400], 0.8, "f-1")])
    tN = np.arange(0.0, 61.0, 0.1)                 # north along x = 48, 5 s past the object
    rbF.feed(200 + tN, np.zeros_like(tN) + 48.0, 10 + 0.5 * tN, np.zeros_like(tN) + math.pi / 2)
    snapF = snap_of(rbF)
    cF = snapF.contacts[0]
    rowF, colF = snapF.cell_of(cF.x, cF.y)
    cellF = int(rowF) * snapF.width + int(colF)
    csF = contact_posterior(cF, modB, CFG, cellF, cell_misses(snapF, modB, cellF))
    check("H-2 on the real core: a look that found nothing is not yet in sectors_missed "
          "(Belief books it when the look closes); the planner reads it from the cell record",
          cF.sectors_missed == () and [g_ for g_, _ in csF.missed] == [0] and
          abs(csF.missed[0][1] - 0.7) < 1e-6 and csF.p_tilde < 0.8 - 1e-3,
          f"{cF.sectors_missed} {csF.missed} {csF.p_tilde}")
    # ---- the evidence horizon (records mode: as in a live run)
    rbH = fed(lineP, "records")
    gH_ = rbH.grid()
    snapH_ = validate_grid(grid_to_dict(gH_))
    tauH = rbH.horizon(gH_)
    stride_s = rbH.stride / CFG.speed_mps
    lagH = lineP.t_end - tauH
    check("evidence horizon: the grid lags the last ping by one to two image strides",
          stride_s - 0.2 <= lagH <= 2 * stride_s + 0.2 and abs(snapH_.stamp - (T0 + tauH)) < 1e-9
          and snapH_.model["coverage_source"] == "records", f"lag {lagH:.2f} s")
    lkH, _ = pairs_pred(lineP, snapH_, modB)
    srH = snapH_.searched.reshape(-1, 12)
    inH = srH[lkH.cell, lkH.g] | srH[lkH.cell, lkH.g + 6]
    bef, aft = lkH.t <= tauH - 0.5, lkH.t > tauH + 0.5
    check("H-3: predicted looks at or before the stamp are in the grid; none after it is",
          inH[bef].mean() > 0.97 and inH[aft].mean() < 0.01 and aft.sum() > 100,
          f"before {inH[bef].mean():.4f} ({bef.sum()}), after {inH[aft].mean():.4f} ({aft.sum()})")
    rbH.finish()
    snapHf = snap_of(rbH)
    check("after finish() the whole pass is credited (the last short image is reported)",
          len(pairs_belief(snapHf)) == len(B_all))

