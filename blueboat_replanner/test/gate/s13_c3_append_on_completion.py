"""Gate section [13]: C3 append-on-completion.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [13]
print("[13] C3 append-on-completion")
pol = policy_for("C3")
plan = pol.initial_plan(cov)
dps = pol.decision_points(plan)
i1 = last_pass1_line(plan)
check("C3 starts from the C2 plan; one decision point at pass 1's end",
      plan is cov.c2 and len(dps) == 1 and dps[0][1] == i1)
cpt, di = dps[0]
B = Budget(plan.t0, plan.duration)
contacts = [contact(1, 30, 30, first_sector=0), contact(2, 90, 40, first_sector=3),
            contact(3, 60, 15, first_sector=1, state=1), contact(4, 100, 25, first_sector=0)]
snapC = validate_grid(make_grid(contacts))
ctx = EpochContext(CFG, RECT, plan, cpt, di, B, belief=snapC, coverage=cov)
dec = pol.on_epoch(ctx)
check("C3 replaces the remainder", dec.action == "REPLACE" and dec.tail is not None)
adm = dec.info.get("admitted", [])
check("confirmed contacts are not revisited", 3 not in adm and len(adm) >= 1, f"{adm}")
kinds = [s.kind for s in dec.tail.segments]
check("B-4: bodies, then a rejoin, then pass-2 lines",
      "body" in kinds and "rejoin" in kinds and
      kinds.index("rejoin") > max(i for i, k in enumerate(kinds) if k == "body") and
      all(s.info.get("pass_no") == 2 for s in dec.tail.segments if s.kind == "line"))
check("tail starts at c with the plan's pose", dec.tail.t0 == cpt and
      max(abs(a - b) for a, b in zip(dec.tail.segments[0].start_pose(),
                                       plan.pose_at(cpt))) < 1e-9)
check("tail ends within the budget", dec.tail.t_end <= B.t_limit + 1e-9)
check("tail inside F_margin", plan_in_area(dec.tail, RECT, CFG.margin_m, 0.05)[0])
tt3 = dec.tail.sample(CFG.sample_ds_m)
check("tail request ok", request_check("r1-0001", *tt3)[0] == "ok")
m0 = ControlPathMirror()
m0.splice(float("nan"), t, x, y, w, "r1-0000")
check("tail continuous with the mirror at c", continuity_check(m0, cpt, *tt3, 0.05)[0] == "ok")
ok_geom = True
for seg in dec.tail.segments:
    if seg.kind != "body":
        continue
    info = seg.info
    cobj = next(cc for cc in snapC.contacts if cc.uid == info["uid"])
    b = info["bearing"]
    side = info["side"]
    perp = geo_mod.wrap_scalar((b - (snapC.sector_centre(cobj.first_sector) + math.pi / 2)))
    ok_geom &= min(abs(perp), abs(abs(perp) - math.pi)) < 1e-9
    bx, by, bth = seg.start_pose()
    u = np.array([math.cos(bth), math.sin(bth)])
    rel = np.array([cobj.x - bx, cobj.y - by])
    dist = abs(u[0] * rel[1] - u[1] * rel[0])
    cross = u[0] * rel[1] - u[1] * rel[0]
    ok_geom &= abs(dist - CFG.r_star_m) < 1e-9 and (np.sign(cross) == side)
    ok_geom &= abs(geo_mod.wrap_scalar(bth + side * math.pi / 2 - b)) < 1e-9
check("bodies look perpendicular to the first detection, at r*, on the stated side", ok_geom)
dec2 = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=snapC, coverage=cov))
check("deterministic decision", dec2.info == dec.info and
      all(np.array_equal(a, b) for a, b in zip(dec2.tail.sample(0.25), tt3)))
dec_cap = pol.on_epoch(EpochContext(CFG.with_overrides({"rho_rev": 0.0}), RECT, plan, cpt, di,
                                    B, belief=snapC, coverage=cov))
check("revisit cap 0 -> CONTINUE", dec_cap.action == "CONTINUE")
dec_b = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, Budget(plan.t0, cpt - plan.t0 + 1.0),
                                  belief=snapC, coverage=cov))
check("no budget left -> CONTINUE", dec_b.action == "CONTINUE")
check("stale belief -> CONTINUE (run continues as C2)",
      pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=None,
                                coverage=cov)).action == "CONTINUE")
g255 = make_grid([contact(5, 40, 30, first_sector=0)])
g255["contacts"][0]["first_sector"] = 255
d255 = pol.on_epoch(EpochContext(CFG, RECT, plan, cpt, di, B, belief=validate_grid(g255),
                                 coverage=cov))
bod = [s for s in d255.tail.segments if s.kind == "body"] if d255.tail else []
check("first_sector 255 -> fallback bearing along the pass-1 lines",
      len(bod) == 1 and min(abs(geo_mod.wrap_scalar(bod[0].info["bearing"] - cov.theta1)),
                            abs(abs(geo_mod.wrap_scalar(bod[0].info["bearing"] - cov.theta1))
                                - math.pi)) < 1e-9)
used = [e["dT"] for e in dec.scored if e.get("admitted")]
check("revisit time within rho_rev * B", sum(used) <= CFG.rho_rev * B.B + 1e-9 and
      abs(dec.info["dT_revisit"] - used[-1]) < 1e-6 if used else False)

