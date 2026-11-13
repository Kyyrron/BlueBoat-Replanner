"""Gate section [22]: dev loop on the real Belief core (small scenario).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [22]
print("[22] dev loop on the real Belief core (small scenario)")
if not BELIEF_OK:
    skip("dev loop on the real Belief core", "BlueBoat-Belief not found beside this repository")
else:
    from blueboat_replanner.eval.devloop import LABEL, Scenario, run_mission  # noqa: E402
    scS = Scenario(width_m=70.0, height_m=45.0, world={"n_targets": 4, "n_clutter": 3})
    rC2 = run_mission(5, "C2", scS)
    rC3 = run_mission(5, "C3", scS)
    rC4 = run_mission(5, "C4", scS)
    rC4b = run_mission(5, "C4", scS)
    check("8: C2 sends exactly one request", rC2["requests"] == 1 and rC2["decisions"] == 0)
    check("C3: one decision", rC3["decisions"] == 1)
    check("C4: decisions taken, no refusal, no missed lead",
          rC4["decisions"] > 0 and not rC4["refusals"] and rC4["missed_lead"] == 0)
    check("C3/C4 within the budget", rC3["duration"] <= rC3["B"] + 1e-9 and
          rC4["duration"] <= rC4["B"] + 1e-9)
    check("C4 revisit time within rho_rev * B",
          rC4["revisit_used"] <= CFG.rho_rev * rC4["B"] + 1e-9)
    check("7: abandonment bounds revisits per contact",
          all(v <= CFG.n_max for v in rC4["n_rev"].values()))
    check("9: C4 deterministic on the same seed", rC4["final"] == rC4b["final"] and
          rC4["requests"] == rC4b["requests"] and rC4["anytime"] == rC4b["anytime"])
    from blueboat_replanner.eval.devloop import rand_table  # noqa: E402
    check("C4_rand's table: drawn over the grid's folded sectors, seeded",
          rand_table(7, 3) == rand_table(7, 3) and set(rand_table(7, 3)) <= {0, 1, 2} and
          len(rand_table(7, 6)) == 64 and max(rand_table(7, 6)) <= 5)
    check("paired worlds: same seed -> same world for every condition",
          rC2["B_full"] == rC4["B_full"])
    check("the dev loop runs the node's executor: it ends done, its records carry the "
          "executor's labels, the policy's compute takes no path time (no ageing)",
          all(r_["end_state"] == "done" for r_ in (rC2, rC3, rC4)) and
          rC2["record_labels"] == [] and  # C2 decides nothing
          all("model-conditional" in r_["record_labels"] for r_ in (rC3, rC4)) and
          rC4["compute_ageing_s"] == 0.0,
          f"{[(r_['end_state'], r_['end_reason']) for r_ in (rC2, rC3, rC4)]}")
    bC4 = rC4["belief"]
    check("the emulator's looks at every object = Belief's aspect_counts at its cell",
          all(r_["belief"]["look_mismatch"] == 0 and r_["belief"]["looks_tracked"] > 0
              for r_ in (rC2, rC3, rC4)), f"{[r_['belief']['look_mismatch'] for r_ in (rC2, rC3, rC4)]}")
    hC4 = bC4["health"]
    check("Belief's run health: no input lost, every ping applied, unclassified detections "
          "counted as such, images never overlap more than their neighbours",
          all(v == 0 for r_ in (rC2, rC3, rC4) for v in r_["belief"]["health"]["losses"].values())
          and hC4["stats"]["pings"] == bC4["pings"] and
          hC4["stats"]["pings_covered"] == bC4["pings"] and
          hC4["counters"].get("unknown_class") == hC4["stats"]["detections"] == bC4["detections"]
          and hC4["stats"]["records"] == bC4["images"] and hC4["stats"]["records_overlap"] == 0
          and hC4["stats"]["records_without_array"] == 0 and hC4["health"][0] == "OK",
          f"{hC4}")
    str_s = bC4["image_stride_m"] / CFG.speed_mps
    check("decisions saw the grid one to two image strides behind the boat (plus the trigger)",
          bC4["miss_credit"] == "records" and str_s - 0.5 <= bC4["horizon_lag_mean_s"] and
          bC4["horizon_lag_max_s"] <= 2 * str_s + 1.5, f"{bC4['horizon_lag_mean_s']} "
          f"{bC4['horizon_lag_max_s']}")
    check("every result says what it is: label, Belief settings, uncalibrated scores marked",
          rC4["label"] == LABEL == "real-Belief dev loop, preliminary" and
          bC4["class_info"] == "none" and bC4["model"]["detector_calibrated"] is False and
          "score-conditional" in rC4["labels"] and
          "detector.class_names" in bC4["belief_config_changed"])
    from blueboat_replanner.eval import compare as cmp_  # noqa: E402
    with tempfile.TemporaryDirectory() as tdc_:
        with open(os.path.join(tdc_, "results.jsonl"), "w") as fh:
            for r_ in (rC2, rC3, rC4):
                fh.write(__import__("json").dumps(r_) + "\n")
        byc = cmp_.load(tdc_)
        pc = cmp_.paired(byc, "C4", "C2", 1.0)
        pa = cmp_.paired(byc, "C4", "C2", 1.0, "confirmed_true", at=0.5)
    check("compare: the paired difference of two conditions on the same seeds",
          pc["n"] == 1 and pc["mean"] == rC4["final"]["score"] - rC2["final"]["score"] and
          pa["mean"] == rC4["anytime"][1]["confirmed_true"] - rC2["anytime"][1]["confirmed_true"]
          and cmp_.paired(byc, "C4", "C9", 1.0) is None and "C4 − C2" in cmp_.table([pc]))
    from blueboat_replanner.eval import runner as run_  # noqa: E402
    gridT = [{"spacing_factor": f_, "pass1_direction": "long_axis", "interlace": "auto"}
             for f_ in (0.8, 0.9, 1.0)]
    base_sc = [3, 0, 4, 1, 5, 2, 0, 3, 4, 1]               # very different worlds
    rowsT = []
    for s_ in range(10):
        rowsT.append({"k": 0, "seed": s_, "score": base_sc[s_], "duration": 2500.0})   # reference
        rowsT.append({"k": 1, "seed": s_, "score": base_sc[s_] + 1.0, "duration": 2200.0})  # best
        # shorter, and worse than the best by 0.5 on EVERY seed: not tied on the same worlds
        rowsT.append({"k": 2, "seed": s_, "score": base_sc[s_] + 0.5, "duration": 2100.0})
    selT = run_.select_c2(rowsT, gridT)
    unpaired_tied = selT["stats"][2]["score_mean"] >= selT["best"]["score_mean"] - \
        selT["best"]["se"]
    check("C2 tie rule (paired yardstick): a shorter plan that is worse on every seed is not "
          "tied, though the between-world standard error would call it tied",
          selT["best"]["k"] == 1 and selT["chosen"]["k"] == 1 and not selT["stats"][2]["tied"]
          and unpaired_tied and abs(selT["stats"][2]["diff_to_best"] - 0.5) < 1e-12 and
          selT["B_cap"] == 2500.0)
    for r_ in rowsT:
        if r_["k"] == 2:           # now better on half of the seeds, worse on the other half
            r_["score"] = base_sc[r_["seed"]] + (2.3 if r_["seed"] % 2 else -0.5)
    selT2 = run_.select_c2(rowsT, gridT)
    check("C2 tie rule: a shorter plan within one paired standard error of the best is chosen",
          selT2["best"]["k"] == 1 and selT2["stats"][2]["tied"] and selT2["chosen"]["k"] == 2)
    check("--belief-set: section.key=value pairs, typed; a malformed one is refused",
          run_.parse_belief_set("contact.confirm_distinct_sectors=true,pd.plateau=0.6") ==
          {"contact": {"confirm_distinct_sectors": True}, "pd": {"plateau": 0.6}} and
          run_.parse_belief_set("") == {} and
          raises(lambda: run_.parse_belief_set("contact=true"), SystemExit))
    rD = run_mission(5, "C2", scS, belief_overrides={"contact": {"confirm_distinct_sectors":
                                                                 True}})
    check("a Belief setting changed for a sensitivity run is recorded with the result",
          rD["belief"]["belief_config_changed"].get("contact.confirm_distinct_sectors") is True
          and rD["final"]["confirmed_true"] <= rC2["final"]["confirmed_true"] and
          rD["belief"]["look_mismatch"] == 0)
    rU = run_mission(5, "C2", scS, miss_credit="ungated")
    check("miss credit 'ungated': no image delay, labelled in the grid",
          rU["belief"]["model"]["negative_evidence"] == "ungated" and rU["belief"]["images"] == 0
          and rU["belief"]["look_mismatch"] == 0)
    rStop = run_mission(5, "C4", scS, overrides={"rho_floor": 10.0})
    check("H-8: with rho_floor > 0 the mission stops at the first decision (a hold is sent); "
          "with 0 it never does", rStop["mission_stop"] and rStop["decisions"] == 1 and
          rStop["accepted"] == 2 and rStop["duration"] < 0.5 * rC4["duration"] and
          not rC4["mission_stop"], f"{rStop['decisions']} {rStop['accepted']}")

