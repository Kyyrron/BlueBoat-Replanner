# Development loop — method and results

> **Real-Belief dev loop, preliminary. Development evidence only, never E4.**
>
> - The belief map is **BlueBoat-Belief's own core**, driven through its documented offline
>   interface (`REAL_BELIEF_BINDING.md`). It is still offline: no ROS, no sonar image, no detector.
> - Belief's PD and ρ, and the planner's aspect windows, are **assumed** placeholders.
> - The detector is **emulated**, and the world's detection model consists of placeholders
>   matched to the planner's. It is not fitted from Gazebo, so per synthesis §6.2 ⚠ no number
>   here may enter the thesis as a result.
> - Tracking is perfect and decisions take no mission time.
> - For E4 everything is re-run in Gazebo with a trained, calibrated detector.

**State on 2026-11-13:** the loop is built and gated on Belief's core (`test/smoke_test.py`
[20], [22]); the result runs of §5 were made on 2026-11-12 (690 missions, plus 150 in §5.9).
Since 2026-11-13 the loop's missions are executed by the node's own executor; 40 stored
missions re-run that way equal their stored rows (§1), so §5 stands as it is.

## 1. What runs

```
world (eval/world.py) ── pings along the executed path ──► BlueBoat-Belief core (eval/belief_real.py)
   ▲      │ looks at world objects (eval/emulator.py)            │ belief_grid/0 (to_grid_dict)
   │      └─► one detection draw per look ─► images ─────────────┘ header stamp = evidence horizon
   │                                                              ▼
   └──────────── executed path ◄── Control stub ◄── the node's executor (ros/executor.py)
                                                      └─ replanner policy (core/policy/*)
```

**Execution model** (`eval/devloop.py`):
- **Tracking.** The boat flies the accepted path exactly at nominal speed (τ = time), simulated in
  1 s steps, with one ping every 0.1 s (0.05 m of track).
- **First path.** Every condition sends the same first path (the C1 or C2 plan from the hold)
  through the same stub.
- **Decisions.** `ros/executor.Executor`, the node's own code, runs every mission against the
  stub (DESIGN §5.4): trigger at t_dp − (lead_min + T_decide + margin), the policy, the
  pre-send checks (lead, Control's request rules, the area, the junction), the request, the
  splice, the budget accounting and the refusal handling. Each 1 s step the loop feeds it the
  stub's progress (with `ros_time` on Belief's clock), the odometry (the mirror pose) and, where
  it will read one (the start, a due decision), Belief's grid; then it ticks it once.
- **What is not the node's.** A policy computes in zero path time, so the ageing of
  `min_change_point` by the compute time runs with 0 (`compute_ageing_s` = 0 in every row) and
  results do not depend on machine load. Answers come at once, so the answer time-outs never
  fire. `decision_time_*_s` is the wall time of each deciding tick, measured outside the
  executor.
- **What a decision sees.** The newest grid of Belief, whose header stamp says up to which
  path time it is complete. The planner treats later looks as pending (MATH §11.3). No fixed
  delay is emulated: the delay is Belief's own.

**The executor and the stored rows (2026-11-13).** When the loop was moved onto the executor,
40 stored missions were re-run and compared field by field with their rows (everything except
wall and decision times):
- the sample: from `main`, seeds 2000, 2007, 2015 and 2029 × the 8 conditions; from
  `budget_sweep`, 4 at f = 0.75 and 0.5; 2 from `class_true`; 2 from `ungated`;
- the result: **40 of 40 identical**, floats included (largest relative difference 0). Run three
  times: after the move, after the grid feed was limited to where it is read, and after the
  [P2] refactors (the admission helper, the next-line helper, the C4_rand table drawn over the
  grid's G);
- a control run of the same script on the code before the change gave 40 of 40 as well;
- wall time over the 40, on 4 workers: 663 s before, 580–676 s after.

The stored rows were made on Belief's core `fcd9531597e593b3…` and re-run on `ac686bf69cf07f6a…`
(comments only, `REAL_BELIEF_BINDING.md`). Rows written since then carry four more fields:
`end_state`, `end_reason`, `record_labels` (every label of the executor's records) and
`compute_ageing_s`.

## 2. Belief (BlueBoat-Belief's core)

Belief's model is Belief's (`BlueBoat-Belief/blueboat_belief/docs/MATH.md`). What matters here:

| Item | In Belief |
|---|---|
| Grid | 0.5 m cells over its survey area: the scenario rectangle, about the scenario's datum |
| Look | a run of pings from one directed aspect sector with gaps ≤ 20 s. There is no "pass" |
| PD | trapezoid: 0.7 on [3, 25] m from the transducer, 3 m ramps (a placeholder) |
| ρ | Paull's pairwise rule on the folded line angle: the same line adds nothing, a perpendicular line is independent, linear between (an assumption of the source) |
| Turns | the inner side beyond r* = v/\|ω\| counts for nothing |
| Contacts | association inside 2 m and a χ² gate; **confirmed on two detected looks**, not necessarily from different sectors; a miss is booked when its look closes |
| Coverage | order-dependent beyond two looks: a planning aid, never a metric between conditions |

**Settings changed from Belief's defaults**, all written into every manifest
(`belief_config_changed`):
- `alignment.origin_lat/lon` = the datum, `override_sigma_m` 0: the dev loop's poses are already
  in the map frame;
- `detector.class_names`: the five planner families, without `other` (see "class information");
- `detector.calibrated` and `detector.base_rate`: true and the real fraction of the calibration
  detections once the emulator's confidences are calibrated (§3); false and 0.5 before;
- `coverage.source: records` (main runs) or `coverage.require_detector: false` (the sensitivity
  run).

Belief's prior density stays at its default, 5 /ha. The scenario holds 10 targets per hectare.

**How a miss is credited** (`--miss-credit`):
- `records` (main runs): as in a live run. Pings are cut into seabed images of 256 rows at a
  stride of 128 rows (2.13 m of track at 20 m range). For every image the loop feeds one
  detection array, empty ones included, then one image record. Belief credits a ping once the
  images containing it are reported: the grid lags the boat by one to two strides (4 to 9 s).
- `ungated` (sensitivity): every ping counts as searched at once.

## 3. World and emulator

**World:**
- 100 × 60 m area at 2 m depth; hold pose (12, 12, 0);
- 6 targets: classes uniform over block, pipe, cage, tyre, plate; continuous orientation uniform
  on [0, 2π);
- 6 clutter objects;
- every object at least 8 m from the edge.

**World detection probability:** PD = 0.7 · s_k · A_k(b − φ) on [3, min(25, r_geo)] m from the
transducer, with the planner's placeholder windows. Clutter is detected with probability 0.2 per
look.

**Draws:**
- **One draw per look at an object**, the look being defined as Belief defines it, from the ping
  poses only (`eval/emulator.py`): a run of pings whose samples fall in the object's cell, from
  one directed sector, with gaps ≤ 20 s, after the turn gate. The draw is made at the first ping
  of the look where the world's PD is above zero.
- After every mission the looks the emulator counted at each object are compared with Belief's
  published `aspect_counts` at that cell (`look_mismatch` in the result; 0 in every gate mission).
- Detection position noise σ = 0.5 m. The detection's covariance follows Perception's
  convention: it contains the assumed navigation σ of 1 m, which Belief removes as common-mode.
- Raw score: Beta(4, 2) for a target, Beta(2, 4) for clutter.

**Common random numbers.** Every draw comes from `SeedSequence([seed, 2, object, look number])`.
So paired conditions share the world (`SeedSequence([seed, 1])`), and the k-th look at an object
gets the same draw whatever path produced it.

**Calibration** (brief §9.4). Raw scores are mapped to confidences by Platt scaling, fitted on C2
runs of the calibration seeds 900–919 with the frozen C2 settings. The ECE of the evaluation
runs is reported.

**Class information** (`--class-info`), a world-model choice (owner, 2026-11-12: both, apart):
- `none` (main runs): the emulated detector gives no class. Every detection is `unclassified`;
  Belief counts it and leaves the class probabilities uniform over the five families. The
  planner then chooses bearings on a uniform class mixture.
- `true` (a second, smaller run): a perfect classifier. A target reports its class; a clutter
  object reports one fixed class drawn for it, so the class says nothing about real against
  clutter. This is an upper bound on what class knowledge buys.

## 4. Protocol

- **Unit:** one mission (one world).
- **n:** the number of evaluation seeds (paired across conditions).
- **Seeds:** tuning 1000–1019; calibration 900–919; evaluation 2000 onwards. All disjoint.
- **C2 first.** C2 is tuned and frozen per `C2_TUNING_PROTOCOL.md` (dev-loop freeze,
  preliminary), and every condition uses the frozen coverage settings. The runner refuses C3/C4
  without the freeze, never overwrites a freeze, and every result row carries its sha256.
- **Budget.** B = the duration of the frozen C2 plan from the hold. Every plan is truncated at
  f·B (budget sweep f ∈ {1.0, 0.75, 0.5}). C1 ends earlier, and its metrics are held to the end.
- **Metrics,** against world truth, eval only:
  - confirmed contacts are matched one-to-one to true targets, greedy nearest within 2 m;
  - **score** = confirmed true − κ·false confirmations (κ = 1, the E4 primary quantity);
  - **detected** = targets matched by any contact;
  - **anytime** = the score at 25/50/75/100 % of the budget, from the grid Belief has then;
  - **targets confirmed only here / only by C2**, per seed;
  - aspects per target;
  - decision time (wall, eval only);
  - refusals and missed leads;
  - Belief side: the grid's lag at the decisions, looks tracked and how many differ, draws on one
    object less than a look time-out apart.
- **Paired statistics vs C2:** mean ± sd of the difference, ≈ 95 % CI (mean ± 2 sd/√n), and
  better / equal / worse counts.
- **Exploratory.** Every ablation is exploratory. No sample size was chosen for power, and no
  significance is claimed.
- **Failures.** A mission that raises is reported, not dropped.

**Batches** (`.claude/tools/devloop_batches.sh`, in this order; it stops before a batch if a
simulation recording is running, and resumes where it stopped):

| Folder | What | Missions |
|---|---|---|
| `c2_tuning/` | the C2 grid on the tuning seeds; writes the freeze | 400 |
| `main/` | 8 conditions × 30 seeds, full budget, no class information (calibration first: 20) | 240 |
| `budget_sweep/`, `budget_merged/` | C2, C3, C4, C4_iso, C4_rand at f = 0.75 and 0.5, 20 seeds; merged with `main/` | 200 |
| `class_true/` | C2, C4, C4_iso, C4_rand with the true class, 30 seeds | 120 |
| `ungated/` | C2, C3, C4, C4_rand with misses credited at once, 30 seeds | 120 |
| `timing_uncontended/` | C4 and C4_depth1, 5 seeds, one process | 10 |

Measured on 2026-11-12 on the laptop: the whole set took 45 minutes on 4 workers (system
python, numpy 1.26.4); one mission of this scenario takes 7 s (C2) to 16 s (C4) alone.

## 5. Results — real-Belief dev loop, preliminary

Run on 2026-11-12, 19:27–20:12, on the operator laptop, by `.claude/tools/devloop_batches.sh`
(4 workers; no simulation recording was running). Outputs: `results/devloop_real_2026-11-12/`.
The numbers below are copied from the `summary.md` / `summary.json` of each folder and from
`eval/compare.py` on their `results.jsonl`.

- Module revision `c53fe2a+uncommitted`; `/usr/bin/python3` 3.12.3, numpy 1.26.4.
- Belief: revision `093841a` with uncommitted changes, core sha256 `fcd9531597e593b3…`
  (its own gate: 240 checks, 0 skipped). Belief's sources changed at 20:19, after these runs
  (`ac686bf69cf07f6a…`, comments only): 40 missions of the main run re-run on them are
  identical to their stored rows, and in them Belief reports no lost input
  (`REAL_BELIEF_BINDING.md`).
- Frozen C2: `config/c2_tuned.yaml`, sha256 `f0bba92ffd4168bb…`: f = 1.0, pass 1 along the long
  axis, interlace auto. B(C2) = 2111 s from the hold. The tuning grid is in
  `C2_TUNING_RESULT.md`; the two candidates of the tie rule are compared in §5.8.
- Calibration: Platt fitted on 355 C2 detections of seeds 900–919, a = 2.135, b = 0.519; 61.4 %
  of them were real targets (Belief's `detector.base_rate` for the evaluation runs).
- **690 missions:** main 240, budget sweep 200, true class 120, ungated 120, timing 10.
  A further 150 missions were run later that evening with one Belief setting changed; they are
  a separate, labelled sensitivity result (§5.9) and are not counted here.
  **No mission failed. 0 refusals, 0 missed leads.** In every mission the emulator's looks at
  the objects equal Belief's `aspect_counts` at their cells (0 differences over 54 635 looks).

### 5.1 Full budget (f = 1.0, n = 30 paired seeds, no class information, misses credited as live)

Score = confirmed true − false confirmations (κ = 1). Δ = paired difference vs C2 on the same
seed; CI ≈ mean ± 2 sd/√n.

| Condition | score | confirmed true | false conf. | detected | Δ score vs C2 | ≈95 % CI | better / equal / worse | only here / only C2 | aspects per target | revisit time s | revisits per mission | decision s mean / max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 | 1.03 ± 1.03 | 1.37 | 0.33 | 3.73 | −0.83 ± 1.74 | [−1.47, −0.20] | 6 / 7 / 17 | 0 / 56 | 1.28 | 0 | — | — |
| **C2** | **1.87 ± 1.70** | 3.23 | 1.37 | 4.70 | — | — | — | — | 2.23 | 0 | — | — |
| C3 | 1.73 ± 1.66 | 3.53 | 1.80 | 4.93 | −0.13 ± 1.07 | [−0.53, +0.26] | 7 / 13 / 10 | 16 / 7 | 2.71 | 427 | one schedule | 0.04 / 0.14 |
| C4 | 2.27 ± 1.17 | 3.97 | 1.70 | 4.93 | +0.40 ± 1.54 | [−0.16, +0.96] | 13 / 11 / 6 | 27 / 5 | 3.81 | 865 | 6.0 | 0.66 / 2.54 |
| C4_iso | 2.23 ± 1.59 | 4.00 | 1.77 | 4.93 | +0.37 ± 1.47 | [−0.17, +0.90] | 13 / 8 / 9 | 25 / 2 | 3.68 | 739 | 4.8 | 0.70 / 2.57 |
| C4_rand | 1.87 ± 1.31 | 3.97 | 2.10 | 5.10 | +0.00 ± 1.44 | [−0.53, +0.53] | 8 / 12 / 10 | 29 / 7 | 3.76 | 879 | 6.4 | 0.69 / 2.76 |
| C4_depth1 | 2.30 ± 1.29 | 4.10 | 1.80 | 5.03 | +0.43 ± 1.52 | [−0.12, +0.99] | 13 / 9 / 8 | 31 / 5 | 3.89 | 844 | 5.9 | 0.23 / 0.85 |
| C4_degraded | 2.20 ± 1.49 | 3.80 | 1.60 | 4.80 | +0.33 ± 1.30 | [−0.14, +0.81] | 14 / 7 / 9 | 22 / 5 | 3.68 | 809 | 5.4 | 0.68 / 2.62 |

Each world holds 6 targets and 6 clutter objects. ECE of the calibrated confidences on the
evaluation runs: 0.039–0.057. Decision times here are with 4 missions in parallel (§5.6 has the
uncontended ones). C3 inserted its revisits in 29 of 30 missions.

**Anytime** (mean score at a fraction of the budget; paired Δ vs C2 in the last two columns):

| Condition | 25 % | 50 % | 75 % | 100 % | Δ at 50 %, ≈95 % CI, b / e / w | Δ at 75 %, ≈95 % CI |
|---|---|---|---|---|---|---|
| C1 | 0.37 | 1.03 | 1.03 | 1.03 | — | — |
| C2 | 0.37 | 1.03 | 1.33 | 1.87 | — | — |
| C3 | 0.37 | 1.03 | 1.83 | 1.73 | 0.00 (identical to C2 up to the pass boundary) | +0.50 [+0.05, +0.95] |
| C4 | 0.67 | 1.90 | 2.20 | 2.27 | +0.87 [+0.35, +1.38], 16 / 10 / 4 | +0.87 [+0.25, +1.49] |
| C4_iso | 0.80 | 1.83 | 2.13 | 2.23 | +0.80 [+0.34, +1.26], 15 / 13 / 2 | — |
| C4_rand | 0.50 | 1.73 | 1.93 | 1.87 | +0.70 [+0.35, +1.05], 17 / 12 / 1 | +0.60 [−0.02, +1.22] |
| C4_depth1 | 0.73 | 2.00 | 2.27 | 2.30 | +0.97 [+0.45, +1.48], 16 / 12 / 2 | — |
| C4_degraded | 0.80 | 1.80 | 2.10 | 2.20 | +0.77 [+0.29, +1.24], 17 / 10 / 3 | — |

**Belief side** (the same for every C4 variant within rounding): the grid a decision saw was
6.7 s behind the boat on average, 8.7 s at most. Draws on one object less than a look time-out
apart (two sectors in one sweep): 0.7 per mission in C2, 1.5 in C4, 1.8 in C4_rand.

### 5.2 Return-visit effect (H1): C3 and C4 against C2

All of the following holds **in this dev loop only**.

1. **Single pass is clearly worse than C2** (Δ −0.83, CI [−1.47, −0.20]). The C2 baseline is not
   a strawman here: it is tuned and frozen first (CM-12).
2. **At the full budget no condition with return visits is distinguishable from C2 in score.**
   C4 +0.40, CI [−0.16, +0.96]; C3 −0.13, CI [−0.53, +0.26]. n = 30 was not chosen for power
   and no significance is claimed.
3. **Return visits confirm more of everything.** Against C2, C4 confirms more true targets
   (+0.73, CI [+0.36, +1.10]) and more clutter (+0.33, CI [0.00, +0.67]); C3 confirms more
   clutter (+0.43, CI [+0.13, +0.73]) and slightly more targets (+0.30, CI [−0.03, +0.63]).
   Under Belief's rule a contact is confirmed by any two detected looks, whatever their
   sectors. A return visit is one more look at whatever is there, and κ = 1 charges a confirmed
   clutter object as much as a confirmed target earns.
4. **Mid-mission the adaptive conditions are clearly ahead.** At 50 % of the budget every C4
   variant leads C2 by +0.7 to +1.0, each CI excluding 0. By then C2 has flown about pass 1
   and equals C1 at its end (1.03); C4 has already revisited contacts found during pass 1.
5. **C4 spends 41 % of the budget on return visits** (865 s of 2111 s, 6.0 revisits per
   mission; the cap ρ_rev is 50 %), and still ends level with or above C2: the coverage it
   displaces is charged through the truncation.

### 5.3 Bearing effect (H2): C4 against C4_rand and C4_iso

Paired with C4 on the same seeds, full budget (`eval/compare.py`):

| A − B | class information | score difference | ≈95 % CI | better / equal / worse | what differs |
|---|---|---|---|---|---|
| C4 − C4_iso (aspect-aware against aspect-agnostic planner) | none | +0.03 ± 0.93 | [−0.31, +0.37] | 7 / 16 / 7 | nothing measurable |
| C4 − C4_iso | true | +0.17 ± 1.34 | [−0.32, +0.66] | 12 / 10 / 8 | nothing measurable |
| C4 − C4_rand (C4's revisits, random bearings) | none | +0.40 ± 1.13 | [−0.01, +0.81] | 15 / 8 / 7 | false confirmations −0.40 [−0.78, −0.02]; confirmed true +0.00 [−0.33, +0.33] |
| C4 − C4_rand | true | +0.57 ± 1.22 | [+0.12, +1.01] | 12 / 13 / 5 | false confirmations −0.50 [−0.84, −0.16]; confirmed true +0.07 [−0.28, +0.41] |
| C4_iso − C4_rand | none | +0.37 ± 1.25 | [−0.09, +0.82] | 13 / 10 / 7 | — |
| C4_degraded − C4 (planner windows ×2, PD −20 %) | none | −0.07 ± 0.94 | [−0.41, +0.28] | 6 / 13 / 11 | nothing measurable |
| C4 − C4_rand, mission cut at 0.75 B | none | +0.05 ± 0.89 | [−0.35, +0.45] | 5 / 11 / 4 | — |
| C4 − C4_rand, mission cut at 0.5 B | none | +0.00 ± 0.97 | [−0.44, +0.44] | 4 / 13 / 3 | — |
| C4 − C4_rand, misses credited at once | none | +0.17 ± 1.26 | [−0.29, +0.63] | 9 / 12 / 9 | — |

**Reading:**
1. **The aspect model shows no effect.** The planner with the aspect windows (C4) and the one
   without (C4_iso) score the same, with or without class knowledge. Knowing the true class
   changes C4 by +0.13, CI [−0.18, +0.45].
2. **A chosen bearing does a little better than a random one at the full budget, through fewer
   confirmed clutter objects, not through more confirmed targets.** The aspect-agnostic planner
   shows the same margin over random bearings as the aspect-aware one. So what helps is that
   the planner chooses the revisit leg by the value of the whole remaining path (its length,
   what it looks at on the way), not that it knows how targets respond to aspect.
3. **The margin is not robust:** it disappears when the mission is cut, and when misses are
   credited at once. Nine comparisons are shown; one interval excludes zero. This is exploratory.

**What this would and would not mean.** Belief's ρ is Paull's assumed rule, the aspect windows
are placeholders shared by the planner and the world, and Belief confirms on any second
detection. Nothing here says whether aspect matters at sea; that is the heading-response
experiment E3. What it does say: **with this Belief and these placeholders, the aspect-aware
choice wins nothing in the dev loop.** One reason was proposed: Belief confirms a contact on
any second detected look, so a bearing matters only through the world's detection probability
at that bearing, and the placeholder windows leave little room. Belief has a switch for a
stricter rule (`contact.confirm_distinct_sectors`). **It was tested (§5.9) and does not
explain it:** with the stricter rule the aspect model still shows no effect.

### 5.4 Missions cut short (f ∈ {1.0, 0.75, 0.5}, n = 20 paired seeds 2000–2019)

`budget_sweep/` ran f = 0.75 and 0.5; `budget_merged/` joins them with the f = 1.0 rows of
`main/` on the common seeds. Final score, Δ vs C2 at the same f, ≈95 % CI, better / equal /
worse:

| Condition | f = 1.0 | f = 0.75 | f = 0.5 |
|---|---|---|---|
| C2 | 1.65 ± 1.84 | 0.90 ± 1.33 | 0.75 ± 1.29 |
| C3 | 1.70; +0.05 [−0.42, +0.52]; 5 / 10 / 5 | 1.55; **+0.65 [+0.12, +1.18]**; 13 / 3 / 4 | 1.15; +0.40 [−0.15, +0.95]; 11 / 3 / 6 |
| C4 | 2.35; **+0.70 [+0.02, +1.38]**; 10 / 7 / 3 | 2.10; **+1.20 [+0.61, +1.79]**; 14 / 4 / 2 | 1.65; **+0.90 [+0.38, +1.42]**; 14 / 4 / 2 |
| C4_iso | 2.20; +0.55 [−0.15, +1.25]; 9 / 6 / 5 | 2.45; **+1.55 [+0.96, +2.14]**; 15 / 5 / 0 | 1.55; **+0.80 [+0.26, +1.34]**; 14 / 3 / 3 |
| C4_rand | 1.90; +0.25 [−0.27, +0.77]; 6 / 10 / 4 | 2.05; **+1.15 [+0.58, +1.72]**; 12 / 7 / 1 | 1.65; **+0.90 [+0.47, +1.33]**; 14 / 4 / 2 |

Bold: CI excludes 0. Revisit time at f = 0.75 / 0.5: C4 583 / 381 s, C3 363 / 146 s.

**Reading, and its limit:**
- When a C2-length mission is cut, every C4 variant is clearly ahead of C2, and C4 is ahead of
  C3 at 0.75 (+0.55, CI [+0.10, +1.00]).
- The f = 1.0 column is the 20-seed subset of §5.1. Its C4 interval just excludes 0
  (+0.70 [+0.02, +1.38]); on all 30 seeds it does not (+0.40 [−0.16, +0.96]). **The 30-seed
  value is the one to quote.**
- **The sweep truncates the frozen full-budget C2 plan.** It models a mission stopped early: C2
  at f = 0.5 has flown about pass 1. A C2 *planned* for 0.5·B, for example wider spacing so that
  both passes fit, is **not tested**. The sweep supports "C4 degrades more gracefully when a
  C2-length mission is cut". It does not support "C4 beats a two-pass survey designed for a
  shorter budget" (CM-12).

### 5.5 Sensitivity: misses credited at once (`ungated/`, n = 30)

| Condition | score | Δ vs C2 | ≈95 % CI | the same condition, ungated − as live |
|---|---|---|---|---|
| C2 | 1.87 | — | — | 0.00 (identical) |
| C3 | 1.70 | −0.17 | [−0.57, +0.24] | −0.03 [−0.10, +0.03] |
| C4 | 2.17 | +0.30 | [−0.33, +0.93] | −0.10 [−0.36, +0.16] |
| C4_rand | 2.00 | +0.13 | [−0.37, +0.64] | — |

The 4 to 9 s by which Belief's grid lags the boat in a live-like run does not change the
outcome measurably. The evidence-horizon rule (MATH §11.3) is exercised in both.

### 5.6 Decision time without contention, and determinism (`timing_uncontended/`)

C4 and C4_depth1, seeds 2000–2004, one process, nothing else running.

| Policy | decisions per mission | decision time mean | max | allowance `t_decide_s` |
|---|---|---|---|---|
| C4 (two-step lookahead) | 12.0 | 0.52 s | 1.79 s | 2.0 s |
| C4_depth1 (one-step) | 11.8 | 0.19 s | 0.49 s | 2.0 s |

- **Uncontended, both fit the allowance** on these 60 and 59 decisions. With four missions in
  parallel C4's slowest decision took 2.5 to 2.8 s (§5.1), C4_depth1's 0.85 s. A live run shares
  the laptop with Gazebo and the ROS nodes: C4's margin is small, C4_depth1's is large.
- **C4_depth1 − C4 = +0.03 ± 0.93, CI [−0.31, +0.37], 8 / 14 / 8** (§5.1 seeds): the one-step
  lookahead does as well as the two-step one in this loop.
- A late decision becomes `missed_lead` and is not sent: safe, and a lost decision. The dev
  loop does not charge decision time to the mission clock. In the node the time a decision took
  is measured and counted against the lead before sending (DESIGN §5.2 rule 2; the first run
  under ROS showed that it was not, `REPORT.md` §F).
- **Determinism:** each of these 10 missions is identical to its row of `main/` (final
  metrics, anytime metrics and requests). Results do not depend on the job count.

### 5.7 What cannot be said from any of this

- Anything about E4, Gazebo, a real detector or water.
- That C4 beats a two-pass survey designed for a shorter budget.
- Any significance (exploratory; n not chosen for power; many comparisons).
- That aspect matters, or does not matter, at sea.
- Anything from `coverage`: it is not used as a metric (it depends on the order of looks).

### 5.8 The C2 freeze and its tie rule

The rule of `C2_TUNING_PROTOCOL.md` §5 was applied as written: the best mean among the feasible
configurations, and among those within one standard error of it, the shortest plan.

| Candidate | mean score ± sd (20 tuning seeds) | confirmed true | false conf. | duration |
|---|---|---|---|---|
| best mean: f = 0.9, long axis | 1.65 ± 1.81 (SE 0.41) | 3.35 | 1.70 | 2214 s |
| **chosen: f = 1.0, long axis** | 1.50 ± 1.79 | 3.30 | 1.80 | 2111 s |
| the reference effort: f = 0.8, long axis | 1.35 ± 1.50 | 3.60 | 2.25 | 2499 s |

- Ten of the twelve feasible configurations lie within one standard error of the best: with
  this yardstick the rule mostly picks the shortest feasible plan.
- Tighter spacing does **not** help the baseline here: it adds looks, and extra looks confirm
  clutter (false confirmations 2.70 at f = 0.6 against 1.80 at f = 1.0).
- Interlacing changes nothing in this scenario (auto = off).
- **The same candidates compared on the same tuning worlds** (a check made after the freeze,
  60 C2 missions; it reproduces the three means above exactly). The configurations share their
  seeds, so the paired difference is the sharper yardstick:

  | Difference | mean ± sd | paired standard error | ≈95 % CI | better / equal / worse |
  |---|---|---|---|---|
  | f = 0.9 − f = 1.0 | +0.15 ± 0.93 | 0.21 | [−0.27, +0.57] | 6 / 9 / 5 |
  | f = 0.9 − f = 0.8 | +0.30 ± 1.13 | 0.25 | [−0.20, +0.80] | 9 / 6 / 5 |
  | f = 1.0 − f = 0.8 | +0.15 ± 1.04 | 0.23 | [−0.32, +0.62] | 7 / 8 / 5 |

  The best mean and the chosen configuration differ by less than one paired standard error:
  on this evidence they are not distinguishable, and the rule's choice of the shorter plan
  stands under either yardstick. The owner settled on 2026-11-12 that the rule uses the paired
  standard error from now on (`C2_TUNING_PROTOCOL.md` §5); the freeze was not redone.

### 5.9 Sensitivity: Belief confirms only on looks from different sectors (`distinct_sectors/`, n = 30)

Asked for by the owner on 2026-11-12 as a labelled extra comparison; run 20:47–20:58. 150
missions: C2, C3, C4, C4_iso, C4_rand on seeds 2000–2029, with **one Belief setting changed
through `--belief-set`: `contact.confirm_distinct_sectors: true`** (written in the manifest).
Belief's code and defaults are untouched. Everything else is as in `main/`: the same seeds and
world draws, the same frozen C2 (not re-tuned for this setting), the calibration reused from
`main/`, misses credited as live, no class information. Belief core `ac686bf69cf07f6a…`.
No mission failed; 0 refusals, 0 missed leads; 0 differences over 13 711 looks; Belief reports
no lost input.

**What the setting does** (read in Belief's `core/contacts.py`): a contact is confirmed when its
detected looks come from at least two different **directed** sectors (12 sectors of 30°). The
two sides of one line are opposite directed sectors, so two detections from opposite sides
still confirm. Only two detected looks from the same directed sector stop confirming. The
setting is not in `model_params_json`: the planner cannot see which rule is in force.

| Condition | score | confirmed true | false conf. | Δ score vs C2 (mean ± sd) | ≈95 % CI | better / equal / worse | this run − `main/`, same seeds |
|---|---|---|---|---|---|---|---|
| C2 | 2.03 | 3.20 | 1.17 | — | — | — | +0.17 ± 0.46 [−0.00, +0.34], 6 / 23 / 1 |
| C3 | 1.80 | 3.40 | 1.60 | −0.23 ± 1.07 | [−0.63, +0.16] | 6 / 13 / 11 | +0.07 ± 0.74 [−0.20, +0.34], 5 / 20 / 5 |
| C4 | 2.23 | 3.87 | 1.63 | +0.20 ± 1.58 | [−0.38, +0.78] | 11 / 11 / 8 | −0.03 ± 0.32 [−0.15, +0.08], 1 / 27 / 2 |
| C4_iso | 2.10 | 3.87 | 1.77 | +0.07 ± 1.55 | [−0.50, +0.63] | 12 / 7 / 11 | −0.13 ± 0.35 [−0.26, −0.01], 0 / 26 / 4 |
| C4_rand | 1.93 | 3.83 | 1.90 | −0.10 ± 1.75 | [−0.74, +0.54] | 10 / 7 / 13 | +0.07 ± 0.94 [−0.28, +0.41], 6 / 21 / 3 |

| A − B (this run) | score difference | ≈95 % CI | better / equal / worse | the same pair in `main/` |
|---|---|---|---|---|
| C4 − C4_iso | +0.13 ± 1.07 | [−0.26, +0.53] | 7 / 17 / 6 | +0.03 [−0.31, +0.37] |
| C4 − C4_rand | +0.30 ± 1.24 | [−0.15, +0.75] | 14 / 8 / 8 | +0.40 [−0.01, +0.81] |
| C4_iso − C4_rand | +0.17 ± 1.39 | [−0.34, +0.67] | 14 / 7 / 9 | +0.37 [−0.09, +0.82] |
| C4 − C2 at 50 % of the budget | +0.87 ± 1.38 | [+0.36, +1.37] | 17 / 9 / 4 | +0.87 [+0.35, +1.38] |

In C4 − C4_rand: false confirmations −0.27 [−0.64, +0.10], confirmed true +0.03 [−0.32, +0.39].

**Reading:**
1. **The stricter rule bites rarely.** C2 flies the same path in both runs and Belief holds the
   same contacts in all 30 seeds; its confirmations change in 7 of them (six lose one false
   confirmation, one loses a true one). Each other condition ends with the same score in 20 to
   27 of its 30 missions.
2. **The aspect model still shows no effect** (C4 − C4_iso = +0.13, CI [−0.26, +0.53]). The
   reason proposed in §5.3 is not supported by this run.
3. **The return-visit pattern is unchanged:** not distinguishable from C2 at the full budget
   (+0.20, CI [−0.38, +0.78]), clearly ahead at half of it (+0.87, CI [+0.36, +1.37]).
4. The rule removes a few clutter confirmations of the fixed survey (C2: false confirmations
   −0.20, CI [−0.35, −0.05]). C4's margin over random bearings is +0.30 and its interval now
   includes zero.

**Why it bites rarely** (a reading of the rule, not a measurement): a second look at an object
almost always comes from another directed sector anyway. In the survey it comes from the
neighbouring line, which sees the object from the other side, or from the pass at right
angles; a return leg passes on a bearing of its own. A rule on **folded** sectors (the line
angle, which is what Belief's ρ is taken on) would be stricter. Belief has no such switch, and
this module does not imitate one: if the owner wants that test, it is a request to Belief.

**Weight:** a sensitivity run, dev loop only, exploratory, C2 not re-tuned under the changed
rule. It is not a result about Belief's default behaviour and not a proposal to change it.

