# Report — BlueBoat-Replanner, cloud session REPLAN-1 (2026-11-04)

What this session built, and **how much weight each statement can carry**. The six categories
below are kept apart on purpose (brief §16.2; CM-13). A statement belongs to the weakest
category that any of its parts needs.

| Category | Meaning |
|---|---|
| **A. Checked against a neighbour's code** | Our code reproduces another module's code, run here read-only |
| **B. Tested offline** | Our own gate, against properties derived in MATH / DESIGN |
| **C. Reference-belief-conditional, preliminary** | Dev-loop numbers: stand-in belief, placeholder world, perfect tracking |
| **D. Assumed** | Placeholders and design choices, labelled in `core/config.py` (`category`) |
| **E. Not run** | Code that exists but has never executed in its real environment |
| **F. Needs the in-machine session** | Items of `LOCAL_CHECKLIST.md`, with commands and expected results |

Nothing here is an E4 result. The headline comparison of the thesis is the Gazebo campaign with
the real Belief core (synthesis §4, §6.2 ⚠, §8.4), and none of it has run.

## A. Checked against a neighbour's code (read-only, no file written in another module)

| What | Against | Gate |
|---|---|---|
| Equirectangular geodesy: function bodies identical (AST), `EARTH_RADIUS_M` identical, 2000 random points bit-identical | BlueBoat-MCS `mcs/core/geo.py` | [2] |
| The C2 first path, and a tail spliced at a segment end, are accepted; a 0.5 m offset tail is refused with the same code | BlueBoat-Control `path_update.PathUpdater` (ROS-free core of `/blueboat/update_path`) | [8] |
| Our request check agrees with Control's `validate_samples` on 40 cases | same | [8] |
| The Control stub of the dev loop answers 5 requests exactly as `PathUpdater` does (accept and refusal codes) | same | [21] |
| Control's own ROS-free tests pass here: `test_path_update.py` 102/102, `test_path_update_log.py` 24/24 | — | Phase 0 |

These cover the **ROS-free cores** only. The service, its QoS and its timing under ROS are not
covered (F: L-C1…L-C4).

## B. Tested offline — the gate

`cd blueboat_replanner && python3 -m test.smoke_test`: **188 passed, 0 failed, 1 skipped**
(2026-11-04, system python 3.13.18, numpy 2.4.4, no ROS; ~35 s, peak RSS ~0.76 GB). The skip is
the predictor check against the real Belief core, which is not in this workspace.

| Section | Checks | What it establishes |
|---|---|---|
| [1] configuration | 13 | every parameter has a unit, a category and a source; unknown keys raise |
| [3]–[6] areas, spacing, Dubins, routing | 27 | single coverage per pass verified by brute force (0.01 m); Dubins endpoints; routes stay inside the margin |
| [7]–[8] sampling, mirror | 21 | Control's request rules (≥ 2 ms steps, ≤ 19 000 samples, continuity 0.20 m / 10°), splice semantics |
| [9]–[10], [12]–[13] coverage, budget, C1/C2/C3 | 32 | C2 = two orthogonal passes; truncation at the budget; C3 inserts revisits at the pass boundary with fallback bearing θ₁ |
| [11] contract `belief_grid/0` | 17 | every C-3/C-4 field validated, each failure names its field |
| [14] purity | 3 | no rclpy, clock or randomness in `core/`, `io/`, adapters; no online code names a detection, sonar or simulation topic (CM-9) |
| [15] mission file, archive | 9 | exactly one area; `planning_depth_m` required; write-once archive |
| [16] background calibration (closure A) | 6 | A-1…A-5: the class-mixture root reproduces Belief's residual; clamped when unreachable |
| [17] sensor and contact model | 8 | folded sectors, posterior and value properties (MATH §6–§8) |
| [18]–[19] closure B/C identities | 5 | union = sequential (100 random cases); worked example flips A → B; decomposition = direct union value (6 random detours); CONTINUE with no detour = K alone; all-pending looks are worth 0 |
| [20] look predictor | 2 | > 97 % agreement both ways with the reference belief's look geometry (the real-Belief check is the skip) |
| [22] dev loop, small scenario | 8 | C2 one request; C3 one decision; C4 no refusal and no missed lead; budget and ρ_rev respected; deterministic per seed; paired worlds |
| [23] executor vs Control stub | 16 | first path; C2 to `done`; C3 decision accepted; stop D-1/D-2; override and E-STOP disarm D-3/D-4; re-arm checks D-5; missed lead G-1; stale belief; lost answer reconciled; foreign update |
| [24] nodes over stand-in rclpy | 13 | node graph and BeliefGrid QoS; `~/start`; a C3 run through the node with 2 accepted requests; `/replanner/plan`, status and decision schemas, run records; emulator draws, positions, ids and fill convention; launch arguments = typed node parameters |

[24] uses **stand-in** modules written in the test. It proves the wiring of the two node files
and the launch files, nothing about DDS, QoS matching, executors or the real message classes. It
found four defects, now fixed (`HANDOVER_CLOUD.md` Phase 4).

## C. Reference-belief-conditional, preliminary — the dev loop

Full method and tables: `DEVLOOP.md` §5. Outputs: `results/devloop_2026-11-04/`
(`c2_tuning/`, `main/`, `budget_sweep/`, `budget_merged/`, `timing_uncontended/`).

**Why every number here is weak evidence:**
- the belief is a stand-in written from the briefs, not BlueBoat-Belief;
- the world's detection model **is** the planner's placeholder model (not fitted from Gazebo,
  synthesis §6.2 ⚠);
- the boat tracks the plan perfectly;
- decisions take no mission time;
- every plan segment counts as a pass;
- one 100 × 60 m scenario at 2 m depth, with 6 targets and 6 clutter objects.

**C2 freeze (dev loop):** f = 1.0, pass 1 along the long axis, interlace auto; B = 2111 s. The best
mean was f = 0.8 (1.85 ± 1.66). The protocol's tie rule (within 1 SE, the shorter duration) chose
f = 1.0 (1.80 ± 1.85).

**Main run** (n = 30 paired seeds, f = 1.0). Score = confirmed true − false confirmations:

| | C1 | C2 | C3 | C4 | C4_iso | C4_rand | C4_depth1 | C4_degraded |
|---|---|---|---|---|---|---|---|---|
| score | 1.03 | **2.00** | 1.90 | 2.27 | 2.20 | 2.27 | 2.57 | 2.40 |
| Δ vs C2, ≈95 % CI | −0.97 [−1.43, −0.50] | — | −0.10 [−0.48, +0.28] | +0.27 [−0.32, +0.86] | +0.20 [−0.29, +0.69] | +0.27 [−0.28, +0.82] | +0.57 [−0.12, +1.25] | +0.40 [−0.02, +0.82] |
| Δ vs C2 at 50 % of B | — | — | 0.00 | **+0.73** [+0.25, +1.22] | **+0.73** | **+0.70** | **+0.80** | **+0.63** |

**Budget sweep** (n = 20). Δ vs C2 at the same budget, C2's plan truncated:

| | f = 1.0 | f = 0.75 | f = 0.5 |
|---|---|---|---|
| C3 | −0.05 | +0.25 | **+0.50** [+0.13, +0.87] |
| C4 | +0.25 | **+1.00** [+0.44, +1.56] | **+0.80** [+0.35, +1.25] |
| C4_rand | +0.25 | **+1.00** [+0.38, +1.62] | **+1.20** [+0.73, +1.67] |

**What can be said (dev loop only):**
1. C2 is a real baseline: single pass is clearly worse.
2. At the full budget, no adaptive condition is distinguishable from C2.
3. Mid-mission, and when a C2-length mission is cut, the adaptive conditions are clearly ahead.
4. **The aspect-aware bearing choice has no measurable effect here.** C4 = C4_rand (+0.00 paired)
   and C4 ≈ C4_iso. The gain comes from revisiting early, because any revisit is a new pass.
   **This is the most important finding for the thesis: the dev loop does not exercise the
   aspect effect the replanner is built for.** A measured aspect response (E3) and Belief's pass
   definition (L-B6) decide whether it ever can.
5. The one-step lookahead is at least as good as the two-step one, and 3× faster.

**What cannot be said:**
- anything about E4, Gazebo, the real Belief or water;
- that C4 beats a two-pass survey *designed* for a shorter budget (not tested, CM-12);
- any significance (exploratory, n not chosen for power).

**Execution in the dev loop:**
- 0 refusals and 0 missed leads over all 450 missions (main 240, sweep 200, timing 10);
- no mission failed;
- results are identical whatever the number of worker processes.

**Timing:** C4 decisions take up to 5.6 s uncontended, against the 2.0 s allowance;
C4_depth1 takes up to 1.5 s. This is an open user decision (`HANDOVER_CLOUD.md`).

## D. Assumed (placeholders and design choices)

Every value below is a labelled placeholder in `core/config.py` or `eval/world.py`. None is
fitted from data. The categories are those of MATH §15.

| Item | Value | Why it matters | Settled by |
|---|---|---|---|
| Planner aspect windows A_k | block 4 / 20° / 0.4 / 1.0, pipe 2 / 10° / 0.1 / 1.0, cage 4 / 25° / 0.3 / 0.9, tyre aspect-free 0.6, plate 0.3 | they are the whole reason C4 exists; the world uses the **same** windows, so the dev loop measures the policy under a correct model, not robustness to a wrong one (C4_degraded only probes that) | E3 (heading-response experiment); a Gazebo-fitted world model (synthesis §6.2 ⚠) |
| Detection band and PD | 0.7 on [3, min(25, r_geo)] m | value of every look | Belief's model (L-B6) |
| Clutter re-detection f | 0.2 | the false-confirmation term | O-5 |
| Belief latency t_lat | 10 s | the pending-look window | L-B5 |
| r_turn, t_settle, margin | 3 m, 2 s, 2 m | time of every turn and so every budget | L-K1, L-K2, L-K4 |
| Speed | 0.5 m/s (user) | above Control's ≤ 0.45 m/s simulation advice | L-K3 |
| r_near, grazing option | 6 m, off | line spacing | L-S1 |
| Reference belief semantics | look = (cell, sector, pass), ρ_repeat 0.3, confirm on 2 passes | everything in C | the real Belief (L-B7) |
| C4 choices | two-step lookahead, top-K proxies, hysteresis ε_J 0.02, rules' thresholds | C4 behaviour | sensitivity sweeps (not run) |
| Calibration reading of `coverage` | class-mixture residual (closure A) | background value | L-B6 |
| DESIGN decisions 21–23 | all conditions start from the C2 plan; whole-tail valuation; mixture calibration | the comparison's structure | approved 2026-11-04 |

## E. Not run

- `ros/replanner_node.py`, `eval/emulator_node.py`, both launch files: never executed under ROS
  (only through the stand-ins of [24]).
- `colcon build` of the package: never run (`package.xml` and `setup.py` are unverified by a
  build).
- Any Gazebo mission, any mission with the real Belief core, anything on water.
- C4_live (mid-line interruption): a flag only; RECOVER candidates: not built (MATH §10).
- Sensitivity sweeps of κ, λ, ε_J and the rule thresholds (C2_TUNING_PROTOCOL §7): not run.

## F. Needs the in-machine session

`LOCAL_CHECKLIST.md`, in its suggested order. The items that gate a thesis result
(`.claude/TODO.md` [P0]):
1. Belief binding: topic, QoS, datum, transform form, evidence topic (L-B1…L-B4, Q-9), then the
   predictor check and the dev loop on the real core (L-B7).
2. Gazebo missions C2 and C4 with live path updates (L-G1…L-G4), through the composition until
   Control R-3 lands (L-G5).
3. 0.5 m/s and 3 m turns achieved in Gazebo (L-K3, L-K4).
4. The E4 C2 freeze in Gazebo on tuning worlds (C2_TUNING_PROTOCOL §6).
5. Real-water authority after a supervised dry run (Q-13).
