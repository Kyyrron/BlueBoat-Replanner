# Report — BlueBoat-Replanner

What exists, and **how much weight each statement can carry**. State of 2026-11-13
(`HANDOVER.md`). The six categories
below are kept apart on purpose (brief §16.2; CM-13). A statement belongs to the weakest category
that any of its parts needs.

| Category | Meaning |
|---|---|
| **A. Checked against a neighbour's code** | Our code reproduces or reads another module's code, run here read-only |
| **B. Tested offline** | Our own gate, against properties derived in MATH / DESIGN |
| **C. Real-Belief dev loop, preliminary** | Dev-loop numbers: Belief's own core offline, an emulated detector, a placeholder world, perfect tracking |
| **D. Assumed** | Placeholders and design choices, labelled in `core/config.py` (`category`) |
| **E. Not run** | Code that exists but has never executed in its real environment |
| **F. Under ROS** | What ran on an isolated ROS graph with made-up inputs (F.1), and what still needs Control's node, Gazebo or the boat (F.2) |

Nothing here is an E4 result. The headline comparison of the thesis is the Gazebo campaign with
a trained, calibrated detector (synthesis §4, §6.2 ⚠, §8.4), and none of it has run. Three things
it needs do not exist: detector weights, a `path_source` argument in the built-world launch
(Control R-3), and the Evaluation module.

## A. Checked against a neighbour's code (read-only, no file written in another module)

| What | Against | Gate |
|---|---|---|
| Equirectangular geodesy: function bodies identical (AST), `EARTH_RADIUS_M` identical, 2000 random points bit-identical | BlueBoat-MCS `mcs/core/geo.py` | [2] |
| The C2 first path, and a tail spliced at a segment end, are accepted; a 0.5 m offset tail is refused with the same code | BlueBoat-Control `path_update.PathUpdater` (ROS-free core of `/blueboat/update_path`) | [8] |
| Our request check agrees with Control's `validate_samples` on 40 cases | same | [8] |
| The Control stub of the dev loop answers 5 requests exactly as `PathUpdater` does (accept and refusal codes) | same | [21] |
| **A grid of Belief's core passes our validator with no fallback; an unaligned one is refused** | BlueBoat-Belief core, revision `093841a` + uncommitted changes, through its `OFFLINE_CORE_API.md`. Checked against the core sources of 2026-11-12 afternoon (sha256 `fcd9531597e593b3…`) and of 20:19 that evening (`ac686bf69cf07f6a…`, comments only changed) | [20] |
| **Look predictor against Belief's bookkeeping, on (cell, folded sector): a straight line 100 % both ways; a path with turns 99.9 % (92.9 % without the turn gate)** | same | [20] |
| **Contact fields mean what we read:** `first_confidence` is the first score, `first_sector` its sector, two detected looks confirm, a sector can be both detected and missed, a miss is in the cell record before it is in `sectors_missed` | same | [20] |
| **A detector that never ran: looks counted, no sector searched, coverage 0** | same | [20] |
| **The evidence horizon:** the grid lags the boat by one to two image strides (6.5 s on a straight pass at 0.5 m/s); predicted looks at or before the stamp are in the grid, none after | same | [20] |
| **The emulator's looks at world objects equal Belief's `aspect_counts` at their cells** | same | [22] |

These cover the **ROS-free cores** only. What ran under ROS, and what did not, is in F.

## B. Tested offline — the gate

`cd blueboat_replanner && python3 -B -m test.smoke_test`: **278 passed, 0 failed, 0 skipped**
(2026-11-13, on the laptop, in three environments: `/usr/bin/python3` 3.12 / numpy 1.26.4;
`~/ros2_ws/.venv/bin/python3` / numpy 2.4.4; and `/usr/bin/python3` in a shell with the ROS
workspace sourced. On 2026-11-12 the last of these exposed that the binding loaded Belief's
installed copy instead of its checkout, corrected the same evening. No ROS is used; about 90 s
on one core). `colcon test --packages-select blueboat_replanner` runs the same gate as one
pytest test (`test/test_gate.py`): 1 test, 0 failures. Sections [20] and [22] need the
BlueBoat-Belief checkout beside this repository and skip visibly without it. Each section is
one file (`test/gate/`); a section that raises counts as one failure and the rest still run.

| Section | Checks | What it establishes |
|---|---|---|
| [1] configuration | 15 | every parameter has a unit, a category and a source; unknown keys raise; `config/replanner.yaml` lists every default; `class_families` is empty and validated |
| [2]–[6] geodesy, areas, spacing, Dubins, routing | 33 | single coverage per pass verified by brute force (0.01 m); Dubins endpoints; routes stay inside the margin |
| [7]–[8] sampling, mirror | 21 | Control's request rules (≥ 2 ms steps, ≤ 19 000 samples, continuity 0.20 m / 10°), splice semantics |
| [9]–[10], [12]–[13] coverage, budget, C1/C2/C3 | 32 | C2 = two orthogonal passes; truncation at the budget; C3 inserts revisits at the pass boundary with fallback bearing θ₁ |
| [11] contract `belief_grid/0` | 27 | every field validated, each failure names its field; `aspect_log_miss`, `model_params_json`, the stamp, Belief's area, sector edges; a sector count whose folded count reaches the look-key stride (64) is refused |
| [14] purity | 3 | no rclpy, clock or randomness in `core/`, `io/`, adapters; no online code names a detection, sonar or simulation topic (CM-9) |
| [15] mission file, archive | 17 | exactly one area; `planning_depth_m` required; write-once archive; C4_live and C4_degraded refused, the other conditions accepted; the C2 freeze file read with its sha256, applied over the node's configuration, a conflicting mission refused; `mission_example.yaml` parses with the shipped freeze |
| [16] background (MATH §9.1, H-1) | 9 | the class-mixture root reproduces Belief's residual; searched sectors come from `aspect_log_miss`; the `p_background` consistency value |
| [17] sensor and contact model (H-2, H-6) | 14 | folded sectors, posterior and value properties; misses from the cell record at their strength; class families |
| [18]–[19] sequence identities (MATH §10.4, §11.1) | 5 | union = sequential (100 random cases); decomposition = direct union value; pending looks are worth 0 |
| [20] Belief's core (H-1, H-2, H-3, H-5, H-7) | 19 | see A; the core is loaded from the checkout, also when an installed copy is on the path |
| [21] Control stub | 3 | see A |
| [22] dev loop on Belief's core | 21 | the missions are run by the node's executor (they end `done`, its records and labels, no compute-time ageing); C4_rand's table over the grid's sectors; C2 one request; C3 one decision; C4 no refusal and no missed lead; budget and ρ_rev respected; deterministic per seed; paired worlds; looks = Belief's; Belief's run health (no input lost); the grid's lag; labels; the paired-difference tool; the ungated mode; the mission stop; the C2 tie rule on paired differences (`select_c2`); a Belief setting changed for one run and written to the manifest (`--belief-set`) |
| [23] executor vs Control stub | 35 | **the pre-send checks: a tail moved 3 m into the margin band is never sent, one moved 0.3 m fails the junction check, unsent; no first request while the service is not ready (named at the start time-out); an answer handled at its arrival time**; first path; C2 to `done`; C3 decision accepted; stop and disarm (D-1…D-5); missed lead (G-1); stale belief; lost answer; foreign update; **horizon conversion and its fallbacks, Belief restart, area refusal, mission stop, record fields, end record; a grid that is received but not usable is reported with its reason; the decision time is measured only with the node's clock, written to the records, and counted against the lead (a decision that used it up is not sent)** |
| [24] nodes over fake rclpy | 15 | node graph and BeliefGrid QoS profile; `~/start`; a C3 run through the node; plan, status and decision schemas; run records including `belief.json`, `end.json`, the C2 freeze in the manifest and each answer's arrival time; emulator: one draw per look, one array per image, fill convention; launch arguments = typed node parameters |
| [25] emulator | 9 | seabed images by track distance; looks by sector and time-out; the turn gate; the reported class |

[24] uses **fake** modules written in the test. It proves the wiring of the two node files and
the launch files, nothing about DDS, QoS matching, executors or the real message classes: those
were seen in the run of F.1.

## C. Real-Belief dev loop, preliminary

Full method and tables: `DEVLOOP.md` §5. Outputs: `results/devloop_real_2026-11-12/`
(`c2_tuning/`, `main/`, `budget_sweep/`, `budget_merged/`, `class_true/`, `ungated/`,
`timing_uncontended/`, and the extra comparison `distinct_sectors/`). 690 missions on
2026-11-12, plus 150 in the extra comparison; none failed; 0 refusals, 0 missed leads.

**The loop runs the node's executor** (since 2026-11-13, DESIGN §5.4): trigger, policy call,
pre-send checks, request, splice and budget accounting are `ros/executor.py`. The results were
made before that change by the loop's own copy of that logic; 40 of them re-run on the executor
are identical to their stored rows, field by field (`DEVLOOP.md` §1). What the loop still does
not exercise: compute time (it is 0 path seconds there), answer time-outs (the stub answers at
once), freshness failures.

**Why every number here is weak evidence:**
- Belief's core runs offline, on emulated detections, with its assumed PD and ρ;
- the world's detection model **is** the planner's placeholder model (not fitted from Gazebo,
  synthesis §6.2 ⚠);
- the boat tracks the plan perfectly, and decisions take no mission time;
- one 100 × 60 m scenario at 2 m depth, with 6 targets and 6 clutter objects.

**C2 freeze (dev loop):** f = 1.0, pass 1 along the long axis, interlace auto; B = 2111 s. The
best mean was f = 0.9 (1.65 ± 1.81); the protocol's tie rule (within one standard error, the
shorter plan) chose f = 1.0 (1.50 ± 1.79). `DEVLOOP.md` §5.8.

**Main run** (n = 30 paired seeds, full budget, no class information). Score = confirmed true −
false confirmations:

| | C1 | C2 | C3 | C4 | C4_iso | C4_rand | C4_depth1 | C4_degraded |
|---|---|---|---|---|---|---|---|---|
| score | 1.03 | **1.87** | 1.73 | 2.27 | 2.23 | 1.87 | 2.30 | 2.20 |
| confirmed true / false | 1.37 / 0.33 | 3.23 / 1.37 | 3.53 / 1.80 | 3.97 / 1.70 | 4.00 / 1.77 | 3.97 / 2.10 | 4.10 / 1.80 | 3.80 / 1.60 |
| Δ vs C2, ≈95 % CI | −0.83 [−1.47, −0.20] | — | −0.13 [−0.53, +0.26] | +0.40 [−0.16, +0.96] | +0.37 [−0.17, +0.90] | +0.00 [−0.53, +0.53] | +0.43 [−0.12, +0.99] | +0.33 [−0.14, +0.81] |
| Δ vs C2 at 50 % of B | — | — | 0.00 | **+0.87** [+0.35, +1.38] | **+0.80** | **+0.70** | **+0.97** | **+0.77** |

**Missions cut short** (n = 20). Δ vs C2 at the same budget, C2's plan truncated:

| | f = 1.0 | f = 0.75 | f = 0.5 |
|---|---|---|---|
| C3 | +0.05 | **+0.65** [+0.12, +1.18] | +0.40 [−0.15, +0.95] |
| C4 | +0.70 [+0.02, +1.38] (30 seeds: +0.40 [−0.16, +0.96]) | **+1.20** [+0.61, +1.79] | **+0.90** [+0.38, +1.42] |
| C4_rand | +0.25 | **+1.15** [+0.58, +1.72] | **+0.90** [+0.47, +1.33] |

**What can be said (this dev loop only), with the two effects kept apart:**

*Return visits (H1).*
1. C2 is a real baseline: single pass is clearly worse.
2. At the full budget no condition with return visits is distinguishable from C2 in score.
3. Return visits confirm more targets **and** more clutter (C4 against C2: +0.73 true,
   +0.33 false). Belief confirms a contact on any two detected looks.
4. Mid-mission, and when a C2-length mission is cut, the adaptive conditions are clearly ahead.

*Bearing choice (H2).*
5. **The aspect model shows no effect:** C4 − C4_iso = +0.03 [−0.31, +0.37]; with the true class
   +0.17 [−0.32, +0.66]. Knowing the class changes C4 by +0.13 [−0.18, +0.45].
6. A chosen bearing does slightly better than a random one at the full budget
   (C4 − C4_rand = +0.40 [−0.01, +0.81]; with the true class +0.57 [+0.12, +1.01]), through
   fewer confirmed clutter objects. The aspect-agnostic planner has the same margin, so it comes
   from choosing the leg by the value of the whole remaining path, not from aspect knowledge. It
   vanishes when the mission is cut and when misses are credited at once.

*Other.*
7. The one-step lookahead equals the two-step one (C4_depth1 − C4 = +0.03 [−0.31, +0.37]) and
   decides in a third of the time.
8. The 4 to 9 s by which Belief's grid lags the boat changes nothing measurable.

*A labelled extra comparison (owner's request, 2026-11-12 evening; `distinct_sectors/`, 150
missions; `DEVLOOP.md` §5.9).* One Belief setting was changed for this run only:
`contact.confirm_distinct_sectors: true`, under which a contact is confirmed only by detected
looks from two different directed sectors. Belief's code and defaults are untouched, and C2 was
not re-tuned under it.
9. The rule changes few outcomes: C2's score moves by +0.17 [−0.00, +0.34] (it loses a few
   clutter confirmations), C4's by −0.03 [−0.15, +0.08].
10. **The aspect model still shows no effect** (C4 − C4_iso = +0.13 [−0.26, +0.53]). The
    explanation proposed for finding 5, that Belief confirms on any second look, is not
    supported: a second look almost always comes from another directed sector anyway.
11. The return-visit pattern is the same: +0.20 [−0.38, +0.78] at the full budget, +0.87
    [+0.36, +1.37] at half of it.

**What cannot be said:**
- anything about E4, Gazebo, a real detector or water;
- that C4 beats a two-pass survey *designed* for a shorter budget (not tested, CM-12);
- any significance (exploratory, n not chosen for power, many comparisons);
- that aspect matters, or does not, at sea: Belief's ρ is an assumed rule and the aspect windows
  are placeholders shared by the planner and the world. A bearing effect here would be a
  property of the model; its absence is one too.

**Execution in the dev loop:** 0 refusals and 0 missed leads over all 690 missions; no mission
failed; Belief reports no lost input (its counters, read in 40 re-run missions that equal their
stored rows); results are identical whatever the number of worker processes; the emulator's looks at
the objects equal Belief's `aspect_counts` (0 differences over 54 635 looks).

**Timing:** uncontended, C4's slowest decision took 1.79 s and C4_depth1's 0.49 s, against the
2.0 s allowance; with four missions in parallel C4 reached 2.5 to 2.8 s. An open owner decision
(`HANDOVER.md` §4).

## D. Assumed (placeholders and design choices)

Every value below is a labelled placeholder in `core/config.py`, `eval/world.py` or Belief's
configuration. None is fitted from data. The categories are those of MATH §15.

| Item | Value | Why it matters | Settled by |
|---|---|---|---|
| Planner aspect windows A_k | block 4 / 20° / 0.4 / 1.0, pipe 2 / 10° / 0.1 / 1.0, cage 4 / 25° / 0.3 / 0.9, tyre aspect-free 0.6, plate 0.3 | they are the whole reason C4 exists; the world uses the **same** windows | E3 (heading-response experiment); a Gazebo-fitted world model |
| Class → aspect family map | empty | with no entry a Belief class is isotropic: the aspect choice is inert for it | the owner (which detector class is which family) |
| Planner PD | 0.7 on [3, min(25, r_geo)] m, a box | value of every look | E3; Belief's is a trapezoid with 3 m ramps (MATH §6) |
| Belief's PD, ρ, prior density | 0.7 trapezoid; Paull's rule; 5 /ha | what the grid says | Belief's owner; labelled runs |
| Detector scores | raw while `detector_calibrated` is false | p̃ and every contact value (`score-conditional`) | Perception's calibration |
| Clutter re-detection f | 0.2 | the false-confirmation term | O-5 |
| Fallback belief latency t_lat | 10 s | used only when the grid's stamp cannot be | — |
| r_turn, t_settle, margin | 3 m, 2 s, 2 m | time of every turn and so every budget | L-K1, L-K2, L-K4 |
| Speed | 0.5 m/s (user) | above Control's ≤ 0.45 m/s simulation advice | L-K3 |
| r_near, grazing option | 6 m, off | line spacing | L-S1 |
| C4 choices | two-step lookahead, top-K proxies, hysteresis ε_J 0.02, rules' thresholds, ρ_floor 0 | C4 behaviour | sensitivity sweeps (not run) |
| Emulated detector's class output | none (main), true (second run) | whether C4 can differ from C4_iso | owner, 2026-11-12: both, apart |
| DESIGN decisions 21–23 | all conditions start from the C2 plan; whole-tail valuation; mixture calibration | the comparison's structure | approved 2026-11-04 |

## E. Not run

- The node beside **Control's node**: the path-update service of F.1 was a made-up Control.
- Any Gazebo mission, any run on simulated time, anything with a sonar image, a detector or
  Perception, anything on water.
- The emulator node on a built world (it ran on a made-up world file).
- C4_live (mid-line interruption): a flag only; RECOVER candidates: not built (MATH §10).
- Sensitivity sweeps of κ, λ, ε_J and the rule thresholds (C2_TUNING_PROTOCOL §7): not run.

## F. Under ROS

### F.1 What ran on 2026-11-12: an isolated graph, made-up inputs, no simulator

`.claude/tools/ros_check.sh` → `test/ros_graph_check.py`, ROS domain 47. **Last run, 21:27–21:52:
42 checks, 0 failed.** Every check and number: `LOCAL_CHECKLIST.md` L-X.

**It ran on the code of 2026-11-12.** The node changed on 2026-11-13:
- the C2 freeze is now applied to every mission, so its coverage spacing is 1.0, no longer 0.8;
- the pre-send area and junction checks;
- the readiness check;
- each answer's arrival time;
- the unguarded executor callback.

These are tested offline ([15], [23], [24]) and the package is built into `~/ros2_ws/install`,
but **the ROS check has not been re-run on them**.

| Real | Made up by the check |
|---|---|
| Belief's installed node (core sha256 `fcd9531597e593b3…`, the core the dev-loop results were made on) | sonar pings without image content, odometry, GPS fixes |
| this package's node and emulator node, from their executables and from their launch files | a world file with 8 objects |
| DDS, QoS matching, every message and service type | Control: the path progress and a `/blueboat/update_path` server of the real type, answered by the dev loop's Control test double |

**What this run establishes:**
1. The package builds (`package.xml`, `setup.py`, entry points, launch files).
2. `BeliefGrid` is exchanged with the contract's QoS on both ends, and every live grid passes
   the adapter and the validator with no fallback. An unaligned grid is refused with its reason.
3. A C3 and a C4 mission run to `done` through the real `UpdatePath` type: first path, decisions,
   accepted path changes, the latched plan and status, complete run records.
4. The evidence horizon is read from the grid's stamp in every decision (here Control's clock
   and the sonar stamps are one clock by construction).
5. A restarted Belief is noticed, counted, logged, and its new grid used.
6. With the emulator node publishing one array per seabed image, Belief credits the search with
   its **default** rule (`negative_evidence: gated`, `coverage_source: arrays`), and its
   contacts lie on the made-up objects: the frames agree through Belief's transform.
7. In the C4 mission at real speed: 10 decisions, 5 path changes, **all accepted, none
   refused, no missed lead**. Decision time: mean 0.46 s, slowest 1.74 s, against the 2.0 s
   allowance; the slowest left 2.05 path-s of lead where 2.0 are needed.

**What it found, and what was corrected the same evening:** a decision's own compute time was
not counted against the lead before sending (now it is: DESIGN §5.2 rule 2, gate [23]; seen
acting under ROS, the node dropped 2 late decisions itself at a tenfold path clock); decision
times were not recorded (now `timing` in every decision record and in `end.json`); in a shell
with ROS sourced the dev loop loaded Belief's installed copy (now the checkout comes first; no
stored result is affected).

**What this run is not evidence of:**
- Control's behaviour, timing or refusals: the made-up Control accepts what the ROS-free
  `PathUpdater` accepts (A), instantly, and tracks the path perfectly;
- anything on simulated time, with Gazebo's load, or with a sonar image;
- the decision time of a full mission: the area was 70 × 45 m with 8 objects, on a machine
  with 16 cores of which three were busy. The slowest C4 decision already used 1.74 of the
  2.0 s. This is one more indication for the lookahead decision (`HANDOVER.md` §4, decision 1),
  not the measurement it waits for;
- any survey result: the detections are draws on a made-up world.

### F.2 What still needs Control's node, Gazebo or the boat

`LOCAL_CHECKLIST.md`. The items that gate a thesis result (`.claude/TODO.md` [P0]):
1. Control's `/blueboat/update_path` live (L-C).
2. Gazebo missions C2 and C4 with live path updates (L-G1…L-G4), through the composition until
   Control R-3 lands (L-G5); the decision time under load is read from their records.
3. 0.5 m/s and 3 m turns achieved in Gazebo (L-K3, L-K4).
4. The E4 C2 freeze in Gazebo on tuning worlds (C2_TUNING_PROTOCOL §6).
5. Real-water authority after a supervised dry run (DESIGN §12, O-7).
