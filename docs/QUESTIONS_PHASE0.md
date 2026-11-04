# Phase 0 — questions for the user (stop gate)

## Answers (2026-11-04)

| Q | Answer | Label |
|---|---|---|
| Q-1 speed | **0.5 m/s** | [DECIDED] user |
| Q-2 swath | **One side.** Spacing is a function of range, from a literature check → `MATH.md` §2 (NOAA HSSD 2025.0.01 §8.3; Yordanova & Gips 2020 §III-A) | [DECIDED] user; the function's parameters are [ASSUMPTION] |
| Q-3, Q-4, Q-5 inputs | "The brief overrides anything else": the mission file (brief Q9), `/blueboat/odom` with fallback (brief §6.2), the BeliefGrid, Control's progress | [DECIDED] user |
| Q-7 C3 | **Append-on-completion is built first.** The policy architecture is modular so that a later algorithm can replan live and interrupt the current area (`DESIGN.md` §6) | [DECIDED] user |
| Q-8 C4_rand | Pre-drawn bearing table | [DECIDED] user |
| Q-18 context pack | It does not exist and will not be provided. Project rules come from the root `.claude/CLAUDE.md` and `project_synthesis.md`; layout and gate from the existing modules. Facts that are not there are [OPEN]; v1-dependent comparisons are [NOT AVAILABLE] | [DECIDED] user |
| Q-6, Q-9 (ii), Q-10, Q-11, Q-12, Q-13, Q-14, Q-15, Q-16, Q-17, Q-19 | "Make the best choice": the recommendation (a) of each is adopted as a labelled default in `DESIGN.md` §2 | [ASSUMPTION] default |
| Q-9 (i) | Which evidence topic Belief consumes: still [OPEN], routed as `REQUESTS.md` RQ-3 | [OPEN] |

**Phase 1 closure (2026-11-04)** (see `PHASE1_CLOSURE.md` §H):
- The grazing default is now off, per brief §3; the option is kept.
- Override and E-STOP disarm the replanner until `~/start`, per brief §7.2.
- Still [OPEN]: O-5 (clutter f), O-8 (multiple areas), Q-9 (i) (Belief's evidence topic), t_lat
  (L-B5).

The original questions follow, unchanged.

Each question gives what is known, the options, a **recommendation** (first) and what each option
changes. Evidence ids (`E-nn`) point to `EVIDENCE_REGISTER.md`, discrepancy ids (`D-xx`) to
`PHASE0_VERIFICATION.md` §3. **P0** = needed before Phase 1 can be designed; **P1** = Phase 1 can
proceed with the recommendation as a labelled default.

Not a question, for information (brief §14 Q1): Control's mid-run mechanism exists and is
verified — `/blueboat/update_path` (replace the rest of the path from a change point in path
seconds; lead ≥ max(path_time + 0.1 s, 3 m / U_d); continuity 0.20 m and 10° at the change point;
a refusal changes nothing; progress τ on `/blueboat/path_progress`). It constrains the design
in three ways that I will design for, not around: (1) Control does **not** check turn rate,
radius or areas, so feasibility is this module's job; (2) decisions must reach Control at least
`lead_min` ahead of τ, which is 7.5 s at 0.4 m/s for PID/LoS; (3) built-world simulations cannot
run replanner-owned runs yet (Q-12). Details: `PHASE0_VERIFICATION.md` §1.

---

## P0

**Q-1 — Survey speed (D-S1).** The brief (C2) and synthesis §8.5 say 1.0 m/s. The Gazebo boat
cannot reach it: it tops out at ~0.78 m/s under Control's 2 × 20 N limit, and Control advises
≤ 0.45 m/s in simulation. 1.0 m/s has never been flown; field route means were 0.32–0.41 m/s.
- **(a, recommended)** One speed for every condition, **0.40 m/s**, in the dev loop and Gazebo
  (real water decided before E6). The C2 duration, and hence every budget, is ~2.5× longer than at
  1.0 m/s; `lead_min` (PID/LoS) is 7.5 s. You would amend synthesis §8.5.
- (b) Keep 1.0 m/s: the governor slows τ to what the boat achieves. Planned and real time
  diverge, so the equal-time budget (Q2 decision) is no longer controlled.
- (c) Different speeds in sim and real: the sim-to-real comparison of conditions loses one
  controlled variable.

**Q-2 — What "usable swath" means, hence line spacing (D-S2).** Synthesis §8.5: spacing
~0.8 × usable swath, "verify against the measured swath". The brief §3: "usable ground swath ≈ 6 m
… √(20² − depth²)". Nothing in the workspace measures it.
- **(a, recommended)** Usable swath **W = one-sided usable ground width = r_far − r_near**, with
  r_near = 6 m (brief §3, placeholder) and r_far = √(20² − h²). Spacing s = 0.8·W (≈ 11.1 m at
  h = 1.5 m). Because s ≤ W, each line's nadir gap is covered by its neighbours in a single pass,
  with 20 % overlap. r_near and the measured r_far are then checked locally from Perception's
  `sides.*.ground_range_m` (`LOCAL_CHECKLIST.md` L-S1).
- (b) W = both sides = 2·(r_far − r_near) → s ≈ 22 m. Durations and budgets halve; a single pass
  leaves every nadir gap unsearched and only the orthogonal pass covers them.
- Sub-question: r_near 6 m (brief §3) or 5 m (synthesis §8.5, the training ground range)?
  Recommended: 6 m, labelled placeholder until the near-field floor is measured (brief Q7).

**Q-3 — Where mission areas (and the anchor) come from (D-A1).** Brief Q9 [DECIDED]: the
Replanner reads a mission file of WGS-84 polygons. Synthesis §5.1 ⚠: "Belief carries the mission's
survey area and anchor with it, so the Replanner needs no other input". Contract `belief_grid/0`
publishes the datum but **no polygon** (only NaN outside Belief's valid polygon). No MCS file holds
a WGS-84 survey area today.
- **(a, recommended)** The Replanner reads the mission file (brief §6.4 + a `geo_anchor` block),
  and at start checks it against the grid: datum equal to the anchor, polygons inside Belief's
  non-NaN region; any mismatch is a `ContractError` and no plan. **One mission file for both
  modules** (Belief reads `areas` + `geo_anchor` from the same file). You would amend synthesis
  §5.1 ⚠ and tell the Belief session the file and schema name (`blueboat_survey_mission/1`
  proposed).
- (b) Belief adds the polygons to `BeliefGrid` (a contract `[ADDED]` field). Matches synthesis
  §5.1 literally; needs the Belief session and a contract revision; the condition and overrides
  would still need another source.
- (c) Derive areas from the grid's NaN mask. No contract change, but the boundary is quantised
  to 0.5 m and Belief's valid polygon need not be the mission area.

**Q-4 — Reading of CM-9 (brief §16.1, D-A2).** CM-9 literally says "the Replanner reads only
Belief"; root §4.1 and Control R-2 require the Replanner to read `/blueboat/path_progress` and the
service answers. Proposed reading: Belief is the **only online source of target / environment
evidence**. Operational inputs are allowed: Control's progress, events and answers; the mission
file (areas, anchor, condition, overrides); this module's configuration. Detections, sonar, sim
ground truth and any `/sim/*` or `ground_truth/*` topic are never read online.
- **(a, recommended)** Confirm, and reword CM-9 / synthesis §5.1 ⚠ yourself (I do not edit the
  root).
- (b) Strict literal reading: impossible with Control's contract (the module could not learn τ
  or `min_change_point`).

**Q-5 — Subscribe `/blueboat/odom` or not (D-O1).** Real odometry values are fine but stamped
sec 0 (SSS TODO, 2026-10-27); simulation is fine.
- **(a, recommended)** **No odometry subscription.** Before the first path the pose is Control's
  `hold`; afterwards the decision pose is this module's own copy of the accepted path evaluated
  at τ (Control's reference, which the boat trails by 0.04–1.2 m median in Gazebo); the event
  messages' `boat` field is logged for comparison. One fewer exception to CM-9, no stamp problem.
  Control already refuses updates without odometry (`not_ready`).
- (b) Subscribe as brief §6.2, freshness by arrival time. Decisions use the measured pose; one
  more input to keep fresh.

**Q-6 — Turn and timing model (D-K1).** No turn rate or settle time is measured anywhere. The
brief's 30 °/s at 0.4 m/s is a 0.76 m radius; field missions used 1.5–6 m.
- **(a, recommended)** Turns are **arcs of radius r_turn** (placeholder 3 m) flown at survey
  speed and written into the trajectory, so planned time = path length / U plus a settle
  allowance `t_settle` (placeholder 2 s) before looks count as useful. No stationary corners:
  a stationary reference triggers Control's station-keeping law. Same model in every condition.
  Measured locally in Gazebo and from field poslogs (L-K1, L-K2).
- (b) Keep `|Δψ|/ω_turn + t_settle` with point turns. The cost model and the flown path disagree
  at corners, and Control's 10° heading check makes point corners at a change point infeasible.

**Q-7 — "Append-on-completion" (brief §17.2, D-C3).** Synthesis §7.3 condition 3 is "Adaptive,
append-on-completion replanner"; brief C3 is described (pass 1, then one perpendicular revisit
per candidate, nearest-neighbour order, then recover lines) but not named so.
- **(a, recommended)** Append-on-completion **= C3 = synthesis condition 3**. "Completion" = the
  end of the single-pass coverage plan (C1's plan). Then appended, in nearest-neighbour order:
  one revisit leg per unresolved contact at the bearing perpendicular to its `first_sector`, then
  orthogonal recover lines while the budget lasts. One decision epoch at completion, then one per
  appended leg (only to pick the next nearest). It is both a schedule and a heuristic policy.
- (b) "Completion" = each contact's resolution event (revisits interleaved during coverage): it
  becomes a weaker C4 and the ladder loses its "append at the end" rung.
- (c) A distinct fifth condition: more runs per comparison.

**Q-8 — C4_rand and the "randomness only in `eval/`" rule (brief §17.3, D-R1).**
- **(a, recommended)** C4_rand is deployable **without randomness online**: `eval/` draws, from
  a seeded `numpy.random.Generator`, a table of random bearing offsets per (contact rank, revisit
  index) and writes it into the mission's `config`. The core reads it deterministically. C4_rand
  then runs through the same node in Gazebo, and the rule holds. Seeds: independent streams for
  world, detection draws and policy table, from one `SeedSequence` per scenario, recorded in the
  manifest. Paired runs share the world and the detection stream; the detection stream is keyed
  by (object, pass index), so different paths do not shift each other's draws.
- (b) C4_rand is evaluation-only (dev loop): it cannot appear in Gazebo E4.
- (c) Allow a seeded `Generator` in the core for C4_rand only: changes your rule.

**Q-9 — What Belief consumes, and where the emulator lives (brief §14 Q4–Q5, D-B1).** The contract
says Belief subscribes `/sss_ai/detections`. Root CLAUDE.md, CM-9 and synthesis §5.1 say Belief
reads **`/sss_ai/image_records`**. Perception publishes both today (MCS starts it for every
personalized-world launch).
- (i) Which one does Belief consume? This is for you or the Belief session to decide. The
  emulator will publish exactly that.
- (ii) **Recommended:** a single emulator for both modules, in this module's `eval/`, run
  **instead of** Perception (never beside it: one publisher per topic). If the answer to (i) is
  image records, the emulator must also produce negative evidence (one record per image with
  `detections: []` and footprints). That is a larger Phase 4 job, which I would scope then.

**Q-11 — Belief import in `eval/`, and which background quantity (brief §11, §17.6; B.3).**
- **(a, recommended)** Import the real Belief core only in `eval/` and `test/`, behind an
  importability guard; online code never imports it. Background: recompute the
  orientation-resolved residual from `coverage` and `aspect_counts` (brief §8.4), and use
  `p_background` **only** as a consistency check in the A ≡ 1 limit (a test plus a runtime
  warning), never both summed.
- (b) Vendored fixture of Belief's core (copy): drifts from Belief.
- (c) Use `p_background` directly: loses the orientation mix the value model needs.

**Q-12 — Gazebo missions while built worlds cannot host replanner runs (D-R3).**
- **(a, recommended)** You order Control's request R-3 in BlueBoat-SSS-Sim: one argument
  `path_source` passed to its `master_control`. Then the Phase 4 C2/C4 missions run in a built
  world through `full_mission_launch.py`.
- (b) Development composition, available now: `Sim_launch.py path_source:=replanner` plus
  `sss_sim_launch.py world_dir:=<world>`. The sonar renders the world's seabed and objects from
  the pose, but Gazebo flies the boat in the stock ocean, with no walls or limits. Fit for the
  first live check, not for E4.
- (c) Defer the Gazebo missions until R-3 lands.

**Q-13 — Real-water authority (brief §14 Q3).**
- **(a, recommended)** Path changes are autonomous **within an operator-started run** and
  inside the mission areas. They stop on `~/stop`, run stop, manual target, arrows or E-STOP;
  Control already refuses updates in all of these. The first water use comes after Control's own
  on-water path-update test (`PATH_UPDATE_REAL_WATER_TEST.md`) and one supervised dry run. MCS has
  no Replanner mode yet (R-1), so a real replanner session is started through MCS *Extra args*.
- (b) Each change confirmed by the operator: needs an operator interface that no module has.

---

## P1

**Q-10 — Evidential status of the dev loop (D-W1).** Synthesis §6.2 ⚠: a surrogate's detection
probability "must be fitted from Tier-1 Gazebo runs, never invented"; §7.4 sweeps are Tier 2.
- **(a, recommended)** The dev loop is for development and property tests. Its Phase 3 report is
  labelled *reference-belief-conditional, placeholder world model, preliminary*. It is never a
  thesis result. The world-model block is swappable, so that a fitted P(detect | class, range,
  aspect, seabed) from D7 can be dropped in later. No sweep enters the thesis from it.
- (b) Report dev-loop sweeps as exploratory thesis material, against synthesis §6.2 ⚠.

**Q-14 — Decision lead and controller (brief §14 Q2).**
- **(a, recommended)** The change point is the current segment's end. The decision is triggered
  when τ reaches `t_end − (lead_min + T_decide + 2 s)`, with `lead_min` read from progress and
  `T_decide` = the 2 s compute budget. The rule is identical for every condition. **One
  controller for every condition.** I propose PID: it is the controller of Control's first
  on-water path-update test. MPC would be a sensitivity run.
- (b) MPC everywhere: best sim tracking (0.04 m median), but its lead floor is 6.1 s, and the real
  MPC solve time has never been measured.

**Q-15 — Planner classes vs simulator types (D-C1).** Belief's class list wins (contract).
Proposed emulator mapping for the dev loop and evaluation:
- `concrete_block` → block;
- `pipe_pvc` → pipe;
- `mesh_crate` → cage;
- `tire_car` → tyre;
- every other type → "other" (clutter-like target, not counted as a class-k target).

Plate has no simulator class, so it is evaluated only if SSS-Sim adds one (its decision).
Confirm or give the mapping.

**Q-16 — GTSP directive (D-L2).** `literature_conclusions.md` §5 directs a turn-penalised GTSP
over (contact, heading). The brief's decided strategy uses nearest-feasible ordering plus a
two-step lookahead. **Recommended:** keep the brief. Record the GTSP as a considered alternative,
justified by the 3–10 contact scale and receding-horizon re-planning.

**Q-17 — S-44 and literature statuses (D-L1, D-L3).**
- **Recommended:** keep **S-44 Ed. 6.1.0** as named, with "Exclusive Order" wording. Note that
  6.2.0 is adopted. Use §3.7 only as context for "search radius ≥ 3σ", not as a derivation of the
  segment-length formula (brief §20.1).
- Keep brief §15 statuses unchanged. Hollinger 2013, Ji 2007, Golovin 2011 and Jadidi 2019 are
  not in `literature_conclusions.md`, which has Jadidi **2017** arXiv [full]. Before Phase 1 cites
  any of them, I check the DOI landing pages and record exactly what was read. Shall I do that
  checking now (web), or leave it to the thesis-writing pass?

**Q-18 — Missing inputs (D-M2).** The context pack `00`–`03` and `papers_to_be_checked.md` are not
in the workspace. **Recommended:** proceed without them. The brief summarises them, and the module
layout follows the existing modules (Perception's layout is the closest analogue). Upload them if
`02` §5 (module skeleton) or the paper list holds anything the brief omits.

**Q-19 — "Tuned seriously" for C2 (CM-12).** **Recommended:** before any C4 run, a written, frozen
tuning protocol for C2. It fixes the first-pass orientation (the area's long axis, plus a check of
the alternative), the spacing factor (0.8 swept over 0.6–1.0 against recall and time), the pass
order, and the turn radius. C2 and C4 share the coverage generator, speed, turns and executor.
The protocol is pre-registered in `docs/` and never revisited after C4 results exist.
