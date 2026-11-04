# Handover — cloud session (REPLAN-1)

**Delivery rule (user, 2026-11-04):** every deliverable lives in this repository, on branch
`claude/nice-johnson-ang1j1`, **uncommitted**, until the user asks for a push. No archives. The
earlier scratchpad archives were deleted.

## Phase 5 — 2026-11-04 — handover (final state of the cloud session)

**Start here:** `docs/REPORT.md`. It sorts every statement into one of six categories:
checked against neighbour code, tested offline, reference-belief-conditional, assumed, not run,
and needs the in-machine session.

**Repository state:**
- branch `claude/nice-johnson-ang1j1`, everything **uncommitted** (the user commits and pushes);
- no archive;
- `git status` lists only this module's own untracked tree (`.claude/`, `.gitignore`,
  `blueboat_replanner/`, `docs/`, `results/`);
- every other module (superproject, SSS, SSS-Sim, Perception, Control, MCS) shows **0 changes**.

**Built:**
- the package `blueboat_replanner/` (ament_python):
  - `core/`: coverage, Dubins and routing, plans, sampling and mirror, contract validation,
    sensor and value models, the J/J2 evaluator, and the policies C1, C2, C3 and the C4 family;
  - `io/`: mission file, trajectory archive, run records;
  - `ros/`: adapters, the executor, the node;
  - `eval/`: reference belief, world and emulator, Control stub, dev loop, runner, emulator node;
  - `config/`, `launch/`, and `test/smoke_test.py`;
- the documents in `docs/` (DESIGN, MATH, topics, contract, protocol, DEVLOOP, INTEGRATION,
  REPORT, LOCAL_CHECKLIST, phase records);
- `.claude/` (CLAUDE.md, TODO.md, rules, the Stop hook).

**Tests that actually passed (cloud, 2026-11-04):**
- **The gate:** `cd blueboat_replanner && python3 -m test.smoke_test` → **188 passed, 0 failed,
  1 skipped**, in ~35 s with peak RSS ~0.76 GB. This was with system python 3.13.18 and numpy 2.4.4,
  and no ROS.
- **Control's ROS-free tests:** `test_path_update.py` 102/102 and `test_path_update_log.py` 24/24
  (Phase 0, read-only).
- **The dev loop:** C2 tuned and frozen, then 450 missions with 0 failures. The results do not
  depend on the number of worker processes.

**Dependencies:**
- offline: python ≥ 3.10, numpy, PyYAML, matplotlib (plots only);
- ROS (Jazzy): `rclpy`, `std_msgs`, `std_srvs`, `nav_msgs`, `geometry_msgs`,
  `blueboat_interfaces` (Control), `blueboat_belief_interfaces` (Belief, not visible here),
  `vision_msgs` (emulator only), `launch`, `launch_ros`.

**Must be built and run locally:** `LOCAL_CHECKLIST.md`, in its suggested order. In short:
1. `colcon build`;
2. the gate in the workspace;
3. Control's service live (L-C);
4. the Belief binding (L-B1…L-B4);
5. the node graph and QoS (L-R3…L-R6);
6. the emulator (L-E);
7. 0.5 m/s and 3 m turns (L-K3, L-K4);
8. the Gazebo C2 and C4 missions with live updates (L-G1…L-G4);
9. Belief's latency, model and pass definition, then the dev loop on the real core (L-B5…L-B7).

**Open user decisions** (none blocks the in-machine session):
1. **Decision time.** C4 decisions take up to 5.6 s uncontended, against the 2.0 s
   `t_decide_s` allowance. The options are:
   - raise `t_decide_s` from the L-G4 measurement;
   - deploy the one-step lookahead (C4_depth1: max 1.5 s, at least as good in the dev loop);
   - keep both and accept `missed_lead`s live.

   Either change means re-running every condition on `S_eval` (C2_TUNING_PROTOCOL §7).
2. **The budget sweep's baseline.** It truncates the full-budget C2. Either re-tune C2 for each
   budget fraction before claiming anything about shorter budgets (CM-12), or restrict the claim
   to "missions cut short".
3. **The aspect effect.** The dev loop shows no effect of the bearing choice (C4 = C4_rand).
   Any world aspect response used later must come from E3 or Gazebo data, never be chosen to make
   C4 win. Decide whether the thesis argument rests on the revisit effect, on the aspect effect,
   or on both, and design E4 accordingly.
4. **C2 freeze tie rule.** The rule chose the shortest C2 (f = 1.0, 2111 s) over the best mean
   (f = 0.8, 2499 s, +0.05). Since B = T(C2), every condition gets the shorter budget. This
   follows the protocol; confirm it, or amend the protocol **before** the E4 freeze.
5. **Root lines.** Apply `INTEGRATION.md` §4 (root CLAUDE.md rows, CM-9 wording,
   `project_synthesis.md` §5.1) if you agree.
6. Still open from earlier phases:
   - Q-9, Belief's evidence topic;
   - O-5, clutter f;
   - O-8, multiple areas;
   - Q-13, real-water authority;
   - L-K, kinematics.

---

## Phase 3 — 2026-11-04 — dev loop run (reference-belief-conditional, preliminary)

**C2 first** (`C2_TUNING_PROTOCOL.md`, seeds 1000–1019):
- grid f {0.6…1.0} × direction {long axis, +90°} × interlace {auto, off};
- B_cap = 2499 s;
- frozen: **f = 1.0, long axis, interlace auto**; B = 2111 s;
- `config/c2_tuned.yaml`, sha256 `abff9534…`;
- the runner refused C3/C4 without the freeze, and every result row carries the hash.

**Evaluation**, seeds from 2000, paired, all on the frozen C2:
- main: n = 30 at f = 1.0, conditions C1, C2, C3, C4, C4_iso, C4_rand, C4_depth1, C4_degraded;
- budget sweep: n = 20 at f = 0.75 and 0.5, conditions C2, C3, C4, C4_iso, C4_rand;
- merged with main's f = 1.0 rows on the common seeds;
- an uncontended timing run.

Numbers: `DEVLOOP.md` §5; outputs: `results/devloop_2026-11-04/`.

**Findings (dev loop only):**
- C1 is clearly worse than C2.
- At f = 1.0 no C4 variant is distinguishable from C2 (C4 +0.27, CI [−0.32, +0.86]).
- At 50 % of the budget, and under budget cuts, every C4 variant is clearly ahead.
- **C4 = C4_rand** (paired +0.00): the gain comes from revisiting early, not from the bearing.
  Every plan segment is a pass in the reference belief.
- C4_depth1 is at least as good as C4, and 3× faster.

**Fixes and additions in this phase:**
- `runner.py --from-results`, which merges finished runs without re-running and refuses mismatched
  freezes or calibrations;
- DEVLOOP §2 now states the pass definition;
- LOCAL_CHECKLIST L-B6 asks for Belief's pass segmentation;
- EVIDENCE_REGISTER E-82…E-87, A-09, A-10.

---

## Phase 4 — 2026-11-04 — ROS code written, wiring checked offline, never run under ROS

**Written:**
- `ros/executor.py` (ROS-free):
  - the state machine of DESIGN §7: start, `disarmed` on override or E-STOP, re-arm only through
    `~/start` plus checks, and the stop states;
  - one outstanding request, an injected clock, the pre-send check (`missed_lead`), the answer
    timeout with reconciliation from progress, the refusal table and `foreign_update`.
- `ros/replanner_node.py`, the only rclpy code. It has:
  - the subscriptions BeliefGrid (RELIABLE, TRANSIENT_LOCAL, depth 1), progress, events and odom;
  - the `/blueboat/update_path` client;
  - the services `~/start` and `~/stop`;
  - the outputs `/replanner/plan`, `/decision` and `/status`;
  - run records under `<root>/data/Replanner_data/`.
- `eval/emulator_node.py`: `output: detections` only; `image_records` raises [OPEN Q-9].
- Two launch files, with every parameter typed through `ParameterValue`.

**Gate** (`python3 -m test.smoke_test`, system python 3.13, numpy 2.4.4, no ROS): **188 passed,
0 failed, 1 skipped**. The skip is the real-Belief check of [20]. The sections behind Phase 4 are:
- [23] the executor against the Control stub with a fake clock: D-1…D-5, G-1, a stale belief, a
  lost answer reconciled, a foreign update;
- [24], new in this phase: `replanner_node.py`, `emulator_node.py` and both launch files driven
  through **stand-in** rclpy, message, service and launch modules written in the test:
  - the node graph and the BeliefGrid QoS profile;
  - `~/start`, then a whole C3 run (1 decision, 2 accepted requests, `done`);
  - `/replanner/plan`, the status and decision schemas, and the run folder;
  - `~/stop` after `done`;
  - for the emulator: one draw per object per pass over 4 passes, detections near the true
    objects with unique ids in Perception's fill convention, and `image_records` refused;
  - the launch arguments = declared node parameters, with the same types.

  The stand-ins prove the wiring of these files only. **This is not ROS integration:** DDS, QoS
  matching, executors, the real message classes and the real Belief are untested.
- Static check: `ruff --select F,E9` is clean on the package (no undefined names in either
  node). It is not a configured gate.

**Defects found by [24] and fixed:**
1. `/replanner/plan` published every mirror segment in full, so the part of a superseded segment
   after the change point appeared and the stamps went backwards. Each segment is now published
   up to the next one's start.
2. `requests.jsonl` was appended once a second with the latest request, which duplicated it and
   logged it before its answer. It now gets one line per request, once answered or
   reconciled.
3. The emulator stamped from `port_stamp`, which is zero on a starboard-only row. Its detection
   ids also repeated across messages. It now uses the present side's stamp and unique ids.
4. Launch arguments were passed untyped. launch_ros YAML-parses them, so `calib_a:=2` would have
   been an int against a float parameter. They are now typed with `ParameterValue`.

**Not run:** everything in `LOCAL_CHECKLIST.md` L-R, L-E and L-G.

---

## Phase 2 — 2026-11-04 — core built, gate green

**Approval.** The user approved the revised Phase 1 ("move on to the next part of the work").

**Built** — `blueboat_replanner/` (ament_python):
- `core/`: config (every parameter with a category), geo (MCS copy) and frames, areas, spacing,
  Dubins, routing, plan (sampling and mirror), validate (Control's request rules), coverage,
  evidence (contract validation), candidates, mission_plan, policy/{base, coverage (C1, C2),
  append (C3)};
- `io/`: mission and trajectory archive;
- `ros/adapters.py` (ROS-free, duck-typed).

**Gate.** `python3 -m test.smoke_test`, run from `blueboat_replanner/`: **128 passed, 0 failed,
0 skipped** (2026-11-04, system python 3.13, numpy 2.4.4).
- It includes the bit-identity check against MCS `geo.py` (AST plus 2000 values).
- It includes cross-checks against Control's ROS-free `path_update.PathUpdater`: the first path
  and our tail are accepted, an offset tail is refused, and our request check agrees with
  `validate_samples` on 40 cases.
- No file was written in another module (`sys.dont_write_bytecode`; all other repos clean).

**MATH clarifications made during implementation** (MATH first, then code):
- lines are spread evenly (§2.2);
- route nodes are joined by Dubins paths (§3.3);
- truncation drops trailing non-productive segments (§4);
- C3 fallback bearing = θ₁, not "heading + π/2", and NN keyed on each contact's cheapest leg
  (§13).

---

## Phase 1 closure — 2026-11-04 — STOP GATE (waiting for explicit approval of the revised design)

**Reviewed:** issues A–I raised by the user on DESIGN.md / MATH.md. Full record in
`docs/PHASE1_CLOSURE.md`.

**Corrections:**
- **A.** The brief's p_eff = 1 − Q^{1/n} reproduces Belief's residual only when every class has
  s_k = 1. p is now the root of the class-mixture equation (unique; bisection; clamped when
  unreachable). Demonstrated numerically: Q = 0.25, n = 2 → brief 0.403, calibrated 0.250.
- **B.** The rejoin to coverage was not charged. Candidates are now valued on the **whole executed
  tail** (detour ⊕ rejoin ⊕ remaining coverage truncated at the budget). C2, C3 and C4 all start
  from the C2 plan; C4 with no detour is exactly C2.
- **C.** The second lookahead step is chosen by the combined-sequence value. The union identity
  (no double counting) is proven and numerically checked. The worked example flips the choice A → B.
  Pending looks [τ − t_lat, c] are counted as seen.
- **D.** Stop states:
  - `stop_requested`, `stop_accepted`, `stopped_holding`, `stop_rejected` ("STOP NOT IN EFFECT"),
    `run_ended`;
  - override or E-STOP → `disarmed`; re-arm only by `~/start` plus checks.
- **E.** No reliance on turning in place. The first path is a Dubins manoeuvre from the hold pose
  with the boat's yaw, charged to B.
- **F.** The coverage band (6 m … √(R² − h²)) is used only for spacing. The detection band
  ([3, 25] m, geometric cap) is used for value.
- **G.** Verified Control bounds are separated from our margins. A pre-send check against the
  freshest `min_change_point`; `missed_lead` → not sent.
- **H.** The NOAA grazing term is now **off by default** (brief §3 formula), kept as an option.
  Other placeholders are confirmed; O-5 and O-8 stay open.
- **I.** `docs/C2_TUNING_PROTOCOL.md`, with a freeze enforced by the runner. Not run.

**Documents:**
- new: `PHASE1_CLOSURE.md`, `C2_TUNING_PROTOCOL.md`;
- rewritten: `MATH.md`, `DESIGN.md`;
- updated: `topics.md` (states, decision schema), `CONTRACT_CONSUMED.md` (p_background check),
  `LOCAL_CHECKLIST.md` (L-B5, L-B6, L-C5, L-C6), `EVIDENCE_REGISTER.md` (E-76…E-81, A-06…A-08),
  `QUESTIONS_PHASE0.md`, `REQUESTS.md` (RQ-2 item 4).

**Checks run:** numerical checks (calibration table, identity, worked example, spacing table), in a
scratch script outside the repo. No code exists; no gate yet.

**Open:**
- O-1 Belief contract binding and evidence topic;
- O-2 R-3;
- O-5 clutter f;
- O-8 multiple areas;
- t_lat (L-B5);
- Belief detection model (L-B6).

**Next, after approval:** Phase 2 core (C1/C2/C3, smoke gate), then Phases 3–5 without further
approval requests.

---

## Phase 1 — 2026-11-04 — STOP GATE (waiting for approval of the design)

**User answers, 2026-11-04:**
- speed 0.5 m/s;
- swath = one side, with spacing a function of range from a literature check;
- the brief overrides other sources for the inputs;
- append-on-completion is built first, inside a modular policy architecture that allows live
  interruption later;
- C4_rand uses a pre-drawn bearing table;
- the context pack does not exist; v1 comparisons are [NOT AVAILABLE].

For every other question I adopted my recommendation as a labelled default (`QUESTIONS_PHASE0.md`
"Answers").

**Written (documents only, no code):**
- `DESIGN.md`: 20 decisions; structure; frames; the path-change transaction; refusal handling;
  stop truncation; policies and epochs; state machine; freshness; safety table; outputs;
  traceability; 7 open items.
- `MATH.md`: the spacing function (NOAA HSSD 2025.0.01 §8.3 grazing floor, Yordanova & Gips 2020
  §III-A, verified by brute force); coverage geometry (exact numpy predicates, Dubins turns,
  visibility-graph transits); time and budget; the look predictor; the folded sensor model; the
  contact posterior with fallbacks; the background residual; candidates; two-step selection; the
  three rules; the C3 schedule; 22 test properties; the parameter table.
- `topics.md`: every input and output, the two JSON schemas, files.
- `CONTRACT_CONSUMED.md`: every C-3 and C-4 field.
- `REQUESTS.md`: RQ-1 to RQ-4.
- Updates to `QUESTIONS_PHASE0.md`, `EVIDENCE_REGISTER.md` (E-70…E-75, L-01…L-03),
  `CONTRACT_DEVIATIONS.md` (CD-2 settled), `LOCAL_CHECKLIST.md` (L-K3, L-K4).

**Literature read:**
- NOAA HSSD 2025.0.01 §8.3 (full section);
- Yordanova & Gips 2020 §III-A (arXiv preprint).

Vendor and secondary sources were seen and are not used as evidence.

**Gate pass count:** no gate yet (Phase 2).

**Design points worth the user's attention:**
1. In water 1 m deep or less, the grazing floor halves the spacing (s ≈ 4–5 m), which roughly
   doubles the C2 budget. It is one switch.
2. One mission area only: a route between disjoint areas would leave the areas (O-8).
3. C4_rand keeps C4's choice of revisits and randomises only the bearings, so the number of
   revisits is matched.
4. 0.5 m/s is above Control's ≤ 0.45 m/s simulation advice; L-K3 checks it in-machine.

**Next, after approval:**
1. Phase 2a: core, coverage, executor model, C1/C2/C3, smoke gate.
2. Phase 3a: dev loop for C1–C3.
3. Then C4.

---

## Phase 0 — 2026-11-04 — STOP GATE (waiting for the user's answers)

**What exists (documents only, no code):**
- `docs/REPLANNER_BRIEF.md`: the build guide, copied read-only
  (sha256 `b9aa7ff6…56ca5a`). The older name `docs/BUILD_GUIDE.md` is not used (prompt §B.3).
- `docs/PHASE0_VERIFICATION.md`: brief §7.1 and §12 answered from the code, with the revisions
  inspected and 20 discrepancies.
- `docs/EVIDENCE_REGISTER.md`: E-01…E-69 and A-01…A-05, labelled.
- `docs/QUESTIONS_PHASE0.md`: Q-1…Q-19, each with a recommendation and the consequences of each
  option.
- `docs/LOCAL_CHECKLIST.md`: the ROS, Gazebo, Belief and kinematics checks for `REPLAN-2-LOCAL`.
- `docs/CONTRACT_DEVIATIONS.md`: none adopted; 4 candidates.
- `.claude/CLAUDE.md`: a minimal stub with the session rules and *Status*.

**Gate pass count:** no gate exists yet (Phase 2).

**Tests of other modules run here (read-only, `PYTHONDONTWRITEBYTECODE=1`):**
- Control `test/test_path_update.py`: 102 passed, 0 failed.
- Control `test/test_path_update_log.py`: 24 passed, 0 failed.

**Key findings:**
1. The mid-run mechanism exists: `/blueboat/update_path` (Control, 2026-10-15). It is verified in
   code and its ROS-free test passes, so the brief §7.1 stop condition does not apply.
2. Built-world simulations cannot run replanner-owned runs yet (Control R-3); only
   `Sim_launch.py` can.
3. 1.0 m/s is infeasible in Gazebo (top speed ~0.78 m/s; Control advises ≤ 0.45 m/s).
4. "Usable swath" is undefined, which is a factor-two ambiguity in line spacing and in every
   budget.
5. Three sources disagree on where mission areas come from (brief Q9, synthesis §5.1 ⚠,
   contract).
6. The workspace says Belief reads `/sss_ai/image_records`; the contract says
   `/sss_ai/detections`.
7. No turn rate or settle time is measured anywhere.
8. Synthesis §6.2 ⚠ limits what dev-loop sweeps may claim.

**Deviations from the prompt:**
- Plan mode was not switched on as a tool, because Phase 0 requires writing these files. Only
  documents were written; no code.
- The context pack `00`–`03` and `papers_to_be_checked.md` are absent from the workspace.

**Open:** Q-1 … Q-19 (P0: Q-1 to Q-9 and Q-11 to Q-13).

**Next:** after the answers, Phase 1 writes `DESIGN.md`, `MATH.md`, `topics.md` and
`CONTRACT_CONSUMED.md`, then stops for approval.
