# BlueBoat-Replanner — design

Status: **approved by the owner on 2026-11-04, built, and bound to BlueBoat-Belief's own core
on 2026-11-12.**
- the ROS-free core and the dev loop are tested offline; the dev loop's belief map is
  BlueBoat-Belief's core, through its documented offline interface (`REAL_BELIEF_BINDING.md`),
  and its missions are executed by the node's own executor (§5.4);
- the ROS node and the emulator node are driven through fake rclpy modules in the gate ([24]);
  on 2026-11-12 both **ran under ROS on an isolated graph beside Belief's node**, with made-up
  sonar, navigation and Control (`test/ros_graph_check.py`, `REPORT.md` §F). Built into
  `~/ros2_ws/install` on 2026-11-13 (again 2026-11-18). The replanner node ran once beside
  Control's node, in Gazebo, on 2026-11-18 (a C2 development run in a built world,
  `LOCAL_CHECKLIST.md` L-G1); the emulator node not yet; neither on the boat (L-C, L-G2…L-G4).

Where this document and the code disagree, the code wins: fix the document.

Companion documents:
- the mathematics: `MATH.md`;
- every interface: `topics.md`;
- the Belief contract, field by field: `CONTRACT_CONSUMED.md`;
- the C2 baseline tuning procedure: `C2_TUNING_PROTOCOL.md`.

**Labels** (brief §16.2): [DECIDED] [CODE] [DOC] [TEST] [DEMO] [ASSUMPTION] [OPEN], plus
[NOT AVAILABLE] for comparisons with an earlier prototype or oracle that does not exist.

**Evidence ids:** `E-nn`, `L-nn` and `A-nn` are in `EVIDENCE_REGISTER.md`; `L-X`, `L-C`, `L-G`, …
are the in-machine checks of `LOCAL_CHECKLIST.md`; `O-n` are the open items of §12.

## 1. Purpose and scope

At decision points during a run, the module decides whether the boat:
- continues its survey;
- revisits a contact from a chosen viewing bearing; or
- flies a coverage line out of order.

It makes the boat do it within the same run, through Control's live path update. It also generates
every survey condition: single pass, two-pass orthogonal, append-on-completion, the informative
replanner and its ablations. All of them go through **one** coverage generator and **one**
executor, so every condition pays the same execution costs (CM-12, synthesis §7.3; E-54, E-65).

**Deliberately not built** (brief §11):
- a message package;
- a GTSP solver, a POMDP, learning;
- a GUI;
- a second path server or any path-change mechanism of our own;
- anything that moves the real boat without an operator-started run.

## 2. Decisions

| # | Decision | Label | Source |
|---|---|---|---|
| 1 | The path is changed mid-run only through `/blueboat/update_path` | [CODE] | E-01…E-19 |
| 2 | Inputs are the brief's (§6.2): the BeliefGrid (only target evidence), `/blueboat/odom`, the mission file, Control's progress, events and answers, and an optional read-only Belief-config cross-check. Never online: detections, sonar, simulation ground truth, `/sim/*` | [DECIDED] user 2026-11-04 | brief §4.3, §6.2, §16.1; E-72 |
| 3 | Mission areas come from the mission file (WGS-84 polygons, brief §6.4), converted with Belief's published datum; the polygon must lie inside Belief's own area | [DECIDED] user 2026-11-04 | brief Q9; E-68 |
| 4 | Survey speed **0.5 m/s** in every condition | [DECIDED] user | E-70 |
| 5 | Swath = **one side**. Line spacing is a function of range: W = √(R² − h²) − r_near (brief §3); the NOAA grazing term is an option, **off by default** | [DECIDED] (one side, function of range, brief §3); [ASSUMPTION] (option) | E-71, L-01, L-02 |
| 6 | Append-on-completion = C3, built first. The policy architecture is modular, so that a later policy may interrupt a line | [DECIDED] user | E-73 |
| 7 | C4_rand uses a bearing table pre-drawn in `eval/`. No online randomness | [DECIDED] user | E-74 |
| 8 | Plans are made in Belief's map frame M; exported paths are in Control's `world` frame W; one conversion module | [DOC] brief §6.5; C-5 | E-10, E-33 |
| 9 | Turns are Dubins arcs (r_turn 3 m), with lead-ins for settling. No stationary corners and **no reliance on turning in place**; the first path is a Dubins manoeuvre from the hold pose (MATH §3.5) | [ASSUMPTION] (values); [DOC] (field radii 1.5–6 m) | E-23, E-41 |
| 10 | Decisions are taken at decision points: the ends of coverage legs and revisit bodies (brief Q10). Mid-line change points only for a policy that declares `may_interrupt` | [DECIDED] | — |
| 11 | One controller for every condition; PID by default | [ASSUMPTION] default, user 2026-11-04 | E-24, E-25 |
| 12 | Planner sensor parameters are labelled placeholders; an aspect-agnostic variant exists | [DOC] brief §4.5 | A-01 |
| 13 | Planner model ≠ world model is configuration, not code | [DOC] brief §4.6 | — |
| 14 | Belief's core and Control's `path_update.py` are imported only in `eval/` and `test/`, behind guards | [ASSUMPTION] default, user 2026-11-04 | CM-3 |
| 15 | A single emulator in `eval/`, run instead of Perception, never beside it. It publishes `/sss_ai/detections`, one array per seabed image, empty ones included (the input Belief requires; settled 2026-11-12) | [ASSUMPTION] | Belief `topics.md` §1 |
| 16 | Dev-loop results are development evidence only (synthesis §6.2 ⚠), never E4 | [DOC] | synthesis §6.2, §7.4 |
| 17 | Class mapping `concrete_block`→block, `pipe_pvc`→pipe, `mesh_crate`→cage, `tire_car`→tyre (world side). Plate has no simulator class. The planner's map from **Belief's** class names to aspect families (`class_families`) is empty until the owner fills it (RQ-6) | [ASSUMPTION] default, user 2026-11-04 | E-38 |
| 18 | No GTSP: nearest-feasible ordering plus a two-step lookahead (the superproject's `literature_conclusions.md` §5 advises a turn-penalised GTSP; the brief's user decision stands) | [DECIDED] brief §5, §8.5 | E-60 |
| 19 | The C2 tuning protocol is frozen before any C3/C4 evaluation result (`C2_TUNING_PROTOCOL.md`) | [ASSUMPTION] default; CM-12 | — |
| 20 | Comparisons with the brief §9.5 prototype | [NOT AVAILABLE] | E-75 |
| 21 | **C2, C3 and the C4 family all start from the C2 plan.** C4 with no detour equals C2; C3 inserts its revisits at the pass-1/pass-2 boundary; C1 is pass 1 only | [ASSUMPTION] design choice, approved 2026-11-04 | MATH §10.1 |
| 22 | Candidates are valued on the **whole executed tail over the remaining budget** (detour, rejoin, truncated remainder); the second lookahead step is chosen by the same combined value | [ASSUMPTION] design choice, approved 2026-11-04 | MATH §10–§11 |
| 23 | Background residual calibrated on the class-weighted mixture (MATH §9.1) | [ASSUMPTION] design choice (corrects the brief's formula, valid only for s ≡ 1), approved 2026-11-04 | MATH §9.1 |
| 24 | Manual override or E-STOP **disarms** the replanner; it resumes only after an explicit `~/start` and the re-arm checks | [DOC] brief §7.2 ("stops on … manual override or E-STOP") | §7 |

## 3. Structure (layout copied from the existing modules)

An ament_python package one level below the git root, like BlueBoat-Perception and
BlueBoat-SSS-Sim.

```
BlueBoat-Replanner/
├── README.md · .gitignore
├── .claude/  CLAUDE.md · TODO.md · rules/{core-purity,models-apart,frames-trajectory}.md
│             settings.json (Stop hook → tools/smoke_gate.sh)
│             tools/{smoke_gate, devloop_batches, ros_check}.sh
├── docs/     HANDOVER, REPORT, DESIGN, MATH, topics, CONTRACT_CONSUMED, REAL_BELIEF_BINDING,
│             DEVLOOP, C2_TUNING_{PROTOCOL,RESULT}, EVIDENCE_REGISTER, LOCAL_CHECKLIST, REQUESTS,
│             REPLANNER_BRIEF (the specification, read-only)
├── results/  devloop_real_<date>/{c2_tuning, main, budget_sweep, budget_merged, class_true,
│             ungated, timing_uncontended, distinct_sectors}  (dev-loop outputs on Belief's
│             core: preliminary; DEVLOOP §5; made by .claude/tools/devloop_batches.sh)
└── blueboat_replanner/                   package root (ament_python); gate: python3 -m test.smoke_test
    ├── package.xml · setup.py · setup.cfg · resource/
    ├── blueboat_replanner/
    │   ├── core/   ROS-free, deterministic (numpy + stdlib)
    │   │   config · geo (MCS copy, frames) · areas · spacing · dubins · routing · coverage
    │   │   plan (Segment/Plan, sampling, ControlPathMirror, transform_segments) · validate
    │   │   evidence (contract validation) · candidates (revisit legs) · mission_plan
    │   │   model (sensor model) · value (posterior, background calibration) · looks (predictor)
    │   │   sequence (J / J2 evaluator) · policy/{base, coverage (C1, C2), append (C3),
    │   │   informative (C4 family)}
    │   ├── io/     mission · trajectory_yaml · recordlog (data root, O_EXCL run folders) — PyYAML here
    │   ├── ros/    adapters (the ONE field-name table, ROS-free) · executor (ROS-free; the only
    │   │           code that knows Control's mechanism) · replanner_node (the only rclpy code)
    │   └── eval/   belief_real (the binding to BlueBoat-Belief's core) · emulator (looks at
    │               objects, seabed images) · world (+ calibrator) · control_stub (+ its
    │               executor client) · devloop (missions run by ros/executor) · runner ·
    │               compare (paired differences) · emulator_node  ← all randomness is here
    ├── config/   replanner.yaml (every default + category) · mission_example.yaml ·
    │             c2_tuned.yaml (the C2 freeze, written by the runner; applied by the node to
    │             every mission, `c2_tuned_file`, §5.1)
    ├── launch/   replanner_launch.py · emulator_launch.py (launch imports optional)
    └── test/     smoke_test.py (the gate's runner: gate/common.py, then the sections
                  gate/s01_*.py … s25_*.py in one shared namespace; a section that raises is
                  one failure and the next still runs; exit != 0 on failure; [20], [22] run on
                  BlueBoat-Belief's core; [24] = the two nodes and both launch files over fake
                  rclpy / launch modules) · test_gate.py (the gate as one test, in a
                  subprocess: `colcon test`) · ros_graph_check.py (run by hand on an isolated
                  ROS domain: the installed nodes beside Belief's node, made-up inputs; not
                  installed, not part of the gate)
```

**Core purity**, enforced by the gate's import sweep:
- no `rclpy`, Qt, OpenCV, torch, `random`, `time` or `numpy.random` in `core/`, `io/` or `ros/`
  (`ros/` uses a clock only through the node: `time.time()` or rclpy's node clock, chosen by the
  node parameter `time_source`, MATH §11.3);
- `Policy.on_epoch` is pure.

## 4. Frames

| Frame | Definition | Source |
|---|---|---|
| M (map) | ENU about Belief's datum (`datum_lat`, `datum_lon`); metres; yaw in rad, CCW from East | C-3, C-5 |
| W (world) | Control's frame = `/blueboat/odom` local ENU. Origin: the launch point (real); Gazebo origin = spawn = world `geo_anchor` (sim) | E-02, E-19, E-37 |
| WGS-84 | mission areas, archive anchors | brief §6.4 |

**Conversions** (`core/geo.py`, one place):
- **WGS-84 → M:** the MCS pair, copied verbatim; bit-identity is tested against MCS's file.
- **W → M:** `x_M = R(θ) x_W + t`, from `odom_to_map_*`. M → W is the inverse, with
  `yaw_W = yaw_M − θ`.
- **Plans** are made in M. **Requests** are sent in W.
- **Junctions** are computed in W from `ControlPathMirror`, so continuity never depends on the
  transform staying constant.
- **Round trip** tested to 1e-9 m.
- **Transform drift:** a jump larger than `transform_jump_m` (0.5 m) between decisions is logged.
  Already-sent path is untouched.
- **Datum or geometry change** between snapshots: `ContractError`, and the mission stops.

## 5. Execution model — the path-change transaction

Control's mechanism is used as documented (`BlueBoat-Control/docs/PATH_UPDATE_INTERFACE.md`;
E-01…E-19). **`ros/executor.py` is the only code that knows it.**
- Its offline twin `eval/control_stub.py` enforces the same rules. It is written from the interface
  document only, like Control's demo client. The gate ([23]) and the dev loop (§5.4) drive the
  executor against it.
- When importable, Control's ROS-free `path_update.PathUpdater` cross-checks it in the gate.

### 5.1 One run

1. **Ready** when all of these hold:
   - progress `state == awaiting_path` with `hold`;
   - the grid is valid (`odom_to_map_valid`) and fresh;
   - the mission is valid;
   - `/blueboat/update_path` is served (`client.ready()`);
   - `~/start` has been called.

   What is missing is named in the `start_timeout` failure (§7).

   **The mission's configuration** is layered: `config/replanner.yaml` (the node's
   `config_file`) < the C2 freeze (`config/c2_tuned.yaml`, the node's `c2_tuned_file`, default
   the installed copy) < the mission file's `config`. The freeze is applied to every condition,
   as in the dev loop; a mission file that sets a frozen key to another value is refused at
   `~/start`, and a missing freeze file refuses every start. The run manifest records the file,
   its sha256 and its settings.
2. **First path** (MATH §3.5):
   - an initial Dubins manoeuvre from (`hold.x`, `hold.y`, ψ₀), where ψ₀ is the fresh odometry yaw,
     else `hold.yaw`, to the first coverage leg;
   - then the condition's plan: C1 = pass 1; **C2, C3, C4 = the C2 plan**;
   - converted to W, sampled, and sent with `change_point = NaN`, `id = r<run>-0000`.

   The design does not rely on in-place turning. A hold outside F_margin, or no
   feasible manoeuvre, is a mission error at `~/start`.
3. **Follow.** Track `tau`, `following_id`, `end_reached`. The plan's t axis is Control's t axis.
4. **Decision point.** For a policy with epochs, the trigger (§5.2) fires before a decision point.
   - The policy returns CONTINUE, REPLACE(tail) or STOP.
   - REPLACE builds the executed tail (MATH §10.2): detour ⊕ rejoin ⊕ remaining K, truncated at the
     budget.
5. **Change point.** c = the decision point's end time. The new tail starts at the mirror pose at c,
   with the mirror's tangent, which satisfies 0.20 m / 10° by construction (E-07).
6. **Always the whole remainder.** Every request carries the full intended remainder up to the
   budget end. Without further decisions, the mission completes as planned.
7. **End.** On `end_reached` with nothing to append, the run is `done`, and Control holds the last
   pose (E-17).

### 5.2 Timing, requests, answers, refusals

**Timing.** Verified Control bounds are separated from our configurable margins:

| Quantity | Value | Kind |
|---|---|---|
| lead_min | max(path_time + 0.1 s, 3 m / U_d) = 6.0 s for PID/LoS at 0.5 m/s | **Control, verified** (E-06) |
| `min_change_point` | τ + lead_min, at 10 Hz and after every answer | **Control, verified** (E-11) |
| τ rate | ≤ 1 path-s per s (governor), so path-second margins are conservative in wall time | **Control, verified** (E-14) |
| Handling time; round trip | ≤ 2.2 ms; ≤ 2.9 ms | Control **[DOC]**, Gazebo-measured (E-24) |
| T_decide | 2 s, a compute budget, not a guarantee | ours |
| latency_margin | 2 path-s | ours |
| answer_timeout; reconcile_timeout | 5 s; then 10 s | ours |

**Requests.**
1. **Trigger.** A decision fires when τ ≥ t_dp − (lead_min + T_decide + latency_margin), with
   lead_min from the latest progress and t_dp the decision point.
2. **Pre-send check.** With the latest progress (age ≤ 0.2 s), send only if
   c ≥ `min_change_point` + latency_margin. Otherwise **do not send**: log `missed_lead`, and
   re-decide at the next decision point. Acceptance is never assumed.

   **Every request is then checked before it is sent** (`Executor._send`), in this order:
   - Control's request rules (`request_check`);
   - the area: every sample of the plan in F_(margin − slack), `area_check_slack_m` = 0.5 m
     (owner, 2026-11-13; MATH §3.6);
   - for a change (not the first path), the junction at c against the mirror, as Control checks
     it (0.20 m, 10° unless the old path is stationary over the governor window h = 0.05 s;
     `continuity_check`).

   A request that fails any of them is a defect here: it is never sent, the run is `failed`
   (`request_invalid:<reason>`), and Control keeps the last accepted path. Plans are built
   inside F_margin and their tails start on the mirror, so none fails by construction; the
   checks hold that against the re-expression of §4 and against a defect.
   **What "latest" means in the node.** The node computes a decision inside its timer callback,
   so no progress message is handled meanwhile: the progress it holds after computing dates
   from before the computation. The executor therefore ages `min_change_point` by the measured
   compute time (τ advances by at most 1 path-s per second, the table above), and sends only if
   c ≥ `min_change_point` + compute time + latency_margin. A decision that takes longer than
   about T_decide is thus dropped, never sent late. The decision record carries the compute
   time and the lead that was left (`timing`, topics.md §5.1).
3. **One request outstanding.** No decision while one is pending.
4. **Answer timeout** (5 s) — reconcile from progress and events:
   - `active_id == id` → accepted;
   - `last_answer.id == id` with `accepted == false` → refused;
   - neither within 10 s more → `failed`. Control keeps the last accepted path.
5. **Refusals.** Never a blind retry.

   | Refusal | Category | Response |
   |---|---|---|
   | `operator_override`, `not_ready` | state | `disarmed` (§7) |
   | `no_run` | state | `run_ended` |
   | `mission_owned` | state | `failed` (the run is not replanner-owned) |
   | `lead_too_short` | geometry | back to `running`; the **next** decision point decides again |
   | `discontinuous_position`, `discontinuous_heading` | geometry | back to `running` (the next decision point builds from the mirror again); a second occurrence in the run → `failed` |
   | `bad_request`, `duplicate_id`, `too_few_points`, `too_many_points`, `not_finite`, `time_not_increasing`, `time_out_of_range`, `too_fast`, `heading_incoherent`, `change_point_outside` | request | `failed` (a defect here) |

6. **The mirror changes only on acceptance.** A refusal never mutates the accepted-plan state.
7. **Logging.** Every request and answer is logged in the decision topic, `requests.jsonl`, and
   Control's own `-pathlog.csv` (E-13).
8. **The client.** The executor sends through `client.send(request, on_answer)`; the client
   calls `on_answer(answer, now)` with the answer's **arrival** time (`requests.jsonl`:
   `sent`, `answered`). In the node, an exception inside the executor's handling of an answer
   is not caught as a failed call: only the service call's own result is guarded.

### 5.3 Stop

`~/stop` asks for a truncation at the earliest feasible c (≥ `min_change_point` +
latency_margin). It is two samples at the mirror pose (x_c, y_c, yaw_c), at t = c and c + 1 s.
Checked against Control's code:
- zero displacement, so the segment is not "moving" and gets no coherence check;
- position gap 0;
- heading equal to the old path's, so it passes the 10° check.

| Situation | Status | Boat |
|---|---|---|
| `~/stop` received | `stop_requested` | follows the current path |
| Truncation accepted | `stop_accepted` (reports c and the stop pose) | follows the old path until τ = c, then holds |
| progress `end_reached` with `active_id` = the stop id | `stopped_holding` | holds the stop pose (Control's station keeping, E-17) |
| Truncation refused | `stop_rejected`: "**STOP NOT IN EFFECT** — the boat continues the previous path; use E-STOP, manual target or run stop" | follows the old path |
| `cleared` / `no_run` | `run_ended` | per Control (idle, zero thrust); **not** "holding" |

**Retries.** At most 3, then `stop_rejected` stays, with the reasons:
- `lead_too_short`: a fresh c;
- `discontinuous_*`: re-read the mirror once;
- state refusals: wait until progress is `following` again. A stop is an operator command, not
  autonomous replanning.

Only `stopped_holding` asserts that the boat holds.

### 5.4 The dev loop runs this executor

`eval/devloop.run_mission` executes every dev-loop mission with `Executor` against the Control
stub (`StubClient`, answers at once), on the loop's clock = path time (perfect tracking). Each
step it feeds the executor:
- Control's progress, with `ros_time` = Belief's clock (T0 + path time);
- the odometry (the mirror pose);
- Belief's grid when the executor will read it: at the start and on a tick where a decision
  is due (`Executor.decision_due`, the trigger of rule 1).

It ticks once, then flies the step. So the trigger, the policy call, the pre-send checks, the
request, the splice, the budget accounting, the refusal handling and the horizon reading are
the node's own code.

What differs from the node, and why:
- a policy computes in zero path time (the world stands still meanwhile), so the compute-time
  ageing of rule 2 runs with 0, and results do not depend on machine load. The wall time of
  each deciding tick is measured outside the executor;
- `tracking_lag_m` = 0 (MATH §11.3);
- two fields of `Mission` that a mission file cannot set: `planner_config` (C4_degraded's
  planner block) and `budget_fraction` (missions cut short).

On 2026-11-13, 40 stored missions re-run this way equal their stored rows (`DEVLOOP.md` §1).

## 6. Policies and decision points (modular, user 2026-11-04)

```
Policy.on_epoch(ctx: EpochContext) -> Decision
  ctx      : belief arrays, pending looks, mirror + plan state (K), tau, c, budget state, mission, config
  Decision : action ∈ {CONTINUE, REPLACE(tail), STOP}, rules_fired, scored[]
Policy.decision_points(plan) -> list[DecisionPoint]     # default: ends of `line` and `body` segments
Policy.may_interrupt -> bool                            # mid-segment change points allowed
Policy.extra_triggers() -> [Periodic(dt) | OnNewContact]  # for interrupting policies
```

- **Default decision points.** The ends of `line` (coverage leg) and `body` (revisit leg) segments.
  Never `connection`, `transit` or `rejoin`.
  - With PID/LoS at 0.5 m/s, lead_min is 6.0 s, so the trigger fires about 10 s (≈ 5 m) before the
    decision point.
  - A leg shorter than that window passes its decision point without a decision (`missed_lead`).
- **Interrupting policies (designed, not built).** `Periodic` or `OnNewContact` (a new uid) would
  fire at the earliest feasible c on the current segment, the unflown part of an interrupted line
  staying in K as a RECOVER item. Today `may_interrupt` is a flag that nothing reads, and
  `extra_triggers` does not exist.

| Condition | Policy | Initial plan | Decisions | `may_interrupt` | In E4 |
|---|---|---|---|---|---|
| C1 | `CoveragePolicy(passes=1)` | pass 1 | none | no | reference only |
| C2 | `CoveragePolicy(passes=2)` | C2 plan | none | no | **baseline** |
| C3 | `AppendOnCompletionPolicy` | C2 plan | one: the end of pass 1's last leg (if missed, the run continues as C2; logged) | no | yes |
| C4 | `InformativePolicy` | C2 plan | every default decision point | no (brief Q10) | yes |
| C4_live | `InformativePolicy(may_interrupt=True)` — **the flag only; no `OnNewContact` trigger, not run** | C2 plan | as C4 | flag | no |
| C4_iso | `InformativePolicy`, A ≡ 1 | C2 plan | default | no | ablation |
| C4_rand | `InformativePolicy`, bearings from `config.c4_rand.bearing_table` | C2 plan | default | no | ablation |
| C4_degraded, C4_depth1 | dev loop (C4_degraded's planner block is set by `eval/` only) | C2 plan | default | no | no |

**Build order** (user: append-on-completion first):
1. C1, C2, C3, with the executor and the dev loop;
2. C4 and its ablations.

C1 and C2 send exactly one request.

**Online, a mission file names C1, C2, C3, C4, C4_iso, C4_rand or C4_depth1.** `C4_live` and
`C4_degraded` are refused there: they would run exactly as C4 (no interruption; no degraded
block outside `eval/`). C4_depth1 stays online until decision 7 (Gazebo timing).

## 7. State machine

| State | Entered when | Leaves to |
|---|---|---|
| `idle` | node start; after a terminal state with a new mission | `waiting` on `~/start` |
| `waiting` | `~/start` accepted | `starting` when ready; `failed` after `start_timeout_s` (60 s), naming what is missing |
| `starting` | first path sent | `running` on acceptance; `failed` on a request refusal; `waiting` on a transient state refusal |
| `running` | following | `deciding` (trigger); `disarmed` (progress `override` or `not_ready`); `done` (end, nothing to append); `stop_requested` (`~/stop`); `run_ended` (`cleared`, `no_run`) |
| `deciding` | trigger | `requesting` (REPLACE); `running` (CONTINUE, `missed_lead`); `stop_requested` (a policy STOP, `mission_stop:rho_floor`) |
| `requesting` | request sent | `running` (accepted, or a handled refusal); `failed` (§5.2) |
| `disarmed` | operator override (manual target, arrows) or E-STOP | `running` only on `~/start` **and** re-arm checks: progress `following`, `active_id` == the mirror's last id, fresh inputs, no stop pending. Never automatic |
| `stop_requested` → `stop_accepted` → `stopped_holding` | §5.3 | `idle` (after `stopped_holding`) |
| `stop_rejected` | truncation refused (≤ 3 attempts) | `stop_accepted` on a later accepted retry; `run_ended` if the operator ends the run |
| `done` | plan exhausted | `idle` |
| `run_ended` | the operator stopped or cleared the run | `idle` (a new run and `~/start` needed) |
| `failed` | §5.2; a persistent `ContractError`; a mission error; `foreign_update` | `idle` |

- **No concurrency:** one decision and one request at a time.
- **Restart:** a node restart never resumes a mission. It comes back `idle`, and a new run plus
  `~/start` are needed.
- **Authority:** the plan is ours only while progress `active_id` equals the mirror's last accepted
  id. Any other id → `failed`, reason `foreign_update`.
- **Disarmed means Control continues.** While `disarmed`, Control keeps its own behaviour: on the
  operator's release it resumes the loaded path (E-16). The replanner neither changes nor stops it
  until re-armed.

## 8. Input freshness and degraded operation

All ages are **arrival times on the node's time source**, never `header.stamp`: real odometry is
stamped 0 (E-40), and the grid stamp is sim time under Gazebo. The time source is the node
parameter `time_source`: `wall` (default, the boat) or `ros` (the node clock; in Gazebo with
`use_sim_time:=true`, so the limits below are simulated seconds in a run slowed or paused by the
ray-cast sonar; MATH §11.3).

| Input | Rule | When not fresh at a decision |
|---|---|---|
| BeliefGrid | `grid_seq` strictly increasing; age ≤ 5 s [ASSUMPTION] | CONTINUE, logged `stale_belief`. C3's single decision then continues as C2 |
| Grid validity | contract checks; `odom_to_map_valid` | no first path; CONTINUE afterwards |
| Progress | age ≤ 1 s (10 Hz, E-11); pre-send age ≤ 0.2 s | no request |
| Odometry | age ≤ 1 s | decision pose = mirror pose at τ (`pose_source: mirror`, brief §6.2 fallback) |
| Pending looks | mirror path over [τ_h, c] counts as seen, τ_h = the grid stamp converted to path time, minus the tracking margin (MATH §11.3); each fallback (τ − t_lat, t_lat = 10 s) is named in the record | — |
| Late joiner | TRANSIENT_LOCAL depth 1 receives the last snapshot. Both ends were seen RELIABLE + TRANSIENT_LOCAL on a live isolated graph (2026-11-12, L-B1); a node joining long after Belief was not tested as such | — |

The odometry pose is used for the record and for ψ₀ (MATH §3.5). Tails always start on the mirror.

## 9. Safety table

| Trigger | Response | Evidence / authority |
|---|---|---|
| Mission start | Never autonomous: `~/start` **and** an operator-started run (real: MCS START RUN in a `path_source:=replanner` session) | brief §7.2, §17.12; E-21 |
| Areas | Plans are feasible by construction: coverage, transits and revisit legs are built and checked inside F_margin (MATH §3). **Every request is re-checked before it is sent**: all samples in F_(margin − 0.5 m), else never sent and the run `failed` (§5.2 rule 2) | brief Q9; MATH §3.6; owner 2026-11-13 |
| Start outside the area / no room to turn | Mission error; the replanner never leaves the area to manoeuvre | MATH §3.5 |
| Stale or inconsistent inputs | No request (§8) | brief §17.10 |
| Manual target, arrows, E-STOP | Control refuses updates (E-16); the replanner is **`disarmed`** until an explicit `~/start` with the re-arm checks | brief §7.2; E-16; Control N10 |
| Run stop / cleared | `run_ended` | E-18 |
| Rejection or unknown acceptance | §5.2: no blind retry; Control keeps the accepted path | E-09 |
| Plan exhausted | Control holds the last pose; `done` | E-17 |
| `~/stop` | §5.3. "Stopped" is reported only as `stopped_holding`; a rejected stop is reported as **not in effect** | brief §7.2 |
| Real-water autonomy | Changes are autonomous within an operator-started run and the areas. The first use comes only after Control's own on-water path-update test and a supervised dry run | [ASSUMPTION] default, user 2026-11-04 (open: O-7) | E-25 |

## 10. Outputs (schemas in `topics.md`)

| Output | Type | Content |
|---|---|---|
| `/replanner/plan` | `nav_msgs/Path`, frame `world`, TRANSIENT_LOCAL depth 1 | the mirror |
| `/replanner/decision` | JSON `blueboat_replanner_decision/1` | one per decision |
| `/replanner/status` | JSON `blueboat_replanner_status/1`, TRANSIENT_LOCAL depth 1 | 1 Hz and on change, with the §7 states |
| `~/start`, `~/stop` | `std_srvs/Trigger` | arm and stop |

**Files** go to `<root>/data/Replanner_data/<YYYY_MM_DD-HH_MM_SS>-<mission>-<condition>/`:
- `<root>`: `data_dir` → `$BLUEBOAT_DATA_DIR` → parent of the first `$COLCON_PREFIX_PATH` entry →
  cwd;
- the directory is claimed with `O_EXCL` (suffixes `-2`, `-3`, …); nothing is overwritten;
- contents:
  - `manifest.json`, `end.json`;
  - `mission.yaml`; `belief.json` (the grid the first path was planned on);
  - `plans/<k>-<id>.yaml` (`blueboat_trajectory/1` in frame M, with `geo_anchor` = datum);
  - `decisions.jsonl`, `requests.jsonl`, `status.jsonl`, `events.jsonl` (Control's events plus
    `belief_restart` and `transform_jump`). Schemas: `topics.md` §5, §6.

## 11. Traceability

| Requirement | Source | Design element | Test |
|---|---|---|---|
| Path change only via Control | brief §7; E-01 | `ros/executor.py`, `eval/control_stub.py` | stub ≡ `PathUpdater` on the same requests (guarded); Gazebo L-G1 |
| Continuity at c | E-07 | mirror, change-point rule | every generated tail passes the stub |
| A refusal leaves the plan unchanged | E-09; brief §17.14 | mirror updated only on acceptance | property 17 |
| Belief is the only target evidence | CM-9; decision 2 | source sweep (no detection, sonar, `ground_truth` or `/sim/` names in `core/`, `ros/`) | gate sweep |
| Areas and margin | brief Q9, §17.8 | `areas.py`, `coverage.py`, `routing.py`; `Executor._send` (send-time check) | every sample in F_margin; concave and narrow cases; a tail 3 m out is never sent ([23]) |
| Trajectory contract | E-08, E-30 | `plan.sample()` | t increasing ≥ 1 ms; speed ≤ 1.05 U; yaw = tangent; ≤ 20 000 samples |
| Spacing as a function of range | user; brief §3 | `spacing.py` | MATH §2.4 table; random brute force |
| Geo bit-identical to MCS | brief §6.5 | `core/geo.py` | value and source comparison |
| Initial manoeuvre without in-place turning | MATH §3.5 | `coverage.initial_manoeuvre` | E-1 |
| Candidate cost = executed tail | MATH §10 | `sequence.py` | B-1…B-5 |
| Lookahead consistency | MATH §11 | `sequence.py`, `policy/informative.py` | C-1…C-5 |
| Background calibration | MATH §9.1 | `value.calibrate_p` | A-1…A-5 |
| Detection band vs coverage band | MATH §2, §6 | `looks.py`, `spacing.py` | F-1, F-2 |
| Timing: no send under a missed lead | §5.2 | executor pre-send check | G-1 |
| The dev loop runs the node's executor | §5.4 | `eval/devloop.py` → `ros/executor.py` | gate [22]; 40 stored missions re-run identical (`DEVLOOP.md` §1) |
| Stop and disarm semantics | §5.3, §7; brief §7.2 | executor + state machine | D-1…D-5 |
| Three rules | synthesis §8.1; brief §8.6 | `core/policy/informative.py` (MATH §12) | abandonment bound; budget never exceeded; C1/C2 never change |
| Value-model properties | brief §13, §17.14 | `value.py`, `model.py` | MATH §14 |
| Look predictor ≡ Belief's look geometry | C-5; brief §9.2; Belief `topics.md` §4 | `looks.py` | gate [20] against BlueBoat-Belief's core: 100 % both ways on a line, 99.9 % with turns (MATH §5) |
| Belief read by meaning | Belief `topics.md` §3a and §11 (notes for a consumer) | `evidence.py`, `value.py`, `ros/executor.py` | H-1…H-8 (MATH §14) |
| Determinism | brief §13 | pure core | byte-identical decisions |
| CM-12 fairness | synthesis §7.3 | one generator, one executor, one speed and turn model; all adaptive conditions = C2 plus detours; frozen C2 protocol | `C2_TUNING_PROTOCOL.md`; runner freeze check |
| No online randomness | user rule | `eval/` only | gate sweep |
| Model-conditional results | CM-13 | labels | `REPORT.md` |

## 12. Open items

| Id | Item | Owner | Blocks |
|---|---|---|---|
| O-2 | R-3: `path_source` in SSS-Sim `full_mission_launch.py` — **done 2026-11-18** (RQ-1) | user → SSS-Sim | nothing now |
| O-3 | Turn radius, settle time, 0.5 m/s achieved, initial-manoeuvre tracking (L-K1…L-K4, L-C5) | in-machine | none (placeholders) |
| O-4 | r_near, measured r_far (L-S1) | in-machine | none (placeholders) |
| O-5 | Clutter re-detection probability f; planner aspect windows (E3) | user / E3 | C4 values (swept) |
| O-6 | Belief configuration file format, for the optional cross-check | Belief session | nothing |
| O-7 | Real-water autonomy, confirmed after a dry run | user | E6 |
| O-8 | Missions with more than one area (how areas connect) | user | multi-area missions only |

## 13. Scientific background (from the README)

*Moved verbatim from `README.md` §2 on 2026-11-18 (session 11); the README keeps a pointer.*


Only references the project has recorded are cited. Their reading status is recorded in
`docs/MATH.md` §16 (read in full by this module) or in the brief `docs/REPLANNER_BRIEF.md` §15
(status as the project recorded it, not re-read here). Each entry says what it supports here and
what it does not.

**Coverage and line spacing**
- **NOAA Office of Coast Survey**, *Hydrographic Survey Specifications and Deliverables*,
  v2025.0.01, §8.3 (read in full). Altitude 8–20 % of the range scale; less claimed coverage when
  the effective range shrinks. Used only by an optional grazing limit, off by default. It gives
  no rule for a hull-mounted sonar and no spacing formula.
- **Yordanova & Gips**, IEEE RA-L 5(3), 2020, DOI 10.1109/LRA.2020.3003886 (§III-A read in full).
  The effective range is where the detection curve crosses a threshold, and the gap under the
  boat (nadir) must be covered by neighbouring lines. This is the shape of the spacing rule of
  `docs/MATH.md` §2. It does not give the 0.8 spacing factor.

**Search with detections and non-detections**
- **Bourgault, Furukawa & Durrant-Whyte**, "Optimal search for a lost target in a Bayesian
  world", *Field and Service Robotics*, STAR 24:209–222, 2006 (read in full). It gives the
  non-detection update and the residual probability that a target was missed, the form of the
  background value of `MATH.md` §9. Its result that a planning horizon beats a greedy choice
  motivates the lookahead; it is no evidence about this module's two-step policy.

**Why the direction of a look matters**
- **Paull, Saeedi, Seto & Li**, "Sensor-driven online coverage planning for AUVs", IEEE/ASME
  T-Mech 18(6):1827–1838, 2013, DOI 10.1109/TMECH.2012.2213607 (read in full).
  Two looks along parallel lines are dependent; two looks at right angles are independent; in
  between it assumes a linear rule. The planner's folded-sector model keeps the two end cases
  and replaces the linear part by a step. Paull motivates it and does not calibrate it.
  BlueBoat-Belief uses Paull's rule itself.
- **Fawcett, Myers, Hopkin, Crawford, Couillard & Zerr**, "Multiaspect classification of
  sidescan images", IEEE JOE 35(4):863–876, 2010, DOI 10.1109/JOE.2010.2069832 (read in
  full). On real Klein 5500 data, a second look improves classification when the training data
  represent the features; fusing two poor looks does not help. It has **no result as a function
  of the angle between looks**, so it cannot support "perpendicular is best".
- **Myers & Williams**, "Adaptive multiview target classification in SAS images
  using a POMDP", IEEE JOE 37(1), 2012 (brief: full). Choosing the next aspect for
  classification; class-dependent stopping.
- **Ji, Parr & Carin**, "Nonmyopic multiaspect sensing with POMDPs", IEEE TSP
  55(6):2720–2730, 2007 (brief: abstract only). Target orientation as
  discrete sectors with a stationary response in each: the motivation for the planner's sectors.

**Is adapting worth it?**
- **Hollinger, Englot, Hover, Mitra & Sukhatme**, "Active planning for underwater inspection and
  the benefit of adaptivity", IJRR 32(1):3–18, 2013, DOI 10.1177/0278364912467485 (read in
  full). Under its assumptions the benefit of adapting is bounded by a constant, and in its
  experiments adaptive planning was "only marginally better". Its theorem does not apply here
  (travel time dominates, and the objective can go negative), but it is the reason a tie with C2
  is a plausible outcome.
- **Golovin & Krause**, "Adaptive submodularity", JAIR 42, 2011 (brief: abstract only). Greedy
  guarantees need assumptions that path-dependent travel costs break, so a short lookahead is
  used instead.
- **Hollinger & Sukhatme**, "Sampling-based robotic information gathering", IJRR
  33(9), 2014 (brief: full). Budgeted information gathering.
- **Kapetanović, Mišković, Tahirović, Bibuli & Caccia**, "A side-scan sonar data-driven coverage
  planning and tracking framework", *Annual Reviews in Control* 46:268–280, 2018, DOI
  10.1016/j.arcontrol.2018.10.012 (read in full). The closest precedent: a lawnmower survey that
  replans online to see "interesting" seabed from the opposite side. It pays when objects are
  sparse and costs more when they are dense, with a break-even point. It has no detector, no
  target evidence and no angle dependence, and its planner was not run at sea.

**Choosing the views and the revisits**
- **Sethuraman, Baldoni, Skinner & McMahon**, "Learning which side to scan", ICRA 2024:8348–8354,
  DOI 10.1109/ICRA57147.2024.10611077 (read in full). In simulation, informed views beat random
  views of the same number slightly (0.95 / 0.93 against 0.92 / 0.90). It supports **having** a
  random-bearing ablation (C4_rand), not its outcome here.
- **Wiig, Krogstad & Midtgaard**, "Autonomous identification planning for mine
  countermeasures", IEEE/OES AUV 2012 (read in full). Filter contacts first, respect a turning
  radius, add lead-in and lead-out legs over a target, penalise sharp turns. The thresholds,
  Dubins turns and lead-in lengths used here are this module's own choices.

**Confidence of a detector**
- **Guo et al.**, "On calibration of modern neural networks", ICML 2017
  (brief: full). A detector's score is not a probability until calibrated. Here, every value
  built on a detection score is labelled `score-conditional` while Belief reports the detector
  as uncalibrated.


## 14. How it works, in brief (from the README)

*Moved verbatim from `README.md` §4 on 2026-11-18 (session 11); the README keeps a pointer.*


The mathematics is in `docs/MATH.md`, the design in `docs/DESIGN.md`. In brief:

**Mission set-up** (`core/mission_plan.py`). The mission polygon is converted into Belief's map
frame with Belief's datum. The **budget B is the duration of the C2 plan** from the boat's hold
pose, the same for every condition, so all of them get the same time.

**One coverage generator** (`core/spacing.py`, `coverage.py`, `dubins.py`, `routing.py`;
MATH §2–§4). Each line covers one side of the boat, from 6 m to the geometric range limit
√(R² − h²) (R = 20 m sonar range, h = altitude). Line spacing is the largest that still covers
every point of the area, checked by brute force. Pass 1 runs along the area's long axis and
pass 2 at right angles. Turns are Dubins arcs of radius 3 m: the boat never turns in place,
and every sample stays inside the area minus a 2 m margin.

**The conditions** (`core/policy/`):

| Condition | What it does |
|---|---|
| C1 | single pass (reference only) |
| **C2** | **two passes at right angles: the baseline**, sent once, never changed |
| C3 | C2, plus one decision at the end of pass 1: revisit the unconfirmed contacts at right angles to their first look, nearest first, within the budget, then pass 2 |
| C4 | C2, with a decision at the end of every line and every revisit leg: carry on, or revisit one contact from the bearing of highest value (below) |
| C4_iso | C4 with the aspect model switched off (every bearing is worth the same): isolates H2 |
| C4_rand | C4 choosing *whether* and *which* contact, but flying a pre-drawn random bearing |
| C4_depth1 | C4 looking one decision ahead instead of two |
| C4_degraded | C4 with a deliberately wrong planner model (dev loop only; a mission file cannot name it, nor C4_live) |

**What a look is worth** (MATH §5–§11). For a candidate path the **look predictor** lists every
(cell, folded sector) the sonar would see, with Belief's own geometry: the 0.285 m transducer
offset, nothing on the inner side of a tight turn. Opposite sides of one line fall in the same
folded sector, so they count as the same look. Then:

- **Contacts.** Each contact gets a posterior probability p̃ of being real: its first score as a
  prior, updated by the sectors where it was detected and those where it was searched and
  missed, under an aspect model per target family. Each family has a window of good directions,
  and the windows are placeholders until the heading-response experiment measures them. Its
  **confirmation value** is the expected gain in confirmed real targets, minus κ times the
  expected false confirmations of clutter.
- **Background.** For cells with no contact, the residual chance of an undetected target
  (calibrated so that it reproduces Belief's own coverage, MATH §9.1) gives the expected number
  of new detections a path would make.
- **The value of a choice** is J = λ·(new detections) + Σ (confirmation values), taken over the
  **whole executed remainder**: the detour, the way back, and the rest of the survey cut at the
  budget. A revisit is never valued on its own, so it has to beat what the remaining survey
  would have seen anyway.
- **Two-step lookahead.** C4 values each first action by the best second action after a
  detection and after a miss. It switches away from "carry on" only if that gains more than a
  margin ε_J = 0.02 expected targets. Only the first action is sent; the next decision point
  decides again.
- **Three rules** (MATH §12): a contact is *resolved* when it is confirmed or its p̃ falls
  below 0.05; it is *abandoned* after 3 revisits, or when two revisits change nothing; the
  *budget* rule says no plan exceeds B, and revisits use at most half of it. An optional
  mission stop exists and is off by default.
- **What the map does not hold yet.** Belief's grid lags the boat by one to two seabed images.
  Its header stamp says up to which moment it is complete, and looks the boat has already flown
  beyond that are counted as **pending**, never as new value (MATH §11.3).

**The executor and safety** (`ros/executor.py`, DESIGN §5–§9). It is the only code that knows
Control's path update.
- **When to decide.** A decision is triggered early enough to reach Control before the change
  point: the lead Control requires, plus a 2 s compute allowance, plus a margin. A decision that
  finishes too late is dropped (`missed_lead`), never sent late.
- **What is sent.** Every request carries the full intended remainder of the mission.
- **Never autonomous.** It never starts on its own (`/replanner/start` and an operator-started
  run are both needed). An operator override or E-STOP **disarms** it until an explicit
  re-start passes the checks.
- **Honest stop reporting.** "Stopped" is reported only once Control confirms the boat holds
  (`stopped_holding`); a refused stop is reported as "not in effect".
- **Records.** Every decision is written with its inputs, candidates, rules fired, timing and
  labels saying what the numbers are conditional on.
