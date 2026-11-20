# BlueBoat-Replanner

The **aspect-aware replanner** of the BlueBoat side-scan sonar survey (thesis deliverable D6).
While a survey runs, it decides at the end of each survey line whether the boat should carry on,
or turn back to look at a possible target again **from a chosen direction**. It then makes the
boat fly that change within the same run, through Control's live path update. It also plans
every survey type the thesis compares, so all of them share one coverage generator, one turn
model and one executor.

| | State (2026-11-18) |
|---|---|
| Built | ROS-free planning core, executor, ROS node, detection-emulator node, launch files |
| Offline gate | `test/smoke_test.py`: **294 passed, 0 failed, 0 skipped** (2026-11-18; no ROS, about 90 s); `colcon test` runs it |
| Development loop | runs on **BlueBoat-Belief's own core**, its missions executed by the node's own executor: C2 tuned and frozen, 690 missions plus a labelled extra comparison of 150 (`docs/DEVLOOP.md` §5). Preliminary, never a thesis result |
| Under ROS | ran on an isolated ROS graph beside Belief's installed node, with made-up sonar, navigation and Control (`docs/LOCAL_CHECKLIST.md` L-X; on the code of 2026-11-12). Built into `~/ros2_ws/install` (2026-11-18) |
| In Gazebo | once, 2026-11-18: a C2 development run in a built world (the simulator's `full_mission_launch.py path_source:=replanner`) beside Control's node, Belief, Perception's dummy detector and the GCS, the node on simulated time (`use_sim_time:=true time_source:=ros`): first path accepted, plan flown to `done`, no stale input through the render holds (`docs/LOCAL_CHECKLIST.md` L-G1). A check, not a result |
| Not yet | C4 or the stop test in Gazebo, a trained detector, the boat |
| Simulator | frozen 2026-11-18 (`sim-freeze-v1.0-2026-11-18`, literal acceptance verdict NOT READY, used as a model-conditional tool); simulated results follow the superproject's `SIM_VALIDITY_CONDITIONS.md`; the CR-BEL-1 re-run of the dev loop on Belief's new core (CR-REP-1) is open (superproject `docs/OPEN_ITEMS.md`) |

Where the work stands and what comes next: **`docs/HANDOVER.md`**.

---

## 1. Why: the question this module answers

A side-scan sonar sees a seabed object from the side. Whether a target shows up, and whether it
can be told apart from clutter, depends on the direction it is seen from (its **aspect**): a
pipe seen end-on or a box seen corner-on can be nearly invisible, the same object seen
broadside is obvious. A fixed survey pattern sees each spot from one or two directions decided
before the mission.

The thesis asks whether a survey that **reacts to what it has seen** confirms more real targets,
and fewer false ones, than a good fixed survey of the same duration. Two effects are kept apart:

- **H1, return visits.** Going back to a possible target at all, for a second look.
- **H2, the bearing of the return visit.** Choosing the direction of that second look from a
  model of how the target's visibility depends on aspect.

The comparison is fair only if the fixed survey is strong. The baseline is therefore the
**two-pass orthogonal survey** (C2: the area flown twice, the second time at right angles),
tuned seriously and frozen before any adaptive result is looked at (CM-12,
`docs/C2_TUNING_PROTOCOL.md`). Every result is stated as **model-conditional**: it holds for the
sensor and target models used, which are placeholders until measured (CM-13).

## 2. Scientific background (full: `docs/DESIGN.md` §13)

Coverage and line spacing (NOAA Office of Coast Survey specifications §8.3; Yordanova & Gips 2020), search with
detections and non-detections, why the direction of a look matters, whether adapting is worth it, choosing the
views and the revisits, and the confidence of a detector; each source with what it supports here and what it
does not.
Reading status: `docs/MATH.md` §16 and `docs/REPLANNER_BRIEF.md` §15.

## 3. Place in the system

```
 sonar ──► Perception ──► Belief ──BeliefGrid──► Replanner ──/blueboat/update_path──► Control ──► boat
 (images)  (detector)    (belief map)              ▲    ▲                               │
                                       mission file │    └── /blueboat/path_progress, ──┘
                                                         /path_update_event, /blueboat/odom
```

- **The belief map is the only target evidence** (root CM-9). The map is
  BlueBoat-Belief's `BeliefGrid` (contract `belief_grid/0`). For each 0.5 m cell it holds how
  often and from which direction the cell has been searched, and it lists the possible targets
  ("contacts") with their scores. The module never reads detections, sonar, simulation ground
  truth or `/sim/*` online; the gate checks the source for those names.
- **Operational inputs, which carry no target evidence:** Control's path progress, events and
  answers; odometry; a mission file (one WGS-84 polygon, the planning depth, the condition),
  which must lie inside Belief's survey area; the C2 freeze (`config/c2_tuned.yaml`), which
  every mission flies.
- **The one way to move the boat:** `/blueboat/update_path`, Control's live path update. It
  replaces the rest of the path from a change point; a refusal changes nothing.
- **Outputs:** `/replanner/plan` (the path Control holds), `/replanner/decision` (one record
  per decision), `/replanner/status`, the services `/replanner/start` and `/replanner/stop`,
  and a write-once run folder under `<root>/data/Replanner_data/`. Every name, type and QoS:
  `docs/topics.md`.

| Needs | From | State |
|---|---|---|
| `/blueboat/update_path`, progress, events; a `path_source:=replanner` run | BlueBoat-Control | exists; its ROS-free core is cross-checked by the gate; run against Control's node once, in Gazebo (2026-11-18, `LOCAL_CHECKLIST.md` L-G1: the first path accepted, followed to its end) |
| `BeliefGrid`, and Belief's Python core for the dev loop | BlueBoat-Belief | exists; read by meaning (`docs/CONTRACT_CONSUMED.md`), core bound offline (`docs/REAL_BELIEF_BINDING.md`) |
| Replanner runs in built simulation worlds | BlueBoat-SSS-Sim (`path_source` in `full_mission_launch.py`, `docs/REQUESTS.md` RQ-1) | exists since 2026-11-18 (`full_mission_launch.py path_source:=replanner`; the node with `use_sim_time:=true time_source:=ros`, `LOCAL_CHECKLIST.md` L-G) |
| A "Replanner mode" in the mission station | BlueBoat-MCS (Control R-1) | missing; a real session uses *Extra args* `path_source:=replanner` |
| A trained, calibrated detector | BlueBoat-Perception | missing; the dev loop and Gazebo use the emulator of `eval/` instead |

## 4. How it works (full: `docs/DESIGN.md` §14; mathematics `docs/MATH.md`)

The mission polygon is converted into Belief's map frame; the budget is the duration of the C2 plan, the
same for every condition. One coverage generator (lines from 6 m to the geometric range limit, Dubins turns of
radius 3 m, a 2 m margin) builds every condition: C1 single pass, **C2 two passes at right angles (the
baseline)**, C3 append-on-completion, C4 informative and its ablations.

## 5. How it is evaluated, and what the development loop shows

**The development loop** (`eval/`, `docs/DEVLOOP.md`): no ROS.
- **Belief.** BlueBoat-Belief's own core, driven through its documented offline interface.
- **World and detector.** A seeded world (100 × 60 m, 2 m deep, 6 targets, 6 clutter objects)
  and an emulated detector that draws one detection per look, defined as Belief defines a look.
- **Pairing.** Paired seeds, so every condition flies the same worlds.
- **Order.** C2 is tuned and frozen first.
- **Score.** Confirmed real targets minus false confirmations.
- **Limits.** The boat tracks the plan perfectly. The world's detection model is the planner's
  own placeholder.

These numbers are therefore **"real-Belief dev loop, preliminary"**. They are never a thesis
result: those need Gazebo runs with a trained, calibrated detector (`docs/REPORT.md`).

| Δ score against C2 (≈95 % CI, n = 30) | C1 | C3 | C4 | C4_iso | C4_rand |
|---|---|---|---|---|---|
| full budget | −0.83 [−1.47, −0.20] | −0.13 [−0.53, +0.26] | +0.40 [−0.16, +0.96] | +0.37 [−0.17, +0.90] | +0.00 [−0.53, +0.53] |
| at 50 % of the budget | — | 0.00 | **+0.87** [+0.35, +1.38] | **+0.80** | **+0.70** |

What the loop indicates:
- C2 is a real baseline: a single pass is clearly worse.
- At the full budget, no condition with return visits is distinguishable from C2. Return
  visits confirm more targets **and** more clutter, because Belief confirms a contact on any two
  detected looks.
- Mid-mission, and when a mission is cut short, the adaptive conditions are clearly ahead (H1).
- **The aspect model shows no effect** (C4 − C4_iso = +0.03 [−0.31, +0.37]), also with the true
  class known, and also when Belief confirms only on looks from different directions (H2). In
  this loop that is a property of the placeholder models, not a finding about the sea.

Full tables and what cannot be claimed: `docs/DEVLOOP.md` §5, `docs/REPORT.md` §C.

## 6. Running it

```bash
cd blueboat_replanner && python3 -B -m test.smoke_test        # the offline gate (no ROS, ~90 s)
.claude/tools/devloop_batches.sh                              # dev loop on Belief's core (~45 min, 4 workers)
.claude/tools/ros_check.sh                                    # scratch build + both nodes on an isolated ROS domain (~28 min)
cd ~/ros2_ws && colcon build --packages-select blueboat_replanner   # needs blueboat_interfaces, blueboat_belief_interfaces
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission.yaml>   # in Gazebo add: use_sim_time:=true time_source:=ros
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```

- **Neighbours.** The gate's sections [20] and [22] need the BlueBoat-Belief checkout beside this
  repository. The cross-checks with MCS's geodesy and Control's `PathUpdater` need those
  checkouts too; each skips visibly when its neighbour is missing.
- **Before any long or ROS run,** check that no simulation is being recorded:
  `pgrep -af 'record_ru[n]\.py'`.
- **Gazebo missions:** `docs/LOCAL_CHECKLIST.md` L-G.

## 7. Repository and documents

```
.claude/        CLAUDE.md (rules for working here) · TODO.md · rules/ · tools/ · settings.json (Stop hook = the gate)
docs/           see the table below
results/        devloop_real_2026-11-12/ (dev-loop outputs on Belief's core)
blueboat_replanner/   the ament_python package
  blueboat_replanner/core/   ROS-free, deterministic planning (no clock, no randomness)
                     io/     mission file, trajectory export, run records
                     ros/    adapters (field names), executor (Control's path update), replanner_node (rclpy)
                     eval/   dev loop: Belief binding, world, emulator, Control stub, runner, emulator node
  config/ · launch/ · test/smoke_test.py (the gate's runner) + test/gate/ (one file per section)
  · test/test_gate.py (the gate for colcon test) · test/ros_graph_check.py (ROS check, by hand)
```

| Document | For |
|---|---|
| `docs/HANDOVER.md` | **where the work stands, the owner's decisions, what comes next** |
| `docs/REPORT.md` | what was built and how much weight each claim carries |
| `docs/DESIGN.md`, `docs/MATH.md` | the design and the mathematics |
| `docs/topics.md` | every topic, service, schema and file |
| `docs/CONTRACT_CONSUMED.md`, `docs/REAL_BELIEF_BINDING.md` | how Belief's grid is read, and how its core is driven offline |
| `docs/DEVLOOP.md`, `docs/C2_TUNING_PROTOCOL.md`, `docs/C2_TUNING_RESULT.md` | the dev loop's method and results; the baseline's tuning and freeze |
| `docs/LOCAL_CHECKLIST.md` | every ROS, Gazebo and boat check, with its command and expected result |
| `docs/EVIDENCE_REGISTER.md`, `docs/REQUESTS.md` | evidence ids; requests to other modules and to the owner |
| `docs/REPLANNER_BRIEF.md` | the original specification (read-only) |
