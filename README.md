# BlueBoat-Replanner

The **aspect-aware replanner** of the BlueBoat side-scan sonar survey (thesis deliverable D6).
While a survey runs, it decides at the end of each survey line whether the boat should carry on,
or turn back to look at a possible target again **from a chosen direction**. It then makes the
boat fly that change within the same run, through Control's live path update. It also plans
every survey type the thesis compares, so all of them share one coverage generator, one turn
model and one executor.

| | State (2026-11-13) |
|---|---|
| Built | ROS-free planning core, executor, ROS node, detection-emulator node, launch files |
| Offline gate | `test/smoke_test.py`: **278 passed, 0 failed, 0 skipped** (no ROS, about 90 s); `colcon test` runs it |
| Development loop | runs on **BlueBoat-Belief's own core**, its missions executed by the node's own executor: C2 tuned and frozen, 690 missions plus a labelled extra comparison of 150 (`docs/DEVLOOP.md` §5). Preliminary, never a thesis result |
| Under ROS | ran on an isolated ROS graph beside Belief's installed node, with made-up sonar, navigation and Control (`docs/LOCAL_CHECKLIST.md` L-X; on the code of 2026-11-12). Built into `~/ros2_ws/install` (2026-11-13) |
| Not yet | run with Control's real node, in Gazebo, with a sonar image or a trained detector, or on the boat |

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

## 2. Scientific background

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
| `/blueboat/update_path`, progress, events; a `path_source:=replanner` run | BlueBoat-Control | exists; its ROS-free core is cross-checked by the gate; not yet run against Control's node |
| `BeliefGrid`, and Belief's Python core for the dev loop | BlueBoat-Belief | exists; read by meaning (`docs/CONTRACT_CONSUMED.md`), core bound offline (`docs/REAL_BELIEF_BINDING.md`) |
| Replanner runs in built simulation worlds | BlueBoat-SSS-Sim (`path_source` in `full_mission_launch.py`, `docs/REQUESTS.md` RQ-1) | missing; a development composition stands in (`LOCAL_CHECKLIST.md` L-G) |
| A "Replanner mode" in the mission station | BlueBoat-MCS (Control R-1) | missing; a real session uses *Extra args* `path_source:=replanner` |
| A trained, calibrated detector | BlueBoat-Perception | missing; the dev loop and Gazebo use the emulator of `eval/` instead |

## 4. How it works

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
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission.yaml>
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
