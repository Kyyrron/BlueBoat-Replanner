# Local checklist — the in-machine work

Each item says what to run and what result settles it. An item is ticked only when it was run
and its result is written here. Where the work stands and what comes next: `HANDOVER.md`.

**Done on 2026-11-12 (offline, no ROS):** L-B2, L-B3, L-B4, L-B5 (as a published quantity),
L-B6, L-B7 (binding, gate and the dev-loop runs), L-R2.
**Done on 2026-11-12, evening, under ROS on an isolated graph** (section "L-X" below: a scratch
build, ROS domain 47, Belief's installed node, made-up sonar, navigation and Control; no
Gazebo, no Control node, no boat): L-R1 (scratch build), L-R3, L-R4, L-R5, L-R6, L-B1, and the
emulator node on that graph (L-E1…L-E3 in part).
**Done on 2026-11-18 in Gazebo, a built world, with Control's node, Belief, Perception (dummy) and the
GCS:** L-G1 (one C2 development run, below).
**Not done:** the rest of what needs Control's node, Gazebo or the boat (L-C, L-K, L-G2…L-G4,
L-S, L-O, and the emulator in a built world).

**Suggested order:** L-0 → L-R1/L-R2 (build, offline gate) → L-C1…L-C4 (Control) → L-B1…L-B4
(Belief binding) → L-R3…L-R6 (node graph, QoS, adapter) → L-E (emulator) → L-K3/L-K4 (speed,
turns) → L-G1…L-G4 (Gazebo missions) → L-B5/L-B6/L-B7 (latency, model, dev loop on the real core)
→ the rest.

**What the offline gate already covers, so a failure here points at ROS/DDS or a neighbour:**
- the gate (`test/smoke_test.py`, **294 passed / 0 failed / 0 skipped** on 2026-11-18, with
  `/usr/bin/python3` (numpy 1.26.4) and `~/ros2_ws/.venv/bin/python3` (numpy 2.4.4); about 90 s);
- the node's clock ([26]): `time_source: wall` byte-identical to the node before the parameter,
  `ros` feeding the executor simulated seconds through a 16 s render hold at 0.1× real time;
- the executor against a Control stub cross-checked with Control's own `PathUpdater` ([21], [23]);
- the reading of Belief's grid against BlueBoat-Belief's own core ([20], [22]);
- `replanner_node.py` and `eval/emulator_node.py` driven through **fake** rclpy, message and
  service modules ([24]: graph, `~/start`, a C3 run with 2 accepted requests, `/replanner/plan`,
  records; emulator looks, one array per image, fill convention). The fake modules are
  hand-written: they prove the wiring of the two files, not DDS, QoS matching, executors or the
  real message classes.

## L-X The first run under ROS: an isolated graph, no simulator (2026-11-12, evening)

- [x] **Run:** `.claude/tools/ros_check.sh` (a scratch build of this package, then
  `blueboat_replanner/test/ros_graph_check.py`). ROS domain 47, Fast DDS on UDPv4 only, this
  machine only; the check refuses domain 0 and a graph that is not empty. Runs 1 to 4 below
  were made with the same two steps typed by hand in this session's scratch folder; run 5 is
  the script itself.
- **Real:** Belief's installed node (`~/ros2_ws/install`, built at 15:34, core sha256
  `fcd9531597e593b3…`); this package's `replanner_node` and `replanner_emulator_node`, from
  their executables and from their launch files; DDS; every message and service type
  (`BeliefGrid`, `UpdatePath`, `ProcessedSSSPing`, `Detection2DArray`, …).
- **Made up by the check:** the sonar pings (10 Hz, a thinned swath, no image content),
  odometry, GPS fixes, a world file with 8 objects, and Control: `/blueboat/path_progress` and a
  `/blueboat/update_path` server of the real type, answered by the dev loop's Control test
  double. Every clock is the wall clock (`use_sim_time` false).
- **So it is not** Control, not a simulation, not a sonar, not a detector.
- **Result of the last run (21:27–21:52): 42 checks, 0 failed.** The machine: 16 cores, three
  single-core jobs of another session running, load average about 5.

| Group | Checks | What was seen |
|---|---|---|
| [R1] nodes, endpoints, QoS | 11 | `belief_map` and `replanner` up. `/belief_map/belief_grid`: RELIABLE + TRANSIENT_LOCAL on both ends. The node's three publishers with their types, its three other subscriptions, its two `Trigger` services. The status is latched: a late subscriber gets it at once (`idle`) |
| [R2] alignment | 4 | The first grids come from an unaligned Belief: received, not used, and the status says why (`grid_note`: "odom_to_map_valid is false…"). After 10 GPS / odometry pairs the grid is used. Belief's transform is the made-up offset (5.0, −3.0) m, not the identity. `grid_seq` advances by exactly 1 |
| [R3] a C3 mission, path clock × 10 | 11 | `~/start` accepted. The first path goes through the real `UpdatePath` type (2810 samples, frame `world`) and is accepted. The mission runs to `done`. One decision: a REPLACE with two contacts, sent and accepted, no refusal. The evidence horizon is read from the grid stamp (`source: stamp`; 13 path-s behind the boat). Labels `score-conditional` and `belief-ungated` (in this bring-up Belief ran with `coverage.require_detector: false`, since nothing detects). The decision time is in the record (0.009 s). `/replanner/plan` once per accepted path. The run folder is complete (`manifest.json`, `mission.yaml`, `belief.json`, `plans/` × 2, the four logs, `end.json`). States `starting` → `running` → `requesting` → `running` → `done`. `~/stop` after `done` is refused |
| [R4] Belief restarted | 3 | `grid_seq` 143 → 1. The node counts the restart, writes `belief_restart` to `events.jsonl` and uses the new grid |
| [R5] shutdown | 1 | SIGINT: exit code 0, no traceback |
| [R6] second bring-up | 2 | Both nodes of this package started **by their launch files**; Belief with its **default** coverage rule. The emulator is the one publisher of `/sss_ai/detections`; it reads the pings BEST_EFFORT and is matched |
| [R7] a C4 mission at real speed (23 min) | 10 | Runs to `done` (τ 1330 of B = 1399 s; 643 s of return visits). **10 decisions: 5 REPLACE, all sent and accepted; 5 CONTINUE. 6 requests, 6 accepted, 0 refused, 0 missed leads.** Emulator: 310 arrays (290 empty) for 665 m sailed = 311.6 image strides; 20 detections, class "unclassified", unique ids. Belief: `negative_evidence: gated`, `coverage_source: arrays`, every one of the 12 600 cells searched from at least one sector; 7 contacts (4 confirmed), each within 3 m of a made-up object, so the emulator's world frame and Belief's map frame agree through the transform. Decisions: labels `model-conditional`, `planner-placeholders`, `score-conditional` (no `belief-ungated`); horizon from the stamp in all ten, 10.6 to 15.0 path-s behind the boat (one to two image strides plus the 6 s tracking margin); grid age 0.1 to 0.9 s. **Decision time: mean 0.46 s, slowest 1.74 s (the first decision, 7 candidates), none over the 2.0 s allowance; smallest lead left 2.05 path-s against the 2.0 needed.** Both launches end cleanly on SIGINT (exit code 0) |

- **The other runs of that evening** (run 4 is the result above):
  1. 20:57, the first bring-up only: 28 checks, 0 failed.
  2. about 21:07, with the second bring-up and the C4 mission on a path clock ten times faster
     than real time: 40 of 41 passed. **3 of 6 requests were refused by the made-up Control
     (`lead_too_short`)**, the three whose decisions took 0.7 to 1.8 s. Two causes: the speed-up
     (a second of computing cost ten path seconds), and a defect it exposed: the node did not
     count a decision's own compute time before sending (DESIGN §5.2 rule 2; corrected).
  3. real speed, on the node as it was: stopped by this session after 5 minutes, to run again
     on the corrected node (run 4, the result above).
  5. 21:52, after run 4: `.claude/tools/ros_check.sh` itself, with `--c4-speedup 10` to keep it
     short: the script builds and runs (42 checks, 0 failed; at that speed refusals are not
     judged). **It shows the corrected rule acting under ROS: the node itself dropped 2 late
     decisions (`missed_lead`, lead left down to 0.56 path-s) and sent none of them.** 2 other
     requests were still refused, because a path clock ten times faster than real time breaks
     the bound the rule relies on (τ advances by at most 1 path-s per second).
- **Not covered:** Control's node and its real timing; Gazebo; simulated time; a real sonar
  image; Perception; image records; a node that joins long after Belief; the decision time on a
  machine loaded by a simulator; `class_info:=true`; a built world.

## L-0 Revisions

- [ ] L-0.1 Before a run with a neighbour, record the revision of every module in
  `~/ros2_ws/src/BlueBoat-SideScanSonar/` (`git -C <module> log -1 --format=%H`, plus
  `git -C <module> status --short`; for Belief also the core hash of
  `eval/belief_real.belief_info()`). The Control facts this module relies on
  (`EVIDENCE_REGISTER.md` E-01…E-26) were read at Control `1fadbb2`: if Control's
  `_custom_libraries/path_update.py` or `docs/PATH_UPDATE_INTERFACE.md` changed since, re-read
  those rows against them. The gate's cross-check with `PathUpdater` ([8], [21]) runs on the
  current tree.

## L-C Control path update (brief §7.1, re-verified on the machine)

- [ ] L-C1 Build, then start a replanner-owned simulation run.
  ```bash
  cd ~/ros2_ws && colcon build --packages-select blueboat_interfaces blueboat_control
  ```
  ```bash
  ros2 launch blueboat_control Sim_launch.py controller_type:=PID path_source:=replanner
  ```
  Expect: progress `state: awaiting_path` with `hold`
  (`ros2 topic echo /blueboat/path_progress --full-length`).
- [ ] L-C2 Run Control's own worked client, then confirm 13/13.
  ```bash
  python3 test/path_update_demo.py --out /tmp/demo.json
  ```
  Then run `test/gazebo_path_update.py --replanner --out-dir /tmp/gz` and confirm 13/13.
- [ ] L-C3 Inspect the service from the CLI.
  ```bash
  ros2 interface show blueboat_interfaces/srv/UpdatePath
  ```
  ```bash
  ros2 service type /blueboat/update_path
  ```
  Both must match Control's `docs/PATH_UPDATE_INTERFACE.md` §2–§4 (fields of E-02).
- [ ] L-C4 Check progress and event QoS: `ros2 topic info -v /blueboat/path_progress` (reliable,
  volatile, depth 10) and `/blueboat/path_update_event` (reliable, depth 50).

## L-B Belief (contract `belief_grid/0`)

- [x] L-B1 **Live topic and QoS.** Seen on 2026-11-12 on the isolated graph, with both nodes
  up, through the graph API (`get_publishers_info_by_topic`, `get_subscriptions_info_by_topic`):
  `belief_map` publishes and `replanner` subscribes `/belief_map/belief_grid`, RELIABLE and
  TRANSIENT_LOCAL on both ends; the grids arrive and `grid_seq` advances by exactly 1 (L-X [R1],
  [R2]).
- [x] L-B2 **Belief's evidence topics.** `/sss_ai/detections` (required) and
  `/sss_ai/image_records` (optional, exact detector coverage). Source: Belief `topics.md` §1.
  The emulator publishes detections only, one array per image.
- [x] L-B3 **Datum and transform.** Datum = the survey area's `geo_anchor` (a world
  `metadata.yaml` or a `blueboat_survey_area/1` file); `odom_to_map` is a translation, yaw 0 by
  construction; `odom_to_map_valid` is false until Belief's GPS / odom fit has 10 pairs. Source:
  Belief `topics.md` §2. The Replanner refuses an unaligned grid (gate [20], on a real grid).
- [x] L-B4 **Field-by-field diff.** Done: `CONTRACT_CONSUMED.md` (rewritten 2026-11-12). No
  field of `belief_grid/0` is missing or renamed; nine fields are now read by their meaning.
- [x] L-B5 **Belief's latency.** It is not a constant to measure: Belief publishes it. The
  grid's header stamp is the evidence horizon (Belief `topics.md` §2). Measured through Belief's
  core in its live-like mode: 6.5 s behind the boat on a straight pass at 0.5 m/s, between one
  and two image strides (gate [20]). The planner uses the stamp (MATH §11.3); `t_lat` = 10 s is
  only the fallback. **Open under ROS:** that Control's `ros_time` and the sonar stamps are one
  clock in simulation (they should both be sim time), seen in a live run.
- [x] L-B6 **Belief's detection model and looks.** Read in Belief's code and `MATH.md`:
  - PD: a trapezoid, plateau 0.7 on [3, 25] m from the transducer with 3 m ramps; no class
    dependence. The planner keeps its box and reads misses at their recorded strength (MATH §6, §7);
  - ρ: Paull's pairwise rule on the folded line angle (`paull_pairwise`, `fold_opposite`);
  - a look: a run of pings from one directed sector with gaps ≤ 20 s. There is no "pass";
  - confirmation: two detected looks, not necessarily from different sectors;
  - in a turn the inner side beyond r* = v/|ω| counts for nothing.
  The reading of `coverage` as a class-mixture residual (MATH §9.1) is kept, with `n_i` now the
  number of folded sectors a detector searched.

## L-K Kinematics

- [ ] L-K1 Measure turn time and settle time in Gazebo. Use a replanner run with arcs of radius
  1.5 / 3 / 6 m at 0.3 / 0.4 m/s, for PID, LoS and MPC. Measure the cross-track error after the
  arc and the time until |xte| < 0.3 m. Record both in `docs/` with the run ids.
- [ ] L-K2 Measure the same from the field poslogs in `~/ros2_ws/data/Robot_data/`, in the
  lawnmower and kin_square turns: yaw rate, turn radius, settle time.
- [ ] L-K3 Check the achievable speed in Gazebo at **0.5 m/s** (decided 2026-11-04; Control
  advises ≤ 0.45 m/s in simulation, E-23). On straight legs the τ rate must equal the authored
  speed (the governor does not throttle), for PID, LoS and MPC. If it lags, tell the user before
  E4: planned and executed times would then diverge.
- [ ] L-K4 Turn feasibility of the generated plans: Dubins arcs of r_turn = 3 m at 0.5 m/s
  (yaw rate 9.5 °/s). Measure the cross-track error at the junctions and inside the turns.

- [ ] L-C5 (MATH §3.5) **Information only; the design does not depend on it.** Send a first path whose
  first-sample yaw differs from the boat's by 90° and by 180°: a straight line, 0.5 m/s, PID/LoS/MPC.
  Record the time and the excursion until cross-track < 0.3 m. This checks Control's "the boat turns
  in place" statement (interface §5). The replanner itself always starts with the boat's yaw.
- [ ] L-C6 (DESIGN §5.3) **Stop semantics, live.**
  1. In a replanner run, call `/replanner/stop`. Expect the status sequence `stop_requested` →
     `stop_accepted` → `stopped_holding`, with Control's `end_reached` for the stop id.
  2. Repeat while a manual target is engaged. Expect the truncation refused with
     `operator_override`, status `stop_rejected` with `stop_in_effect: false`, the old path
     unchanged in progress, and a retry after release.
  3. Engage arrows during `running`. Expect status `disarmed`, and no request after release until
     `/replanner/start`.

## L-S Swath

- [ ] L-S1 Get the usable ground range r_near and r_far at 20 m range: from Perception image
  records (`sides.*.ground_range_m`) over a GCS session in a built world at depths 1–3 m, and from
  the 2026-10-27 field sessions.

## L-O Odometry

- [ ] L-O1 If odometry is subscribed: check `/blueboat/odom` stamps and arrival rate in
  simulation and in a real run bag (`ros2 bag info`, `ros2 topic hz`).

## L-R Replanner node (first run under ROS on 2026-11-12, section L-X)

- [x] L-R1 Build. **Done into `~/ros2_ws/install` on 2026-11-13**:
  ```bash
  cd ~/ros2_ws && env -i HOME=$HOME PATH=/usr/bin:/bin bash -c 'source /opt/ros/jazzy/setup.bash \
    && source install/setup.bash && colcon build --packages-select blueboat_replanner'
  ```
  (a shell without the venv; plain install, like the rest of the workspace). `1 package
  finished` in 1.5 s. Installed: `lib/blueboat_replanner/{replanner_node,
  replanner_emulator_node, replanner-devloop}`, `share/blueboat_replanner/config/{replanner,
  c2_tuned, mission_example}.yaml` and both launch files; the node's default `c2_tuned_file`
  resolves to the installed `c2_tuned.yaml`. `colcon test --packages-select blueboat_replanner`
  runs the gate (`test/test_gate.py`, pytest): 1 test, 0 failures, 74 s. No node was started.
  Earlier, on 2026-11-12, into a scratch install base (`.claude/tools/ros_check.sh`, which still
  builds its own):
  `colcon build --base-paths <this repo> --packages-select blueboat_replanner --build-base …
  --install-base …`, with `blueboat_interfaces`, `blueboat_belief_interfaces` and
  `blueboat_belief` read from `~/ros2_ws/install`. Result: `1 package finished` in 1 to 3 s, no
  error (the only output on the error stream is "byte-compiling is disabled", which the script
  asks for). Both executables and both launch files are installed.
- [x] L-R2 The offline gate on this machine, 2026-11-13:
  `cd blueboat_replanner && /usr/bin/python3 -B -m test.smoke_test` → **`SMOKE: 278 passed,
  0 failed, 0 skipped`**; the same with `~/ros2_ws/.venv/bin/python3`. Sections [20] and [22]
  need the BlueBoat-Belief checkout beside this repository (they skip visibly without it).
  **Also run from a shell with the ROS workspace sourced** (2026-11-13, the package installed
  in `~/ros2_ws/install`): 278 passed.
  [24] replaces the ROS modules in `sys.modules` by its fake ones and restores them afterwards.
  The first run there failed one check of [20]: the binding had loaded Belief's installed copy
  and not its checkout (corrected: the checkout comes first, `REAL_BELIEF_BINDING.md`).
- [x] L-R3 Start the node and inspect its graph.
  ```bash
  ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission.yaml>
  ```
  Expected, and **seen on 2026-11-12** (L-X [R1]; started from its executable in the first
  bring-up and from its launch file in the second): subscriptions `/belief_map/belief_grid`,
  `/blueboat/path_progress`, `/blueboat/path_update_event`, `/blueboat/odom`; publishers
  `/replanner/plan`, `/replanner/decision`, `/replanner/status` with their types; services
  `/replanner/start` and `/replanner/stop` (`std_srvs/srv/Trigger`); the client
  `/blueboat/update_path`, used with the real service type.
- [x] L-R4 QoS: RELIABLE and TRANSIENT_LOCAL on both ends of `/belief_map/belief_grid`
  (contract C-2), seen on the live graph (L-B1). The node's profile needed no change.
- [x] L-R5 The adapter on the real message: the node passes every live `BeliefGrid` through
  `grid_to_dict` → `validate_grid`. No `ContractError`; the decision records carry
  `belief_fallbacks: []`; a grid of an unaligned Belief is received and refused with its reason
  in the status (L-X [R2]). No line of `GRID_FIELDS` / `CONTACT_FIELDS` had to change.
- [x] L-R6 Status and decisions: the state sequence of a C3 run is `starting` → `running` →
  `requesting` → `running` → `done` (`status.jsonl`), the status is latched, and every
  `/replanner/decision` carries the required keys of topics.md §5.1 (L-X [R3], [R7]).

## L-E Detection emulator node (eval/ only)

**On the isolated graph (2026-11-12, L-X [R6]–[R7]):** the node ran from its launch file on a
made-up world file and made-up pings: one array per seabed image, empty ones included (310 for
311.6 strides); Belief's coverage source `arrays`, its negative evidence gated; one publisher on
the topic; Belief's contacts within 3 m of the objects. **Still to do in a built world, in
Gazebo:** everything below, with the calibrator of that world and the look-by-look comparison.

- [ ] L-E1 Run the emulator (Belief reads `/sss_ai/detections`; settled, L-B2):
  ```bash
  ros2 launch blueboat_replanner emulator_launch.py world_dir:=<world> seed:=1 calib_a:=<a> calib_b:=<b>
  ```
  Take a and b from the `calibration` block of the dev-loop manifest
  (`results/devloop_real_<date>/main/manifest.json`). They are fitted on the dev-loop world: refit
  them on Gazebo C2 runs of the calibration worlds before any reported result.
  Expect on `/sss_ai/detections`: **one array per seabed image, empty ones included** (about one
  every 4.3 s at 0.5 m/s), and Belief's `/diagnostics` `coverage.source` = `arrays` with
  `stats.pings_covered` growing. With `class_info:=true` the class is the object's type in the
  world file: Belief keeps it as `other` unless its `detector.class_names` lists it.
- [ ] L-E2 Run the emulator INSTEAD of Perception: `ros2 node list` must not show
  `perception_node`. One publisher on `/sss_ai/detections`.
- [ ] L-E3 Check the draws: the emulator draws once per look at an object, a look being a run
  of pings from one aspect sector with gaps ≤ 20 s, as in Belief (`eval/emulator.py`). Compare
  the looks it counts with Belief's `aspect_counts` at the objects' cells (the dev loop does this
  after every mission and finds no difference).

## L-B7 Binding to BlueBoat-Belief's core for the dev loop

- [x] `eval/belief_real.py` written on Belief's `OFFLINE_CORE_API.md` (2026-11-12);
  `REAL_BELIEF_BINDING.md` says how.
- [x] Gate [20] against Belief's core: straight line 100 % both ways, a path with turns 99.9 %
  (92.9 % without the turn gate); grid accepted with no fallback; unaligned grid refused; contact
  fields; no detector → nothing searched; evidence horizon; determinism.
- [x] **Dev loop on Belief's core, 2026-11-12, 19:27–20:12:**
  `.claude/tools/devloop_batches.sh devloop_real_2026-11-12 4` → exit 0. C2 frozen (f = 1.0, long
  axis, interlace auto, B = 2111 s; `config/c2_tuned.yaml` sha256 `f0bba92ffd4168bb…`), then 690
  missions: none failed, 0 refusals, 0 missed leads, 0 differences between the emulator's looks
  and Belief's. Results: `DEVLOOP.md` §5; outputs: `results/devloop_real_2026-11-12/`.

## L-G Gazebo missions

**The default since 2026-11-18 is a built world** (R-3 / RQ-1 done): the simulator's
`full_mission_launch.py` takes `path_source:=replanner`, so its `master_control` holds at the spawn
pose until the first accepted `/blueboat/update_path`, with the world's walls, limits and sea. Run
it through the superproject recorder, which also runs the GCS (its seabed images feed Perception)
and starts Perception like MCS does; with `path_source:=replanner` it ends the run on
`/replanner/status` (`tools/sim_record/README.md`, "Replanner-owned runs"). Use separate
terminals, each sourced with `source ~/ros2_ws/env.sh`:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar
python3 tools/sim_record/record_run.py --world <world folder> --trajectory <MCS path of the world> \
    --sensor harbour_fast --name <run> --rtf 2.0 --waves harbour_chop --controller MPC \
    --out ~/ros2_ws/sim_runs/pipeline_dev --private-data-root \
    --extra path_source:=replanner sonar_fouling:=<the world's metadata run.sonar_fouling>
```
The `--trajectory` is only what the launch requires (`trajectory_file`); nothing follows it. When the
recorder's log says "path deployed":
```bash
ros2 launch blueboat_belief belief_launch.py area_file:=<world folder> use_sim_time:=true \
    record_inputs:=true data_dir:=<run folder>
```
```bash
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission_c2.yaml> \
    use_sim_time:=true time_source:=ros data_dir:=<run folder>
```
Once Control's progress is `awaiting_path` and Belief's grid has `odom_to_map_valid`:
```bash
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```

**Clock.** In Gazebo pass `use_sim_time:=true time_source:=ros`: arrival times, freshness, the
answer and reconcile time-outs, the start time-out and every record time are then simulated
seconds, the node clock on `/clock` (the records carry `"time_source": "ros"`, the manifest a wall
`started_wall` beside `started_clock`). This is needed, not a refinement: the production ray-cast
runs slowed (`real_time_factor`) and Gazebo is paused while a slow ping renders (1–16 s near a
quay), so on the wall clock Control's 10 Hz progress (simulated time) looks stale below half real
speed and during every pause, and no path is sent (the gate's [26] shows it). The tick and status
timers are rclpy's, on the node clock in either mode: under `use_sim_time` there is no tick while
Gazebo is paused. The decision time stays wall (`time.perf_counter`); with the simulation at or
below real time, ageing `min_change_point` by it is conservative. On the boat keep the defaults
(`use_sim_time:=false time_source:=wall`); `time_source:=ros` without `use_sim_time` is the system
time and equals `wall`. The evidence horizon does not depend on the node's clock either way: it is
converted through Control's (`ros_time`, τ) pairs (MATH §11.3). Check in L-G1 that
`/replanner/decision` names `source: stamp` for the horizon.

**Fallback (development only, no walls or limits):** the sonar renders the world folder from the
pose while Gazebo flies the boat in the stock ocean world:
```bash
ros2 launch blueboat_control Sim_launch.py controller_type:=PID path_source:=replanner
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<path>/<world>
ros2 launch blueboat_sss SSS_processing_launch.py
ros2 launch blueboat_belief belief_launch.py area_file:=<area.yaml> use_sim_time:=true
ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> seed:=1 calib_a:=<a> calib_b:=<b>
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission_c2.yaml> use_sim_time:=true time_source:=ros
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```
(one terminal each). The area file can come from the mission file, so the two cannot differ:
`ros2 run blueboat_belief blueboat-belief-make-area --out area.yaml --from-mission <mission.yaml>`.
The Replanner refuses a mission whose polygon is not inside Belief's area.

Mission file:
- `areas`: one polygon inside the world's `metadata.yaml` `limits` (WGS-84, as `corners_gps`), clear
  of quays, pontoons and structures, and where the altitude stays inside the validated envelope
  (4.5–7.0 m, `SIM_VALIDITY_CONDITIONS.md` §2: depth ≤ 7.2 m with the 0.22 m transducer depth);
- `planning_depth_m` = the depth over the polygon;
- the C2 settings come from `config/c2_tuned.yaml` (the node applies them);
- `condition: C2`, then repeat with `condition: C4`.

- [x] L-G1 C2 mission. Expect `/replanner/status`: `waiting` → `starting` → `running` → `done`.
  Expect exactly one accepted request, progress `following` then `end_reached`, a run folder under
  `<root>/data/Replanner_data/`, and Control's pathlog CSV with 1 accepted row.
  **Seen on 2026-11-18** (a development run, never a result; `~/ros2_ws/sim_runs/pipeline_dev/
  pipe13_c2_nominal_s2/RUN_NOTES.md`): campaign world `e4_example-nominal-s2`, production ray-cast,
  MPC, `harbour_chop`, ×2 requested with Gazebo held around every ping. `starting` → `running` →
  `done` ("plan exhausted", τ 373.7 of B = 373.6 s); one request, accepted; Control's pathlog
  `request_accepted`, `activated`, `end_reached`; maximum input ages while running 0.10 s
  (progress), 1.05 s (grid), 0.44 s (odometry), simulated time, no stale reason; every record
  `time_source: ros`. C2 has no decision point, so the decision's `source: stamp` is not
  observable here: Belief's grid stamp ran 8.4–8.7 s behind Control's `ros_time`, both
  simulated, inside the stamp rule (to be seen in L-G2).
- [ ] L-G2 C4 mission. Expect at least one `request_accepted` **during the run**, no refusal,
  `missed_lead` 0 or rare. The decision JSON must carry candidates with J/J2. The boat must
  actually fly the revisit leg: `/replanner/plan` must change, and `/blueboat/odom` must follow it.
- [ ] L-G3 Stop test. During L-G2, call `ros2 service call /replanner/stop std_srvs/srv/Trigger {}`.
  Expect `stop_requested` → `stop_accepted` → `stopped_holding`. Repeat with a manual target
  engaged: expect `stop_rejected` with `stop_in_effect: false` (L-C6).
- [ ] L-G4 Record the bag, time per decision, and τ vs wall time (0.5 m/s achieved, L-K3).
  Report everything as [DEMO] with the run id.
- [x] L-G5 R-3 landed on 2026-11-18: L-G1…L-G4 run in a built world by default (above).
