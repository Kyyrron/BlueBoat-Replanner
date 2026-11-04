# Local checklist — input of the in-machine session (`REPLAN-2-LOCAL`)

Everything here needs ROS 2, Gazebo, the real Belief core, recordings or the boat, and was
**[NOT RUN — needs in-machine session]** in the cloud. Each item says what to run and what result
settles it. Final list of the cloud session (Phases 0–5, 2026-11-04).

**Suggested order:** L-0 → L-R1/L-R2 (build, offline gate) → L-C1…L-C4 (Control) → L-B1…L-B4
(Belief binding) → L-R3…L-R6 (node graph, QoS, adapter) → L-E (emulator) → L-K3/L-K4 (speed,
turns) → L-G1…L-G4 (Gazebo missions) → L-B5/L-B6/L-B7 (latency, model, dev loop on the real core)
→ the rest.

**What the cloud already checked offline, so a failure here points at ROS/DDS or a neighbour:**
- the gate (`test/smoke_test.py`, 187 passed / 0 failed / 1 skipped);
- the executor against a Control stub cross-checked with Control's own `PathUpdater` ([21], [23]);
- `replanner_node.py` and `eval/emulator_node.py` driven through **stand-in** rclpy, message and
  service modules ([24]: graph, `~/start`, a C3 run with 2 accepted requests, `/replanner/plan`,
  records; emulator draws and fill convention). The stand-ins are hand-written: they prove the
  wiring of the two files, not DDS, QoS matching, executors or the real message classes.

## L-0 Revisions

- [ ] L-0.1 Record the revision of every module in `~/ros2_ws/src/BlueBoat-SideScanSonar/`.
  `git -C <module> log -1 --format=%H` for each module, plus BlueBoat-Belief. Diff them against
  the cloud revisions in `PHASE0_VERIFICATION.md` §0. If Control differs from `1fadbb2`, re-verify
  `PHASE0_VERIFICATION.md` §1 against `docs/PATH_UPDATE_INTERFACE.md` and
  `_custom_libraries/path_update.py`.

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
  Both must equal `PHASE0_VERIFICATION.md` §1.1.
- [ ] L-C4 Check progress and event QoS: `ros2 topic info -v /blueboat/path_progress` (reliable,
  volatile, depth 10) and `/blueboat/path_update_event` (reliable, depth 50).

## L-B Belief (contract `belief_grid/0`)

- [ ] L-B1 Find Belief's location, build and node names. Check the topic name
  (`/belief_map/belief_grid` provisional) and QoS with `ros2 topic info -v`. The publisher must
  offer TRANSIENT_LOCAL for the consumer of contract C-2.
- [ ] L-B2 Find the topic and type Belief subscribes for evidence: `/sss_ai/image_records` (root
  CLAUDE.md) or `/sss_ai/detections` (contract C-1). See Q-9.
- [ ] L-B3 Find Belief's datum: the mission `geo_anchor`? Check the `odom_to_map_*` form
  (translation only, or with rotation) and its value in simulation and on the real boat.
- [ ] L-B4 Diff Belief's `docs/CONTRACT_PROVIDED.md` against this module's
  `docs/CONTRACT_CONSUMED.md`, field by field (C-9).

- [ ] L-B5 (closure C) **Measure Belief's latency.** In a C3/C4 development run, measure the time
  from a contact passing abeam (`/blueboat/odom` pose, the contact's known position) to the first
  `/belief_map/belief_grid` snapshot whose contact `updated` stamp or sector sets reflect that look.
  It sets `t_lat` (10 s placeholder). Expect a few seconds: GCS images every 128 rows ≈ 6.4 s at
  20 Hz, plus Perception, plus Belief at ~1 Hz.
- [ ] L-B6 (closure A, F) **Belief's detection model.** Read Belief's PD(range) window and plateau,
  its class dependence and its ρ, from its code and `model_params_json`. Compare them with the
  planner's PD_r (0.7 on [3, 25] m). Decide which reading of `coverage` applies (MATH §9.1: a
  class-mixture residual, chosen, vs a unit-detectability residual) and record it in
  `CONTRACT_DEVIATIONS.md`.
  Also read **how Belief segments passes** (per line, per heading change, per time gap). In the
  reference belief every plan segment is a pass, so any revisit can confirm a contact. That
  drives the dev-loop result that C4_rand ≈ C4 (`DEVLOOP.md` §5.2 item 4). If Belief's
  definition differs, re-run the dev loop on the real core (L-B7) before reading anything into
  the C4 ablations.

## L-K Kinematics (Q-6)

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

- [ ] L-C5 (closure E) **Information only; the design does not depend on it.** Send a first path whose
  first-sample yaw differs from the boat's by 90° and by 180°: a straight line, 0.5 m/s, PID/LoS/MPC.
  Record the time and the excursion until cross-track < 0.3 m. This checks Control's "the boat turns
  in place" statement (interface §5). The replanner itself always starts with the boat's yaw.
- [ ] L-C6 (closure D) **Stop semantics, live.**
  1. In a replanner run, call `/replanner/stop`. Expect the status sequence `stop_requested` →
     `stop_accepted` → `stopped_holding`, with Control's `end_reached` for the stop id.
  2. Repeat while a manual target is engaged. Expect the truncation refused with
     `operator_override`, status `stop_rejected` with `stop_in_effect: false`, the old path
     unchanged in progress, and a retry after release.
  3. Engage arrows during `running`. Expect status `disarmed`, and no request after release until
     `/replanner/start`.

## L-S Swath (Q-2)

- [ ] L-S1 Get the usable ground range r_near and r_far at 20 m range: from Perception image
  records (`sides.*.ground_range_m`) over a GCS session in a built world at depths 1–3 m, and from
  the 2026-10-27 field sessions.

## L-O Odometry (Q-5)

- [ ] L-O1 If odometry is subscribed: check `/blueboat/odom` stamps and arrival rate in
  simulation and in a real run bag (`ros2 bag info`, `ros2 topic hz`).

## L-R Replanner node (Phase 4 code, written in the cloud and never run)

- [ ] L-R1 Build. `blueboat_belief_interfaces` must exist (Belief's package) and so must
  `blueboat_interfaces`.
  ```bash
  cd ~/ros2_ws && colcon build --packages-select blueboat_interfaces blueboat_replanner
  ```
  Expect a clean build with 0 errors.
- [ ] L-R2 Run the offline gate in the workspace: `cd src/BlueBoat-SideScanSonar/BlueBoat-Replanner/blueboat_replanner && /usr/bin/python3 -m test.smoke_test`.
  Expect `SMOKE: 187 passed, 0 failed, 1 skipped` (cloud, 2026-11-04), the skip being the
  real-Belief check of [20] until L-B7. With a sourced ROS workspace [24] still uses its
  stand-in modules (it replaces them in `sys.modules` and restores them afterwards).
- [ ] L-R3 Start the node and inspect its graph.
  ```bash
  ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission.yaml>
  ```
  Then `ros2 node info /replanner`. Expect these subscriptions: `/belief_map/belief_grid`,
  `/blueboat/path_progress`, `/blueboat/path_update_event`, `/blueboat/odom`. Expect the
  publishers `/replanner/plan`, `/replanner/decision`, `/replanner/status`, the services
  `/replanner/start` and `/replanner/stop`, and the client `/blueboat/update_path`.
- [ ] L-R4 Check QoS: `ros2 topic info -v /belief_map/belief_grid` must show RELIABLE and
  TRANSIENT_LOCAL on both ends (contract C-2). If Belief offers VOLATILE, change `BELIEF_QOS`
  in `ros/adapters.py` and record the change in `CONTRACT_DEVIATIONS.md`.
- [ ] L-R5 Check the adapter on the real message.
  ```bash
  ros2 topic echo --once /belief_map/belief_grid --qos-durability transient_local
  ```
  Feed it through `grid_to_dict` → `validate_grid`. Any `ContractError` names the field to
  reconcile, and the fix is one line in `GRID_FIELDS` / `CONTACT_FIELDS`.
- [ ] L-R6 Inspect status and decisions: `ros2 topic echo /replanner/status` (state sequence per
  DESIGN §7), and check that `/replanner/decision` JSON validates against topics.md §5.1.

## L-E Detection emulator node (eval/ only)

- [ ] L-E1 Settle the evidence topic first (Q-9, L-B2). If Belief reads `/sss_ai/detections`,
  run:
  ```bash
  ros2 launch blueboat_replanner emulator_launch.py world_dir:=<world> seed:=1 calib_a:=<a> calib_b:=<b>
  ```
  Take a and b from `results/devloop_2026-11-04/main/manifest.json` → `calibration`
  (dev loop: a = 2.136, b = 0.478 — fitted on the dev-loop world, so refit it on Gazebo C2 runs of
  the calibration worlds before any reported result). If Belief reads
  `/sss_ai/image_records`, the `image_records` output must be written first (it raises today).
- [ ] L-E2 Run the emulator INSTEAD of Perception: `ros2 node list` must not show
  `perception_node`. One publisher on `/sss_ai/detections`.
- [ ] L-E3 Check pass segmentation: the emulator splits passes on > 30° heading changes. Check
  that each object gets at most one draw per pass (`ros2 topic echo /sss_ai/detections`).

## L-B7 Real Belief binding for the dev loop

- [ ] Write `eval/belief_real.py` implementing `BeliefPort` (`feed_swath`, `feed_detections`,
  `end_passes`, `snapshot` → `to_grid_dict`) over Belief's ingestion API. Then:
  - re-run gate [20] against the real core (> 97 % agreement both ways);
  - re-run `eval/runner.py` with the real backend, and re-state every dev-loop number.

## L-G Gazebo missions (Phase 4; Q-12)

Until R-3 lands, use the development composition (Q-12 b). The sonar renders the world folder
from the pose, while Gazebo flies the boat in the stock ocean world (no walls or limits).
Use separate terminals, each sourced with `source ~/ros2_ws/env.sh`:

```bash
ros2 launch blueboat_control Sim_launch.py controller_type:=PID path_source:=replanner
```
```bash
ros2 launch blueboat_sss_sim sss_sim_launch.py world_dir:=$HOME/worlds/<path>/<world>
```
```bash
ros2 launch blueboat_sss SSS_processing_launch.py
```
```bash
<Belief's launch — its name is unknown in the cloud (L-B1)>
```
```bash
ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> seed:=1 calib_a:=<a> calib_b:=<b>
```
```bash
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs mission_c2.yaml>
```
```bash
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```

Mission file:
- `areas` = the world's `metadata.yaml` `limits.corners_gps` (SW, SE, NE, NW);
- `planning_depth_m` = the world depth;
- the C2 settings from `config/c2_tuned.yaml`;
- `condition: C2`, then repeat with `condition: C4`.

- [ ] L-G1 C2 mission. Expect `/replanner/status`: `waiting` → `starting` → `running` → `done`.
  Expect exactly one accepted request, progress `following` then `end_reached`, a run folder under
  `<root>/data/Replanner_data/`, and Control's pathlog CSV with 1 accepted row.
- [ ] L-G2 C4 mission. Expect at least one `request_accepted` **during the run**, no refusal,
  `missed_lead` 0 or rare. The decision JSON must carry candidates with J/J2. The boat must
  actually fly the revisit leg: `/replanner/plan` must change, and `/blueboat/odom` must follow it.
- [ ] L-G3 Stop test. During L-G2, call `ros2 service call /replanner/stop std_srvs/srv/Trigger {}`.
  Expect `stop_requested` → `stop_accepted` → `stopped_holding`. Repeat with a manual target
  engaged: expect `stop_rejected` with `stop_in_effect: false` (L-C6).
- [ ] L-G4 Record the bag, time per decision, and τ vs wall time (0.5 m/s achieved, L-K3).
  Report everything as [DEMO] with the run id.
- [ ] L-G5 When R-3 lands, repeat L-G1/L-G2 with `full_mission_launch.py` in a built world.
