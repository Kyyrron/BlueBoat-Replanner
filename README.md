# BlueBoat-Replanner

The aspect-aware replanner of the BlueBoat side-scan sonar survey. During a survey it decides,
at each decision point, whether the boat carries on or goes back to look at a possible target
again **from a chosen direction**. It makes the boat fly that change in the same run, through
Control's live path update. It also plans every survey type the project compares, so they all
share one coverage generator and one executor.

> **Caveats.** The Replanner has **never run on the boat**. In Gazebo it has run once (one C2
> development run). Belief's scores are not probabilities until a calibrated detector exists, so
> every decision built on them is score-conditional, and every result is model-conditional.

## Where it fits

```
sonar ──► Perception ──► Belief ──BeliefGrid──► Replanner ──/blueboat/update_path──► Control ──► boat
                                                   ▲   ▲                                  │
                                      mission file ┘   └── path progress, events, odom ───┘
```

- **Belief's `BeliefGrid` is the only target evidence.** The Replanner never reads detections,
  sonar data, simulation ground truth or `/sim/*`.
- **`/blueboat/update_path` is the only way it moves the boat.** It replaces the rest of the
  path from a change point; a refused request changes nothing.
- In simulation, Control's node must be started in replanner mode:
  `full_mission_launch.py path_source:=replanner` (BlueBoat-SSS-Sim). It then holds the boat at
  the spawn pose until the first accepted path.

## Survey conditions

| Condition | What it does |
|---|---|
| `C1` | single pass |
| `C2` | two passes at right angles: **the baseline**, tuned and frozen first (`config/c2_tuned.yaml`) |
| `C3` | C2, with one revisit of each contact inserted after pass 1, perpendicular to its first detection (a simple heuristic) |
| `C4` | informative: revisits chosen during the survey, with the look direction from an aspect model |
| `C4_iso`, `C4_rand`, `C4_depth1` | ablations of C4: no aspect model; a pre-drawn random bearing; one-step lookahead |

All conditions get the same time budget: the duration of the C2 plan.

## Prerequisites

- ROS 2 Jazzy, workspace `~/ros2_ws`.
- `blueboat_interfaces` (BlueBoat-Control) and `blueboat_belief_interfaces` (BlueBoat-Belief).
- At run time, Belief's `belief_map` node and Control's `master_control` (or the simulator).
- For the development loop only: `blueboat_belief` (its core is imported) and `vision_msgs`.
- Python: `numpy`, `PyYAML`.

## Build

From `~/ros2_ws`, in a shell that has not sourced the venv:

```bash
cd ~/ros2_ws
colcon build --packages-select blueboat_interfaces blueboat_belief_interfaces   # first
colcon build --packages-select blueboat_replanner
source install/setup.bash
```

## Run

```bash
# start the node (in Gazebo add: use_sim_time:=true time_source:=ros)
ros2 launch blueboat_replanner replanner_launch.py mission_file:=/abs/path/mission.yaml

# arm the mission: the node never starts by itself
ros2 service call /replanner/start std_srvs/srv/Trigger {}

# stop: truncate the path and hold
ros2 service call /replanner/stop std_srvs/srv/Trigger {}
```

Launch arguments (`replanner_launch.py`):

| Argument | Default | Meaning |
|---|---|---|
| `mission_file` | required | absolute path to the mission file |
| `config_file` | `""` (built-in defaults, as in `config/replanner.yaml`) | planner parameter overrides (`replanner:` block) |
| `c2_tuned_file` | `""` (installed `config/c2_tuned.yaml`) | the C2 freeze applied to every mission |
| `data_dir` | `""` | data root for run records |
| `belief_topic` | `/belief_map/belief_grid` | where the grid comes from |
| `tick_hz` | `5.0` | node tick rate |
| `use_sim_time` | `false` | `true` in Gazebo |
| `time_source` | `wall` | `ros` in Gazebo (with `use_sim_time:=true`) |

Safety behaviour:
- No autonomous start: only `/replanner/start` arms a mission.
- Operator override or E-STOP disarms the Replanner. It re-arms only through `/replanner/start`.
- `/replanner/stop` returning success means the stop was *requested*. Only
  `/replanner/status` reports whether it took effect (`stopped_holding`).

### Mission file

One WGS-84 polygon, a condition and the planning depth. It must lie inside Belief's survey area.
Example: `blueboat_replanner/config/mission_example.yaml`.

```yaml
mission:
  name: my_survey
  condition: C2            # C1 | C2 | C3 | C4 | C4_iso | C4_rand | C4_depth1
  areas:
    - [[lat, lon], [lat, lon], [lat, lon], [lat, lon]]
  config:
    planning_depth_m: 2.0  # required
```

Unknown keys are refused. Do not repeat the C2 freeze settings in a mission; a different value
refuses the mission.

### Detection emulator (simulation and tests only)

Runs **instead of** Perception, never beside it. It publishes emulated detections from the
world's true objects.

```bash
ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> seed:=1
```

## Topics

| Name | Kind | Type |
|---|---|---|
| `/belief_map/belief_grid` | sub | `blueboat_belief_interfaces/BeliefGrid` (latched) |
| `/blueboat/path_progress` | sub | `std_msgs/String` (JSON, from Control) |
| `/blueboat/path_update_event` | sub | `std_msgs/String` (JSON, from Control) |
| `/blueboat/odom` | sub | `nav_msgs/Odometry` |
| `/blueboat/update_path` | service client | `blueboat_interfaces/srv/UpdatePath` |
| `/replanner/plan` | pub | `nav_msgs/Path` (the path Control holds; display) |
| `/replanner/decision` | pub | `std_msgs/String` (JSON, one record per decision) |
| `/replanner/status` | pub | `std_msgs/String` (JSON, 1 Hz and on change) |
| `/replanner/start`, `/replanner/stop` | services | `std_srvs/Trigger` |

QoS, JSON schemas and files: `docs/topics.md`.

## Configuration

- `config/replanner.yaml`: every planner parameter with its unit and whether it is a user
  decision, a repository fact, from literature, or a **placeholder**. Unknown keys raise.
- A mission file's `config:` block overrides it for that mission.
- `config/c2_tuned.yaml`: the frozen C2 baseline (spacing, first-pass direction, interlace).
- Key defaults: speed 0.5 m/s, sonar range 20 m, turn radius 3 m. The aspect-model values in
  `classes:` are placeholders.
- `class_families` (detector class → aspect family) is empty on purpose: the owner decides it.

## Outputs on disk

Each run gets a write-once folder `<root>/data/Replanner_data/<date-time>-<mission>-<condition>/`
(manifest, mission copy, every plan, decisions, requests, status, events, end record). The root
is `data_dir`, else `$BLUEBOAT_DATA_DIR`, else the workspace (normally `~/ros2_ws`).

## Tests and development loop

```bash
cd BlueBoat-Replanner/blueboat_replanner && python3 -B -m test.smoke_test   # offline gate, no ROS (~90 s)
BlueBoat-Replanner/.claude/tools/devloop_batches.sh     # dev loop on Belief's core (~45 min, 4 workers)
BlueBoat-Replanner/.claude/tools/ros_check.sh           # scratch build + nodes on an isolated ROS domain (~28 min)
```

- `colcon test --packages-select blueboat_replanner` also runs the gate.
- Some gate sections need the BlueBoat-Belief, BlueBoat-MCS and BlueBoat-Control checkouts beside
  this repository. Each skips visibly when its neighbour is missing.
- The **development loop** (`eval/`) runs without ROS: Belief's own core, a seeded synthetic
  world, an emulated detector, a boat that tracks the plan perfectly. Its numbers are
  **preliminary** and never a thesis result. Method and results: `docs/DEVLOOP.md`.
- Before a long run, check that no simulation is being recorded: `pgrep -af 'record_ru[n]\.py'`.

## Status

- Built: planning core, executor, ROS node, emulator node, launch files. Offline gate green.
- Run on an isolated ROS graph beside Belief's node with made-up inputs.
- Run once in Gazebo: a C2 development run in a built world, first path accepted and flown to
  the end.
- **Not yet:** C4 or the stop test in Gazebo, a trained detector, the boat.

Where the work stands and what comes next: `docs/HANDOVER.md`. Open items: `.claude/TODO.md`.
Simulated results follow the superproject's `SIM_VALIDITY_CONDITIONS.md`.

## Layout

```
blueboat_replanner/                 ament_python package
  blueboat_replanner/core/          planning: no ROS, no clock, no randomness
  blueboat_replanner/io/            mission file, trajectory export, run records
  blueboat_replanner/ros/           adapters, executor (the only code that calls update_path), replanner_node
  blueboat_replanner/eval/          dev loop: Belief binding, world, emulator, Control stub, runner
  config/                           replanner.yaml, c2_tuned.yaml, mission_example.yaml
  launch/                           replanner_launch.py, emulator_launch.py
  test/                             smoke_test.py + gate/, test_gate.py, ros_graph_check.py
docs/                               see below
results/                            dev-loop outputs
```

## Documents (`docs/`)

| File | Content |
|---|---|
| `HANDOVER.md` | where the work stands, owner decisions, next steps |
| `REPORT.md` | what was built and how much weight each claim carries |
| `DESIGN.md` | design, and the scientific background (§13) |
| `MATH.md` | the mathematics |
| `topics.md` | every topic, service, JSON schema and file |
| `CONTRACT_CONSUMED.md`, `REAL_BELIEF_BINDING.md` | how Belief's grid is read; how Belief's core is driven offline |
| `DEVLOOP.md` | development loop method and results |
| `C2_TUNING_PROTOCOL.md`, `C2_TUNING_RESULT.md` | how the baseline was tuned and frozen |
| `LOCAL_CHECKLIST.md` | every ROS, Gazebo and boat check, with its command |
| `EVIDENCE_REGISTER.md`, `REQUESTS.md` | evidence ids; requests to other modules |
| `REPLANNER_BRIEF.md` | the original specification |
