# BlueBoat-Replanner — user guide

How to run the Replanner, in detail. For installation, see the [README](../README.md). Every command
assumes `cd ~/ros2_ws && source env.sh` was run first.

## 1. Run a survey

```bash
# start the node (in Gazebo add: use_sim_time:=true time_source:=ros)
ros2 launch blueboat_replanner replanner_launch.py mission_file:=/abs/path/mission.yaml

# arm the mission: the node never starts by itself
ros2 service call /replanner/start std_srvs/srv/Trigger {}

# stop: cut the path short and hold
ros2 service call /replanner/stop std_srvs/srv/Trigger {}
```

Before `/replanner/start`, these must be running: Belief's `belief_map` node (with a survey area that
contains the mission area), and Control's `master_control` in replanner mode (on the boat, or in a
simulated world started with `full_mission_launch.py path_source:=replanner`).

| Launch argument | Default | Meaning |
|---|---|---|
| `mission_file` | required | Absolute path to the mission file |
| `config_file` | `""` (built-in defaults, as in `config/replanner.yaml`) | Planner setting overrides (a `replanner:` block) |
| `c2_tuned_file` | `""` (installed `config/c2_tuned.yaml`) | The frozen C2 baseline applied to every mission |
| `data_dir` | `""` | Data root for the run records |
| `belief_topic` | `/belief_map/belief_grid` | Where the belief map comes from |
| `tick_hz` | `5.0` | How often the node runs its loop |
| `use_sim_time` | `false` | `true` in Gazebo |
| `time_source` | `wall` | `ros` in Gazebo (with `use_sim_time:=true`) |

Safety behaviour:

- No automatic start: only `/replanner/start` arms a mission.
- An operator override or an emergency stop disarms the Replanner. It re-arms only through
  `/replanner/start`.
- `/replanner/stop` returning success means the stop was *requested*. Only `/replanner/status`
  reports whether it took effect (`stopped_holding`).

## 2. The mission file

One GPS polygon, a condition and the planning depth. It must lie inside Belief's survey area.
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

Unknown keys are refused. Do not repeat the C2 baseline settings in a mission: a different value
refuses the mission.

## 3. The detection emulator (simulation and tests only)

It runs **instead of** Perception, never beside it, and publishes emulated detections from the
world's true objects:

```bash
ros2 launch blueboat_replanner emulator_launch.py world_dir:=$HOME/worlds/<path>/<world> seed:=1
```

## 4. Settings

- `config/replanner.yaml` lists every planner setting with its unit and its origin: a user decision,
  a fact of the repository, the literature, or a **placeholder**. Unknown keys raise.
- A mission file's `config:` block overrides it for that mission.
- `config/c2_tuned.yaml` is the frozen C2 baseline (line spacing, first-pass direction, interlacing).
- Key defaults: speed 0.5 m/s, sonar range 20 m, turn radius 3 m. The aspect-model values in
  `classes:` are placeholders.
- `class_families` (which aspect model each detector class uses) is empty on purpose: the owner
  decides it.

## 5. Topics

| Name | Kind | Type |
|---|---|---|
| `/belief_map/belief_grid` | subscribed | `blueboat_belief_interfaces/BeliefGrid` (kept for late subscribers) |
| `/blueboat/path_progress` | subscribed | `std_msgs/String` (JSON, from Control) |
| `/blueboat/path_update_event` | subscribed | `std_msgs/String` (JSON, from Control) |
| `/blueboat/odom` | subscribed | `nav_msgs/Odometry` |
| `/blueboat/update_path` | service client | `blueboat_interfaces/srv/UpdatePath` |
| `/replanner/plan` | published | `nav_msgs/Path` (the path Control holds; display) |
| `/replanner/decision` | published | `std_msgs/String` (JSON, one record per decision) |
| `/replanner/status` | published | `std_msgs/String` (JSON, once per second and on change) |
| `/replanner/start`, `/replanner/stop` | services | `std_srvs/Trigger` |

Delivery settings, JSON schemas and files: [topics.md](topics.md).

## 6. Outputs on disk

Each run gets its own write-once folder
`<root>/data/Replanner_data/<date-time>-<mission>-<condition>/`: manifest, a copy of the mission,
every plan, the decisions, the requests, the status, the events and an end record. The root is
`data_dir`, else `$BLUEBOAT_DATA_DIR`, else the workspace (normally `~/ros2_ws`).

## 7. Where it stands

- Built: the planning core, the executor, the ROS node, the emulator node and the launch files. The
  offline test gate passes.
- Run on an isolated ROS graph beside Belief's node with made-up inputs.
- Run once in Gazebo: a C2 development run in a built world, the first path accepted and flown to
  the end.
- **Not yet**: C4 or the stop test in Gazebo, a trained detector, the boat.

## 8. Folder layout

```
blueboat_replanner/                 ROS 2 package
  blueboat_replanner/core/          planning: no ROS, no clock, no randomness
  blueboat_replanner/io/            mission file, trajectory export, run records
  blueboat_replanner/ros/           adapters, the executor (the only code that calls update_path), the node
  blueboat_replanner/eval/          development loop: Belief binding, world, emulator, Control stand-in, runner
  config/                           replanner.yaml, c2_tuned.yaml, mission_example.yaml
  launch/                           replanner_launch.py, emulator_launch.py
  test/                             the test gate and the ROS check
docs/                               this guide and the design documents
results/                            development-loop outputs
```
