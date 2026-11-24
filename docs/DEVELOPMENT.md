# BlueBoat-Replanner — development guide

How to work on the Replanner: the AI coding assistant setup, the tests, the rules and what to do next.
For using it, see the [README](../README.md) and the [user guide](USER_GUIDE.md).

## 1. Install Claude Code

The project is developed with [Claude Code](https://docs.claude.com/en/docs/claude-code/overview),
an AI coding assistant that runs in the terminal.

```bash
curl -fsSL https://claude.ai/install.sh | bash        # native installer
# or, with Node.js 18 or newer:
npm install -g @anthropic-ai/claude-code
```

Then start it and log in with your Anthropic account:

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar               # the whole project: cross-module rules included
claude
# or only the Replanner (needed for its test hook, below):
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Replanner && claude
```

From the superproject, Claude also reads the project rules, the interface contract and the shared
skills (test commands, file formats, and `sim-run-recording` for simulated runs).

## 2. What Claude reads in this module

| File | Role |
|---|---|
| `.claude/CLAUDE.md` | Loaded at start: the conditions, the layout, the interface, the non-negotiable rules, the commands and the status |
| `.claude/rules/*.md` | Three path rules: keep the core pure, frames and trajectories, keep the planner, world and Belief models apart |
| `.claude/settings.json` | The test hook (below) |
| `.claude/tools/` | `smoke_gate.sh` (the hook's script), `devloop_batches.sh` (the development loop), `ros_check.sh` (the ROS check) |
| `.claude/TODO.md` | Open work, by priority |
| `docs/HANDOVER.md` | Where the work stands: read it first |

Documents describe the current state only: no session records or change logs, one main copy of each
fact.

## 3. What to set up after a fresh clone

Nothing: unlike some other modules, the Replanner tracks its Claude settings and tools in git.

- **The test hook**: when Claude stops after a Python file of the package changed, `.claude/settings.json`
  runs `.claude/tools/smoke_gate.sh`, which runs the offline gate and blocks the stop if it fails.
  It works only when Claude is started inside `BlueBoat-Replanner/`.
- **The neighbouring checkouts** beside this repository (BlueBoat-Belief, BlueBoat-MCS,
  BlueBoat-Control): some gate sections cross-check them, and skip visibly without them. The
  development loop needs Belief's core and stops without it.

## 4. Tests

```bash
cd blueboat_replanner && python3 -B -m test.smoke_test     # the offline gate, no ROS, about 90 s
colcon test --packages-select blueboat_replanner          # the same gate through colcon
.claude/tools/devloop_batches.sh                          # development loop on Belief's core (about 45 min, 4 workers)
.claude/tools/ros_check.sh                                # scratch build + the nodes on an isolated ROS domain (about 28 min)
```

- **The development loop** (`eval/`) runs without ROS: Belief's own core, a seeded synthetic world, an
  emulated detector, a boat that tracks the plan perfectly. Its numbers are preliminary, never a
  thesis result. Method and results: [DEVLOOP.md](DEVLOOP.md).
- **The ROS check** runs on its own ROS domain with made-up inputs, never on a live graph.
- **Before a long run**, check that no simulation is being recorded: `pgrep -af 'record_ru[n]\.py'`.
- The ROS, Gazebo and boat checks, with their commands: [LOCAL_CHECKLIST.md](LOCAL_CHECKLIST.md).

## 5. Rules for changes

- **Edit only inside BlueBoat-Replanner.** Other modules are read-only, Belief included; needs go to
  `docs/REQUESTS.md`.
- **No new message types.** JSON goes on `std_msgs/String` with a `"schema"` field.
- **The path changes only through `/blueboat/update_path`.** Never write Mission Control Station files,
  never start a path service.
- **The belief map is the only online target evidence.** Never read detections, sonar data, ground
  truth or simulation topics online.
- **The core stays pure**: no ROS, no clock, no randomness in `core/`. Randomness lives in `eval/` only,
  from seeded generators.
- **Planner model, world model and Belief's model stay separate**: configuration, not code.
- **C2 is tuned seriously and frozen before any C3 or C4 result. Never tune to make C4 win.**
- **Results are model-conditional**, and "score-conditional" while Belief's detector is not
  calibrated.
- **Mathematics first**: an ambiguity found while coding goes to `docs/MATH.md` before the code.
- **Safety**: no automatic start; an override or emergency stop disarms; "stopped" is reported only
  as `stopped_holding`.
- **Cite nothing from memory.**
- AI sessions never commit, push or tag: the owner does.

## 6. Next steps

From `.claude/TODO.md`, in plain words:

1. **Run the remaining Gazebo missions**: a C4 mission, the stop test and the last check of the list
   (L-G2 to L-G4 in `LOCAL_CHECKLIST.md`). They also measure the decision time under load.
2. **Test Control's live path update on water**, then a supervised dry run of the Replanner on the boat.
3. **Decide which aspect model each detector class uses** (the class → family map). Until then the
   choice of look direction has no effect for any real class.
4. **Show that 0.5 m/s is achieved in Gazebo** and that 3 m turns are tracked; measure the turn
   radius and the boat's lag behind the path, then freeze them.
5. **Freeze C2 in Gazebo** on tuning worlds kept apart from the evaluation worlds, and fit a world
   detection model from Gazebo before any reported comparison.
