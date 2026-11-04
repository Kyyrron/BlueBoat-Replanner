# Integration — BlueBoat-Replanner in the superproject

How the module plugs into the stack, what it needs from its neighbours, and the lines proposed
for the superproject root `.claude/CLAUDE.md`. **The root file is not edited by this module;
the user applies these lines.**

## 1. Position in the chain (root §1)

```
sonar → Perception (→ emulator in dev/sim) → Belief ──BeliefGrid──► Replanner ──/blueboat/update_path──► Control
                                                                     ▲   ▲
                                         mission file (areas, condition) │  /blueboat/path_progress, /path_update_event
```

**Online inputs:**
- `BeliefGrid`: the only target evidence (contract `belief_grid/0`);
- Control's progress, events and answers;
- `/blueboat/odom`: arrival-time freshness; the fallback is the mirror pose;
- the mission file;
- the module's own configuration.

**Never read online:** detections, sonar, simulation ground truth, `/sim/*`. The gate sweeps for
these names.

**Outputs:**
- requests to `/blueboat/update_path` (the only way the path changes);
- `/replanner/plan`, `/replanner/decision`, `/replanner/status`;
- run records under `<root>/data/Replanner_data/`.

## 2. Dependencies

| Needs | From | State |
|---|---|---|
| `/blueboat/update_path`, `/blueboat/path_progress`, `/blueboat/path_update_event` | BlueBoat-Control (2026-10-15, add-only) | exists; verified in code (E-01…E-19); the ROS-free core was cross-checked by our gate |
| `path_source:=replanner` run | Control launch files (sim and real) | exists (E-19) |
| Built-world replanner runs | BlueBoat-SSS-Sim `full_mission_launch.py` | **missing** (R-3 / RQ-1); the composition in `LOCAL_CHECKLIST` L-G is the stop-gap |
| `blueboat_belief_interfaces/BeliefGrid` | BlueBoat-Belief (parallel session) | **not visible**; contract `belief_grid/0` only |
| The evidence topic Belief reads | Belief / user | **[OPEN]** (Q-9, RQ-3) |
| MCS "Replanner mode" | MCS (frozen, add-only) | missing (R-1); a real session is launched with *Extra args* `path_source:=replanner` |

## 3. Running it

| Mode | How |
|---|---|
| Offline gate | `cd blueboat_replanner && python3 -m test.smoke_test` |
| Dev loop (ROS-free) | `python3 -m blueboat_replanner.eval.runner --out <dir> --tune-c2`, then `--c2-tuned <dir>/c2_tuned.yaml --seeds 30 --conditions ...` |
| Node | `ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs yaml>`, then `ros2 service call /replanner/start std_srvs/srv/Trigger {}` |
| Emulator (sim) | `ros2 launch blueboat_replanner emulator_launch.py world_dir:=<world> ...`, instead of Perception |
| Gazebo mission | `LOCAL_CHECKLIST.md` L-G |

## 4. Proposed lines for the root `.claude/CLAUDE.md`

### §1 table, a new row

| Module | Produces |
|---|---|
| **BlueBoat-Replanner** — `blueboat_replanner` | Aspect-aware replanner (D6). It reads the BeliefGrid as its only target evidence, plus Control's path progress and a mission file. It generates every survey condition (C1 single pass, C2 two-pass orthogonal, C3 append-on-completion, C4 informative and its ablations) through one coverage generator. It changes the followed path only through `/blueboat/update_path`. ROS-free deterministic core; the dev loop runs against a reference belief (a stand-in); the ROS node is written but **not yet run** |

### §4.1, new rows

| Name | Type | Producer → Consumers | Notes |
|---|---|---|---|
| `/replanner/plan` | `nav_msgs/Path` | `replanner` → RViz, GCS (optional) | Frame `world`, TRANSIENT_LOCAL depth 1. The mirror of what Control holds; pose stamps = path parameter |
| `/replanner/decision` | `std_msgs/String` (JSON `blueboat_replanner_decision/1`) | `replanner` → bag, analysis | One per decision: candidates with J/J2, rules fired, budget, request |
| `/replanner/status` | `std_msgs/String` (JSON `blueboat_replanner_status/1`) | `replanner` → operator, MCS (future) | TRANSIENT_LOCAL depth 1, 1 Hz + on change. States include `disarmed` (override / E-STOP, re-arm only by `/replanner/start`) and `stop_rejected` (stop **not** in effect) |
| `/replanner/start`, `/replanner/stop` | `std_srvs/Trigger` | operator → `replanner` | Start or re-arm a mission; stop = truncation to a hold |

### §4.1 row `/blueboat/update_path`, amended

> Clients today: Control's tests **and BlueBoat-Replanner** (`ros/executor.py`; written,
> cross-checked offline against `path_update.PathUpdater`, not yet run live).

### CM-9, proposed wording (user decision 2026-11-04: "the brief overrides")

> **CM-9 — The belief layer is the only source of target / environment evidence the replanner
> reads.** Detector, sensor and planner changes interact only through it. The Replanner may also
> read operational inputs that carry no target evidence: Control's path progress, events and
> answers, `/blueboat/odom`, its mission file (areas, condition) and its configuration. It never
> reads detections, sonar data, simulation ground truth or `/sim/*` topics online.

The matching edit in `project_synthesis.md` §5.1 ⚠ ("so the Replanner needs no other input")
becomes "…needs no other target evidence; mission areas come from the mission file it shares with
Belief".

### §4.5 file interfaces, a new row (`file-interfaces` skill)

| Artifact | Path | Nature |
|---|---|---|
| Replanner run record | `<root>/data/Replanner_data/<date>-<mission>-<condition>/` (`manifest.json`, `mission.yaml`, `plans/*.yaml` as `blueboat_trajectory/1` in frame M with the datum as `geo_anchor`, `decisions.jsonl`, `requests.jsonl`, `status.jsonl`, `events.jsonl`) | write-once; folder claimed exclusively (CM-7) |
