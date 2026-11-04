# Phase 0 — verification against the checked-out workspace

Status: **Phase 0 deliverable, cloud session, 2026-11-04.** No code. Answers to brief §7.1 and §12,
with the evidence behind them. Labels follow brief §16.2. Line numbers are those of the revisions
below. They drift, so check the symbol name too.

## 0. What was inspected

| Repository | Where | Revision inspected | Gitlink in superproject `bc18142` | Note |
|---|---|---|---|---|
| Superproject `BlueBoat-SideScanSonar` | `/home/user/BlueBoat-SideScanSonar` | `bc18142` (2026-11-04 "New Modules"), clean | — | Its submodule directories are **empty**; the modules are cloned as siblings under `/home/user/` |
| BlueBoat-Control | `/home/user/BlueBoat-Control` | `1fadbb2` (2026-11-03), clean | `1fadbb2` | equal |
| BlueBoat-MCS | `/home/user/BlueBoat-MCS` | `1d5cb36` (2026-11-02), clean | `1d5cb36` | equal |
| BlueBoat-SSS-Sim | `/home/user/BlueBoat-SSS-Sim` | `86f8e29` (2026-11-04), clean | `86f8e29` | equal |
| BlueBoat-SSS | `/home/user/BlueBoat-SSS` | `f85876b` (2026-11-03), clean | **`ed759e0`** | clone is one commit **ahead** of the gitlink (`f85876b` = processor rate request; not relevant here) |
| BlueBoat-Perception | `/home/user/BlueBoat-Perception` | `8c28025` (2026-11-03), clean | `8c28025` | equal |
| BlueBoat-Replanner | `/home/user/BlueBoat-Replanner` | `ba97fd7` (2026-11-04 "first commit", README only) | `ba97fd7` | this module |
| **BlueBoat-Belief** | — | **not present** | `3ba9281` | empty directory in the superproject; no clone. Not read (by design: contract only) |
| SSS-Dataset-Aug-Studio | — | not present | `98a523b` | not needed |

Missing inputs named by the brief: the context pack `00`–`03` (2026-10-09) and
`papers_to_be_checked.md` are **not in the workspace** (searched `/`). `project_synthesis.md`,
`literature_conclusions.md`, `target_families_and_surrogates.md` are at the superproject root.

Environment: Python 3 with numpy 2.4.4, PyYAML 6.0.3, scipy 1.17.1; **no `rclpy`** (`import rclpy`
fails). Every ROS check is therefore [NOT RUN — needs in-machine session] (`LOCAL_CHECKLIST.md`).

The brief was copied read-only to `docs/REPLANNER_BRIEF.md`
(sha256 `b9aa7ff6723c57c11b198e3b968da52acc7c1feb9f6560c1b449d164df56ca5a`).

---

## 1. Brief §7.1 — how Control lets the boat change its path mid-run

**Answer: a mechanism exists.** It is the *live path update*, added by owner order on 2026-10-15
(Control `b94dd7b` "Path update update"; root CLAUDE.md §3, §4.1, D8). The stop condition of brief
§7.1 does not apply.

### 1.1 Entry point, caller, fields

| Item | Fact | Label / evidence |
|---|---|---|
| Entry point | Service `/blueboat/update_path`, type `blueboat_interfaces/srv/UpdatePath`, served by `master_control` in **every** launch (sim and real) | [CODE] `blueboat_control/src/master_control.py` `_init_path_update` l.1494–1537 (service created l.1527); [CODE] `blueboat_interfaces/srv/UpdatePath.srv`; [DOC] `docs/PATH_UPDATE_INTERFACE.md` §2 |
| Request | `string id` (`[A-Za-z0-9._:-]{1,64}`, unique among the run's **accepted** ids), `float64 change_point` (path seconds; NaN or t[0] for a first path), `string frame` (must be `"world"`), `float64[] t, x, y, yaw` | [CODE] `UpdatePath.srv`; `path_update.py` `ID_PATTERN` l.46, `FRAME` l.44 |
| Response | `bool accepted`, `string reason` (`ok` or one of 17 codes), `string message`, `string active_id`, `float64 tau`, `float64 min_change_point` (NaN when none) | [CODE] `UpdatePath.srv`; `path_update.py` `PathUpdater._answer` l.560–569 |
| Who may call | Any client. Nothing in Control restricts the caller. Clients today: only Control's tests (`test/path_update_demo.py`, `test/gazebo_path_update.py`) | [CODE] no caller check in `update_path_callback` l.1596; [DOC] root CLAUDE.md §4.1 row `/blueboat/update_path` |
| Run kind | Only a **replanner-owned** run (`path_source=replanner`) accepts updates; a fixed-mission or pinger run answers `mission_owned` | [CODE] `path_update.py` `PathUpdater.gate` l.414–433; `master_control.py` `path_source` param l.1496–1507 |
| How a run becomes replanner-owned | Launch arg `path_source:=replanner` on `Sim_launch.py` (l.14, 88) or `BlueBoat_launch.py` (l.13, 62); in a real session also the optional `configure` field `"path_source": "replanner"` | [CODE] launch files; `master_control.py` l.1311–1320; [DOC] interface §10.2–10.3 |

### 1.2 What can be changed, and how the change point is expressed

| Item | Fact | Label / evidence |
|---|---|---|
| First path | While `state == awaiting_path`: `change_point` NaN or = t[0]; first sample within 0.20 m of `progress.hold`; any yaw (the boat turns in place). On acceptance τ := t[0] | [CODE] `PathUpdater.handle` l.484–490, 509–512, 527–530; [TEST] `test/test_path_update.py` |
| Later updates | Replace **everything from `change_point` c on**: plan = old path before c + new samples from c; earlier updates with change point ≥ c are discarded; samples of the new request before c are ignored | [CODE] `PathPlan.splice` l.243–246, `segment_at` l.211–215 |
| Append after the end | Allowed: an update whose c is past τ once `end_reached`; the lead is then the floor only and the heading check is skipped (path stationary) | [CODE] `lead_min` l.392–407 (stationary → floor), `handle` l.512–515; [DOC] interface §10.4 item 7 |
| Replace everything | Not as such: τ never goes backwards, so only the part after `τ + lead_min` can change. A new run (`start`/`stop`, or a new launch in sim) clears the plan | [CODE] `reset` l.366–385; [DOC] interface §1, §9 |
| Change-point unit | **Path seconds** on one t axis shared by every path of the run (not wall time, not an index) | [DOC] interface §1 "Path seconds and τ"; [CODE] `handle` l.491–501 |
| Evaluation rule | Linear x, y; short-way yaw (`wrap`); clamped before t[0] and after t[n−1] — the `blueboat_trajectory/1` rule | [CODE] `Segment` uses `yaml_trajectory.YamlTrajectory.from_points` (l.180–186); [DOC] interface §3 |

### 1.3 Acceptance rules (checked in this order; the first failure is the answer; a refusal changes nothing)

| Check | Rule | Code |
|---|---|---|
| State | `not_ready` (E-STOP latched / controller not ready / replanner run without odometry yet), `no_run` (idle real session), `mission_owned`, `operator_override` (manual target or arrows) | `gate` l.414–433 |
| Request | `bad_request`, `duplicate_id`, `too_few_points` (< 2), `too_many_points` (> 20 000), `not_finite`, `time_not_increasing` (step < 1 ms), `time_out_of_range` (t < 0 or ≥ 1e6), `too_fast` (> 2.0 m/s), `heading_incoherent` (yaw > 45° from direction of motion on a moving segment) | `validate_samples` l.261–343; `Config` l.119–128 |
| Change point | first path: NaN or t[0] (±1e-6 s); otherwise c required and `t[0] ≤ c < t[n−1]` | `handle` l.484–501 |
| Lead | `c ≥ τ + lead_min`, `lead_min = max(path_time + 2·dt, lead_factor·gov_Lmax/U_d)` (second term only while U_d ≥ `hold_speed` 0.05 m/s). PID/LoS: 3.0 s at 1.0 m/s, 6.0 s at 0.5 m/s, 10.7 s at 0.28 m/s; MPC sim floor 6.1 s | `lead_min` l.392–407, `handle` l.502–507; [DOC] interface §6 |
| Position continuity | new path at c within **0.20 m** (`path_update_pos_tol`) of the current path at c (first path: of `hold`) | `handle` l.517–533 |
| Heading continuity | yaw at c within **10°** (`path_update_heading_tol`); **not checked** where the current path is stationary at c (past its end, or the pre-path hold) | `handle` l.520–539 |
| Not checked by Control | turn rate, turn radius, path inside any area, speed below 2.0 m/s being feasible | [DOC] interface §10.4 item 6; R-2 in `docs/REQUESTS.md` ("feasibility is the Replanner's") |

On refusal the old path continues unchanged: [CODE] `_refuse` l.549–558 touches counters only;
[DOC] interface §4 "A refusal changes nothing"; [TEST] `test/test_path_update.py` (refusal
immutability is among its 102 checks; run here: **102 passed, 0 failed**, 2026-11-04, system
python, `PYTHONDONTWRITEBYTECODE=1 python3 test/test_path_update.py`).

### 1.4 Progress feedback

`/blueboat/path_progress` (`std_msgs/String`, JSON `blueboat_path_progress/1`, reliable volatile
depth 10, 10 Hz + immediately after every answer or transition): `state` (`not_ready`, `no_run`,
`mission`, `override`, `awaiting_path`, `following`, `end_reached`), `accepting`, `refusal_now`,
`tau`, `U_d`, `lead_min`, `min_change_point`, `following_id`, `active_id`, `pending_ids`,
`t_start`, `t_end`, `end_reached`, `hold` (only while awaiting), `override`, `last_answer`,
counters, `run`, `ros_time`, `wall_time`. [CODE] `PathUpdater.progress` l.632–670;
`master_control.py` publisher l.1524, timer l.1529.

`/blueboat/path_update_event` (JSON `blueboat_path_update_event/1`, reliable, **depth 50**, not
heartbeated): `request_accepted`, `request_refused`, `activated` (τ entered a new update),
`end_reached`, `cleared`; each carries `boat {x, y, psi}`, `tau`, `lead_min`, gaps.
[CODE] `PathUpdater._event` l.672ff, `tick` l.572–597.

**The progress message carries no boat pose** (only `hold` while awaiting). The boat pose is in
the event messages and on `/blueboat/odom`. Progress *along the path* is τ, which is the
governor's parameter: it advances at most at authored speed, slows when the boat falls behind
and freezes at a 3 m along-track gap; it never goes backwards. [DOC] interface §1;
[CODE] `advance_governor` (master_control).

### 1.5 Ownership

- A replanner-owned run never calls `/path_request` and never reads the watched trajectory file:
  `timer_callback` routes it to `_replanner_reference_step`, which composes the window from the
  held plan. [CODE] `master_control.py` l.564–568, l.1646–1670. So the MCS file watcher
  (`path_generation` + `.session/run.yaml`) cannot change a replanner run's path.
  [DOC] interface §10.2 item 3.
- The operator always wins: E-STOP, run stop, the motor gate, manual target and arrows refuse
  updates while engaged and pause τ; releasing them resumes the loaded path. [CODE] `gate`;
  [DOC] interface §9; Control N10.
- Two replanner clients are not arbitrated: any accepted update replaces the remainder. Single
  client is the Replanner's responsibility. [CODE] no caller identity in the request.

### 1.6 End of path, override, E-STOP

- τ ≥ `t_end`: `end_reached` event, the boat **holds the last pose** (station-keeping gate,
  U_d = 0). [CODE] `tick` l.587–594; [DOC] interface §10.4 item 7.
- Manual target / arrows: τ freezes, updates refused with `operator_override`; on release the
  loaded path resumes. E-STOP: `not_ready`; the loaded path resumes after Release.
  [DOC] interface §9.
- Run start/stop (real sessions) and a launch (sim) clear the path and the accepted ids
  (`cleared` event); a new replanner run awaits its first path with a new `hold`.
  [CODE] `reset` l.366–385; `master_control.py` l.1430, l.1457.

### 1.7 Simulation and real boat

Same service, topics, QoS and semantics in both ([DOC] interface header and §2; [CODE] one
implementation in `master_control`). Differences: real runs are session-gated (`no_run`) and the
pre-path hold needs odometry; simulation has no E-STOP and no run session.
**Two limits that matter for this module:**
1. **Built-world simulations cannot run replanner-owned runs yet.** BlueBoat-SSS-Sim's
   `full_mission_launch.py` starts its own copies of Control's nodes and does not pass
   `path_source` (Control `docs/REQUESTS.md` R-3; Control `.claude/TODO.md`). Only
   `Sim_launch.py` (stock `ocean` world, no objects) supports it today.
2. **MCS has no Replanner mode** (R-1). A real replanner session can be started through MCS's
   *Extra args* (`path_source:=replanner`), which makes every run of that session
   replanner-owned. [DOC] `docs/REQUESTS.md` R-1.

### 1.8 How it was tested, documented limits

| Evidence | Result | Label |
|---|---|---|
| `test/test_path_update.py` (ROS-free) | 102/102 — **re-run here 2026-11-04: 102 passed, 0 failed** | [TEST] |
| `test/test_path_update_log.py` | 24/24 — **re-run here: 24 passed, 0 failed** | [TEST] |
| `test/mc_harness.py --replanner` | 3 × (19 + 11) checks, offline closed loop, all controllers | [DOC] `docs/PATH_UPDATE_REPORT.md` (not re-run: needs ROS + acados) |
| `test/gazebo_path_update.py --replanner` | 13/13 per controller; target trail never jumps; boat–target median PID 0.89 m, LoS 1.16 m, MPC 0.04 m | [DOC] report (not re-run) |
| Real water | **Never tested.** Protocol `docs/PATH_UPDATE_REAL_WATER_TEST.md` | [DOC] report "Assumed, not demonstrated" |

Documented limits relevant to the design: the Replanner owns feasibility (speed, turn radius);
"author at least 0.28 m/s … in simulation stay at or below 0.45 m/s; the simulated boat tops out
at about 0.78 m/s"; field missions used turn radii 1.5–6 m ([DOC] interface §10.4 item 6). The
yaw-coherence guard does not catch every deg/rad error (Control TODO).

---

## 2. Brief §12 — the other verification items

### 2.1 Starting a headless simulated mission with an initial plan (§12 item 1, second half)

| Launch | Replanner-owned run? | World | Evidence |
|---|---|---|---|
| `blueboat_control Sim_launch.py controller_type:=… path_source:=replanner` | **Yes.** The launch is the run; the boat holds at the spawn pose until the first path; `trajectory:=` is ignored; `/set_path` still shows that argument's path | stock `ocean` world, no objects | [CODE] `Sim_launch.py` l.14, 88; [DOC] interface §10.3 |
| `blueboat_sss_sim full_mission_launch.py world_dir:=… with_control:=true trajectory_file:=… headless:=true` | **No.** Starts `simulation_interface`, `path_generation`, `path_publisher`, `master_control` directly and passes no `path_source`, so `master_control` stays `mission` | built world (objects, walls, anchor) | [CODE] SSS-Sim `launch/full_mission_launch.py` l.163–193 (arguments), l.305–323 (Control nodes); grep `path_source` in SSS-Sim: no hit; [DOC] Control `docs/REQUESTS.md` R-3 |
| `blueboat_sss_sim sss_sim_launch.py world_dir:=…` (sonar only; `with_mavros_shim`, `auto_ping`, `sonar_seed`, …) | n/a (no control) | renders the world folder from the received pose | [CODE] `launch/sss_sim_launch.py` l.70–81 |

The sonar is not a Gazebo plugin: `sss_sim_node` renders the world folder's `scene.npz` from the
pose it receives ([DOC] SSS-Sim CLAUDE.md §2). A composition `Sim_launch.py path_source:=replanner`
+ `sss_sim_launch.py world_dir:=<world>` would therefore render a built world's seabed while Gazebo
simulates the boat in the stock ocean world (no walls, no limit collision, no visible objects).
Whether that is acceptable for this module's Gazebo missions, or whether R-3 should be done first,
is a user question (Q-12). **Not run here.**

### 2.2 `blueboat_trajectory/1` (§12 item 2)

| Item | Fact | Evidence |
|---|---|---|
| Format key | `format: blueboat_trajectory/1` (key is `format`, not `schema`); anything else raises `ValueError` | [CODE] Control `yaml_trajectory.py` `SUPPORTED_FORMAT` l.23, `__init__` l.29–47 |
| Required | `points`: non-empty N×4 array `[t s, x m, y m, yaw rad]`; otherwise `ValueError` | [CODE] l.37–39; [DOC] MCS `docs/05_trajectory_format.md` l.28–29 |
| Optional, read | `name` (""), `loop` (False) | [CODE] l.40–41 |
| Ignored by the loader | `generator`, `created`, `frame`, `speed`, `length_m`, `duration_s`, `geo_anchor`, `deployed_from`, `placed_at`, … | [CODE] l.29–47 |
| `geo_anchor` | `{lat0, lon0, theta_deg}`, `theta_deg` 0 for files written today (non-zero legacy, still honoured by the deploy step) | [DOC] `05_trajectory_format.md` l.53–60; [CODE] MCS `designer/io_yaml.py` `save_mission` l.115–120, `deploy_mission` l.188–237 |
| Yaw | rad, ENU, CCW from East, path tangent; designer writes `arctan2(dy, dx)`, last row repeats | [DOC] `UpdatePath.srv`; [CODE] MCS `designer/sampling.py` l.111–112 |
| Evaluation | clamped outside `[t[0], t[n−1]]`; linear x, y; short-way yaw `wrap(y0 + u·wrap(y1 − y0))`; `loop` wraps `t % t[-1]` | [CODE] `pose` l.71–90 |
| **Not validated by the loader** | `t[0] = 0`, strictly increasing `t`, finite values (the doc says "strictly increasing t" is REQUIRED; the code does not enforce it) | [CODE] l.29–47 vs [DOC] `05_trajectory_format.md` l.28 — **discrepancy D-T1** |
| Designer conventions | `t[0] = 0.0`, `t = s/speed`, `ds = 0.25 m`; `save_mission` rounds t to 3 decimals | [CODE] `sampling.py` l.66, 81; `io_yaml.py` l.98–100 |
| Doc vs code | loop wraps "modulo `duration_s`" (doc) vs `t[-1]` (code) | **discrepancy D-T2** (not relevant if this module writes `loop: false`) |

Path-update requests are stricter than files: t ≥ 0, steps ≥ 1 ms, finite, ≤ 20 000 samples,
speed ≤ 2.0 m/s, yaw coherent within 45°; a first path may start at any t[0] ≥ 0 and later paths
continue the same axis ([CODE] `path_update.validate_samples` l.261–343). So the brief's §9.3
"t[0] = 0" holds for files, not for update requests.

### 2.3 BlueBoat-Belief (§12 item 3)

**Not in this workspace** (empty superproject directory, gitlink `3ba9281`, no sibling clone). By
the session design the contract `belief_grid/0` (prompt §B) is the only source. What the
workspace *does* say about Belief:
- Root CLAUDE.md §1 and CM-9 (l.476–480), `project_synthesis.md` §5.1 (l.216) and §5.2: Belief
  reads **only Perception's per-image records** (`/sss_ai/image_records`) plus the mission's
  survey area and anchor, and sends only to the Replanner.
- The contract C-1 says Belief **subscribes `/sss_ai/detections`** (`vision_msgs/Detection2DArray`)
  and the Replanner's emulator publishes that same topic. **Discrepancy D-B1** (Q-9).
- Perception's image record carries the per-row look directions `track.look_port_rad` =
  ψ + π/2 and `look_stbd_rad` = ψ − π/2 ([CODE] Perception `core/geometry.py:90`
  `look_azimuths`; [DOC] `docs/topics.md` §4), **consistent with contract C-5** (s = +1 port).
- Node / topic names, QoS, alignment mode, datum: unknown here → contract C-2/C-3 provisional,
  [OPEN] until the in-machine session (`LOCAL_CHECKLIST.md` L-B*).

### 2.4 `mcs/core/geo.py` (§12 item 4)

[CODE] MCS `mcs/core/geo.py` (rev `1d5cb36`):
```
38  EARTH_RADIUS_M = 6378137.0
42  def latlon_to_local_en(lat, lon, lat0, lon0):
44      east = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(math.radians(lat0))
45      north = math.radians(lat - lat0) * EARTH_RADIUS_M
46      return east, north
49  def local_en_to_latlon(east, north, lat0, lon0):
50      lat = lat0 + math.degrees(north / EARTH_RADIUS_M)
51      lon = lon0 + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))
52      return lat, lon
```
Identical to contract C-5. `GeoFit` (l.78–117) is translation-only (`EN = world + t`). Other
copies: SSS-Sim `core/geo.py` (numpy, array-capable; claims bit-equality in its docstring, not
proven here), GCS `mapping/geo.py` (same bodies, wrapped signatures) and GCS `utils/geodesy.py`
(**anchor-first argument order**). This module will copy the MCS pair verbatim (math, scalars) and
test bit-identity against MCS when MCS is importable (it is, as a plain file, in the cloud).

### 2.5 World `metadata.yaml` (`blueboat_world_meta/1`, §12 item 5)

[CODE] SSS-Sim `worldgen/world_folder.py` `_write_metadata` l.371–461; [DOC]
`docs/configuration_guide.md` l.631–652, `docs/integration_guide.md` l.43–62.
- Top-level: `format`, `world_name`, `created`, `source_path{name, file, design_offset}`,
  `geo_anchor{lat0, lon0, theta_deg}` (θ must be 0), `spawn{x: 0, y: 0, lat, lon}`,
  `limits{local{x_min, y_min, x_max, y_max}, corners_gps[SW, SE, NE, NW]}`, `walls[]`,
  `objects[]`, `depth`, `seabed_material`, `seed`.
- Per object (l.428–444): `object_id`, `type` (CATALOG key, no class label), `x`, `y` (world ENU
  m), `lat`, `lon`, `yaw` (rad, ENU CCW from East, the **long-axis** heading), `yaw_deg`,
  `roll_deg`, `pitch_deg`, `rest`, `orientation`, `length` (along local x), `width`,
  `proud_height`, `burial`, `fill_fraction`, `material`.
- World (0, 0) = spawn = `geo_anchor` by construction (l.407–410; SSS-Sim CLAUDE.md §2).
- CATALOG (`worldgen/objects.py` l.208–370), 29 types. The brief's planner classes have no
  verbatim match: block ≈ `concrete_block`, pipe ≈ `pipe_pvc`, cage ≈ `mesh_crate` /
  `shopping_trolley`, tyre ≈ `tire_car` / `tire_bicycle`, **plate: no simulator class**
  (`project_synthesis.md` §7.1 class E, a field surrogate). Mapping is Q-15.

### 2.6 Turn rate and settle time (§12 item 6)

**No measured value exists in any checked-out repository.** Control's 3-DOF model is unidentified
(Control report 2026-11-03, P01 "WEAK / DIVERGES"; yaw dynamics not identified). SSS-Sim records
only *disturbance* yaw-rate std (sim 1.2–1.3 °/s, field 5–14 °/s; SSS-Sim
`docs/sea_state_validation.md` l.278–291) and plans step / zig-zag identification
(`docs/engineering/validation_and_experiment_plan.md` l.30, 74). The only kinematic guidance:
field missions used **turn radii 1.5–6 m**; author ≥ 0.28 m/s; in simulation ≤ 0.45 m/s; the
simulated boat tops out at ~0.78 m/s ([DOC] Control interface §10.4 item 6; Control report §5.4
"Gazebo plant needs 59 N … cannot reach 1.0 m/s"). Field speed means 0.32–0.41 m/s on the Sin
route ([CODE-data] Control `controller_comparison/metrics_all.csv`). The brief's 30 °/s + 2 s stay
**placeholders** [ASSUMPTION]; measurement → `LOCAL_CHECKLIST.md` L-K1/L-K2; model choice Q-6.

### 2.7 `/blueboat/odom` validity (§12 item 7)

- **Simulation:** Gazebo `OdometryPublisher` in `blueboat.xacro`, 20 Hz, world ENU with
  (0, 0) = spawn ([DOC] SSS-Sim `docs/sea_state_model.md` l.398–399; Control CLAUDE.md §8).
  Valid.
- **Real boat:** values are local ENU with absolute yaw (re-zeroed position only; [CODE]
  `robot_interface.odom_callback`, Control report l.164). The open defect is the **stamp**: mavros
  inherits `use_sim_time` with no `/clock`, so `/mavros/local_position/odom` and hence
  `/blueboat/odom` carry **sec 0** ([DOC] SSS `.claude/TODO.md` l.150–166, cause found live
  2026-10-27; Control is the owner). Rate ~4 Hz, ~13 Hz while the sonar's interval request holds
  (root CLAUDE.md §4.4, UNCERTAIN). Field path-following runs exist, so the values themselves are
  usable [DOC]. The brief's "reads zero" wording is older than the 2026-10-27 diagnosis.
- Consequence for this module: any freshness test must use **arrival time**, never
  `header.stamp`. Whether to subscribe odometry at all is Q-5: Control's progress (`hold`, τ) and
  this module's own copy of the accepted path give the decision pose without it.

---

## 3. Discrepancies found (guide vs documents vs code)

Recorded, not resolved. Each is either a question (Q-n in `QUESTIONS_PHASE0.md`) or a note
carried into Phase 1.

| Id | Between | Discrepancy | Evidence | Disposition |
|---|---|---|---|---|
| D-S1 | brief §8.7 C2 "1.0 m/s", synthesis §8.5 "1.0 m/s nominal" **vs** Control | Simulated boat tops out at ~0.78 m/s (2 × 20 N limit); Control advises ≤ 0.45 m/s in sim; 1.0 m/s never flown (fastest authored field route 0.7 m/s) | Control interface §10.4.6; Control report §5.4 l.1447, P01 l.374 | **Q-1** |
| D-S2 | synthesis §8.5 "line spacing ~0.8 × usable swath" **vs** brief §3 "usable ground swath ≈ 6 m … √(20² − depth²)" **vs** TF near-field floor 9–10 m slant | "Usable swath" is undefined (one side? both sides? nadir gap?) and no document measures it; a factor-of-two ambiguity in line spacing and hence in the two-pass duration that sets every budget | synthesis l.570–573; TF l.39–42; SSS-Sim `configuration_guide.md` l.564 (no number) | **Q-2** |
| D-A1 | brief Q9 (mission file of WGS-84 polygons read by the Replanner) **vs** synthesis §5.1 ⚠ l.224–226 ("Belief carries the mission's survey area and anchor … so the Replanner needs no other input") **vs** contract C-3 (no polygon field) | Three sources, three answers for where mission areas come from; no MCS file holds a WGS-84 survey area (MCS designer areas are design-frame lawnmower parameters; the only WGS-84 rectangle is a world's `limits.corners_gps`) | MCS `designer/patterns.py` l.9–43; SSS-Sim `world_folder.py` l.411–417 | **Q-3** |
| D-A2 | CM-9 literal ("the Replanner reads only Belief") **vs** root §4.1 (`/blueboat/path_progress` → "replanner (future)") and Control R-2 (the Replanner must read progress and answers) | The rule's literal reading forbids inputs Control's own contract requires | root CLAUDE.md l.476–480, §4.1 row `/blueboat/path_progress`; Control `docs/REQUESTS.md` R-2 | **Q-4** (§16.1 interpretation) |
| D-B1 | contract C-1 (Belief subscribes `/sss_ai/detections`) **vs** root §1, CM-9, synthesis §5.1 (Belief reads `/sss_ai/image_records`) | The emulator's output topic/type depends on it; Perception is the one publisher of both today (MCS starts it for every personalized-world launch), so an emulator beside it would make two publishers | root CLAUDE.md §1, §4.3; Perception CLAUDE.md §2 | **Q-9** |
| D-R3 | brief §13 Phase 4 "one C2 and one C4 mission in Gazebo" **vs** Control R-3 | Built worlds cannot host replanner-owned runs yet | §2.1 above | **Q-12** |
| D-C3 | brief title "append-on-completion" **vs** brief §8.7 | Synthesis §7.3 condition 3 is "Adaptive, append-on-completion replanner"; brief C3 is described (pass 1, perpendicular revisits, recover lines) but not named so; the phrase is defined nowhere | synthesis l.461–464 | **Q-7** |
| D-R1 | prompt rule "randomness only in `eval/`" **vs** brief §8.7 "C4_rand … deployable" | — | brief §17.3 | **Q-8** |
| D-W1 | brief §10 dev loop "ablations and sweeps" with placeholder detection model **vs** synthesis §6.2 ⚠ "The probability function must be fitted from Tier-1 Gazebo runs, never invented" and §7.4 (sweeps are Tier 2) | Whether dev-loop sweeps may be reported at all | synthesis l.371–377, l.471–482 | **Q-10** |
| D-K1 | brief §8.5 turn model `|Δψ|/ω_turn + t_settle` (30 °/s, 2 s) **vs** Control | No turn rate is measured; field turn radii 1.5–6 m; 30 °/s at 0.4 m/s is a 0.76 m radius, tighter than any flown | §2.6 | **Q-6** |
| D-G1 | brief §6.5 ("datum default: polygon centroid") **vs** contract C-3 (mission `geo_anchor`) | Already resolved by the prompt (B.3): implement the general transform, read the datum | prompt §B.3 | Phase 1 |
| D-L1 | brief §15 lists Hollinger et al. 2013, Ji et al. 2007, Golovin & Krause 2011, Jadidi et al. 2019 as "checked, abstract" **vs** `literature_conclusions.md` | None of the four is in `literature_conclusions.md`, which uses only `[full]` / `[abs]` (no `[checked]`); it has Jadidi et al. **2017 arXiv** `[full]` and Hollinger & Sukhatme 2014 `[full]` | LC l.6–8, l.457, l.527 | Report; statuses not upgraded (Q-17) |
| D-L2 | brief §8.5 "a TSP/GTSP is not needed" **vs** `literature_conclusions.md` §5 directive 4 ("a turn-penalised generalised TSP over (contact, heading) pairs") | The brief's §5 strategy is a user decision; LC is a directive | LC l.501–522 | **Q-16** |
| D-L3 | synthesis l.113–115 "S-44 Special Order" **vs** LC l.36–38 ("should be Exclusive Order") **vs** brief "Ed. 6.1.0, Exclusive Order" | Order naming and edition | — | **Q-17** |
| D-T1 | MCS doc "strictly increasing t, REQUIRED" **vs** Control loader (not enforced) | — | §2.2 | note; this module validates its own output |
| D-T2 | MCS doc loop "modulo `duration_s`" **vs** code `t[-1]` | — | §2.2 | note; this module writes `loop: false` |
| D-O1 | brief §6.2 "odom reads zero in real runs" **vs** SSS TODO 2026-10-27 (zero **stamp**, values fine) | — | §2.7 | Q-5 |
| D-C1 | brief §8.2 class list (block, pipe, cage, tyre, plate) **vs** simulator CATALOG | No verbatim match; no plate class | §2.5 | **Q-15** |
| D-M1 | brief §3 "D4 Belief … built, core tested" | Not visible here | prompt §B.3 | contract only |
| D-M2 | brief §2 context pack `00`–`03`, `papers_to_be_checked.md` | Not in the workspace | §0 | Q-18 |
