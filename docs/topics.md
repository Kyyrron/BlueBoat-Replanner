# BlueBoat-Replanner — interfaces

Every topic, service and file this module reads or writes. This module defines no message type;
its JSON payloads travel on `std_msgs/String` with a `"schema"` field (CM-1). Names are absolute.
Status: implemented in `ros/replanner_node.py` and `eval/emulator_node.py`. Their wiring (names,
QoS profiles, services, schemas, records) is checked through fake rclpy modules (gate [24]).
**On 2026-11-12 both ran under ROS on an isolated graph beside BlueBoat-Belief's node**, with
made-up sonar, navigation and Control (`test/ros_graph_check.py`; what it showed and what it
did not: `REPORT.md` §F). They have never run beside Control's node, in Gazebo or on the boat.

## 1. Online subscriptions (`replanner_node`)

| Name | Type | QoS (this end) | Other end | Frame | Timestamp use | Use | Evidence |
|---|---|---|---|---|---|---|---|
| `/belief_map/belief_grid` (C-2; Belief `topics.md` §1) | `blueboat_belief_interfaces/BeliefGrid` | RELIABLE, **TRANSIENT_LOCAL**, KEEP_LAST 1 | Belief (offers TRANSIENT_LOCAL, 1 Hz on a steady clock) | M (`frame_id` "map", label only) | freshness by arrival time. **The header stamp is Belief's evidence horizon** (sonar-profile clock): it is converted to path time through the progress clock (MATH §11.3). `grid_seq`: equal = a repeat, lower = a Belief restart | the only target evidence | contract C-2, C-3; `CONTRACT_CONSUMED.md` |
| `/blueboat/path_progress` | `std_msgs/String` JSON `blueboat_path_progress/1` | RELIABLE, VOLATILE, depth 10 | `master_control` (same) | W | freshness by arrival time; **`ros_time` (Control's node clock: sim time in Gazebo, wall time on the boat) with `tau` gives the path time at the grid's stamp** | τ, state, `min_change_point`, `hold`, `active_id`, `end_reached` | E-11 |
| `/blueboat/path_update_event` | `std_msgs/String` JSON `blueboat_path_update_event/1` | RELIABLE, depth **50** | `master_control` (same) | W | logged | `cleared`, `activated`, `end_reached`, the answers' trace, `boat` pose | E-13 |
| `/blueboat/odom` | `nav_msgs/Odometry` | default, depth 10 (root §4.1) | `robot_interface` / Gazebo bridge | W | **arrival time only** (real stamps are sec 0, E-40) | decision pose for the record; fallback = mirror pose at τ | brief §6.2; E-39, E-40 |

**Never subscribed online** (enforced by the gate's source sweep):
- `/sss_ai/*`, `/side_scan_sonar/*`, `/sss_processor/*`;
- any `ground_truth`, `/sim/*`, `/replay/*`.

## 2. Online client

| Name | Type | Use | Evidence |
|---|---|---|---|
| `/blueboat/update_path` | service `blueboat_interfaces/srv/UpdatePath` | the first path and every change of the run's path. Request in frame `"world"`; ids `r<run>-<k>`. No first request until the service is ready; answers are handled at their arrival time (DESIGN §5.1, §5.2) | E-01, E-02 |

## 3. Online publications

| Name | Type | QoS | Frame | Content |
|---|---|---|---|---|
| `/replanner/plan` | `nav_msgs/Path` | RELIABLE, TRANSIENT_LOCAL, depth 1 | `world` | the mirror of the path Control holds, republished on every accepted change. Pose stamps = path parameter (sec, nanosec of t), as Control's `_compose_window` does; display only |
| `/replanner/decision` | `std_msgs/String` JSON `blueboat_replanner_decision/1` | RELIABLE, depth 50 | M for geometry, W for the request | one message per epoch (below) |
| `/replanner/status` | `std_msgs/String` JSON `blueboat_replanner_status/1` | RELIABLE, TRANSIENT_LOCAL, depth 1 | — | 1 Hz + on every change (below) |

## 4. Services served

| Name | Type | Semantics |
|---|---|---|
| `~/start` (`/replanner/start`) | `std_srvs/Trigger` | arms the mission (from `idle`), or re-arms it (from `disarmed`, if the re-arm checks of DESIGN §7 pass). `success` = moved to `waiting` / `running`; `message` names the missing inputs or the failed check |
| `~/stop` (`/replanner/stop`) | `std_srvs/Trigger` | `success` = the truncation request was issued (`stop_requested`). Acceptance and holding are reported **only** on status: `stop_accepted` → `stopped_holding`, or `stop_rejected` (DESIGN §5.3) |

## 5. JSON schemas

**Versioning rules** (Control's rule, E-11):
- fields may be **added** within a version;
- a rename, a removal or a change of meaning bumps the version;
- readers ignore unknown keys;
- a reader refuses a payload whose `schema` name is unknown, or whose version is greater than it
  knows.

### 5.1 `blueboat_replanner_decision/1`

```json
{"schema": "blueboat_replanner_decision/1", "seq": 7, "mission": "beach_2026_10_12",
 "condition": "C4", "run": 2, "ros_time": 412.3, "wall_time": 1793760000.1,
 "tau": 398.0,
 "epoch": {"source": "decision_point", "segment_id": "L05", "t_dp": 408.6, "change_point": 408.6,
           "min_change_point_at_send": 402.1, "outcome": "sent"},
 "inputs": {"grid_seq": 412, "grid_age_s": 0.6, "grid_stamp": 405.9, "progress_age_s": 0.05,
            "pose_source": "odom", "odom_age_s": 0.2, "pose_xy_psi": [12.1, 40.3, 1.571],
            "odom_to_map": [3.2, -1.0, 0.0], "odom_to_map_sigma_m": 0.4, "transform_jump_m": 0.0,
            "belief_model": {"detector_calibrated": false, "negative_evidence": "gated",
                             "coverage_source": "records", "rho_ordering": "paull_pairwise",
                             "fold_opposite": true, "look_timeout_s": 20.0},
            "belief_fallbacks": [], "belief_restarts": 0, "unmapped_classes": ["other"]},
 "budget": {"B": 2210.0, "elapsed": 398.0, "remaining": 1812.0,
            "revisit_used": 95.0, "revisit_cap": 1105.0},
 "calibration": {"cells": 5120, "clamped": 12, "fallback_no_sector": 0,
                 "searched_source": "aspect_log_miss", "p_background_median_rel_diff": 1e-7},
 "pending": {"tau_horizon": 385.9, "source": "stamp", "keys": 1830, "contact_keys": 1902,
             "contact_guard_s": 2.0},
 "candidates": [
   {"id": "REV:17:g3", "kind": "revisit", "uid": 17, "folded_sector": 3,
    "bearing_deg": 105.0, "side": "port", "T_detour": 84.0, "dT": 61.0,
    "J": 3.41, "J2": 3.52, "best_a2": "CONT", "feasible": true, "admissible": true, "reason": ""},
   {"id": "CONT", "kind": "continue", "T_detour": 23.0, "dT": 0.0, "J": 3.30, "J2": 3.38}],
 "chosen": "REV:17:g3", "J2_continue": 3.38, "epsilon_J": 0.02,
 "rules_fired": [{"rule": "resolution", "uid": 9, "why": "confirmed"}],
 "contacts": [{"uid": 17, "p_tilde": 0.62, "q0": 0.55, "state": 0, "n_rev": 0,
               "resolved": false, "abandoned": false, "pending": false, "detected_g": [0],
               "missed_g": [[3, 0.7]], "fallbacks": []}],
 "timing": {"compute_s": 0.61, "allowance_s": 2.0, "over_allowance": false, "lead_left_s": 3.3},
 "request": {"id": "r2-0007", "change_point": 408.6, "n_samples": 912,
             "t_first": 408.6, "t_last": 864.1},
 "labels": ["model-conditional", "planner-placeholders", "score-conditional"]}
```

`timing` says how long the policy took and what that left:
- `compute_s`: wall seconds of the decision, measured by the node; null when the executor is
  driven without a clock (the gate, the dev loop);
- `allowance_s`: `t_decide_s`, the time the trigger reserves for deciding; `over_allowance` is
  `compute_s > allowance_s`. In a simulation slower than real time the comparison is
  conservative (path time then advances by less than the wall time spent);
- `lead_left_s`: path seconds between Control's earliest allowed change point, aged by
  `compute_s`, and the change point. A REPLACE or STOP is sent only when it is at least
  `latency_margin_s`; otherwise the epoch is `missed_lead` (DESIGN §5.2 rule 2).

`pending.source` says how the evidence horizon was obtained (MATH §11.3): `stamp`,
`stamp_before_history`, `t_lat:no_stamp`, `t_lat:no_clock` or `t_lat:clock_mismatch`.
`labels` say what the numbers are conditional on (`CONTRACT_CONSUMED.md`, last table).
`action` is `CONTINUE`, `REPLACE` or `STOP` (the mission stop of MATH §12).

Required keys: `schema`, `seq`, `condition`, `tau`, `epoch`, `inputs`, `budget`, `chosen`,
`request`.
- `request` is null for CONTINUE and for an epoch whose `outcome` is `missed_lead`,
  `stale_belief` or `request_invalid` (the request failed a pre-send check and was never sent:
  the run is `failed`, DESIGN §5.2 rule 2).
- `J` and `J2` are the full-horizon values of MATH §10–§11, in expected confirmed targets (κ, λ
  applied). `dT` is the extra time charged to the revisit cap.
- The rest is optional per condition: C1 and C2 publish none; C3 publishes one, with its appended
  schedule as `candidates`.

### 5.2 `blueboat_replanner_status/1`

```json
{"schema": "blueboat_replanner_status/1", "state": "running", "reason": "",
 "mission": "beach_2026_10_12", "condition": "C4", "run": 2, "tau": 398.0,
 "active_id": "r2-0006", "mirror_ids": ["r2-0000", "r2-0006"],
 "last_request": {"id": "r2-0006", "accepted": true, "reason": "ok", "message": "…"},
 "counts": {"decisions": 7, "requests": 3, "accepted": 3, "refused": 0},
 "inputs": {"grid_ok": true, "grid_seq_received": 412, "grid_note": null, "grid_age_s": 0.6,
            "progress_age_s": 0.05, "odom_age_s": 0.2, "horizon_source": "stamp",
            "detector_calibrated": false},
 "budget": {"B": 2210.0, "elapsed": 398.0}, "output_dir": "/…/Replanner_data/…",
 "wall_time": 1793760000.2}
```

`inputs.grid_ok` is false while no usable grid is held. `grid_seq_received` then says whether
grids arrive at all, and `grid_note` why the last one is not usable (for example
"odom_to_map_valid is false: Belief's alignment has not converged yet").
`counts` also holds `belief_restarts` and `transform_jumps`.

**`state` enum** (DESIGN §7): `idle`, `waiting`, `starting`, `running`, `deciding`,
`requesting`, `disarmed`, `stop_requested`, `stop_accepted`, `stopped_holding`, `stop_rejected`,
`done`, `run_ended`, `failed`.
- Only `stopped_holding` asserts that the boat holds.
- `stop_rejected` carries `"stop_in_effect": false` and the message "STOP NOT IN EFFECT — the boat
  continues the previous path; use E-STOP, manual target or run stop".
- `disarmed` carries `"rearm_required": true` (an explicit `~/start`).
- `reason` is free text, plus a code:
  - `contract_error:<field>`;
  - `request_refused:<code>`;
  - `foreign_update`;
  - `answer_lost`;
  - `mission_error:<what>`;
  - `operator_override`;
  - `estop`;
  - `run_cleared`;
  - `missed_lead`.

## 6. Files

| Path | Format | Read / write | Online? |
|---|---|---|---|
| mission file (parameter `mission_file`) | YAML, brief §6.4: `mission: {name, condition, areas, config}`; unknown keys raise; `condition` one of C1, C2, C3, C4, C4_iso, C4_rand, C4_depth1 (C4_live and C4_degraded are dev-loop only and refused) | read | yes |
| `config/replanner.yaml` (installed share; parameter `config_file`) | YAML, every default with its category (`MATH.md` §15) | read | yes |
| `config/c2_tuned.yaml` (parameter `c2_tuned_file`, default the installed share) | YAML `c2_tuned: {spacing_factor, pass1_direction, interlace}`, the C2 freeze (`C2_TUNING_PROTOCOL.md` §6). Applied to every mission over `config_file`; a mission setting a frozen key to another value is refused; a missing file refuses `~/start` | read, at every `~/start` | yes |
| Belief configuration (parameter `belief_config_path`, optional) | YAML, format [OPEN] O-6 | read, cross-check only | yes (optional) |
| `<root>/data/Replanner_data/<YYYY_MM_DD-HH_MM_SS>-<mission>-<condition>/` | directory, `O_EXCL`, `-2`, `-3` … | write-once | yes |
| ↳ `manifest.json` | effective config, mission, the C2 freeze applied (`c2_tuned`: file, sha256, settings), datum, transform at start, versions (module git rev if available, python, numpy), contract version, Control schema names, B, spacing record | write once | yes |
| ↳ `mission.yaml` | copy | write once | yes |
| ↳ `plans/<k>-<id>.yaml` | `blueboat_trajectory/1`, **frame M**, `geo_anchor` = datum (θ 0), `loop: false` | write once each | yes |
| ↳ `belief.json` | `blueboat_replanner_belief/1`: the grid the first path was planned on (datum, transform, classes, Belief's area, the model flags of `model_params_json`) | write once | yes |
| ↳ `decisions.jsonl`, `requests.jsonl`, `status.jsonl`, `events.jsonl` | one JSON per line. A `requests.jsonl` line: `id`, `kind`, `change_point`, `n`, `sent` and, once answered, `answer` and `answered` (its arrival time). `events.jsonl` holds Control's events and this module's own (`blueboat_replanner_event/1`: `belief_restart`, `transform_jump`) | append | yes |
| ↳ `end.json` | `blueboat_replanner_end/1`: final state, reason, counts, end τ and wall time, and `decision_time_s` (`n`, `mean`, `max`, `allowance_s`, `over_allowance` = how many decisions took longer than the allowance; null when nothing was measured); written when the run reaches `done`, `failed`, `run_ended` or `stopped_holding`, else at shutdown | write once | yes |
| world `metadata.yaml` (`blueboat_world_meta/1`) | read-only (E-36) | read | **no — `eval/` only** |
| `eval/` results: `<root>/data/Replanner_eval/<name>/results.jsonl`, `summary.json`, `manifest.json` | JSONL (resumable) | write | no |

The data root is resolved as in the file-interfaces skill: `data_dir` → `$BLUEBOAT_DATA_DIR` →
parent of the first `$COLCON_PREFIX_PATH` entry → cwd.

## 7. Evaluation-only interfaces (`eval/`, never in the online node)

| Name | Type | Direction | Note |
|---|---|---|---|
| `/sss_ai/detections` | `vision_msgs/Detection2DArray`, Perception's fill convention. **One array per seabed image, empty ones included**, stamped with the image's newest row (Belief credits a ping as searched only between two arrays). Class: `unclassified`, or the object's type in the world file (`class_info:=true`). Image records are not published | pub (`emulator_node`) | run **instead of** Perception, never beside it (one publisher per topic) |
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | sub (`emulator_node`) | swath timing for emulated looks (brief §10) |
| world folder `metadata.yaml` | file | read | true objects for the emulator and scoring |

`emulator_node` parameters (`launch/emulator_launch.py`): `world_dir`, `seed`, `class_info`,
`calib_a`, `calib_b`, `range_m`; Belief's grid and look parameters `resolution_m`,
`sector_count`, `sector_origin_deg`, `look_timeout_s`; the sonar's mounting `sensor_depth_m`
(0.22 m) and `lever_arm_m` (0.285 m). Every default is the real stack's.

The superproject's `.claude/interface-contract.md` §4.1 carries this module's rows (applied on
2026-11-13).
