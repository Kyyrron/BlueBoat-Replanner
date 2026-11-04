# BlueBoat-Replanner — interfaces

Every topic, service and file this module reads or writes. This module defines no message type;
its JSON payloads travel on `std_msgs/String` with a `"schema"` field (CM-1). Names are absolute.
Status: implemented in `ros/replanner_node.py` and `eval/emulator_node.py`. Their wiring (names,
QoS profiles, services, schemas, records) is checked through stand-in rclpy modules (gate [24]).
They have **never run under ROS [NOT RUN]**; the QoS of both ends is verified in-machine
(`LOCAL_CHECKLIST.md` L-R4).

## 1. Online subscriptions (`replanner_node`)

| Name | Type | QoS (this end) | Other end | Frame | Timestamp use | Use | Evidence |
|---|---|---|---|---|---|---|---|
| `/belief_map/belief_grid` (provisional, C-2) | `blueboat_belief_interfaces/BeliefGrid` | RELIABLE, **TRANSIENT_LOCAL**, KEEP_LAST 1 | Belief (offers TRANSIENT_LOCAL, C-2) | M (`frame_id` "map", label only) | **ignored for freshness** (arrival time + `grid_seq`); logged | the only target evidence | contract C-2, C-3 |
| `/blueboat/path_progress` | `std_msgs/String` JSON `blueboat_path_progress/1` | RELIABLE, VOLATILE, depth 10 | `master_control` (same) | W | arrival time; `ros_time` logged | τ, state, `min_change_point`, `hold`, `active_id`, `end_reached` | E-11 |
| `/blueboat/path_update_event` | `std_msgs/String` JSON `blueboat_path_update_event/1` | RELIABLE, depth **50** | `master_control` (same) | W | logged | `cleared`, `activated`, `end_reached`, the answers' trace, `boat` pose | E-13 |
| `/blueboat/odom` | `nav_msgs/Odometry` | default, depth 10 (root §4.1) | `robot_interface` / Gazebo bridge | W | **arrival time only** (real stamps are sec 0, E-40) | decision pose for the record; fallback = mirror pose at τ | brief §6.2; E-39, E-40 |

**Never subscribed online** (enforced by the gate's source sweep):
- `/sss_ai/*`, `/side_scan_sonar/*`, `/sss_processor/*`;
- any `ground_truth`, `/sim/*`, `/replay/*`.

## 2. Online client

| Name | Type | Use | Evidence |
|---|---|---|---|
| `/blueboat/update_path` | service `blueboat_interfaces/srv/UpdatePath` | the first path and every change of the run's path. Request in frame `"world"`; ids `r<run>-<k>` | E-01, E-02 |

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
 "inputs": {"grid_seq": 412, "grid_age_s": 0.6, "progress_age_s": 0.05,
            "pose_source": "odom", "odom_age_s": 0.2, "pose_xy_psi": [12.1, 40.3, 1.571],
            "odom_to_map": [3.2, -1.0, 0.0], "transform_jump_m": 0.0},
 "budget": {"B": 2210.0, "elapsed": 398.0, "remaining": 1812.0,
            "revisit_used": 95.0, "revisit_cap": 1105.0},
 "calibration": {"cells": 5120, "clamped": 12, "fallback_no_sector": 0, "p_background_median_rel_diff": 0.01},
 "pending": {"t_lat": 10.0, "keys": 1830},
 "candidates": [
   {"id": "REV:17:g3", "kind": "revisit", "uid": 17, "folded_sector": 3,
    "bearing_deg": 105.0, "side": "port", "T_detour": 84.0, "dT": 61.0,
    "J": 3.41, "J2": 3.52, "best_a2": "CONT", "feasible": true, "admissible": true, "reason": ""},
   {"id": "CONT", "kind": "continue", "T_detour": 23.0, "dT": 0.0, "J": 3.30, "J2": 3.38}],
 "chosen": "REV:17:g3", "J2_continue": 3.38, "epsilon_J": 0.02,
 "rules_fired": [{"rule": "resolution", "uid": 9, "why": "confirmed"}],
 "contacts": [{"uid": 17, "p_tilde": 0.62, "q0": 0.55, "state": 0, "n_rev": 0,
               "resolved": false, "abandoned": false, "fallbacks": []}],
 "request": {"id": "r2-0007", "change_point": 408.6, "n_samples": 912,
             "t_first": 408.6, "t_last": 864.1},
 "labels": ["model-conditional", "planner-placeholders"]}
```

Required keys: `schema`, `seq`, `condition`, `tau`, `epoch`, `inputs`, `budget`, `chosen`,
`request`.
- `request` is null for CONTINUE and for an epoch whose `outcome` is `missed_lead` or
  `stale_belief`.
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
 "inputs": {"grid_ok": true, "grid_age_s": 0.6, "progress_age_s": 0.05, "odom_age_s": 0.2},
 "budget": {"B": 2210.0, "elapsed": 398.0}, "output_dir": "/…/Replanner_data/…",
 "wall_time": 1793760000.2}
```

**`state` enum** (DESIGN §7, closure D): `idle`, `waiting`, `starting`, `running`, `deciding`,
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
| mission file (parameter `mission_file`) | YAML, brief §6.4: `mission: {name, condition, areas, config}`; unknown keys raise | read | yes |
| `config/replanner.yaml` (installed share) | YAML, every default with its category (`MATH.md` §15) | read | yes |
| Belief configuration (parameter `belief_config_path`, optional) | YAML, format [OPEN] O-6 | read, cross-check only | yes (optional) |
| `<root>/data/Replanner_data/<YYYY_MM_DD-HH_MM_SS>-<mission>-<condition>/` | directory, `O_EXCL`, `-2`, `-3` … | write-once | yes |
| ↳ `manifest.json` | effective config, mission, datum, transform at start, versions (module git rev if available, python, numpy), contract version, Control schema names, B, spacing record | write once | yes |
| ↳ `mission.yaml` | copy | write once | yes |
| ↳ `plans/<k>-<id>.yaml` | `blueboat_trajectory/1`, **frame M**, `geo_anchor` = datum (θ 0), `loop: false` | write once each | yes |
| ↳ `decisions.jsonl`, `requests.jsonl`, `status.jsonl` | one JSON per line | append | yes |
| ↳ `end.json` | final state, reason, counts, end τ and wall time | write once | yes |
| world `metadata.yaml` (`blueboat_world_meta/1`) | read-only (E-36) | read | **no — `eval/` only** |
| `eval/` results: `<root>/data/Replanner_eval/<name>/results.jsonl`, `summary.json`, `manifest.json` | JSONL (resumable) | write | no |

The data root is resolved as in the file-interfaces skill: `data_dir` → `$BLUEBOAT_DATA_DIR` →
parent of the first `$COLCON_PREFIX_PATH` entry → cwd.

## 7. Evaluation-only interfaces (`eval/`, never in the online node)

| Name | Type | Direction | Note |
|---|---|---|---|
| emulator output | **[OPEN] O-1 / Q-9**: `/sss_ai/image_records` (`std_msgs/String`, `blueboat_perception/image_record/1`) **or** `/sss_ai/detections` (`vision_msgs/Detection2DArray`), whichever Belief consumes | pub (`emulator_node`) | run **instead of** Perception, never beside it (one publisher per topic) |
| `/sss_processor/processed` | `blueboat_interfaces/ProcessedSSSPing` | sub (`emulator_node`) | swath timing for emulated looks (brief §10) |
| world folder `metadata.yaml` | file | read | true objects for the emulator and scoring |

## 8. Lines proposed for the root `.claude/CLAUDE.md` §4 tables (Phase 5; the user applies them)

To be written in Phase 5, once the names are verified in-machine.
