# Evidence register

Every project-dependent statement the design will rely on, with its source and status label
(brief §16.2). Revisions: Control `1fadbb2`, MCS `1d5cb36`, SSS-Sim `86f8e29`, SSS `f85876b`,
Perception `8c28025`, superproject `bc18142` (all clean, inspected 2026-11-04). Belief: **not
present** (contract `belief_grid/0` only). Line numbers drift; the symbol is authoritative.

Labels: [DECIDED] user decision · [CODE] checked-out implementation · [DOC] project document ·
[TEST] automated test (command + result) · [DEMO] sim / hardware run · [ASSUMPTION] · [OPEN].

## Control — the path-change mechanism

| Id | Statement | Label | Source |
|---|---|---|---|
| E-01 | Mid-run path change = service `/blueboat/update_path` (`blueboat_interfaces/srv/UpdatePath`), served by `master_control` in every launch | [CODE] | `master_control.py` `_init_path_update` l.1494–1537; `blueboat_interfaces/srv/UpdatePath.srv` |
| E-02 | Request `id, change_point, frame="world", t[], x[], y[], yaw[]`; response `accepted, reason, message, active_id, tau, min_change_point` | [CODE] | `UpdatePath.srv` |
| E-03 | Only `path_source=replanner` runs accept updates; others → `mission_owned` | [CODE] | `path_update.py` `PathUpdater.gate` l.414–433 |
| E-04 | Accepted update replaces everything from c on; later-starting earlier updates discarded | [CODE] | `PathPlan.splice` l.243–246 |
| E-05 | First path: c = NaN or t[0]; first sample within 0.20 m of `hold`; τ := t[0] | [CODE] | `handle` l.484–490, 517–533 |
| E-06 | Lead `c ≥ τ + max(path_time + 2·dt, gov_Lmax/U_d)` (2nd term iff U_d ≥ 0.05 m/s) | [CODE] | `lead_min` l.392–407; `handle` l.502–507 |
| E-07 | Continuity at c: 0.20 m, 10° (heading skipped where current path stationary) | [CODE] | `handle` l.517–539; `Config` l.119–128 |
| E-08 | Request validity: ≥ 2 and ≤ 20 000 samples, finite, t ≥ 0 and < 1e6, steps ≥ 1 ms, speed ≤ 2.0 m/s, yaw within 45° of motion | [CODE] | `validate_samples` l.261–343 |
| E-09 | A refusal changes nothing; refused ids reusable, accepted ids not | [CODE] [TEST] | `_refuse` l.549–558; `test/test_path_update.py` → **102 passed, 0 failed** (run here 2026-11-04, `PYTHONDONTWRITEBYTECODE=1 python3 test/test_path_update.py`) |
| E-10 | Path evaluation: clamp outside, linear x/y, short-way yaw (= `blueboat_trajectory/1` rule) | [CODE] [DOC] | `Segment` → `YamlTrajectory.from_points`; interface §3 |
| E-11 | Progress `/blueboat/path_progress` JSON `blueboat_path_progress/1`, reliable volatile depth 10, 10 Hz + on change; fields incl. `state`, `tau`, `U_d`, `lead_min`, `min_change_point`, `following_id`, `active_id`, `t_end`, `end_reached`, `hold` | [CODE] | `PathUpdater.progress` l.632–670; `master_control.py` l.1524, 1529 |
| E-12 | Progress carries **no boat pose** (only `hold` while awaiting); events carry `boat {x, y, psi}` | [CODE] | `progress` l.632–670; `_event` l.672ff |
| E-13 | Events `/blueboat/path_update_event`, reliable depth 50: `request_accepted`, `request_refused`, `activated`, `end_reached`, `cleared` | [CODE] | `tick` l.572–597; interface §8 |
| E-14 | τ never goes back; advances ≤ authored speed; freezes at 3 m along-track gap | [DOC] [CODE] | interface §1; Control src/CLAUDE.md §4 governor |
| E-15 | Replanner run never calls `/path_request` nor reads the session file → MCS watcher cannot change it | [CODE] | `master_control.py` l.564–568, `_replanner_reference_step` l.1646–1670 |
| E-16 | Operator wins: E-STOP, run stop, motor gate, manual target, arrows refuse updates and pause τ; release resumes loaded path | [CODE] [DOC] | `gate`; interface §9; Control N10 |
| E-17 | End of path: hold last pose, `end_reached`; append allowed with floor-only lead, heading free | [CODE] [DOC] | `tick` l.587–594; interface §10.4.7 |
| E-18 | Run boundaries (start/stop, launch) clear plan and ids; new run awaits first path | [CODE] | `reset` l.366–385; `master_control.py` l.1430, 1457 |
| E-19 | Same interface in sim and real; sim launch: `Sim_launch.py path_source:=replanner` | [CODE] [DOC] | `Sim_launch.py` l.14, 88; `BlueBoat_launch.py` l.13, 62; interface §10.3 |
| E-20 | Built-world sims (`full_mission_launch.py`) cannot run replanner-owned runs (no `path_source`) | [CODE] [DOC] | SSS-Sim `launch/full_mission_launch.py` l.163–193, 305–323; Control `docs/REQUESTS.md` R-3 |
| E-21 | No MCS Replanner mode; real replanner sessions via MCS *Extra args* `path_source:=replanner` | [CODE] [DOC] | MCS `git grep` no hit; `run_session.py` l.185–190; Control R-1 |
| E-22 | Control does not check turn rate / radius / areas; feasibility is the Replanner's | [DOC] | interface §10.4.6; `REQUESTS.md` R-2 |
| E-23 | Speed guidance: ≥ 0.28 m/s; sim ≤ 0.45 m/s; sim top ~0.78 m/s; field turn radii 1.5–6 m | [DOC] | interface §10.4.6; Control report §5.4, P01 |
| E-24 | Gazebo: 13/13 per controller; boat–target median PID 0.89, LoS 1.16, MPC 0.04 m | [DOC] (not re-run) | `docs/PATH_UPDATE_REPORT.md` |
| E-25 | Never tested on water | [DOC] | `PATH_UPDATE_REPORT.md` "Assumed"; Control TODO |
| E-26 | `path_time`: PID/LoS 0.05 s; MPC 6.0 s sim / 2.5 s real | [DOC] [CODE] | Control src/CLAUDE.md §4 table |

## Trajectory, geodesy, worlds

| Id | Statement | Label | Source |
|---|---|---|---|
| E-30 | `blueboat_trajectory/1`: key `format`; N×4 `points` `[t, x, y, yaw]`; optional `name`, `loop`; other keys ignored by the loader | [CODE] | Control `yaml_trajectory.py` l.23–47 |
| E-31 | Loader does not validate increasing t (doc says required) | [CODE] vs [DOC] | `yaml_trajectory.py`; MCS `docs/05_trajectory_format.md` l.28 |
| E-32 | `geo_anchor {lat0, lon0, theta_deg}` (θ = 0 today) | [DOC] [CODE] | `05_trajectory_format.md` l.53–60; MCS `io_yaml.save_mission` l.115–120 |
| E-33 | MCS geodesy: `EARTH_RADIUS_M = 6378137.0`; `latlon_to_local_en`, `local_en_to_latlon` (equirectangular, cos lat0) | [CODE] | MCS `mcs/core/geo.py` l.38–52 |
| E-34 | MCS `GeoFit` translation-only, `EN = world + t` | [CODE] | `geo.py` l.78–117 |
| E-35 | No WGS-84 survey-area polygon in any MCS file; world `limits.corners_gps` is the only WGS-84 rectangle | [CODE] | MCS `designer/patterns.py` l.9–43; SSS-Sim `world_folder.py` l.411–417 |
| E-36 | World `metadata.yaml` objects: `object_id, type, x, y, lat, lon, yaw (rad ENU, long axis), yaw_deg, …` | [CODE] | SSS-Sim `world_folder._write_metadata` l.428–444 |
| E-37 | World (0, 0) = spawn = `geo_anchor` | [CODE] [DOC] | `world_folder.py` l.407–410; SSS-Sim CLAUDE.md §2 |
| E-38 | CATALOG: 29 types; no verbatim block/pipe/cage/tyre/plate; no plate | [CODE] | SSS-Sim `worldgen/objects.py` l.208–370 |
| E-39 | Sim `/blueboat/odom`: Gazebo, 20 Hz, world ENU | [DOC] | SSS-Sim `docs/sea_state_model.md` l.398–399 |
| E-40 | Real `/blueboat/odom`: values local ENU absolute yaw; **stamp sec 0** (mavros `use_sim_time`, no `/clock`) | [DOC] | SSS `.claude/TODO.md` l.150–166; Control report l.164 |
| E-41 | No measured turn rate / settle time anywhere | [DOC] (absence) | Control report P01; SSS-Sim `docs/sea_state_validation.md` l.278–291 |
| E-42 | Field route mean speeds 0.32–0.41 m/s (Sin route) | [CODE-data] | Control `controller_comparison/metrics_all.csv` |
| E-43 | Perception look directions: port = ψ + π/2, starboard = ψ − π/2 (consistent with contract C-5) | [CODE] | Perception `core/geometry.py:90` `look_azimuths` |
| E-44 | Sonar mount: 0.22 m deep, ±0.285 m lateral, −0.345 m fore/aft (sim); range 20 m, 1200 bins | [CODE] [DOC] | SSS-Sim `config/default_sonar.yaml` l.36–41; root §4.3 |
| E-45 | No document gives a usable-swath number | [DOC] (absence) | SSS-Sim `configuration_guide.md` l.564 |

## Project rules and decisions

| Id | Statement | Label | Source |
|---|---|---|---|
| E-50 | CM-9 text: "the Replanner reads only Belief" | [DOC] | root `.claude/CLAUDE.md` l.476–480 |
| E-51 | Synthesis §5.1 ⚠: Belief carries survey area and anchor, "so the Replanner needs no other input" | [DOC] | `project_synthesis.md` l.224–226 |
| E-52 | Belief reads Perception's `/sss_ai/image_records` + mission area/anchor | [DOC] | root §1; synthesis l.216 |
| E-53 | Three rules (resolution, budget, abandonment) are the scientific content | [DOC] | synthesis §8.1 l.507–521 |
| E-54 | Baseline two-pass orthogonal, tuned seriously; ladder 1–4 incl. "append-on-completion" | [DOC] | synthesis §7.3 l.453–469; §8.6 l.585 |
| E-55 | 20 m range every condition; speed 1.0 m/s nominal; spacing ~0.8 × usable swath | [DOC] | synthesis §8.5 l.566–579 |
| E-56 | Surrogate probability must be fitted from Tier-1 Gazebo runs, never invented | [DOC] | synthesis §6.2 ⚠ l.371–377 |
| E-57 | Investigation not advocacy; model-conditional results | [DOC] | synthesis l.103–105, §4.1; CM-13 |
| E-58 | Sessions never commit / push; Control and MCS add-only | [DOC] | root §2.1, §3 |
| E-60 | Strategy: contact-centric aspect-aware value per unit time + aspect-resolved search; short lookahead; no POMDP / learning | [DECIDED] | brief §5 "Strategy" |
| E-61 | κ = 1, λ = 0.5, both swept | [DECIDED] | brief §5 Q1 |
| E-62 | Equal mission time = tuned two-pass duration; anytime curves | [DECIDED] | brief §5 Q2 |
| E-63 | Own orientation-resolved residual from Belief's published fields | [DECIDED] | brief §5 Q4 |
| E-64 | Detection emulator stands in for D3, labelled | [DECIDED] | brief §5 Q5 |
| E-65 | The Replanner generates the coverage for every condition | [DECIDED] | brief §5 Q6 |
| E-66 | r* = 10 m placeholder | [DECIDED] | brief §5 Q7 |
| E-67 | One range setting, 20 m | [DECIDED] | brief §5 Q8 |
| E-68 | Mission areas defined at launch in a WGS-84 polygon file, margin | [DECIDED] (but see D-A1) | brief §5 Q9 |
| E-69 | Decisions at segment ends, executed within the run | [DECIDED] | brief §5 Q10 |

## User decisions of 2026-11-04 (answers to the Phase 0 questions)

| Id | Statement | Label | Source |
|---|---|---|---|
| E-70 | Survey speed 0.5 m/s, every condition | [DECIDED] | user, Q-1 |
| E-71 | Swath = one side; line spacing is a function of range | [DECIDED] | user, Q-2 |
| E-72 | The brief overrides other sources for this module's inputs (mission file, `/blueboat/odom`, BeliefGrid, progress) | [DECIDED] | user, Q-3/4/5 |
| E-73 | Append-on-completion (C3) built first; modular policies that allow live interruption later | [DECIDED] | user, Q-7 |
| E-74 | C4_rand via a pre-drawn bearing table | [DECIDED] | user, Q-8 |
| E-75 | No context pack exists. Rules come from the root CLAUDE.md and the synthesis; v1 comparisons are [NOT AVAILABLE] | [DECIDED] | user, 2026-11-04 |

## Literature read for the spacing function (2026-11-04)

| Id | Statement | Label | Source |
|---|---|---|---|
| L-01 | "Field Units must operate the sensor with a towfish height above the bottom (altitude) of 8 percent to 20 percent of the range scale in use"; max range scale 100 m; a reduced effective range ⇒ reduced claimed swath coverage | [DOC] external, full text of §8.3 read | NOAA OCS, *HSSD* Version 2025.0.01, §8.3 + "Data Gaps", https://nauticalcharts.noaa.gov/publications/documents/HSSD_2025-0-01.pdf |
| L-02 | Effective range = Pd(range) curve ∩ admissibility threshold; nadir gap r_min covered by paired tracks; eq. (1) r_max ≥ 3·r_min | [DOC] external, §III-A read | Yordanova & Gips, IEEE RA-L 5(3) 2020, DOI 10.1109/LRA.2020.3003886; arXiv 2006.12896v1 |
| L-04 | Paull et al. 2013 eq. 22–24: acute-angle dependence; parallel = dependent (max), perpendicular = independent, linear in between ("it is assumed") | [DOC] external, full text (user PDF) | MATH §16 |
| L-05 | Bourgault et al. 2006 eq. 4–9: miss update; static target, greedy .59 vs horizon .77 | [DOC] external, full text | MATH §16 |
| L-06 | Hollinger et al. 2013 Theorem 2: adaptivity gap e/(e−1) for monotone submodular variance reduction with observation-cost dominance; experiments "only marginally better" | [DOC] external, full text | MATH §16 |
| L-07 | Sethuraman et al. 2024 Table II: informed 0.95/0.93 vs random 0.92/0.90 (simulation); the 30-min claim is uncited | [DOC] external, full text | MATH §16 |
| L-08 | Fawcett et al. 2010: a second look improves classification when training is representative; no result vs the angle between looks | [DOC] external, full text | MATH §16 |
| L-09 | Wiig et al. 2012: confidence filter < 20 %, lead-in/out, turning radius, sharp-turn penalty < 90°, GA-TSP; HUGIN trial 2012 | [DOC] external, full text | MATH §16 |
| L-03 | Vendor and secondary practice: 15/45 line plans, 65–75 % of range, 20–30 % overlap, "12.5 × towfish height" | not used as evidence (not read in full) | search results 2026-11-04 (`MATH.md` §16) |

## Phase 1 closure (2026-11-04)

| Id | Statement | Label | Source |
|---|---|---|---|
| E-76 | Control documents that the first path "may have any yaw, since the boat turns in place"; the heading check is skipped at a stationary hold | [DOC] | Control `docs/PATH_UPDATE_INTERFACE.md` §5, §10.4 item 2 |
| E-77 | Before the first path, the boat holds with the manual-target keep-location law (`solve_LoS(…, keep_location=True)` on the hold point) | [CODE] | Control `master_control.py` l.700–706 (`_awaiting_hold_active`) |
| E-78 | No measurement of in-place turning time or radius exists | [DOC] (absence) | E-41 |
| E-79 | The brief's p_eff = 1 − Q^{1/n} reproduces Belief's residual only if s_k = 1 for every class of positive prior | derivation | `PHASE1_CLOSURE.md` §A |
| E-80 | Union-of-looks value = branch-weighted sequential value, for a fixed second action (contacts and background) | derivation + numerical check | `PHASE1_CLOSURE.md` §C; MATH §11.1 |
| E-81 | Brief §3: "Usable ground swath ≈ 6 m … √(20² − depth²)" — no grazing term | [DOC] | `REPLANNER_BRIEF.md` §3 |

## Phases 3–4 (2026-11-04): dev loop and offline checks

Every E-82…E-86 entry is **reference-belief-conditional, preliminary** or an offline check; none
is E4.

| Id | Statement | Label | Source |
|---|---|---|---|
| E-82 | Dev-loop C2 freeze: f = 1.0, long axis, interlace auto (tie rule within 1 SE over f = 0.8); B(C2) = 2111 s; interlace has no effect | [TEST] dev loop | `C2_TUNING_RESULT.md`; `config/c2_tuned.yaml` sha256 `abff9534…` |
| E-83 | At f = 1.0 (n = 30), no C4 variant differs from C2 by more than the noise (C4 +0.27, CI [−0.32, +0.86]); C1 is clearly worse (−0.97, CI [−1.43, −0.50]) | [TEST] dev loop | `DEVLOOP.md` §5.1 |
| E-84 | C4 = C4_rand (+0.00, CI [−0.41, +0.41]) and C4 ≈ C4_iso: no effect of the aspect-aware bearing choice in the dev loop, where every segment is a pass | [TEST] dev loop | `DEVLOOP.md` §5.2 |
| E-85 | At 50 % of the budget every C4 variant leads C2 by ≈ +0.7, CI excluding 0 | [TEST] dev loop | `DEVLOOP.md` §5.1 |
| E-86 | C4 decision time up to 8.1 s wall under 4-way contention, vs the `t_decide_s` allowance of 2.0 s | [TEST] dev loop | `DEVLOOP.md` §5.2 item 8, §5.4 |
| E-87 | The node, the emulator node and both launch files run through stand-in rclpy modules: graph, QoS profiles, a C3 run with 2 accepted requests, records, typed parameters | [TEST] stand-in, not ROS | gate [24] |

## Assumptions carried (until a question or a measurement settles them)

| Id | Assumption | Consequence | How tested / varied |
|---|---|---|---|
| A-01 | Planner aspect windows, class scales, PD(range) are placeholders (brief §8.2) | Aspect-aware value depends on them | Aspect-agnostic variant; degraded ablation; E3 replaces them |
| A-02 | Turn radius 3 m, settle 2 s (pending Q-6) | Time accounting | L-K1/L-K2 measurement |
| A-03 | r_near 6 m, r_far √(20² − h²) (brief §3; NOAA grazing option off) | Spacing, budget | L-S1 measurement; grazing option in the C2 protocol sensitivity |
| A-04 | Contract `belief_grid/0` names and semantics | Adapter | `CONTRACT_CONSUMED.md`; in-machine diff |
| A-05 | Emulator confidences calibrated against the emulator's own world | Contact value | Reliability diagram in `eval/` |
| A-06 | `coverage` is a class-mixture residual (MATH §9.1) | Background calibration | L-B6 |
| A-07 | Belief latency t_lat = 10 s | Pending-look window; lookahead's information assumption | L-B5 |
| A-08 | Hysteresis ε_J = 0.02 expected confirmed targets | Switching | swept |
| A-09 | A pass = one plan segment (reference belief and dev-loop emulator) | Confirmation rule, hence every dev-loop score | L-B6 |
| A-10 | Decisions take no mission time in the dev loop (`t_decide_s` 2.0 s is only the trigger allowance) | Missed leads live | L-G4 |

## Open

Q-1 … Q-19 in `QUESTIONS_PHASE0.md`; ROS-runtime checks in `LOCAL_CHECKLIST.md`.
