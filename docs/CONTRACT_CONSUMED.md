# Contract consumed — `belief_grid/0` (consumer side, C-9)

**Status: implemented and checked against BlueBoat-Belief's own core, 2026-11-12.** Read from a
live ROS graph that evening: Belief's installed node on an
isolated domain, fed with made-up sonar and navigation (`LOCAL_CHECKLIST.md` L-X, L-B1,
L-R4, L-R5). Every live grid passed the validator with no fallback; an unaligned one was
refused. Not yet read in Gazebo or with a real detector.

- The producer's text is `BlueBoat-Belief/blueboat_belief/docs/topics.md` (the contract; its
  §10 conformance and §11 notes for a consumer) and `MATH.md` (model). Where this file and those
  disagree about what a field means, Belief's text wins and this file is corrected.
- "Read in" names the file and symbol. "Gate" names the section of `test/smoke_test.py`.
- The single field-name table lives in `ros/adapters.py` (C-7). Belief's names are the contract's.
- The core reads duck-typed objects: a ROS message (attributes) **or** Belief's `to_grid_dict`
  output (keys, numpy scalars, a float header stamp). Both forms are tested ([11], [20]).

**Status values:**
- **[USED]** read, and required by the plan;
- **[FALLBACK]** read; a documented degradation applies when it is missing, and it is recorded;
- **[IGNORED]** not read, or logged only.

## What changed on 2026-11-12, and why

The Replanner at `c53fe2a` read several fields by their name and not by their meaning. Each row
is a statement about Belief, verified in Belief's code and documents.

| Field | Read before | Read now | Why (Belief's meaning) |
|---|---|---|---|
| `aspect_counts` | the sectors a cell was seen from | fallback only | it counts every look, also one with no detector coverage. With a dummy detector, or none, every look is counted and nothing was searched |
| `aspect_log_miss` | ignored | **the searched sectors**, and the strength of a contact's misses | `< 0` ⟺ a detector searched that sector with PD > 0; the value is `ln(1 − max covered PD)`. It is the set Belief's own ρ is taken against |
| `sectors_missed` | a contact's misses | kept, with the cell record beside it | Belief writes a miss only when the look closes (a sector change, or 20 s later). The cell record has it at once, and has the misses made before the contact existed |
| `header.stamp` | logged | **the evidence horizon** | every ping at or before it is in the grid, none after. It lags the boat by one to two image strides |
| `grid_seq` | a decrease was dropped | a decrease is **a Belief restart** | a restarted Belief starts again at 1 |
| `model_params_json` | ignored | `detector_calibrated`, `rho_model`, `negative_evidence`, `coverage_source`, `look_timeout_s`, `turn_gating`, `pd_model` | they say what the scores and the coverage are |
| `first_confidence` | a calibrated probability | the same use, **labelled** | it is the raw score of the creating detection while `detector_calibrated` is false |
| `class_names` | looked up as aspect families | looked up **through `class_families`** | it is Belief's configured detector class list; an unknown detector class arrives as `other` |
| `area_x`, `area_y` | not read | the mission polygon is checked against them | outside Belief's area there is no evidence |

## C-2 transport

| Item | Contract | Consumer | Status | Check |
|---|---|---|---|---|
| Topic | `/belief_map/belief_grid` | parameter `belief_topic`, default = contract | [USED] | seen live 2026-11-12 (L-R4) |
| QoS | RELIABLE, TRANSIENT_LOCAL, KEEP_LAST 1 | the same (`ros/replanner_node.py`) | [USED] | gate [24] (profile); matching seen live 2026-11-12, both ends (L-B1, L-R4) |
| Rate | 1 Hz on a steady clock | freshness by arrival time, `belief_max_age_s` = 5 s | [USED] | gate [23]; live grid ages 0.4 to 0.9 s at the decisions (L-X) |
| `QueryBelief` | optional | not used | [IGNORED] | — |

## C-3 `BeliefGrid`

| Field | Read in | Use | Status | Validation (ContractError names the field) | Gate |
|---|---|---|---|---|---|
| `header.stamp` | `ros/adapters._stamp`, `ros/executor._horizon` | the evidence horizon: converted to path time through Control's progress clock (MATH §11.3) | [USED]; fallback to `t_lat` when zero, missing or on another clock, named in the record | — | [11], [20], [23] |
| `header.frame_id` | — | label only (no TF in the project) | [IGNORED] | — | — |
| `contract_version` | `evidence.validate_grid` | must equal `"belief_grid/0"` | [USED] | equality | [11] |
| `grid_seq` | `ros/executor.on_grid` | equal = a repeat (ignored); lower = a Belief restart (accepted, counted, logged, per-contact memory reset) | [USED] | integer ≥ 0 | [11], [23] |
| `width`, `height` | `evidence.py` | array shapes | [USED] | > 0; every per-cell array has length width·height | [11] |
| `resolution` | `evidence.py`, `looks.py` | cell size Δ | [USED] | > 0, finite | [11] |
| `origin_x`, `origin_y` | `evidence.py`, `looks.py` | lower-left corner of cell (0, 0) | [USED] | finite | [11] |
| `datum_lat`, `datum_lon` | `geo.py` | WGS-84 → M for the mission area and the archive | [USED] | finite, \|lat\| < 90 | [2], [11] |
| `odom_to_map_x/_y/_yaw` | `geo.py` | W ↔ M (general form; Belief's yaw is 0 by construction) | [USED] | finite | [2] round trip |
| `odom_to_map_sigma_m` | `evidence.py` | recorded in the decision `inputs` | [IGNORED] (logged) | NaN allowed | [23] |
| `odom_to_map_valid` | `evidence.py`, executor | no plan while false | [USED] | bool | [11], [20] (a real unaligned grid is refused) |
| `aspect_sector_count` | `model.py`, `looks.py` | S; folding needs S even, else directed [FALLBACK] | [USED] | ≥ 1 | [11] |
| `aspect_sector_origin_deg` | `model.py`, `looks.py` | sector origin o | [USED] | finite | [11] (edge bearings) |
| `class_names` | `model.py`, `value.py` | class index; each name is mapped to an aspect family through `class_families`, else isotropic | [USED]; unmapped names [FALLBACK], recorded | non-empty, unique | [11], [17] |
| `prior_density_per_ha` | `value.py` | p₀ (MATH §9). Belief's default is 5 /ha, a placeholder | [USED] | ≥ 0, finite | [16] |
| `model_params_json` | `evidence.parse_model_params` | the keys of the table above; written into every decision record, `belief.json` and every dev-loop manifest | [USED]; absent keys are None and recorded | valid JSON object when non-empty | [11], [20] |
| `p_background` | `value.build_background` | consistency value only (MATH §9.2), in every decision record | [FALLBACK] (skipped if empty) | length; same NaN mask as `coverage` | [11], [16] |
| `p_target` | — | mixes background and contacts | [IGNORED] | — | — |
| `coverage` | `value.py` | Q = 1 − coverage; calibrates p_i on the class mixture (MATH §9.1). A planning aid: it depends on the order of three or more looks, so it is never a metric for comparing conditions | [USED] | length; in [0, 1] or NaN; NaN ⇔ outside Belief's area | [11], [16] |
| `n_looks`, `n_pings`, `n_aspects` | — | geometry only | [IGNORED] | — | — |
| `depth_m` | `candidates.py` | altitude for revisit legs (MATH §2.3) | [USED] | length; NaN allowed | [13] |
| `last_intensity_db`, `max_confidence` | — | — | [IGNORED] | — | — |
| `aspect_counts` | `value.py` | searched sectors **only when `aspect_log_miss` is absent** | [FALLBACK] (`searched_from_aspect_counts`) | length width·height·S; ≥ 0 | [11], [16] |
| `aspect_log_miss` | `evidence.py` (`miss_pd`, `searched`), `value.py` | searched folded sectors S_i (MATH §9); a contact's misses and their strength at its cell (MATH §7) | [USED] | length 0 or width·height·S; ≤ 0 or NaN | [11], [16], [17], [20] |
| `area_x`, `area_y` | `evidence.mission_area_check` | the mission polygon must lie inside (tolerance 0.05 m); also every cell inside the mission polygon must have a finite `coverage` | [USED]; absent → the cell test alone, recorded | equal length ≥ 3, finite | [11], [20], [23] |
| `contacts` | `value.py`, `candidates.py` | contact term | [USED] | uids unique | [11], [17] |

## C-4 `BeliefContact`

| Field | Use | Status | Validation | Gate |
|---|---|---|---|---|
| `uid` | identity, revisit counter, abandonment. Unique within one Belief run; after a restart the counters are reset | [USED] | unique within the snapshot | [11], [23] |
| `x`, `y` | contact mean m_c (M) | [USED] | finite | [13] |
| `covariance` | σ_along of the revisit leg; 3σ exclusion zone of the background. Belief adds the navigation common-mode error once (about 1.2 m per axis with Perception's convention) | [USED] | 4 finite values, symmetric (tol 1e-9·scale), PSD | [11] |
| `p_exists` | fallback for q₀ only (MATH §7). Belief's update is sequential: a stronger later detection on the same line does not raise it | [FALLBACK] | in [0, 1] | [17] |
| `state` | 0 candidate, 1 confirmed (value 0). Belief confirms on two detected looks, not necessarily from different sectors | [USED] | ∈ {0, 1} | [13], [20] |
| `class_probabilities` | P(k \| c). Belief's Dirichlet mean over its class list | [USED] (empty → uniform [FALLBACK]) | length = len(`class_names`) or 0; finite ≥ 0; renormalised if the sum deviates by > 1e-6 | [11], [20] |
| `sectors_detected` | D | [USED] | sorted, unique, < S | [11], [20] |
| `sectors_missed` | part of Mi, beside the cell record (MATH §7). A sector can be in both sets; it then counts as detected | [USED] | sorted, unique, < S | [17], [20] |
| `n_looks_detected` | — | [IGNORED] | — | — |
| `first_confidence` | q₀, the prior P(real). **A raw score while `model_params_json.detector_calibrated` is false**: everything built on it is labelled `score-conditional` | [USED] (NaN → p̃ = `p_exists` [FALLBACK]) | in [0, 1] or NaN | [17], [20] |
| `first_sector` | k₁ | [FALLBACK] (255 → averaged over D_f) | < S or 255 | [13], [17], [20] |
| `max_confidence` | — | [IGNORED] | — | — |
| `shadow_known`, `has_shadow` | always false in Belief today (no shadow input) | [IGNORED] | — | — |
| `created`, `updated` | — | [IGNORED] | — | — |
| `look_bearings_deg`, `look_ranges_m`, `look_detected`, `detection_ids` | — (the cell record gives what the planner needs) | [IGNORED] | — | — |

## What the labels of a decision record say about the grid

| Label | When |
|---|---|
| `score-conditional` | `detector_calibrated` is not true: p̃ and every value built on it rest on a raw score |
| `belief-ungated` | `negative_evidence` is `ungated` (Belief's development mode) |
| `fold-mismatch` | Belief does not fold opposite sides (`rho_model.fold_opposite` false) while the planner does |
| `searched-from-aspect-counts` | the grid has no `aspect_log_miss` |
| `aspect-inert` | no class of the grid has an aspect family: C4 = C4_iso by construction |
| `belief-restarted` | a Belief restart happened in this run: not evaluation evidence |

## Deviations from the contract

**None on this side.** The consumer is built to `belief_grid/0` as Belief implements it. Belief's
own deviations from the original contract text are listed in Belief's `topics.md` §10
(D-C1…D-C6), and the consumer follows them: the datum is the survey area's anchor (D-C1),
`class_names` is Belief's configured list plus `other` (D-C2), `first_confidence` is raw while
uncalibrated (D-C3), `area_x` / `area_y` exist and are read (D-C4), shadows are always false
(D-C6).

Questions raised on 2026-11-04 and how they were settled:
- **Evidence topic.** Belief reads `/sss_ai/detections` (required) and `/sss_ai/image_records`
  (optional); the emulator publishes detections, one array per image.
- **Survey area.** The Replanner reads its mission file; Belief publishes its own area
  (`area_x`, `area_y`) and can make it from the same mission file
  (`blueboat-belief-make-area --from-mission`); the Replanner refuses a mission polygon outside
  Belief's area before the first path.
- **Look geometry.** Perception's per-row look directions match the contract (port = ψ + π/2).
- **Belief's configuration file.** Not read: shared values come from the message
  (`model_params_json`).
