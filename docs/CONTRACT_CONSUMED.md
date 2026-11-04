# Contract consumed — `belief_grid/0` (consumer side, C-9)

**Status: Phase 1 design, 2026-11-04. Nothing implemented yet.**
- "Read in" names the planned file and symbol.
- "Test" names the planned smoke-gate section.
- The single field-name table lives in `ros/adapters.py` (C-7). Renaming a field costs one line
  there.
- The core reads duck-typed objects: attributes **or** dict keys.

**Status values:**
- **[USED]** read, and required by the plan;
- **[FALLBACK]** read; a documented degradation applies when it is missing;
- **[IGNORED]** not read, or logged only.

## C-2 transport

| Item | Contract | Consumer | Status | Test |
|---|---|---|---|---|
| Topic | `/belief_map/belief_grid` | parameter `belief_topic`, default = contract | [USED] | in-machine L-B1 |
| QoS | RELIABLE, TRANSIENT_LOCAL, depth 1 | same, the constant `BELIEF_QOS` in `ros/adapters.py` | [USED] | in-machine L-B1 (`ros2 topic info -v`) |
| `QueryBelief` | optional | not used | [IGNORED] | — |

## C-3 `BeliefGrid`

| Field | Read in | Use | Status | Validation (ContractError names the field) | Test |
|---|---|---|---|---|---|
| `header.stamp` | `evidence.py` | logged only; freshness uses arrival time + `grid_seq` | [IGNORED] (logged) | — | [8] |
| `header.frame_id` | — | label only (no TF in the project) | [IGNORED] | — | — |
| `contract_version` | `evidence.validate` | must equal `"belief_grid/0"` | [USED] | equality | [8] |
| `grid_seq` | `evidence.py` | drop repeats and out-of-order snapshots | [USED] | uint32 | [8] |
| `width`, `height` | `evidence.py` | array shapes | [USED] | > 0; every per-cell array has length width·height | [8] |
| `resolution` | `evidence.py`, `looks.py` | cell size Δ | [USED] | > 0, finite | [8] |
| `origin_x`, `origin_y` | `evidence.py`, `looks.py` | lower-left corner of cell (0, 0) | [USED] | finite | [8] |
| `datum_lat`, `datum_lon` | `geo.py` | WGS-84 → M for areas and the archive | [USED] | finite, \|lat\| < 90 | [3], [8] |
| `odom_to_map_x/_y/_yaw` | `geo.py` | W ↔ M (general form, rotation allowed) | [USED] | finite | [3] round trip |
| `odom_to_map_sigma_m` | `evidence.py` | logged; recorded in the decision `inputs` | [IGNORED] (logged) | NaN allowed | — |
| `odom_to_map_valid` | `evidence.py`, node | no plan while false | [USED] | bool | [8] |
| `aspect_sector_count` | `model.py`, `looks.py` | S; folding needs S even, else directed [FALLBACK] | [USED] | ≥ 1 | [6] |
| `aspect_sector_origin_deg` | `model.py`, `looks.py` | sector origin o | [USED] | finite | [6] boundary case |
| `class_names` | `model.py`, `value.py` | class index; planner parameters looked up by name | [USED] | non-empty, unique | [8] |
| `prior_density_per_ha` | `value.py` | p₀ (MATH §9) | [USED] | ≥ 0, finite | [9] |
| `model_params_json` | — | logged into the manifest; no key depended on | [IGNORED] (logged) | JSON with `schema` 1 if non-empty | — |
| `p_background` | `value.py` | consistency check only: with A ≡ 1, in reachable cells, q_i = p₀Q/(1 − p₀ + p₀Q) for any s_k after the closure-A calibration (MATH §9.1–9.2) | [FALLBACK] (check skipped if empty) | length or empty | [9] |
| `p_target` | — | mixes background and contacts | [IGNORED] | — | — |
| `coverage` | `value.py` | Q = 1 − coverage; calibrates p_i on the class mixture (MATH §9.1) | [USED] | length; in [0, 1] or NaN; NaN ⇔ outside Belief's polygon | [9] |
| `n_looks`, `n_pings`, `n_aspects` | — | not used (diversity comes from `aspect_counts`, brief §9.2) | [IGNORED] | — | — |
| `depth_m` | `spacing.py`, `candidates.py` | altitude for revisit legs (MATH §2.3) | [USED] | length; NaN allowed | [2] |
| `last_intensity_db`, `max_confidence` | — | — | [IGNORED] | — | — |
| `aspect_counts` | `value.py` | seen folded sectors S_i | [USED] | length width·height·S; index `cell·S + s` | [9] |
| `aspect_log_miss` | — | optional per-sector ln Q; a possible future refinement of p_eff | [IGNORED] | length 0 or width·height·S | — |
| `contacts` | `value.py`, `candidates.py` | contact term | [USED] | uids unique | [7], [8] |

## C-4 `BeliefContact`

| Field | Use | Status | Validation | Test |
|---|---|---|---|---|
| `uid` | identity, revisit counter, abandonment | [USED] | unique within the snapshot; never reused | [8], [12] |
| `x`, `y` | contact mean m_c (M) | [USED] | finite | [10] |
| `covariance` | σ_along of the revisit leg; 3σ exclusion zone of the background | [USED] | 4 finite values, symmetric (tol 1e-9·scale), PSD | [8], [10] |
| `p_exists` | fallback for q₀ (MATH §7) | [FALLBACK] | in [0, 1] | [7] |
| `state` | 0 candidate, 1 confirmed (value 0) | [USED] | ∈ {0, 1} | [7], [13] |
| `class_probabilities` | P(k \| c) | [USED] (empty → uniform [FALLBACK]) | length = len(`class_names`) or 0; finite ≥ 0; renormalised if the sum deviates by > 1e-6 (logged) | [7] |
| `sectors_detected` | D | [USED] | sorted, unique, < S | [7], [8] |
| `sectors_missed` | Mi | [USED] | sorted, unique, < S | [7], [8] |
| `n_looks_detected` | — | [IGNORED] (logged) | — | — |
| `first_confidence` | q₀ | [FALLBACK] (NaN → p̃ = `p_exists`) | in [0, 1] or NaN | [7] both paths |
| `first_sector` | k₁ | [FALLBACK] (255 → averaged over D_f) | < S or 255 | [7] both paths |
| `max_confidence` | — | [IGNORED] (logged) | — | — |
| `shadow_known`, `has_shadow` | Belief's `state` already includes shadow confirmation (brief §9.2) | [IGNORED] (logged) | — | — |
| `created`, `updated` | — | [IGNORED] (logged) | — | — |
| optional `look_bearings_deg`, `look_ranges_m`, `detection_ids` | — | [IGNORED] | — | — |

Test sections refer to the planned smoke gate:
- [2] spacing;
- [3] geo and frames;
- [6] sensor model;
- [7] contact posterior;
- [8] contract validation;
- [9] background;
- [10] candidates;
- [12] rules;
- [13] C3.

The section numbers are fixed in Phase 2.
