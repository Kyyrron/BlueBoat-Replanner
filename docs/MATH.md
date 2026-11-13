# BlueBoat-Replanner — mathematics

Status: **approved by the owner on 2026-11-04 and implemented; revised on 2026-11-12 against
BlueBoat-Belief's code.** The gate checks the properties listed in each section
(`test/smoke_test.py`, §14).
- The code implements this document. It does not fill its gaps in code: an ambiguity found while
  implementing comes back here first (brief §18).
- Labels as in `DESIGN.md`. Every parameter appears in §15 with its category.
- What follows Belief's own model is marked **(Belief 2026-11-12)** and cites the Belief text
  it follows: §1 (sector rule), §5 (lever arm, turn gate), §6 (class families, range model), §7
  (misses from the cell record), §9 (searched sectors), §11.3 (evidence horizon), §12 (revisit
  count, mission stop).

## 1. Notation and units

| Symbol | Meaning | Unit |
|---|---|---|
| M, W | map frame (Belief), world frame (Control) | — |
| ψ | boat heading, ENU, CCW from East | rad |
| s ∈ {+1, −1} | side: +1 port, −1 starboard | — |
| b = ψ + s·π/2 | **directed** look bearing (contract C-5) | rad |
| S, o | sector count and sector origin, read from the message (`aspect_sector_count`, `aspect_sector_origin_deg`) | —, deg |
| k(b) = ⌊mod(round₉(deg b − o), 360) / (360/S)⌋ | directed sector of b (C-5). round₉ rounds to 1e-9°, so a bearing on a sector edge goes to the upper sector, as in Belief (`topics.md` §4 item 3) **(Belief 2026-11-12)** | — |
| ℓ | lateral lever arm of the transducers = 0.285 m (Belief `topics.md` §4 item 2) | m |
| β_k = rad(o + (k + ½)·360/S) | centre bearing of directed sector k | rad |
| g(k) = k mod (S/2) | **folded** sector (b and b + π identified); requires S even | — |
| U | survey speed = 0.5 m/s [DECIDED] | m/s |
| R | sonar range setting = 20 m [DECIDED, synthesis §8.5] | m |
| h | transducer altitude = depth − 0.22 m (sensor depth, root CLAUDE.md §4.3) | m |
| Δ | grid resolution (`resolution`, default 0.5 m) | m |
| τ | Control's path parameter | path s |
| wrap_P(a) = a − P·round(a/P) | wrap to (−P/2, P/2] | — |
| B, t_start | budget (§4) and the run's first path parameter | path s |

If S is odd, folding is undefined. The planner then uses directed sectors, logged as
[FALLBACK: directed]. The default is S = 12.

## 2. Line spacing as a function of range (coverage band only)

### 2.1 Usable one-sided coverage band

```
r_geo(R, h)   = sqrt( R² − h² )                                   geometric far limit (brief §3)
r_far_cov     = r_geo(R, h)                                        default  (grazing_floor: false)
              = sqrt( min(R, h / 0.08)² − h² )                     option   (grazing_floor: true)
r_near        = 6 m                                                [ASSUMPTION] brief §3
W(R, h)       = r_far_cov − r_near                                 one-sided usable swath (must be > 0)
```

- **Default** = the brief §3 formula ("usable ground swath ≈ 6 m … √(20² − depth²)"). It is a
  function of the range setting, as the user asked, with the swath on one side only.
- **The `grazing_floor` option** applies NOAA HSSD 2025.0.01 §8.3: towed side-scan altitude
  8–20 % of the range scale in use, and the claimed coverage reduced when the effective range is
  reduced. Applying it to a hull-mounted sonar is this module's reading, not NOAA's rule
  [ASSUMPTION]. The option is kept for the sensitivity runs of the C2 protocol and is **off** by default.
- **Effective range = detection range minus nadir gap** (Yordanova & Gips 2020, §III-A): r_far_cov
  and r_near play those two roles. The threshold here is geometric, not a measured Pd curve
  [ASSUMPTION].
- **This band is used only for line spacing.** Predicted detections use the detection band of §6.

### 2.2 Spacing and its verification

```
s0 = f · W          f = 0.8 (synthesis §8.5 "~0.8 × usable swath"; one side per the user)
```

**Lines.** The first and last lines are at the area's cross-track edges
± e, with e = margin + r_turn. Between them, n = ⌈(width − 2e)/s⌉ + 1 lines are spread evenly, so
the actual spacing is ≤ s. If width = 2e there is one line; if width < 2e the pass is infeasible
(mission error).

**Verification.** Cross-track positions x are sampled every 0.05 m over the area width. Every x
must satisfy r_near ≤ |x − y_j| ≤ r_far_cov for at least one line j.

**Fallback.** If s0 fails, bisection finds the largest valid s ∈ (0, s0] in 40 iterations. The
manifest records s0, s, and the minimum and mean multiplicity. If no s > 0 is valid (W ≤ 0, or the
area is too narrow), it is a mission error naming R, h, r_near and r_far_cov.

### 2.3 Altitude used

| When | Water depth used | Source |
|---|---|---|
| Initial coverage (all conditions, mission start) | `config.planning_depth_m` (mission config, required) | identical for every condition (CM-12) |
| Revisit legs (C3, C4) | median finite `depth_m` within 10 m of the contact, else `planning_depth_m` | contract C-3 |

### 2.4 Worked values (R = 20 m, r_near = 6 m, f = 0.8; verified spacing)

| h (m) | default: r_far_cov | W | s | grazing option: r_far_cov | W | s |
|---|---|---|---|---|---|---|
| 0.9 | 19.98 | 13.98 | 11.18 | 11.21 | 5.21 | 4.17 |
| 1.0 | 19.97 | 13.97 | 11.18 | 12.46 | 6.46 | 5.17 |
| 1.5 | 19.94 | 13.94 | 11.15 | 18.69 | 12.69 | 10.15 |
| 2.0 | 19.90 | 13.90 | 11.12 | 19.90 | 13.90 | 11.12 |
| 3.0 | 19.77 | 13.77 | 11.02 | 19.77 | 13.77 | 11.02 |

The gate reproduces this table. Along-track sampling (recorded only): a ~50 ms ping at 20 m
(root CLAUDE.md §4.3) gives U · 0.05 = 0.025 m.

## 3. Areas, coverage geometry, routing, initial manoeuvre

### 3.1 Mission areas

**Input.** The brief §6.4 `areas`, [lat, lon] polygons, converted to M (DESIGN §4).

**Validity:**
- at least 3 vertices;
- consecutive duplicates closer than 1e-6 m removed;
- simple (no edge intersections);
- orientation normalised to CCW;
- area ≥ (2e)²;
- no holes.

**Exactly one area per mission** ([OPEN] O-8): a route between disjoint areas would leave them.

**Feasible set.** F_d = { p : p inside (even-odd ray test) and dist(p, boundary) ≥ d }. Every
path centreline lies in F_margin (margin 2 m [ASSUMPTION]). The predicates are exact and
vectorised, in numpy only, with no offsetting library. Inclusion tolerance: 1e-6 m.

### 3.2 Coverage lines (one generator for C1, C2, C3, C4)

For a pass at direction θ:
- Pass 1 θ is the area's longest extent, i.e. the minimum-width direction of its convex hull.
  Pass 2 is θ + π/2. The C2 protocol may swap them.

1. **Cross-track positions** y_j are those of §2.2, measured along n = (−sin θ, cos θ).
2. **Legs.** Each line is sampled every 0.05 m. Its maximal runs inside F_{margin + r_turn} are
   legs, one or more per line for concave areas. Each leg is shortened by l_in = U · t_settle at
   both ends.
3. **Order.** Legs are flown boustrophedon. If s < 2 r_turn, lines are interlaced every k-th,
   with k = ⌈2 r_turn / s⌉, and the skipped lines are flown on the way back.
4. **Connections.** Each connection is a Dubins path (radius r_turn; LSL, RSR, LSR, RSL, RLR,
   LRL, the shortest feasible) inside F_margin. If none fits, both legs are shortened by 0.5 m and
   the connection retried, at most 10 times. The loss is recorded in the coverage check (§3.4).

The **C2 plan** is pass 1 then pass 2, joined by the shortest feasible route from the last pass-1
leg to the pass-2 leg nearest to it. The pass-2 boustrophedon starts from that leg.

### 3.3 Transits

Used for revisit transits, rejoins, recover legs and the initial manoeuvre.
- **Nodes:** the start, the goal, and every reflex vertex moved inward along its bisector to
  distance margin + r_turn from both adjacent edges.
- **Edges:** straight segments that lie in F_margin when sampled every 0.05 m.
- **Route:** Dijkstra gives the shortest polyline. Each interior node gets the heading of its
  outgoing segment, and consecutive node poses are joined by the shortest feasible Dubins path of
  radius r_turn. The start and goal keep their own headings. It is the same turning model as
  everywhere else (no fillet arcs).
- **Check:** a final sampled check in F_margin. If it fails, the candidate is infeasible. The
  route never leaves the area.

### 3.4 Coverage check (reported)

The look predictor runs over the planned C1 / C2 paths, using the coverage band (§2). The
manifest reports the fraction of area cells with at least one look in [r_near, r_far_cov] for each
pass.

### 3.5 Initial manoeuvre

Control's interface says a first path "may have any yaw, since the boat turns in place" [DOC]. No
measurement of in-place turning exists (E-41), so **this design never relies on it**. The first
path of every condition is built as follows:
- It starts at `hold` (x, y), within 0.20 m (E-05). Its heading ψ₀ is the boat's yaw: the fresh
  odometry yaw, else `hold.yaw` as Control captured it.
- It is a Dubins / transit route of radius r_turn from (hold, ψ₀) to the start pose of the first
  coverage leg, inside F_margin.
- It is **part of the plan and of B**, identical for every condition.
- At `~/start` these are mission errors, never a manoeuvre outside the area:
  - the hold position is not in F_margin;
  - no feasible route exists.

### 3.6 Send-time area check

Before any request is sent, the plan it carries (frame M) is sampled every `geom_step_m`
(0.05 m) and every sample must lie in **F_(margin − s)**, with s = `area_check_slack_m` = 0.5 m
(owner, 2026-11-13), floored at F_0 = the area itself. A plan that fails is never sent, and the
run fails (DESIGN §5.2 rule 2).

Why a slack: plans are built inside F_margin (§3.2–§3.5), and turns touch its boundary. The
executor re-expresses its plan in Belief's current map frame before a decision (DESIGN §4), so
a refinement of `odom_to_map` moves the plan rigidly by the frame change. A check at the full
margin would then fail a mission on a 1 cm alignment drift. The slack equals
`transform_jump_m`, the drift above which a jump is logged: a path that has moved further into
the band than that is refused. The gate's own check stays at the full margin (§14 item 10).

## 4. Sampling, time and budget

**Truncation.** Keep the whole segments that end at or before
t_start + B. Then drop trailing `initial`, `connection`, `transit` and `rejoin` segments, so a
truncated plan ends on a `line`, a `body` or a `hold`.

**Sampling.** A plan is a chain of segments: `initial`, `line`, `connection`, `transit`, `body`,
`rejoin`, `hold`. Each segment is sampled by arc length every Δs = 0.25 m (the MCS designer's
step, E-30), giving t = t₀ + s/U, (x, y) on the curve, and yaw = the analytic tangent.
- If a request would exceed 19 000 samples, Δs = L_total/19 000 (Control's cap is 20 000, E-08).
- Control's checks are satisfied: t is strictly increasing in 0.5 s steps; speed U ≤ 2 m/s; yaw is
  the direction of motion.
- Only `hold` segments (the stop truncation) are stationary.

**Time.** T(P) = L(P)/U, with turns and lead-ins included in L(P). Planned durations equal the sample
times, so what the budget counts is what Control receives.

**Budget.**
- B = T(tuned C2 plan from the run's hold), including the initial manoeuvre, computed at mission
  start by the same generator for every condition (brief Q2 [DECIDED]; the tuned parameters come
  from `C2_TUNING_PROTOCOL.md`).
- Elapsed E = τ − t_start, in path seconds.
- **Charged to the budget:** the τ path seconds of everything flown. That covers the initial
  manoeuvre, coverage legs, connections, transits, revisit bodies, rejoins and turns.
- Wall time and `ros_time` are logged beside it. They differ when the governor slows τ (E-14), and
  the report gives both.
- [ASSUMPTION] The equal-time comparison is in planned path seconds. Measured time is reported
  separately.

## 5. Look predictor (vectorised; brief §9.1, §9.2)

**Input.** Path P in M, sampled every Δ/2 along-track: poses (p_j, ψ_j) and the signed curvature
κ_j of the primitive they lie on (0 on a straight, +1/r on a left arc, −1/r on a right arc).

**Per pose and side:**
- b = ψ_j + s·π/2;
- ground ranges r_m **from the transducer**, from r_lo to r_hi′ in steps of Δ/2, where
  **r_hi′ = min(r_hi, r_geo(R, h))** (the detection band of §6, not the coverage band);
- ground points g = p_j + (r_m + ℓ)(cos b, sin b). Belief places a sample at its offset from the
  boat's centre line, which contains the lever arm ℓ, and evaluates PD at the range from the
  transducer (Belief `topics.md` §4 item 2) **(Belief 2026-11-12)**;
- for each point: its cell, k(b), g(k), and PD_r(r_m).

**Turn gate (Belief 2026-11-12).** Belief counts nothing on the inner side of a turn beyond the
instantaneous turn radius r* = v/|ω| (Belief `MATH.md` §4 item 6; `topics.md` §4 item 6a): no
look, no aspect, no coverage. On a pose with |ω| = U·|κ_j| ≥ ω_min:
- the inner side is port for κ_j > 0, starboard for κ_j < 0;
- inner-side points with r_m + ℓ > 1/|κ_j| are dropped.

ω_min and the on/off switch are read from `model_params_json.turn_gating` when Belief publishes
them, else from the configuration (`turn_gate`, `turn_min_yaw_rate_rad_s`). On an arc of radius
r_turn = 3 m the whole inner detection band is dropped (r_lo + ℓ > 3 m).

**Stated approximations:**
- Belief estimates ω and v from the ping poses over a 1 s window; the predictor uses the exact
  curvature of the planned primitive. They differ within about 1 s of an arc's entry and exit.
- On an arc the far end of the outer swath moves faster than the boat. With poses every Δ/2 the
  predictor leaves gaps there that a 20 Hz sonar does not. The prediction is therefore a subset of
  what Belief credits on the outer side of a turn: conservative. Denser arc sampling would
  multiply the decision time and is not done.

**Keys and reduction.** Points outside the grid are dropped. The remaining points are keyed by
`cell·64 + g` (one key per cell and folded sector; the stride `model.KEY_STRIDE` = 64 holds
while the folded count G < 64, which `validate_grid` checks for every grid), sorted, and reduced
with `np.maximum.reduceat`: the best PD per (cell, folded
sector). Each key also carries the **first sample time** at which it is reached, which the budget
truncation in §10.4 needs. No `np.add.at` is used.

**Contacts.** A contact is looked at when the cell of its mean appears.

**Check against the real Belief core** (gate [20]). The same poses are fed to BlueBoat-Belief
through its offline interface (`OFFLINE_CORE_API.md`) and compared on (cell, folded sector):
- on straight lines, both ways: more than 97 % of the predicted pairs are searched in Belief
  (`aspect_log_miss < 0`), and more than 97 % of the pairs Belief searched at plateau strength are
  predicted. Belief's PD has 3 m ramps where the planner's is zero (§6); those pairs are counted
  and reported apart;
- on a path with turns, one way: more than 97 % of the predicted pairs are searched in Belief.
  The outer-side pairs Belief has and the predictor lacks are reported.

## 6. Planner sensor model (placeholders until E3; brief §8.2)

```
PD_k(r, b, φ) = PD_r(r) · s_k · A_k(b, φ)
PD_r(r)       = p_r if r_lo ≤ r ≤ min(r_hi, r_geo) else 0       p_r = 0.7, [r_lo, r_hi] = [3, 25] m
A_k(b, φ)     = 1 if m_k = 0 ;  1 if |wrap_{2π/m_k}(b − φ)| ≤ w_k  else a_k   (m_k ≥ 1)
```

**Detection band vs coverage band.** The window [3, 25] m is the brief's statement of
Belief's default (brief §8.2), capped at the geometric limit.
- Looks at 3–6 m ground range **do count** for contact and background value: Belief counts them
  in Q, and the planner's predictions must stay consistent with Q (§9).
- They are excluded only from the coverage guarantee (§2), which is the conservative choice and
  applies to every condition.
- r is the ground range **from the transducer** (§5).
- **Belief's actual window (Belief 2026-11-12, read in `core/models.py::pd_range`).** A trapezoid:
  plateau 0.7 on [3, 25] m with linear 3 m ramps outside, so PD > 0 from just above 0 m. The
  planner keeps the box. Consequences, stated:
  - Belief searches the 0–3 m strip at reduced strength; the planner predicts no look there and
    gives it no value (conservative);
  - a miss Belief books in that strip reaches the planner at its recorded strength (§7), not at
    the plateau;
  - the planner model stays a configuration of its own (`pd_plateau`, `pd_r_lo_m`, `pd_r_hi_m`):
    planner, Belief and world models are kept apart.

**Orientation.** φ takes the S values β_0 … β_{S−1}, with a uniform prior π_φ = 1/S [ASSUMPTION;
Ji, Parr & Carin 2007 motivate discrete sectors].

**Class table (placeholders):**

| Class | m_k | w_k | a_k | s_k |
|---|---|---|---|---|
| block | 4 | 20° | 0.4 | 1.0 |
| pipe | 2 | 10° | 0.1 | 1.0 |
| cage | 4 | 25° | 0.3 | 0.9 |
| tyre | 0 | — | — | 0.6 |
| plate | 0 | — | — | 0.3 |

- Validation: s_k ∈ (0, 1]; a_k ∈ [0, 1].
- **Class families (Belief 2026-11-12).** The rows above are *aspect families*. Belief's
  `class_names` is its configured list of detector classes, and a detector class outside that
  list arrives as `other` (Belief `topics.md` §10, D-C2). The configuration block
  `class_families` maps a Belief class name to a family; a name that is itself a family needs no
  entry. The map is empty by default: which detector class belongs to which family is the
  owner's decision.
- A Belief class with no family is isotropic with s = 1 [FALLBACK]. It is listed in every decision
  record (`inputs.unmapped_classes`). When no class with a family carries probability, the
  aspect-aware choice is inert (C4 = C4_iso by construction) and the record is labelled
  `aspect-inert`.
- C4_iso sets A ≡ 1.

**Folded look dependence** (brief §8.2, §17.5):
- one look per (target, folded sector); looks in different folded sectors are independent given
  (k, φ);
- opposite-side looks share a folded sector, so they are redundant. Paull et al. 2013 is the
  motivation only; this is **not** Belief's ρ. Belief folds too when
  `model_params_json.rho_model.fold_opposite` is true (its default); the value is recorded, and a
  false value is labelled `fold-mismatch`;
- a detected history look is taken at its sector centre with p̄ = p_r. A missed history look is
  taken at its sector centre with the strength Belief recorded for it (§7). New looks are taken at
  their predicted sector centre with PD_r at the predicted range.

**Hand-checked angular cases** (S = 12, o = 0):
- parallel: same directed and folded sector;
- antiparallel: different directed sectors, same folded sector, so redundant;
- perpendicular: different folded sectors, independent;
- 29.9° vs 30.1°: sectors 0 and 1.

## 7. Contact posterior (brief §8.3, §17.4)

Per contact c from contract C-4:
- `p_exists`, `state`;
- P(k|c) = `class_probabilities`;
- D = `sectors_detected`, Mi = `sectors_missed`;
- q₀ = `first_confidence`, k₁ = `first_sector`.

```
D_f = { g(k) : k ∈ D },   g₁ = g(k₁),   D' = D_f \ {g₁}
C   = { k : aspect_log_miss[cell(c), k] < 0 }            sectors a detector searched at c's cell
Mi_f = ( { g(k) : k ∈ Mi } ∪ { g(k) : k ∈ C } ) \ D_f
m_g  = max_{k∈C, g(k)=g} ( 1 − exp(aspect_log_miss[cell(c), k]) )      (p̄ if g comes from Mi only)
p_k(φ; β, m) = m · s_k · A_k(β, φ)
w₁(k, φ) ∝ P(k|c) π_φ p_k(φ; β_{k₁}, p̄)
L_real   = Σ w₁ · Π_{g∈D'} p_k(φ; β_g, p̄) · Π_{g∈Mi_f} (1 − p_k(φ; β_g, m_g))
L_clut   = f^{|D'|} · Π_{g∈Mi_f} (1 − f·m_g/p̄)
p̃_c      = q₀ L_real / ( q₀ L_real + (1 − q₀) L_clut )
λ_c(k, φ) ∝ w₁ · Π_{D'} p_k · Π_{Mi_f} (1 − p_k)        (Σ λ = 1)
```

**Misses come from the cell record (Belief 2026-11-12).** Belief writes a miss into
`sectors_missed` only when the contact's look closes: on a sector change, or at the first ping
more than `look_timeout_s` (20 s) after the look's last observation (Belief `MATH.md` §9 items
5–6). Until then a look that found nothing is in neither sector set. The grid cell is updated
at once: `aspect_log_miss[cell, k] = ln(1 − max covered PD in sector k)`, the same quantity
Belief uses for the contact's own miss (the best covered PD at the contact's cell). The planner
therefore reads the cell of the contact's mean:
- every sector searched there and not detected is a miss. That includes the looks made
  **before** the contact was created, which `sectors_missed` never holds;
- m_g is the strength Belief recorded: p̄ on the plateau, less in Belief's range ramps (§6);
- `sectors_missed` is still read: a sector listed there and not searched at the present cell
  (the mean moved to another cell) counts at p̄ [FALLBACK];
- every searched sector counts as seen, whatever its strength: a new look in it has no value
  (§8). This is the set Belief's own ρ is taken against (ρ = 0 on the same line), and it keeps
  the planner conservative;
- the clutter term carries the same range factor m_g/p̄. On the plateau it is the 1 − f of the
  original form;
- when `aspect_log_miss` is absent, Mi_f comes from `sectors_missed` alone, at p̄ [FALLBACK,
  recorded].

**Assumptions [ASSUMPTION]:**
- q₀ is a calibrated probability (brief §8.3). **It is not, today:** Belief publishes the raw
  score of the creating detection, and says so in `model_params_json.detector_calibrated`
  (false; Belief `topics.md` §10, D-C3). While that flag is not true, p̃_c and every
  quantity built on it are **score-conditional**, and every decision record carries that label.
  The dev-loop emulator calibrates its scores against its own world (brief §9.4) and sets the
  flag accordingly.
- Looks are independent given (k, φ) or clutter.
- f = 0.2 is a placeholder, swept: [OPEN] O-5.

**Fallbacks (C-6):**
- q₀ NaN → p̃ = `p_exists`, with no re-update (avoids double-counting Belief's looks); λ still from
  the sector sets.
- k₁ = 255 → w₁ averaged uniformly over the choice of g₁ ∈ D_f.
- Empty `class_probabilities` → uniform.

**Lifecycle:**
- Confirmed → value 0.
- Pruned → forgotten, except this module's revisit counter and log.
- No private posterior: recomputed from the grid at every epoch.

**Operational vs true** (brief §17.7). A false confirmation is resolved operationally and is still
counted as a false positive in evaluation. Contacts are matched to ground truth only in `eval/`.

## 8. Contact value (brief §8.3)

For new folded sectors N at c, i.e. not in D_f ∪ Mi_f ∪ pending (§11.3), with Mi_f as in §7:

```
P_det_real(N) = Σ λ_c(k, φ) [ 1 − Π_{n∈N} (1 − p_k(φ; β_n, r_n)) ]
P_det_clut(N) = 1 − (1 − f)^{|N|}
V_c(N)        = p̃_c P_det_real(N) − κ (1 − p̃_c) P_det_clut(N)          κ = 1 [DECIDED]
```

- V_c = 0 if c is confirmed, resolved or abandoned (§12), or if N = ∅.
- V_c ∈ [−κ, 1]; it is signed and not monotone in N.
- Belief confirms a candidate on a second consistent detection, so P_det_real and P_det_clut are
  the probabilities of a true and a false confirmation.
- Belief also confirms on a **same-sector** repeat (brief §9.2). The planner gives such a repeat no
  value (§6, folded model), so it never buys a same-aspect confirmation. This is deliberate and
  conservative (§11.4, approximation 5).

## 9. Background: aspect-resolved search value (brief §8.4, §17.6)

For each cell i that is inside the area, has a finite `coverage_i`, and lies outside every contact's
Mahalanobis-3 zone:

```
Q_i   = 1 − coverage_i                                   Belief's modelled residual miss probability
S_i   = { g : aspect_log_miss[i, k] < 0, g(k) = g },  n_i = |S_i|
p_i   : solution of  Σ_k π_k (1 − p_i s_k)^{n_i} = Q_i      (§9.1)
L_i(k, φ) = Π_{g∈S_i} (1 − p_i s_k A_k(β_g, φ))            (= 1 if S_i = ∅)
p₀    = density_per_ha · Δ² / 10⁴
M_i   = Σ π_k π_φ L_i(k, φ),     q_i = p₀ M_i / (1 − p₀ + p₀ M_i)
w_i(k, φ) = π_k π_φ L_i(k, φ) / M_i
E_i(N_i)  = q_i Σ w_i(k, φ) [ 1 − Π_{n∈N_i} (1 − PD_r(r_n) s_k A_k(β_n, φ)) ]
N(P)      = Σ_i E_i(N_i(P))
```

π_k is the background class prior, uniform over `class_names` [ASSUMPTION].

**Searched sectors (Belief 2026-11-12).** S_i is the set of folded sectors **a detector searched**
with PD > 0: `aspect_log_miss[i, k] < 0` (Belief `topics.md` §3a). It is the set Belief's own ρ is
taken against.
- `aspect_counts > 0` is **not** that set. It also counts looks that had no detector coverage:
  with a dummy detector, or none, every look is counted and no sector is searched.
- n_i is the number of folded sectors searched, not a number of looks. p_i is then the
  per-sector detection probability which, applied independently over those n_i sectors,
  reproduces Belief's residual Q_i. It absorbs what Belief's own rule does between sectors
  (ρ < 1 at intermediate angles, a stronger repeat on the same line) and the range dependence
  of PD.
- When `aspect_log_miss` is absent from a grid, S_i falls back to `aspect_counts > 0`
  [FALLBACK, recorded as `searched_from_aspect_counts`]. Belief always publishes the field.

### 9.1 Calibration of p_i

The brief's p = 1 − Q^{1/n} satisfies the brief's own property ("the aspect-agnostic case
reproduces Q") **only if s_k = 1 for every class with π_k > 0**. Since 1 − p s_k ≥ 1 − p, with
equality iff s_k = 1:

```
Σ π_k (1 − p s_k)^n ≥ (1 − p)^n = Q
```

The planner therefore solves the class-weighted equation:
- M(p) = Σ π_k (1 − p s_k)^n is strictly decreasing on [0, 1), from 1 to
  Q_min = Σ π_k (1 − s_k)^n.
- A unique root exists iff Q_i ≥ Q_min,i. It is found by vectorised bisection with exactly 60
  iterations (|M − Q| ≤ n·2⁻⁶⁰; deterministic).
- Q_i < Q_min,i: p_i = 1, clamped, counted as `calibration_clamped`. Here the planner's residual
  exceeds Belief's.
- S_i = ∅ with Q_i < 1: n = 1 and A ≡ 1 [FALLBACK, counted].

Closed forms (gate):
- s ≡ 1 → 1 − Q^{1/n};
- s ≡ s̄ → (1 − Q^{1/n})/s̄;
- n = 1 → (1 − Q)/Σ π_k s_k.

**Aspect-aware use.** The same p_i is used. Since A ≤ 1, the aspect-aware residual is ≥ Q_i:
the aspect-agnostic case reproduces Belief, and the aspect windows expose orientations Belief
treats as seen.

**Worked numbers** (uniform prior over the five default classes, s = (1, 1, 0.9, 0.6, 0.3); gate
[16]): Q = 0.25, n = 2 → the brief's p_b = 0.500 gives M(p_b) = 0.403, the calibrated p = 0.709
gives M = 0.250 (Q_min = 0.132). With p₀ = 0.002 the brief's formula gives q = 8.07e-4 (61 %
above Belief's 5.008e-4); the calibrated value reproduces it. The cost: for s = 1 classes the
calibrated per-look probability exceeds Belief's apparent one (0.92 against 0.70 for one look).
**Rejected alternative:** reading Belief's Q as the residual of a unit (s = 1) target, the
brief's formula; its mixture never matches Belief's residual, and the `p_background`
consistency value would fail whenever s_k ≠ 1.

### 9.2 Quantities used and not used (§17.6)

| Field | Use |
|---|---|
| `coverage`, `aspect_log_miss`, `prior_density_per_ha`, `class_names`, geometry | **used** |
| `aspect_counts` | **fallback only**, when `aspect_log_miss` is absent |
| `p_target` | **not used** (it mixes background and contacts) |
| `p_background` | **check only.** Belief's value is p₀Q_i/(1 − p₀ + p₀Q_i) when its PFA is 0 (Belief `MATH.md` §7), which is the planner's q_i with A ≡ 1 in reachable cells, for any s_k. Every decision record carries the median relative difference between that closed form and `p_background` over the eligible cells (`calibration.p_background_median_rel_diff`). It is never added to anything |

**Limits (gate):**
- S_i = ∅ → q_i = p₀;
- density = 0 → N = 0;
- NaN or out-of-area cells excluded;
- every folded sector seen with Q → 0 → q → 0 (reachable case).

## 10. Actions, the executed sequence, and its value

### 10.1 Default continuation K

K is the ordered list of unflown coverage legs, with their connections, of the condition's
current plan:

| Condition | K |
|---|---|
| C1 | pass 1 |
| C2, C3, C4 family | **the C2 plan** (pass 1 then pass 2) |

C4 with no detour is therefore exactly C2. C3 inserts its revisits at the pass-1/pass-2 boundary
(§13).

### 10.2 Actions as detours

At change point c, with mirror pose x_c and heading:

| Action | Detour D_a | Executed tail |
|---|---|---|
| CONTINUE | D_cont = the ordinary connection from x_c to the start pose of K's next leg | D_cont ⊕ K |
| REVISIT(c, g) | transit(x_c → body start) ⊕ body ⊕ rejoin(body end → start pose of K's next leg) | D_a ⊕ K |
| RECOVER(j) | transit(x_c → leg j) ⊕ leg j ⊕ rejoin(→ next leg of K \ {j}) | D_a ⊕ (K \ {j}) |

**Not implemented.** RECOVER is not built: K already contains pass 2, so RECOVER would only
reorder K. It comes back with mid-line interruptions (C4_live, today only a flag that nothing
reads; not run).

Every tail is truncated at the last segment boundary ≤ t_start + B.

**REVISIT body.** One of four legs, b ∈ {β_g, β_g + π} × side s:
- ψ = b − s π/2, u = (cos ψ, sin ψ);
- the leg passes q = m_c − r* (cos b, sin b), with r* = 10 m [DECIDED brief Q7, placeholder]. r*
  must lie in [r_lo, min(r_hi, r_geo)] (the detection band, §6), else
  r* := the midpoint of that band, logged;
- straight length 2·max(3σ_along, L_min/2), with σ_along = √(uᵀ Σ_c u), plus l_in at both ends;
- the feasible leg with the smallest T(D_a) is kept.

S-44 Ed. 6.1.0 §3.7 ("search radius ≥ 3σ") is context only, not the derivation of this formula
(brief §20.1). L_min = 10 m [ASSUMPTION], swept.

### 10.3 Objective: value of the whole executed tail over the remaining budget

```
B_rem   = t_start + B − c
J(a)    = V( union of new looks along D_a ⊕ K_trunc(a) )
V(·)    = Σ_c V_c(N_c) + λ · N(P)        λ = 0.5 [DECIDED Q1];  N_c, N_i over the WHOLE tail
```

- Every option occupies the same B_rem. For C2's remainder T(K) = B_rem exactly, and every detour
  pushes K past the budget. So comparing J **is** comparing value rates J/B_rem: the brief's rate
  rule over the executed sequence, with CONTINUE treated identically.
- Displaced coverage is charged through the truncation.
- A look of K in a key already reached by D_a has no further value.

### 10.4 Exact, efficient evaluation

At each epoch:
- K's keys (cell or contact, folded sector) are precomputed with their first-reach time t and PD.
- V_K(t) is the value of K truncated at t, with contributions summed per key in time order. It is
  a step function.

For a candidate:

```
J(a) = V_touched( D_a ∪ K_{≤ t_a} ) + V_K(t_a) − V_K,touched(t_a),     t_a = B_rem − T(D_a)
```

"Touched" means the cells and contacts reached by D_a. This is exact, because values are additive
across cells and contacts.

### 10.5 Approximations (stated)

- The rejoin is always at the start of K's next unflown leg; K is not re-sequenced. This holds for
  every condition, and is conservative for the adaptive policy.
- Truncation is at segment boundaries, in evaluation and in execution alike.
- K is valued open-loop (no future decisions inside K).

### 10.6 What is charged

| Quantity | Content | Used by |
|---|---|---|
| T(D_a ⊕ K_trunc) | everything in the tail, sampled by the same sampler as the request | J, budget |
| Budget charge | τ path seconds actually flown | budget rule, reports |
| ΔT(a) = T(D_a) − T(D_cont) | the extra time a detour adds | revisit cap R_used (§12), reports |

## 11. Selection with a two-step lookahead

### 11.1 Identity (no double counting)

For any contact and disjoint new-look sets N₁, N₂:

```
V_c(N₁ ∪ N₂) = V_c(N₁) + P_c(miss on N₁) · V_c'(N₂)
```

- V_c' uses p̃' = p̃ Σλ Π₁ / P(miss), λ' ∝ λ Π₁, P(miss) = p̃ Σλ Π₁ + (1 − p̃)(1 − f)^{|N₁|}.
- A detection confirms the contact, after which its value is 0.
- The background has the same identity, with q′ and w′ the posteriors after a miss.

**Proof sketch** (expand both sides):
- the real-hypothesis terms combine to p̃(1 − Σλ Π₁Π₂);
- the clutter terms combine to κ(1 − p̃)(1 − (1 − f)^{|N₁|+|N₂|}).

Checked numerically: differences 0 and 1e-18. Consequences:
- for a fixed second action, the value of a sequence is the value of the union of its looks;
- branching matters only because a₂ is chosen after ω.

### 11.2 Decision points and actions

**Decision points:** the end of each coverage `line` and each revisit `body` segment. Never
`connection`, `transit` or `rejoin`.

**First actions A₁:** CONTINUE, REVISIT(c, g) for unresolved, unabandoned, non-pending
(c, g), and RECOVER(j) (not implemented). The top K₁ = 5 are kept by one-step J; CONTINUE is always
kept.
- **Prefilter, as implemented.** For each contact only the k = `sector_prefilter_k` = 3 folded
  sectors with the highest body-only contact value are built. Second actions are pre-ranked by contact value per unit
  straight-line time, and the top K₂ are then built exactly.
- These proxies only choose which candidates get evaluated exactly. Every candidate that is kept
  is scored by the exact J.

### 11.3 Pending looks and the evidence horizon (Belief 2026-11-12)

A look the boat has made, or will make before the change point c, and that the grid does not
hold yet is **pending**. Pending looks count as **seen** before any valuation:
- for background: their keys join S_i through the union;
- for contacts: their sectors are excluded from N, and a contact with a pending look is not a
  candidate at this epoch.

**What the grid holds.** `BeliefGrid.header.stamp` is Belief's **evidence horizon**, on the clock
of the sonar profiles: every ping at or before it is in the grid, and none after it (Belief
`topics.md` §2). It lags the newest ping by one to two seabed-image strides plus the detector
time: 4 to 9 s at 0.5 m/s. The planner uses the stamp, not a fixed delay.

```
τ_h   = path time the boat had reached at the stamp            (the horizon on the path axis)
grid cells:   pending = mirror looks with t ∈ [τ_h, c]
contacts:     pending = mirror looks at the contact's cell with t ∈ [τ_h − t_g, c],  t_g = d_g / U
```

- **t_g.** A cell is first searched up to one cell-crossing before the row a detection of an
  object in it lies on. In that interval the cell record says "searched", and the detection is
  not applied yet. d_g = 1 m (two cells) [DESIGN].
- Misses need no further delay: they are read from the cell record (§7), which the horizon
  covers.

**τ_h online** (`ros/executor.py`). Control's progress carries its node clock and the path
parameter together (`ros_time`, `tau`; E-11). The executor keeps a short history of these pairs
and reads τ at the stamp by linear interpolation, then subtracts a tracking margin:

```
τ_h = τ(stamp) − L_lag / U,        L_lag = 3 m
```

L_lag is the governor's bound on how far the boat may trail the path parameter
(`gov_Lmax`, E-08): the grid holds what the boat saw, not what the path parameter passed.
The history is cleared when the run changes, on a `cleared` event, and when τ or `ros_time`
steps backwards.

**τ_h in the dev loop.** The dev loop runs the executor (DESIGN §5.4), so τ_h is read by the
rule above. Its progress carries `ros_time` = T0 + τ, the clock of Belief's stamps there, and
its configuration sets L_lag = 0 (perfect tracking): τ_h = τ(stamp) = stamp − T0.

**Fallback: τ_h = τ − t_lat**, with t_lat = 10 s (two image strides at 0.5 m/s plus 2 s), in
these cases, each named in the decision record (`pending.source`):

| Case | `pending.source` | τ_h |
|---|---|---|
| stamp read, inside the history | `stamp` | τ(stamp) − L_lag/U |
| stamp older than the history | `stamp_before_history` | the oldest τ of the history − L_lag/U (everything since is pending) |
| stamp zero or missing | `t_lat:no_stamp` | τ − t_lat |
| no progress history with `ros_time` | `t_lat:no_clock` | τ − t_lat |
| stamp later than the newest `ros_time` by more than 1 s (`clock_tolerance_s`), or older than it by more than 120 s (`horizon_max_lag_s`; the clocks differ, for example sim time against wall time; Belief itself releases a ping after 60 s at most) | `t_lat:clock_mismatch` | τ − t_lat |

Under the folded model the pending rule is idempotent: looks already in the grid add nothing.

**After a Belief restart** (`grid_seq` decreases; Belief `topics.md` §1) the new grid holds no
earlier evidence and its contact uids start again at 1. The executor accepts it, counts and logs
the restart, forgets the per-contact memory of §12 (it is keyed by uid), and labels every later
decision record `belief-restarted`. A run with that label is not evaluation evidence.

### 11.4 Two-step value

```
J₂(a₁) = Σ_ω P(ω) · max_{a₂ ∈ A₂(ω)}  J( a₁ ⊕ a₂ ⊕ K'_trunc | ω )
```

**Outcomes ω:**
- REVISIT(c, ·): ω ∈ {det, miss}, P(det) = p̃ P_det_real + (1 − p̃) P_det_clut.
  - det → c confirmed;
  - miss → c's posterior updated (§11.1);
  - all other contacts and the background take a₁'s looks as seen (union).
- CONTINUE, RECOVER: a single branch.

**Second actions:**
- a₂ starts at a₁'s body exit (for CONTINUE: at the next decision point of K).
- a₂ = CONTINUE means a₁'s own rejoin, then K.
- A₂(ω) is the top K₂ = 5 by one-step J in the post-state, subject to the budget rule.
- a₂ is chosen **by the combined-sequence value J**, never by its own rate.

**Choice.** a₁* = argmax J₂. Switch from CONTINUE only if J₂(a₁*) ≥ J₂(CONTINUE) + ε_J, with
ε_J = 0.02 [ASSUMPTION, swept]. This absolute margin replaces the relative η of the draft, because
J includes the remainder's value.

**Execution.** Only a₁'s tail (a₁ ⊕ rejoin ⊕ K) is sent. a₂ is never sent. The next decision point
re-decides with what was observed (receding horizon).

**C4_depth1:** no second step, J₂ = J. The top K₁ = 5 are still built, which changes nothing:
the best candidate by J is among them.

**C4_rand.** The bearing of the n-th REVISIT decided in the run is read from
`config.c4_rand.bearing_table[n]` instead of being optimised. The entry is an **index into the
contact's unseen folded sectors** (with wrap-around): the first feasible sector from that index on,
within the budget and the revisit cap, is flown; if none is, C4's own choice stands. The table is
pre-drawn in `eval/` (`devloop.rand_table`, 64 entries in [0, G), G = the folded sector count
of Belief's grid, 6 today) [DECIDED]. Whether a revisit happens and of which contact follow C4's own
selection, so the number of revisits matches (brief §8.1).

**Approximations:**
1. Branching only on the revisited contact.
2. ω is assumed known at the next decision point (latency).
3. The second step is restricted to the top K₂.
4. K is valued open-loop.
5. Same-sector repeats have no value (§8).

## 12. The three rules and the mission stop (synthesis §8.1; brief §8.6)

| Rule | Definition (thresholds configurable and swept) |
|---|---|
| **Resolution** | c is resolved if confirmed, **or** p̃_c < p_res (0.05), **or** every REVISIT candidate of c has V_c ≤ 0 |
| **Abandonment** | c is abandoned if n_rev(c) ≥ n_max (3), **or** each of the last two revisits of c changed p̃_c by < δ (0.02). n_rev counts the **accepted** REVISIT requests for c. Δp̃ is measured from the epoch before the body to the first epoch whose horizon covers it: τ_h − t_g ≥ the body end (§11.3). Jadidi et al. motivate this; they do not derive 0.02 or 3 (brief §20.8) |
| **Budget** | an action is admissible only if (c − t_start) + T(D_a) ≤ B and, for a REVISIT, R_used + ΔT(a) ≤ ρ_rev·B, with ρ_rev = 0.5. R_used = Σ ΔT of the accepted revisits. Every tail is truncated at the budget (§10.2) |
| **Mission stop** | if B_rem < T_min (the shortest admissible detour body) and K is empty, the run ends when Control reports `end_reached`. With ρ_floor > 0 (0 = never, the default [ASSUMPTION]): if the best action's value rate max_a J₂(a)/B_rem is below ρ_floor, the policy returns **STOP** and a truncation to a hold at the change point is sent. The run then ends as `stopped_holding` with the reason `mission_stop:rho_floor` |

**Why "accepted" and not "executed" (corrected 2026-11-12).** The decision at the end of a
revisit body is taken before the boat reaches that end (the trigger of DESIGN §5.2 precedes the
change point by the lead, the decision allowance and the margin). Counting only bodies the boat
has passed would leave the body in progress out of n_rev, and allow n_max + 1 revisits. Counting
accepted requests bounds the number of revisits at n_max, which is what the gate checks. The two
counts differ only when a run is stopped or disarmed between the acceptance and the body.

## 13. C3 — append-on-completion [DECIDED user 2026-11-04]

1. **Initial plan:** the C2 plan (K = pass 1 + pass 2).
2. **Completion epoch:** the decision point at the end of pass 1's last leg (trigger per DESIGN
   §5.2). If it is missed (stale belief, refusal, `missed_lead`), C3 simply continues K, i.e. C2,
   and the fallback is logged.
3. **Candidates:** contacts with `state` = 0 at the epoch.
4. **Bearing:** perpendicular to the first detection, b = β_{k₁} + π/2 (folded). Of the four
   legs (§10.2), the feasible one with the smallest transit + body length is used.
   - If k₁ = 255, b = θ₁ (the pass-1 line direction), i.e. perpendicular to the pass-1 look
     bearings θ₁ ± π/2 [FALLBACK].
   - Corrected while implementing: the draft said "pass-1 heading + π/2", which is a pass-1 look bearing
     itself, not the perpendicular one.
5. **Order:** greedy nearest neighbour by the route length to each contact's cheapest leg, from
   the current pose (the change point, then each admitted body's end). Ties go to the lower uid.
6. **Admission:** each revisit is admitted in order while the budget rule (§12, including the
   ρ_rev cap and ΔT accounting) holds; one that does not fit is skipped.
7. **Tail sent:** the transit and body of each admitted revisit in order, then the rejoin to
   pass 2's first leg, then pass 2, truncated at the budget. **One request.** C3 is deterministic
   given the grid at its epoch.
8. **Rules:** resolution, abandonment and the C4 selection do not apply. Each candidate gets
   exactly one revisit. C3 is the heuristic rung of synthesis §7.3; synthesis §8.3 calls
   "perpendicular to last time" a heuristic.

## 14. Invariants and properties tested by the gate

Tolerances: 1e-12 for normalisations and calibration, 1e-9 m for geometry, bitwise for
determinism.

**Original properties:**
1. An isotropic contact values every unseen folded sector equally.
2. Cage, detected at sector 0: best next folded sector 3, hand-computed.
3. Pipe, detected at sector 0: all unseen folded sectors at the floor, hand-computed.
4. With A ≡ 1, E_i equals the aspect-agnostic closed form.
5. Seen sectors (grid **or pending**) add no value.
6. CONTINUE is chosen when no J₂ exceeds J₂(CONTINUE) + ε_J.
7. Abandonment bounds the number of revisits.
8. C1 and C2 send exactly one request.
9. Byte-identical decisions on identical inputs.
10. Every exported sample lies in F_margin, and every request passes `control_stub` (and Control's
    `PathUpdater` when importable). At send time the executor refuses a plan outside
    F_(margin − s) (§3.6) and a tail whose junction at c fails Control's continuity rule.
11. Look predictor against the real Belief core above 97 % (§5: both ways on lines, one way
    with turns).
12. Geo bit-identical to MCS.
13. All probabilities finite and in [0, 1]; Σλ = Σw = 1; V_c ∈ [−κ, 1]; q_i ∈ [0, p₀].
14. P_det_real and P_det_clut are monotone in N; V_c is not asserted monotone.
15. Zero budget admits no positive-duration action.
16. No infeasible candidate is ever selected.
17. A refused request leaves the mirror unchanged.
18. Frame-equivalent inputs give equivalent decisions.
19. Spacing table (§2.4), plus a random brute-force check.
20. Background limits (§9.2).
21. C3: perpendicular bearings, NN order, budget admission, determinism, missed-epoch fallback to C2.
22. Every `ContractError` names its field.

**Design-review properties** (2026-11-04):
- A-1…A-5: calibration.
- B-1…B-5: planned duration = sampled request duration; tail within the budget; CONTINUE = K_trunc
  value; rejoin present; K-shared looks.
- C-1…C-5: identity; worked example; CONTINUE same function; only a₁ sent; pending looks.
- D-1…D-5: stop and disarm semantics against the stub.
- E-1: initial manoeuvre.
- F-1, F-2: detection band vs coverage band.
- G-1: `missed_lead` never sent. Since 2026-11-12 also under the node's clock: the time a
  decision took is counted against the lead before sending (DESIGN §5.2 rule 2).

**Added on 2026-11-12** (reading of the real Belief):
- H-1: searched sectors come from `aspect_log_miss`; a grid with looks and no detector coverage
  has no searched sector.
- H-2: a contact's misses come from its cell record, at the recorded strength; a sector both
  detected and missed counts as detected.
- H-3: pending looks follow the evidence horizon; each fallback names itself.
- H-4: a decrease of `grid_seq` is a restart: accepted, counted, per-contact memory reset.
- H-5: a mission polygon outside Belief's area stops the mission before the first path.
- H-6: an unmapped class is isotropic and is recorded; a mapped one takes its family's window.
- H-7: lever arm, turn gate and sector rounding of the predictor (§5), against the real core.
- H-8: with ρ_floor > 0 a STOP is issued and ends as `stopped_holding`; with 0 it never is.

## 15. Parameters

Categories: **U** user decision · **R** repository default or fact · **L** literature-motivated
· **P** placeholder · **D** design choice.

| Parameter | Default | Unit | Cat. | Swept | Source |
|---|---|---|---|---|---|
| U | 0.5 | m/s | U | no | user 2026-11-04 |
| R | 20 | m | U | no | synthesis §8.5; brief Q8 |
| sensor depth | 0.22 | m | R | no | root CLAUDE.md §4.3 |
| grazing_floor | **off** (option: altitude ≥ 0.08·slant) | — | U (brief §3) / L (option) | on/off in the C2 protocol | brief §3; NOAA HSSD 2025.0.01 §8.3 |
| r_near | 6 | m | P | 6, 9.5 | brief §3; TF l.38 |
| f (spacing factor) | 0.8 | — | R | C2 protocol | synthesis §8.5 |
| planning_depth_m | per mission | m | U | — | — |
| margin | 2 | m | P | — | brief Q9 |
| s (send-time area slack, §3.6) | 0.5 | m | U | no | owner 2026-11-13 |
| r_turn | 3 | m | P | 1.5–6 | E-23 |
| t_settle | 2 | s | P | — | brief §8.5 |
| p_r, r_lo, r_hi | 0.7, 3, 25 | —, m | P | degraded ×0.8 | brief §8.2 |
| ℓ (lever arm) | 0.285 | m | R | no | Belief `topics.md` §4 item 2 (the processor's lateral transducer offset) |
| turn gate, ω_min | on, 0.01 | —, rad/s | R | no | Belief `MATH.md` §4 item 6; `model_params_json.turn_gating` when published |
| class families | empty | — | U | — | owner's decision (§6) |
| m_k, w_k, a_k, s_k | §6 table | — | P | iso; windows ×2 | brief §8.2 |
| π_k (background class prior) | uniform | — | P | — | D |
| f (clutter re-detection) | 0.2 | — | P | 0.05, 0.2, 0.5 | [OPEN] O-5 |
| κ, λ | 1, 0.5 | — | U | yes | brief Q1 |
| r* | 10 | m | U (placeholder) | 6–15 | brief Q7 |
| L_min | 10 | m | P | 5–20 | D |
| K₁, K₂, ε_J | 5, 5, 0.02 | —, —, exp. targets | D/P | depth1; ε_J | §11 |
| k (sector prefilter, §11.2) | 3 | — | D | — | top-K proxy |
| t_lat (fallback belief latency, §11.3) | 10 | s | P | — | two image strides at 0.5 m/s plus 2 s; used only when the stamp cannot be |
| d_g (contact guard, §11.3) | 1 | m | D | — | two cells |
| L_lag (tracking margin, §11.3) | 3 | m | R | — | Control `gov_Lmax` (E-08) |
| clock tolerance (§11.3) | 1 | s | D | — | — |
| horizon_max_lag_s (§11.3) | 120 | s | D | — | a stamp older than the newest `ros_time` by more is another clock |
| p_res, n_max, δ, ρ_rev, ρ_floor | 0.05, 3, 0.02, 0.5, 0 | — | P | yes | brief §8.6 |
| T_decide, latency_margin | 2, 2 | s, path s | D | — | brief §9.1; DESIGN §5.2 |
| answer / reconcile timeouts | 5, 10 | s | D | — | DESIGN §5.2 |
| belief / progress / odom max age | 5, 1, 1 | s | D | — | Belief ~1 Hz (C-2); Control 10 Hz (E-11) |
| transform_jump_m | 0.5 | m | D | — | — |
| calibration bisection iterations | 60 | — | D | — | §9.1 |
| Δs (sampling) | 0.25 | m | R | — | MCS `sampling.py` l.66 |

## 16. References used in this document

Statuses as recorded by the brief (§15, §19), unless a reading date is given here.
The superproject's `literature_conclusions.md` lacks Hollinger 2013, Ji 2007, Golovin & Krause
2011 and Jadidi 2019; it has Jadidi 2017 (arXiv) [full]. Hollinger et al. 2013 was read in full
from a user-provided PDF on 2026-11-04 (below; the brief lists it as abstract only); Ji 2007,
Golovin & Krause 2011 and Jadidi 2019 keep the brief's status.

- **NOAA OCS, *Hydrographic Survey Specifications and Deliverables* v2025.0.01, §8.3** — [full:
  §8.3 + "Data Gaps", read 2026-11-04,
  https://nauticalcharts.noaa.gov/publications/documents/HSSD_2025-0-01.pdf].
  - Supports: altitude 8–20 % of the range scale; reduced claimed coverage under a reduced
    effective range.
  - Does not support: a hull-mounted rule or a spacing formula. Used only by the optional
    `grazing_floor`.
- **Yordanova & Gips, IEEE RA-L 5(3) 2020, DOI 10.1109/LRA.2020.3003886** — [full: §III-A, read
  2026-11-04 from arXiv 2006.12896v1].
  - Supports: effective range = Pd curve ∩ threshold; a nadir gap covered by paired tracks.
  - Does not support: the 0.8 factor.
- **Full texts provided by the user, read 2026-11-04** (exact support and limits):
  - **Paull, Saeedi, Seto, Li, IEEE/ASME T-Mech 18(6):1827–1838, 2013, DOI
    10.1109/TMECH.2012.2213607** [R2].
    - Sec. III-C, eq. 22–24: the acute angle α between two track lines fuses two confidences.
      Parallel (α = 0) are "dependent", c = max(c¹, c²). Perpendicular (α = π/2) are
      "independent", c = 1 − (1 − c¹)(1 − c²). In between, a linear interpolation
      c = (2α/π)c²(1 − c¹) + c¹, stated as an assumption.
    - The folded model (§6) shares the endpoints: opposite-side and parallel looks are redundant,
      perpendicular looks are independent. Between them it is a step at sector boundaries
      (independent once in a different folded sector), not Paull's linear interpolation. This is
      a deliberate simplification, motivated by Paull and **not** calibrated by it.
    - Paull's Pd(y) is an ESPRESSO curve given only in figures. Their in-water runs stop at 95 %
      confidence.
  - **Bourgault, Furukawa, Durrant-Whyte, FSR, STAR 24:209–222, 2006** [R1].
    - Eq. 4–9: the non-detection update p(D̄|x) = 1 − p(D|x); Q_k = Π q_i; P_k = 1 − Q_k. This is
      the form of the residual used in §9.
    - Sec. 4.4: static target, greedy P = .59 vs the quasi-optimal horizon .77. They state that
      longer horizons give better trajectories. This motivates a lookahead; it is no evidence about
      this module's two-step policy.
  - **Hollinger, Englot, Hover, Mitra, Sukhatme, IJRR 32(1):3–18, 2013, DOI
    10.1177/0278364912467485** [R8].
    - Theorem 2: the adaptivity gap is e/(e − 1) for monotone submodular variance reduction under
      a cost budget, with a known stochastic viewing model, independent sensors, and observation
      cost dominating traversal cost (Sec. 4.2).
    - Their experiments: adaptive is "only marginally better".
    - **Here traversal cost dominates and the objective is signed, so the theorem does not apply.**
      It is context for why a tie with C2 is plausible.
  - **Sethuraman, Baldoni, Skinner, McMahon, ICRA 2024:8348–8354, DOI
    10.1109/ICRA57147.2024.10611077** [R4].
    - Simulation only: K = 6 views 60° apart.
    - Table II: informed views 0.95 accuracy / 0.93 recall vs random views with the same count
      0.92 / 0.90 (1.18 views on average, no SDs). This supports *having* the C4_rand ablation,
      not its expected outcome here.
    - Their "in excess of 30 minutes per object" sentence carries no citation in the paper.
  - **Fawcett, Myers, Hopkin, Crawford, Couillard, Zerr, IEEE JOE 35(4):863–876, 2010, DOI
    10.1109/JOE.2010.2069832** [R6].
    - Real Klein 5500 data, two-look fusion. A second look "significantly improves"
      classification when the training set represents the features. Fusing "two poor single-aspect
      results" did not help.
    - **No result as a function of the angle between looks**, so it cannot support
      "perpendicular is best".
  - **Wiig, Krogstad, Midtgaard, "Autonomous identification planning for mine countermeasures"**
    (IEEE/OES AUV 2012 per the brief; the venue is not printed on the PDF) [R5]. It supports the
    design's handling of turning, lead-ins and candidate filtering:
    - targets with confidence < 20 %, clutter and those outside the area are filtered out;
    - a minimum turning radius is respected;
    - lead-in and lead-out over the target (100 m for HUGIN);
    - extra cost for sharp turns (< 90°);
    - a GA-solved TSP;
    - a HUGIN 1000 sea trial, June 2012.
    The resolution threshold p_res, Dubins turns, l_in and the no-GTSP decision are this module's
    own choices, not Wiig's values.
- **Kapetanović, Mišković, Tahirović, Bibuli, Caccia, "A side-scan sonar data-driven coverage
  planning and tracking framework", Annual Reviews in Control 46:268–280, 2018, DOI
  10.1016/j.arcontrol.2018.10.012** — [full: all 13 pages, read 2026-11-12 from
  `~/Downloads/research_paper/sssCovPlanning.pdf`]. The closest precedent for replanning a
  side-scan coverage pattern online from the sonar data.
  - **What it does.** The survey starts as a lawnmower with lanes twice as wide as the sonar
    range (no overlap). When the data of the previous swath shows something "interesting"
    (information gain = local roughness of the backscatter map, Eq. 1–3), a wavefront planner
    (WRbNF-CPP) diverts the path to sonify it **from the opposite side**, to remove its sonic
    shadow, by diverging or by a loop-back. The baseline (CL-CPP) is the classical lawnmower
    that overlaps every sonar range.
  - **Its metric is coverage path length**, relative to CL-CPP. Analytical bounds (Eq. 8, 9):
    at best 50 % shorter (barren seabed, lines much longer than wide); at worst 70.7 % to
    100 % **longer**.
  - **Simulations** (Sec. 4.4.4: 100 maps per parameter pair, sonar range 50 m, 1.5 m/s, areas
    of 1.4–5.5 km²): 20–50 % shorter on average when at most 15 % of the sonar ranges of the
    area contain an object and the lanes are long; equal to the classical pattern at about
    30 %; **worse above that** (−40 % with objects everywhere, Fig. 8f). Shorter than the classical pattern in a
    little over a third of all runs for the longest lanes (Sec. 4.5). The worst cases fall
    below the predicted bound (Sec. 4.4.5).
  - **Replanning time** of the order of 10 s, called online because lanes are at least 50 m
    apart (Sec. 4.8).
  - **Assumptions** (Sec. 2.3–2.4): no nadir gap, perfect localisation, a point-mass vehicle,
    a flat sea floor, sparse objects; turns are not minimised.
  - **At sea** (Sec. 5.4, October 2016): only the line-following controller was tested, on
    the e-URoPe vehicle. The coverage planner itself was not run at sea.
  - **Supports:** that an adaptive side-scan survey pays when objects are sparse and stops
    paying when they are dense, with a measurable break-even; reporting worst cases beside
    means; comparing against a classical fixed pattern; a second look as the purpose of a
    detour.
  - **Does not support:** any detection probability or detection gain (it has no detector, no
    target evidence and no belief map: "interesting" is a binarised cost map); any dependence
    on the angle between looks (its second look is from the opposite side, which this module's
    folded model and Belief's ρ treat as the same line); this module's revisit rules; a
    two-pass orthogonal baseline (its baseline is a one-direction overlap pattern).
- **Not provided as full text:** Ji et al. 2007 [R9], Golovin & Krause 2011 [R10], Ghaffari
  Jadidi et al. 2019 [R11] (motivation only), IHO S-44 Ed. 6.1.0 §3.7 [R14] (context for 3σ; 6.2.0
  noted), Guo et al. 2017 [R13] (calibration matters). Their statuses stay as the brief records
  them.
- Vendor and secondary survey practice was seen and not used as evidence (the 2026-11-04 search): the 15/45 plan, 65–75 % of range, overlap calculators, "12.5 × towfish
  height".
