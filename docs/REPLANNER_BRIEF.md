# BlueBoat-Replanner — Build guide for Claude Code (thesis deliverable D6)

*Written 2026-10-09 from the project files (`00`–`03` context pack of 2026-10-09, `project_synthesis.md`,
`literature_conclusions.md`, `papers_to_be_checked.md`, `target_families_and_surrogates.md`), the
BlueBoat-Belief module, the design decisions the user already took, and the lessons of a throw-away
prototype. It is a **brief, not a specification**: everything marked *to analyse* is yours to verify
and decide (with the user where it says so). Where this guide and the repository disagree, **the
repository wins** — report the discrepancy.*

---

## 0. How to use this file

1. Read §1–§4 fully before touching anything. Then do **Phase 0** (§13): verify the items in §12
   against the local repository and ask the user the open questions.
2. Do not re-derive what is already decided (§5). Do challenge it if the code proves it wrong.
3. Keep it small. The scientific content is the value model and three rules (§8), not infrastructure.
   §11 lists what **not** to build.

---

## 1. The mission in one paragraph

Build **BlueBoat-Replanner**, a new submodule that reads **only the belief map** (CM-9) and decides,
at each decision epoch, whether the boat should **continue** the survey, **revisit a contact from a
chosen viewing bearing**, or **fly an extra coverage line** — and makes the boat fly it **during the
mission**, through the mechanism the control stack provides for changing the path being followed
(§7). It **modifies no other module**. The same code must also
generate the baselines (single pass, **two-pass orthogonal tuned seriously**, append-on-completion) so
every condition of the E4 comparison runs through one execution path.

**Done means:**
- a ROS-free deterministic core with tests;
- a thin ROS 2 node that runs a mission in simulation end to end;
- a ROS-free development loop for sweeps and ablations;
- the module's `.claude/` files;
- documentation of what is demonstrated vs assumed.

Results are always **model-conditional** (CM-13).

---

## 2. Sources and authority

| Source | Use it for |
|---|---|
| `project_synthesis.md` | **Single source of truth**: scope, spine (§2), E1–E7, design rules §8.1 (three rules), §8.3 (aspect rule from E3), §8.4 (metrics), §8.5 (settings), §8.6 (operations) |
| Context pack `00`–`03` (2026-10-09) | Current state, interface contract, CM-1…CM-16, how to add a module (`02` §5), simulator status |
| Superproject `.claude/CLAUDE.md` + each neighbour's `.claude/CLAUDE.md` | Authoritative for each module's internals and rules |
| **BlueBoat-Belief** (`.claude/CLAUDE.md`, `blueboat_belief/docs/MATH.md`, `INTERFACES.md`, `INTEGRATION.md`, the `.msg`/`.srv` files, `core/`) | Your only input. Read the code, not only the docs |
| BlueBoat-Control `blueboat_control/src/TRAJECTORY_SYSTEM.md`, `_custom_libraries/{path_generation,yaml_trajectory}.py`, `master_control.py` | How a path is actually served and followed (read-only) |
| BlueBoat-MCS `docs/05_trajectory_format.md`, `mcs/core/geo.py` | Trajectory file format; the project's equirectangular conversion (copy it bit-identically) |
| BlueBoat-SSS-Sim `docs/integration_guide.md`, `docs/topics.md`, `launch/full_mission_launch.py`, world `metadata.yaml` | Running headless sim missions; ground truth for evaluation **only** |
| `literature_conclusions.md` §5–§6, `papers_to_be_checked.md` | Literature directives (hypotheses, not conclusions) |

---

## 3. Project state that matters to the replanner (2026-10-09)

| Item | State | Consequence for you |
|---|---|---|
| D3 detector | Not built | Development and sim runs need a **detection emulator** (§10) |
| D4 Belief | Separate module **BlueBoat-Belief**, being (re)built in parallel (a v1 existed outside the repo; its status is unverified here). The context pack lists D4 as "no owning module" | **Phase 0:** inspect Belief's current state and record its revision. Contract-dependent items stay [OPEN] until Belief's interface is approved. **Phase 2 needs Belief's core installed and tested** (geometry test, dev loop). If absent at that point, stop and ask |
| D5 mission interface | Pattern Designer → `blueboat_trajectory/1` YAML → `path_generation` (`/path_request`, file **watched**) → `master_control` (path parameter τ, governor). Control is being **updated so the followed path can be changed during a run**; *how* is not fixed by this guide | **Phase 0: find out, in the Control version present, exactly how the path is changed mid-run** (§7), then build the executor on that mechanism. Never improvise another one |
| Control, MCS | Read-only for this module (whatever version is checked out) | Integrate only through the interfaces they expose (CM-3). If something is missing, write a request for the user |
| Simulator | Built and calibrated; aspect response predicted by physics, **not yet field-validated** (E3) | Aspect parameters are placeholders. E4 waits for validation, but development need not |
| Range setting | Implemented default 20 m in real and sim; **user decision: 20 m for every condition** | Usable ground swath ≈ 6 m … √(20² − depth²) |
| Interface rules | CM-1: new message types only in `blueboat_interfaces` (Control, frozen) | **Do not create a message package for this module** (decision for D6: JSON on `std_msgs/String` with a `"schema"` field, `02` §5.3). Note: Belief's own `blueboat_belief_interfaces` package is a deliberate owner decision (Belief brief §4.1) and is a *consumed* dependency here; the rule above concerns this module only. The CM-1 wording is to be updated by the user |

---

## 4. Scientific frame and non-negotiables

**Why a replanner exists** (synthesis §2):
- A surface vehicle cannot choose its altitude, so in shallow water shadows starve.
- Detection therefore relies on the **aspect-dependent** specular highlight.
- One pass misses targets seen from an uninformative heading; multi-aspect revisit is a detection
  requirement.
- Deciding *which contact, from which heading* is the contribution.

**Rules you must implement or respect:**
1. **Investigation, not advocacy.** "Adaptive does not beat two-pass" is a publishable result. Never
   tune a parameter to make the adaptive policy win.
2. **CM-12:** the baseline is **two-pass orthogonal, tuned seriously**, built by the same coverage
   generator, spacing, speed, turns and execution path as the adaptive policies.
3. **CM-9:** the belief layer is the only thing the replanner reads. Never read detections, sonar
   data or sim ground truth online. **Online code never reads sim-only topics** (`ground_truth/*`,
   `/sim/*`); only `eval/` may read ground truth, and only from files.
4. **Three explicit rules are the scientific content** (synthesis §8.1): *resolution* (when a contact
   is done), *budget* (share of time for revisits), *abandonment* (when to give up). No unbounded
   "revisit hard areas".
5. **Aspect rule from measurement** (synthesis §8.3). Until E3 exists, every aspect parameter is a
   **labelled placeholder in configuration**, and an aspect-agnostic planner variant must exist.
6. **Planner model ≠ world model is configuration, not code.** This enables the oracle, degraded
   and aspect-agnostic ablations (synthesis §7.4).
7. **CM-13:** every result is stated model-conditionally. **CM-14:** iteration happens offline
   (rosbags, dev loop), never in the water.
8. **Superproject rules:**
   - Sessions never `git commit`/`push`.
   - Field data and world folders are write-once (CM-7).
   - Every stochastic component takes a seeded `numpy.random.Generator`.
   - Outputs carry a manifest (config, seed, versions).

---

## 5. Decisions already taken by the user (do not re-open without evidence)

| # | Decision |
|---|---|
| Strategy | **Contact-centric, aspect-aware value per unit time**, plus an aspect-resolved search term for undetected targets; short lookahead; every condition through the same machinery. Rejected: full POMDP and learned policy (cannot be validated with E3-sized data; would learn the simulator) |
| Q1 | Objective weights: false-confirmation cost **κ = 1**, search weight **λ = 0.5**; both swept |
| Q2 | Fair comparison at **equal mission time** = duration of the tuned two-pass plan; also report **anytime curves** (metric vs time used) |
| Q4 | The replanner computes its own **orientation-resolved residual** from fields Belief already publishes; it does not ask Belief to change |
| Q5 | A **detection emulator** stands in for D3 (dev loop and simulation), clearly labelled |
| Q6 | The replanner **generates the coverage for every condition** (no MCS-designed baseline files) |
| Q7 | Revisit ground range **r\* = 10 m**, a placeholder until the near-field floor is measured |
| Q8 | **One sonar range setting, 20 m**, for every condition |
| Q9 | **Mission areas are defined at mission launch** (a mission file of WGS-84 polygons); every path stays inside them, with a margin |
| Q10 | Decisions are taken at segment ends (line / leg), never mid-line, and executed **within the same run** by changing the followed path (§7) |

---

## 6. Interfaces — inputs, outputs, connections

### 6.1 Graph position

```
sonar ─► sss_processor ─► /sss_processor/processed ─┐
detector (D3) or emulator ─► detections ────────────┼─► BlueBoat-Belief ─► /belief_map/belief_grid ─┐
/blueboat/odom ─────────────────────────────────────┘                                              │
                                                                                                   ▼
 mission file (areas, condition) ─────────────────────────────────────────────────► BlueBoat-Replanner
                                                                                                   │
   ┌── next path segment, via Control's path-change mechanism (verified in Phase 0, §7) ◄──────────┤
   │                                                         /replanner/plan, /replanner/decision ◄┘
   ▼
 path_generation (/path_request) ─► master_control ─► thrusters
   ▲ progress along the path (however Control exposes it) ── back to the replanner (§7)
```

### 6.2 Inputs

| Input | Type / format | Use |
|---|---|---|
| `/belief_map/belief_grid` (name [OPEN]: Belief brief says "e.g.") | `blueboat_belief_interfaces/BeliefGrid` (reliable; **transient local is [OPEN]**, Belief may publish periodically; verify QoS) | Geometry, datum, odom→map transform, `coverage` (= 1 − Q), `aspect_counts` (cell-major, S = 12 per cell), `p_target` (NaN outside Belief's polygon), `contacts[]` |
| `BeliefContact` (inside the grid) | uid, x, y, 2×2 covariance, `p_exists`, `state` (candidate / confirmed), class names and probabilities, `sectors_detected`, `sectors_missed`, `max_confidence`, `has_shadow` | Candidate contacts and their look history |
| `/belief_map/query_belief` | `QueryBelief` (optional) | Disc queries; you can do without it |
| `/blueboat/odom` | `nav_msgs/Odometry`, local ENU, absolute yaw | Boat pose at a decision. ⚠ Reads zero in real runs today (open issue); in that case fall back to the pose at the end of the last run plan |
| Mission file | YAML (§6.4) | Areas, condition, overrides |
| Belief's configuration file | YAML, read-only | Shared prior density, class list, sector count (single source of shared values) |

### 6.3 Outputs (existing types only)

| Output | Type | Notes |
|---|---|---|
| Path changes | **Whatever Control provides** for changing the followed path mid-run (service, topic or file — verified in Phase 0, §7). Samples dense `[t, x, y, yaw]` in the **world/odom frame** unless Control says otherwise | All calls go through one executor adapter (§7). Log every request and Control's answer. **Never write MCS-owned files** (`.deployed/`, `.session/run.yaml`) |
| Plan archive | `blueboat_trajectory/1` YAML of the initial plan and of every accepted change, with `geo_anchor` | Module's own output folder (data root, `O_EXCL` names, never overwrite) — for replay and the report |
| `/replanner/plan` | `nav_msgs/Path` (frame `map` or `world`, decide and document) | For RViz/GCS display. Latched |
| `/replanner/decision` | `std_msgs/String`, JSON with `"schema": 1` | One record per decision: candidates, values, times, chosen action, rules fired, budget state. A rosbag is then a complete decision log |
| `/replanner/status` | `std_msgs/String`, JSON with `"schema": 1` | State machine (idle / running / deciding / done / stopped / failed) with a reason, including rejected path changes |
| `~/start`, `~/stop` | `std_srvs/Trigger` | Arm and stop a mission |

Document every topic in `docs/topics.md`, and list each one for the root `CLAUDE.md` §4 tables. The
user edits the root; you propose the lines.

### 6.4 Mission file (proposal)

```yaml
mission:
  name: beach_2026_10_12
  condition: C4            # C1 | C2 | C3 | C4 | C4_iso | C4_rand  (+ dev-loop-only ablations)
  areas:                   # >= 1 simple polygon, [latitude, longitude], WGS-84
    - [[34.50000, 133.50000], [34.50000, 133.50098], [34.50063, 133.50098], [34.50063, 133.50000]]
  config: {}               # overrides of the replanner configuration, same keys; unknown keys raise
```

### 6.5 Frames — the thing most likely to go wrong

- **Belief map frame:** ENU at Belief's datum. [OPEN] Belief's brief recommends the mission `geo_anchor`
  (the v1 default, polygon centroid, was rejected there); verify what the checked-out Belief uses.
  Belief publishes the datum and the `odom → map` transform. Its form is [OPEN]: translation only if real
  `/blueboat/odom` yaw is absolute ENU, otherwise `x_map = R(θ)·x_odom + t`. Implement the general form
  (R = I allowed) and read what Belief publishes; do not assume either.
- **World / odom frame** (Control, trajectory files): local ENU at the launch point. In sim, world
  (0,0) = the world's `geo_anchor` = spawn.
- **WGS-84 ↔ local:** the project's single equirectangular formula (sphere R = 6378137 m, longitude
  scaled by cos of the anchor latitude). Keep your own copy and **assert it bit-identical to
  `mcs/core/geo.py`** in a test.
- **Plan in the map frame** (where the evidence is), **write trajectories in world/odom**, and convert
  in one place with one transform. Test the round trip.

---

## 7. Executing plans — verify first, then implement (Phase 0 → Phase 4)

Control is being updated so the boat can **change the path it follows during a run**. This guide
deliberately does **not** say how that works: the mechanism is whatever the Control version in the
workspace implements. Your job is to **find it, understand it, and adapt to it** — never to guess it,
work around it, or build a second one.

### 7.1 Verify (Phase 0, from the code — not only the docs)

Answer each question with file and symbol references, in `docs/DESIGN.md`:

1. **How is the followed path changed mid-run?** Start from `blueboat_control/src/TRAJECTORY_SYSTEM.md`,
   `path_generation.py`, `master_control.py`, the Control `.claude/` files and `blueboat_interfaces`
   (new `.srv` / `.msg`?), and the Control changelog / git log. Identify the entry point (service,
   topic, file, parameter), who may call it, and its request/response or message fields.
2. **What can be changed, and from where?** Replace the remainder from a future point, append after
   the end, replace everything? How is the change point expressed (path parameter, time, index)?
3. **Acceptance rules.** Minimum lead ahead of the boat, position / heading continuity, speed limits,
   sample limits, frames. What happens on rejection — does the old path continue unchanged?
4. **Progress feedback.** How the replanner knows where the boat is **along the path** (path
   parameter, segment reached, end reached). If nothing is exposed, say so — do not infer it from
   odometry silently.
5. **Ownership.** What happens if the MCS file watcher or an operator also changes the path; who wins.
6. **Behaviour at the end of the path** (hold, stop, loop) and after a manual override / E-STOP.
7. **Is it available in simulation and on the real boat** with the same interface?
8. **How was it tested** in Control (tests, demos), and what the Control docs list as limits.

If **no mid-run mechanism exists** in the checked-out Control, **stop and tell the user** — do not
fall back to rewriting the watched file (it replaces the whole function at the current τ and makes the
reference jump) and do not start a second `/path_request` server.

### 7.2 Implement (design in Phase 1, code in Phase 4)

- **One executor adapter** (`ros/executor.py` or similar) is the *only* code that knows the mechanism.
  It turns a `Plan` (sequence of segments, world/odom frame) into the calls Control expects, reports
  progress events (`segment_done`, `end_reached`) and acceptance / rejection to the node.
- The **core stays indifferent to execution**: `decide(belief, boat_pose, plan_state, config) →
  decision`. If Control's mechanism changes later, only the adapter changes.
- **Decision timing** (Q10): decide at the end of a segment, early enough that the change point is
  ahead of the boat by at least Control's minimum lead. Design how: e.g. decide when the boat is
  within a configurable lead of the segment end, splice at that end.
- **Continuity:** every new segment starts **on the current path at the change point** (or exactly as
  Control requires). Test it offline against the acceptance rules you found.
- **Rejections are normal events:** log them, keep following the current plan, re-decide at the next
  epoch. Never retry blindly in a loop.
- **Fairness (CM-12):** C1 and C2 are executed through the **same adapter** (as an initial plan,
  plus appends if that is how the mechanism works), so every condition pays the same execution costs.
- **Real water:** start, stop and any change of mission scope remain operator actions. The replanner
  only changes the path within a mission the operator started, and stops on `~/stop`, manual override
  or E-STOP. Confirm with the user whether autonomous changes on the real boat are allowed for E6 or
  must be confirmed one by one.
- **Offline test of the adapter:** a stub of Control's interface that enforces the acceptance rules
  you found, so Phase 2–3 tests can exercise splices without ROS.

## 8. The algorithm (main ideas — specify precisely in `docs/MATH.md`)

### 8.1 Three models, kept apart

| Model | Where | What |
|---|---|---|
| **World** | `eval/` only | True targets (class, orientation), clutter, detection draws, calibrated confidences |
| **Belief** | BlueBoat-Belief | How evidence is fused and contacts declared (placeholder PD(range), ρ) |
| **Planner** | this module's config | Predictions of what a candidate pass would observe |

Ablations change **only the planner block**:
- matched (planner = world);
- degraded (windows ×2, PD −20 %);
- aspect-agnostic (window ≡ 1);
- random bearings (matched number of revisits).

### 8.2 Planner sensor model (placeholders until E3)

- Detection probability of a look by a class-k target with unknown orientation φ (12 sectors,
  uniform prior): `PD_k(r, b, φ) = PD_r(r) · s_k · A_k(b − φ)`.
- `A_k` is an aspect window: 1 within a half-width `w_k` of one of `m_k` specular directions, else a
  floor `a_k`; `m_k = 0` means isotropic.
- `s_k` is a class detectability scale; `PD_r` is a range window (default = Belief's: 0.7 plateau,
  3–25 m).
- Default classes (Belief's list):

  | Class | Symmetry `m_k` | Half-width `w_k` | Floor `a_k` | `s_k` |
  |---|---|---|---|---|
  | block | 4 | 20° | 0.4 | 1.0 |
  | pipe | 2 | 10° | 0.1 | 1.0 |
  | cage | 4 | 25° | 0.3 | 0.9 |
  | tyre | isotropic | — | — | 0.6 |
  | plate | isotropic | — | — | 0.3 (low-detectability control) |

- **Look dependence:** a look from a sector already seen for that cell or contact adds nothing.
  ⚠ Belief's ρ treats **opposite-side looks** (Δbearing = 180°) as fully independent. Paull's angle
  model (track angle in [0°, 90°]) treats them as dependent. Your planner model must state its choice
  explicitly. Folded (conservative) is recommended, and you must not inherit Belief's choice silently.

### 8.3 Contacts — expected confirmations

For each candidate contact c:
- **Orientation-class posterior:**
  `λ_c(k, φ) ∝ P(k|c) · π_φ · ∏_{s∈detected} s_k A_k(b_s − φ) · ∏_{s∈missed} (1 − PD̄ s_k A_k(b_s − φ))`.
- **Probability it is real:** p̃_c, from its first confidence q₀ (assumed calibrated) and its look
  record, against a clutter process that re-detects with probability f per new look.

Value of a set of new looks at c:

  `V_c = p̃_c · P(≥ 1 detection | real) − κ · (1 − p̃_c) · P(≥ 1 detection | clutter)`

Confirmed contacts are worth 0 (resolved). This targets what E4 counts: **confirmed real targets**
and **false confirmations**.

### 8.4 Background — aspect-resolved search value

Undetected targets per cell: prior `p₀ = density · Δ² / 10⁴`.
- For seen sectors S_i (from `aspect_counts`):
  `L_i(k, φ) = ∏_{s∈S_i} (1 − p_eff,i · s_k A_k(b_s − φ))`.
- `p_eff,i` is chosen so the aspect-agnostic case reproduces **Belief's own residual Q_i**:
  `1 − Q_i^{1/|S_i|}`.
- Then `q_i` = P(undetected target still in i), with its class-orientation mix `w_i ∝ π L_i`.
- Expected new detections of a path: `N(P) = Σ_i q_i Σ_{k,φ} w_i [1 − ∏_{new looks}(1 − PD)]`.

With A ≡ 1 this is ordinary residual-risk gap filling. That is why one code base covers both.

### 8.5 Candidates, cost, selection

- **Candidates at an epoch:**
  - `REVISIT(c, b)`: a straight leg passing c at ground range r\* so the look bearing is b. The
    straight section is `2·max(3σ_along, L_min/2)` (S-44 §3.7) plus lead-in and lead-out. **Both
    sides count**: generate the port and starboard versions and keep the cheaper feasible one. Only
    bearings not yet seen.
  - `RECOVER(j)`: one line of the orthogonal pass (lets the planner reproduce two-pass when that is
    best).
  - `CONTINUE`.
- **Feasibility:** every point of transit and leg inside the mission areas with the margin; routes
  are shortest paths inside the areas (a visibility graph over inward-offset vertices is enough).
- **Value and cost of a candidate path P:**
  - `V(P) = Σ_c V_c + λ · N(P)`, counting every look along P, transit included;
  - `T(P) = length/U + Σ_turns (|Δψ|/ω_turn + t_settle)`, including the turn into the path and
    any switching cost the path-change mechanism imposes (§7), identically for every condition.
- **Selection:** compare value **rates** V/T against the rate of continuing the current plan, with
  hysteresis. Use a **two-step lookahead** over the top-K candidates (states updated over the first
  action's outcomes). **Execute only the first action, then decide again** (receding horizon — see
  §9.1).
- **Ordering:** a TSP/GTSP is not needed at 3–10 contacts. Nearest-feasible plus the lookahead is
  enough.

### 8.6 The three rules (+ mission stop)

| Rule | Default (all configurable, swept) |
|---|---|
| **Resolution** | Contact confirmed by Belief, or p̃_c < 0.05, or no positive-value bearing left |
| **Abandonment** | ≥ 3 revisits of the contact, or two consecutive revisits that moved p̃_c by < 0.02 (a relative-information-contribution analogue, Jadidi et al. 2019) |
| **Budget** | Elapsed + T(candidate) ≤ total budget (= the two-pass duration); revisit time ≤ 50 % of the budget; selection by marginal rate |
| **Mission stop** | Remaining time < the shortest feasible action, or every rate < a floor |

### 8.7 Conditions (one code base)

| Condition | Policy |
|---|---|
| C1 | Single-pass boustrophedon (reference only) |
| **C2** | **Two-pass orthogonal**: spacing 0.8 × usable swath, 1.0 m/s; its duration defines the budget |
| C3 | Pass 1, then one revisit leg per candidate at the bearing perpendicular to its first detection (nearest-neighbour order), then recover lines while the budget lasts |
| C4 | §8.5 at every epoch |
| C4_iso, C4_rand | Ablations (window ≡ 1; random bearings), deployable |
| C4_degraded, C4_depth1 | Dev-loop ablations |

---

## 9. Lessons and pitfalls (from the Belief code and a throw-away prototype)

### 9.1 Defects the prototype had — avoid them

1. **The lookahead executed both actions.** The best pair must only *value* its first action; execute
   that action alone, then re-decide with what it actually observed. Symptom of the bug: one-step
   beat two-step.
2. **Budget overrun.** The remaining-plan time ignored the turn into it; include the entry turn.
3. **Slow scoring.** `np.add.at` scatter-adds dominated runtime; group rows by cell with
   `np.add.reduceat`. Budget ≈ 1–2 s per decision, ~25–50 s per dev-loop mission.

### 9.2 Belief facts that bite (verify in its code)

- Belief updates its map **one ping at a time** and changes its state. Do not score hypotheticals by
  pushing fake pings through it. Write a **vectorised look predictor** with Belief's conventions:
  - sample position `p + r·(cos b, sin b)`;
  - look bearing `b = ψ + s·π/2` (s = +1 port, −1 starboard);
  - Belief's sector formula, 30° sectors, sector 0 = looking East.

  **Test it against what the real Belief core records for the same pass** (aim > 97 % agreement
  both ways).
- `p_target` **mixes background and contacts**. Recompute the background from `coverage` and
  `aspect_counts`.
- `n_looks` counts same-sector repeats; use `aspect_counts` / `n_aspects` for diversity.
- Contact look history is **30° sector sets only** (no bearings, ranges, ρ or log-odds); a missed
  look is stored at its sector centre.
- A detection from a sector **previously missed** adds no evidence in Belief.
- `confirmed` = ≥ 2 position-consistent detected looks **or** a shadow. A same-aspect repeat
  confirms, so `confirmed` is not an information measure.
- Pruned contacts vanish without trace; uids are never reused.

### 9.3 Control and stack facts that bite

- Trajectory samples: `t[0] = 0`, strictly increasing, constant speed on straight edges, **yaw =
  direction of travel**. Yaw is interpolated between samples, so put a corner on **two samples a
  step apart**, not across a leg. Optionally hold at a corner for the cost model's turn time, so
  planned durations equal budget accounting.
- ⚠ **Detection topic types.** The project contract has `/sss_ai/detections` as
  `vision_msgs/Detection2DArray` (GCS listens). Belief may subscribe a different type on the same
  name. Verify which topic and type Belief consumes **in this workspace**. Your emulator publishes
  exactly that, and never two types on one topic name.
- QoS must match at both ends (CM-4). The latched belief grid needs a transient-local subscriber.
- Launch files import `simple_launch` / `launch` optionally, or offline gates die on `lark`.
- In a ROS-sourced shell, run gates with `/usr/bin/python3` and `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

### 9.4 Evaluation pitfalls

- **Uncalibrated emulator confidences make the planner chase clutter** (it trusts q as P(real)). Make
  the emulator's confidences calibrated against its own world, or say clearly that they are not.
- Compare conditions on **paired seeds**; report n, mean ± sd, paired better/equal/worse, targets
  found that C2 missed and vice versa, FP per mission, anytime recall at 25/50/75/100 % of the budget.

### 9.5 Prior exploratory evidence (prototype dev loop, model-conditional, n = 6, not significant)

- At the full two-pass budget, two-pass led: recall 0.73 vs 0.69. Adaptive confirmed ~0.8 more
  clutter per mission.
- Adaptive reached about **2× the recall at 25–50 % of the budget**.
- Informed bearings beat random ones at the end: 0.69 vs 0.63.

**Implication:** the **budget sweep** is the most informative experiment. Hollinger et al. (2013) bound
the benefit of adaptivity under a cost budget, so a tie at full budget is a plausible true result.

---

## 10. Development loop and emulator (`eval/`)

- **Dev loop (ROS-free, model-conditional):** world → pings along the plan → the **real Belief core**
  (imported from BlueBoat-Belief, *in `eval/` only*) → replanner core → next segment (spliced like the
  executor would, through the Control stub of §7.2).
  - The boat flies the plan exactly at nominal speed, with the planner's turn model.
  - Used for development, property tests, ablations and sweeps — **never reported as E4**.
- **Emulator node (sim, until D3):**
  - subscribes `/sss_processor/processed`;
  - draws one detection per object per pass when the object is abeam (shared function with the dev
    loop);
  - publishes on the topic and type Belief consumes.
  - Objects come from the world folder's `metadata.yaml` (a file, allowed in `eval/`), converted to
    the odom frame.
- **Runner:** `--seeds`, `--variants`, results appended as JSONL (resume-able), summary table and
  anytime plot, manifest. The full E4 campaign belongs to D7; keep the runner minimal and reusable.

---

## 11. Possible structure (to analyse — follow `02` §5.2 conventions)

```
BlueBoat-Replanner/
├── .claude/CLAUDE.md          ≤ ~200 lines: purpose, interface table, NC list, commands, status
├── .claude/TODO.md            open items only, [P0]/[P1]/[P2]
├── .claude/rules/*.md         path-scoped rules (core purity, models apart, frames/trajectory contract)
├── .claude/settings.json      optional Stop hook running the gate
├── docs/                      DESIGN.md · MATH.md · topics.md · INTEGRATION.md · REPORT.md (at the end)
└── blueboat_replanner/        ament_python package
    ├── package.xml · setup.py · setup.cfg · resource/
    ├── blueboat_replanner/
    │   ├── core/   config (validated dataclasses, unknown keys raise) · geo (copy of MCS) · areas & routing ·
    │   │           looks (vectorised predictor) · evidence (BeliefGrid → arrays) · model (sensor model) ·
    │   │           value · coverage · candidates · policy (conditions, rules, selection) · trajectory (export)
    │   ├── io/     mission file, trajectory YAML writer, belief-config reader, manifests
    │   ├── ros/    msg adapters (duck-typed, testable without ROS) · executor (the only code that knows
    │   │           Control's path-change mechanism) · replanner_node (thin)
    │   └── eval/   world + emulator · devloop · runner · emulator_node     ← only place with randomness
    ├── config/     replanner.yaml (every default) · mission_example.yaml · world_example.yaml
    ├── launch/     replanner_launch.py · emulator_launch.py
    └── test/       smoke_test.py (gate, `python3 -m test.smoke_test`, no ROS) + unit tests
```

**Belief import boundary [OPEN, ask the user].** Belief's rule CM-3 says consumers never need to import its
code. This guide imports the Belief core in `eval/` only (dev loop and the geometry test of §13). Confirm this
is permitted, or use a vendored/test-fixture route. Online code never imports Belief.

**Deliberately not built:**
- a message package;
- a mission manager inside another module;
- a GTSP solver, a POMDP, learning;
- a GUI;
- a second `/path_request` server, or any path-change mechanism of its own (use Control's);
- anything that moves the real boat without an operator action.

---

## 12. Verify in Phase 0 (local session, write answers into `docs/DESIGN.md`)

1. **Path execution — first and most important.** Every question of §7.1: how the followed path is
   changed mid-run in the checked-out Control, its acceptance rules, progress feedback, ownership,
   end-of-path behaviour, sim/real parity. Also: how to start a headless sim mission with an initial
   plan (`full_mission_launch.py` arguments). **If no mid-run mechanism exists, stop and ask.**
2. **`blueboat_trajectory/1` exact schema** (`yaml_trajectory.py`, MCS `docs/05_trajectory_format.md`):
   required keys, `geo_anchor`, `loop`, units, yaw convention.
3. **BlueBoat-Belief.**
   - Location and build.
   - Node and topic names (expected `/belief_map/belief_grid`), QoS.
   - Detection topic and type it subscribes; alignment mode in sim (fixed) and real.
   - How its datum relates to the world `geo_anchor`.
4. **`mcs/core/geo.py`:** the exact formula, to copy and assert.
5. **World `metadata.yaml`** (`blueboat_world_meta/1`): object positions, headings, classes (for the
   emulator and evaluation).
6. **BlueBoat turn rate and settle time** from existing logs (poslog CSV, `/monitoring_data`
   recordings). Defaults 30 °/s + 2 s are placeholders.
7. **Whether `/blueboat/odom` is valid in sim** (expected yes) and in real runs (known issue).

---

## 13. Working method with Claude Code

**Phases and stop gates:**

| Phase | Output | Gate |
|---|---|---|
| 0 Verify | §12 answers, open questions asked (§14), discrepancies listed | **Stop for user answers** |
| 1 Design | `docs/DESIGN.md` (decisions, execution model, structure, interfaces), `docs/MATH.md` (value model, rules, test properties) | **Stop for approval — no implementation before** |
| 2 Core | `core/`, config, smoke test: closed-form checks, geometry vs Belief, determinism | Gate green |
| 3 Dev loop | emulator, dev loop, runner; C1–C4 + ablations on paired seeds | Report to the user (model-conditional) |
| 4 ROS | node, adapters, **executor on Control's path-change mechanism**, launch; one C2 and one C4 mission in Gazebo with mid-run changes | Live check with the user |
| 5 Handover | `.claude/` final, `docs/INTEGRATION.md`, `REPORT.md` (demonstrated vs assumed), proposed root-table lines | — |

**Tests the core must pass** (property tests, not just coverage):
- An isotropic contact values every unseen bearing equally.
- A two-fold (pipe-like) contact's best next bearing is the one the window predicts.
- With A ≡ 1, the search value equals the aspect-agnostic expected detection.
- A seen sector adds no value.
- "Continue" is chosen when nothing beats the continue rate.
- Abandonment bounds revisits.
- C1/C2 never change their plan.
- Identical inputs give bitwise-identical decisions.
- Every exported path is inside the areas and satisfies the trajectory contract.
- The predicted looks match the Belief core (> 97 % both ways).
- The geo copy is bit-identical to MCS.

**Claude Code tips for this module:**
- **CLAUDE.md:** use the project skeleton (`02` §5.7); keep it **under ~200 lines**. Put
  directory-specific instructions in **`.claude/rules/*.md` with a `paths:` frontmatter**; they
  load only when matching files are touched. Use `@path` imports sparingly (imported files still
  load at launch). Update the *Status* section at the end of every session.
- **Plan mode** (`Shift+Tab` until "plan mode on", or `claude --permission-mode plan`) for Phases 0–1:
  read and propose, no edits until approved.
- **Subagents:** use the *Explore* agent for read-only sweeps of the neighbour modules (Control, MCS,
  Sim, Belief), so findings come back without filling the main context. Use a separate review agent
  for high-stakes checks (the value model vs `MATH.md`; fairness of C2 vs C4).
- **Hooks:** an optional `Stop` hook in `.claude/settings.json` that runs the smoke test and exits 2
  on failure keeps the gate honest (the simulator module does this). A `PostToolUse` hook (matcher
  `Edit|Write`) can run a linter.
- **Context:** `/clear` between phases; keep durable findings in `docs/` and `.claude/TODO.md`, not
  in chat. Resume with `claude --continue`.
- **Ask, don't assume:** frozen modules, interface changes and anything touching the real boat go to
  the user.

---

## 14. Open questions (ask the user in Phase 0; recommendation first)

1. **Path-change mechanism** (§7.1): summarise what you found and any limit that constrains the
   design (lead, continuity, progress feedback). Anything missing becomes a request for the user —
   never a workaround.
2. **Decision lead** (§7.2): how far before a segment end to decide. *Recommendation:* the smallest
   lead Control accepts plus the decision time, same for every condition.
3. **Real water:** may the replanner change the path autonomously within an operator-started mission
   (E6), or must each change be confirmed? *Recommendation:* autonomous within the mission areas,
   with stop on manual override / E-STOP, after a supervised dry run.
4. **BlueBoat-Belief:** where is it in the workspace, and which detection topic and type does it use
   here?
5. **Placement of the dev loop and emulator:** in this module's `eval/` (*recommended*), or in a
   future D7 evaluation module? Belief's brief asks the same question for its own dev-only synthetic
   detector (its [ASK] 5): decide **once** for both modules, so no two emulators exist.
6. **Importing Belief's core in `eval/`** (see §11): allowed under CM-3, or another route?
7. **Belief output contract still open** (Belief brief §4.2, §16): latched vs periodic publication,
   exact topic name, sector convention, `p_background` published or recomputed (§9.2 says recompute from
   `coverage` and `aspect_counts`; drop one source once Belief decides), per-contact look history fields
   (`sectors_detected`/`sectors_missed` here vs "sectors and ranges" there). Resolve with the Belief session.
8. **Append-on-completion, C4_rand boundary, S-44 edition:** see §§17.2, 17.3, 20.

---

## 15. References (verification status as recorded by the project)

| Reference | Status | What it supports |
|---|---|---|
| Bourgault, Furukawa, Durrant-Whyte (2006), *Optimal search for a lost target in a Bayesian world*, FSR, STAR 24 | full (project review) | Miss update; residual risk; lookahead beats greedy (static target) |
| Paull, Saeedi, Seto, Li (2013), *Sensor-driven online coverage planning for AUVs*, IEEE/ASME TMech 18(6) | full | Angle-dependent look fusion (assumed model); IG + completion term beats IG alone |
| Myers, Williams (2012), *Adaptive multiview target classification in SAS images using a POMDP*, IEEE JOE 37(1) | full | Terminal-decision reward; aspect choice = discrimination geometry; class-dependent stopping |
| Sethuraman et al. (2024), *Learning which side to scan*, ICRA | full | > 30 min per object reacquisition; informed vs random heading ablation |
| Wiig, Krogstad, Midtgaard (2012), *Autonomous identification planning for MCM*, IEEE/OES AUV | full | Filter contacts first; turn-penalised revisit ordering |
| Fawcett et al. (2010), *Multiaspect classification of sidescan images*, IEEE JOE 35(4) | full | Fusion cannot fix a poor classifier |
| Hollinger, Sukhatme (2014), *Sampling-based robotic information gathering*, IJRR 33(9) | full | Budgeted information gathering, submodularity |
| **Hollinger, Englot, Hover, Mitra, Sukhatme (2013)**, *Active planning for underwater inspection and the benefit of adaptivity*, IJRR 32(1), 3–18 | checked, abstract | Benefit of adaptivity bounded by a constant under a cost constraint → a tie with two-pass is plausible |
| **Ji, Parr, Carin (2007)**, *Nonmyopic multiaspect sensing with POMDPs*, IEEE TSP 55(6), 2720–2730 | checked, abstract | Target orientation as discrete sectors with stationary response — basis of the φ-sector model |
| **Golovin, Krause (2011)**, *Adaptive submodularity*, JAIR 42 | checked, abstract | Greedy guarantees need adaptive submodularity and item costs — not our path-dependent costs → short lookahead |
| **Ghaffari Jadidi, Valls Miro, Dissanayake (2019)**, *Sampling-based incremental information gathering…*, IJRR 38(6), 658–685 | checked, abstract (published) | Relative-information-contribution convergence; entropy-based mission stop |
| Yordanova, Gips (2020), RA-L 5(3) | full | Residual risk ∏(1 − Pd) as a map |
| IHO S-44 Ed. 6.1.0 (2022) | full | Exclusive Order 200 % search; search radius ≥ 3σ (§3.7) |
| Guo et al. (2017), *On calibration of modern neural networks*, ICML | full | Calibrated confidences (temperature scaling) |

Cite nothing from memory. Anything new is opened and marked [full] / [abs] / [checked] before it
enters `docs/`.

---

# Addendum — design-readiness, evidence, and reference clarifications

*Added 2026-11-03. This is an append-only supplement to §§0–15. The original text, decisions, project-specific names, paths, interfaces, constants, and cross-module references above are intentionally retained verbatim. This supplement does not claim that the repository has been inspected or that any project-status statement has been revalidated. Where a point below depends on the checked-out project, it is a question for Phase 0 or Phase 1, not a new fact.*

## 16. How to interpret and evidence this brief

### 16.1 Meaning of “reads only the belief map”

For the purpose of this brief, distinguish **environmental / target evidence** from **operational context**:

- The belief map is intended to be the replanner's only online source of environmental and target evidence. The replanner must not independently consume raw sonar, detector outputs, or simulation ground truth online.
- The operational inputs already listed in §6.2 (for example, boat pose / odometry, mission areas, execution progress, and read-only configuration) are not alternative sources of target evidence. They support localisation, mission constraints, execution, or configuration.
- This interpretation is a clarification of the wording in the brief, not an amendment to CM-9. In Phase 0, check the exact superproject rule and report any conflict. Do not weaken or reinterpret a cross-module rule without the user's decision.

### 16.2 Status labels for new design artefacts

Use the following labels in `docs/DESIGN.md`, `docs/MATH.md`, `docs/INTEGRATION.md`, and `docs/REPORT.md` wherever a statement could otherwise be mistaken for a verified fact:

| Label | Meaning | Minimum support |
|---|---|---|
| **[DECIDED]** | A decision already taken by the user and recorded in §5 or elsewhere by the user | Exact source / decision reference; reopen only if contradicted by evidence |
| **[CODE]** | Behaviour demonstrated by the checked-out implementation | Repository path, symbol, relevant line range, and revision / commit identifier |
| **[DOC]** | Behaviour or contract stated by a project document | Repository path and section / heading; note if contradicted by code |
| **[TEST]** | Behaviour demonstrated by an automated test | Test path, test name, command, and result |
| **[DEMO]** | Behaviour demonstrated in a simulation or supervised hardware run | Run identifier, configuration, logs / bag, and conditions |
| **[ASSUMPTION]** | A modelling or engineering assumption not yet demonstrated | State its consequence and how it will be tested / varied |
| **[OPEN]** | A question requiring evidence or a user decision | State the owner, options, recommendation (if appropriate), and consequence |

A design choice is not “verified” merely because it appears in this guide. A document statement is not proof that the corresponding code implements it. If the code and documents disagree, preserve both observations and report the discrepancy; do not silently make them agree.

### 16.3 Evidence and citation discipline

- For project behaviour, cite the checked-out repository, not recollection: file path + symbol + line range, and record the revision inspected. Use tests and demos where available.
- For scientific claims, cite the primary paper, standard, or official documentation. Record the full bibliographic metadata and DOI / stable publisher URL where available.
- Distinguish **the source's result** from **the replanner's use of it**. A paper may motivate a modelling choice without validating the replanner's equations, parameter values, simulator, or real-world performance.
- Retain the existing `[full]`, `[abs]`, and `[checked]` status language in §15. Add the date and exact material reviewed when changing a status. Do not upgrade a source from abstract-only to full-text review based on a search result or citation page.
- Do not cite generated summaries, model memory, or a secondary bibliography in place of the primary source when a primary source is available.

## 17. Items that the current brief does not fully specify

These are design questions / requirements to resolve in Phases 0–1. They are not instructions to choose arbitrary values. Where the repository or an existing user decision already answers an item, record the evidence and do not reopen it unnecessarily.

### 17.1 Requirements and decision traceability

For every consequential behaviour, record:

1. the requirement or decision;
2. its source and status label (§16.2);
3. the design element that implements it;
4. the test or experiment that checks it;
5. any known limitation or assumption.

A compact traceability table can live in `docs/DESIGN.md` and `docs/MATH.md`; a separate framework or requirements-management tool is not required. The aim is to prevent a requirement from disappearing between the brief, design, implementation, and report.

### 17.2 Baseline and condition semantics — clarification required

The introduction names three baselines: single pass, two-pass orthogonal, and append-on-completion. §8.7 defines C1, C2, C3, C4, C4_iso, and C4_rand, but does not explicitly map the phrase **append-on-completion** to a condition or fully define its event semantics.

Before implementation, ask the user to confirm:

- whether append-on-completion is C3 or a distinct condition;
- exactly what is appended, at which event, in what order, and under what time / feasibility limits;
- whether “completion” means a contact resolution event, completion of the initial coverage plan, or another event;
- whether a condition is a policy, an execution schedule, or both.

Do not infer this mapping from the condition names. Keep the C1–C4 descriptions in §8.7 unchanged unless the user confirms a correction.

### 17.3 Randomness and the C4_rand boundary — clarification required

The hard rule in the prompt places all randomness in `eval/`, while §8.7 calls C4_rand an ablation and labels it deployable. Resolve this apparent tension with the user before implementation.

The design must state separately:

- which components are allowed to be stochastic;
- whether C4_rand is an evaluation-only comparator or an operational policy;
- which random draws belong to world generation, detection simulation, and policy / bearing selection;
- how random streams are seeded and recorded.

A single seed value does not necessarily create a fair paired comparison if two policies consume random numbers in different orders. Define whether paired runs share the same world and scenario, how detection randomness is coupled (if at all), and how independent streams are derived. Do not change the “only in `eval/`” rule without a user decision.

### 17.4 Contact probability and clutter model — mathematical definition required

§8.3 introduces `p̃_c`, first confidence `q₀`, and a clutter re-detection probability `f`, but it does not yet give the complete update equations. `docs/MATH.md` must state, without ambiguity:

- the prior probability that a candidate contact is real;
- the mapping from the first reported confidence `q₀` to a likelihood or calibrated probability;
- the likelihood of each detected and missed look under the real-target and clutter hypotheses;
- whether observations are conditionally independent, and under which bearing / sector model;
- how class uncertainty and orientation uncertainty enter the update;
- the exact posterior update and its normalisation;
- how contact pruning, uid disappearance, and a Belief state change affect the planner's retained state.

Do not treat a classifier confidence as `P(real)` unless calibration evidence supports that interpretation. Do not invent a clutter model to complete the equations. If the detector / emulator does not expose the information required, record the limitation and ask how the model should proceed.

### 17.5 Aspect dependence and discrete-sector conventions

§8.2 correctly requires the planner to choose its own look-dependence model rather than silently inheriting Belief's `ρ`. `docs/MATH.md` should make the following explicit:

- whether bearings are directed headings or unoriented track lines;
- the exact angular difference operation (e.g. modulo `π` for line orientation, versus modulo `2π` for directed bearing);
- how the continuous aspect window is integrated or sampled over the 12 orientation sectors;
- how a sector-centre approximation is applied to missed looks;
- how duplicate looks, already-seen sectors, and opposite-side looks are treated;
- which choices apply to contact histories and which apply to background-cell histories.

Add hand-checkable tests for parallel, antiparallel, perpendicular, and near-boundary cases. In particular, test the 0° / 180° equivalence or non-equivalence that follows from the chosen convention. Do not conflate the Belief sector count with a claim that the target-orientation prior is empirically uniform; the latter remains a model assumption unless supported by data.

### 17.6 Background residual and double-counting

§8.4 derives an orientation-resolved residual from `coverage` and `aspect_counts`, while §6.2 also lists `p_target`, which the guide notes mixes background and contacts. `docs/MATH.md` must state exactly which grid quantities are used and which are ignored / recomputed.

The specification should demonstrate that:

- confirmed and candidate contacts are not counted again as undetected background targets unless explicitly intended;
- the background term is consistent with Belief's scalar residual in the aspect-agnostic case;
- the same observation is not counted once in the contact term and again in the map term;
- cells outside the valid Belief polygon, NaNs, zero-look cells, and cells at the mission-area boundary have defined handling;
- the prior density and cell area units are dimensionally consistent.

At minimum, include limiting-case tests for no looks, all sectors seen, `A ≡ 1`, zero prior density, and a cell with no valid Belief data.

### 17.7 Meaning of “confirmed” versus “true” and evaluation accounting

The guide says confirmed contacts have zero future value, while the target metrics include false confirmations. `docs/MATH.md` and the evaluation protocol must distinguish:

- Belief's operational state `confirmed`;
- whether a confirmed contact is a true target in the evaluation world;
- the planner's expected utility for an unresolved contact;
- the evaluation metric that counts a false confirmation.

A false confirmation may be operationally resolved (and therefore no longer revisited) while still counting as a false positive in evaluation. State this explicitly if it matches the intended scoring. Define how duplicate contacts, merged / split contacts, pruned contacts, and a contact that changes uid are counted; if the existing Belief contract makes any of these impossible or unobservable, cite that evidence.

### 17.8 Candidate feasibility and mission geometry

§8.5 states that the entire transit and leg must remain inside the mission areas with a margin, and proposes a visibility graph over inward-offset vertices. The detailed design still needs to define or verify:

- polygon validity requirements (winding, self-intersections, duplicate vertices, minimum area);
- whether areas may overlap, touch, contain holes, or be disjoint;
- whether the margin is a fixed distance, a configurable quantity, or already specified elsewhere;
- whether the margin applies to the centreline, the vehicle footprint, or another envelope;
- geometric tolerances and boundary-inclusion rules;
- how inward offsets behave for narrow corridors, concave polygons, and polygons that collapse after offset;
- what happens when a target / contact lies outside the feasible mission region;
- whether the visibility-graph route is shortest in Euclidean distance and whether it satisfies the boat's verified turning constraints.

Do not add support for holes, arbitrary polygons, or a more capable planner unless the mission format and user requirements need it. Define a clear rejection path for invalid or infeasible missions.

### 17.9 Path timing and cost accounting

The cost model includes straight-line travel, turns, settling, entry turns, and path-switching cost. Phase 1 must define what is included in the reported mission time and apply it consistently to every condition.

Explicitly account for:

- time already spent before a decision;
- time to reach the change point / decision point;
- the turn into a candidate and turns inside it;
- Control's verified minimum lead, acceptance latency, and any switching delay (only if Control exposes / measures them);
- settling, corner geometry, and any hold samples;
- rejected path changes and time spent deciding / waiting;
- the remaining unexecuted plan and the definition of “remaining time”.

The model should be calibrated against observed simulation / vehicle timing where evidence exists. Keep nominal model time and measured execution time distinguishable in logs and reports. Do not invent an execution penalty or a path-switch cost that the Control mechanism does not impose or demonstrate.

### 17.10 Input freshness, synchronisation, and degraded operation

The guide identifies QoS and odometry concerns but does not yet define freshness or consistency rules. Phase 0–1 must identify:

- timestamps and frame identifiers available on each input;
- how a BeliefGrid snapshot is associated with the boat pose and decision epoch;
- the maximum acceptable age of the belief grid, odometry, and execution-progress feedback (values require evidence / user decision);
- handling of out-of-order or repeated messages;
- handling of missing, stale, malformed, or mutually inconsistent inputs;
- how map-datum / transform changes are detected, if they can occur;
- whether a late-joining subscriber receives a usable initial BeliefGrid under the actual QoS configuration.

When inputs are not safe to use, the replanner must not issue a speculative path change. Define the exact hold / stop / continue-current-path response in `docs/DESIGN.md` after checking the operational contract. Do not silently fall back to odometry if the brief says progress feedback is unavailable; preserve the explicit warning in §7.1.

### 17.11 Execution state machine, concurrency, and fault handling

The status names in §6.3 are a useful starting point, but Phase 1 should define the transitions and guards between them. At minimum, resolve:

- whether a mission can be started before all required inputs and the executor are ready;
- when a decision is allowed to start and what prevents two concurrent decisions;
- how an outstanding path-change request is tracked until accepted or rejected;
- how duplicate / delayed responses are distinguished from current requests, using only fields / identifiers the verified Control interface supports;
- what happens on timeout, node restart, executor error, invalid plan, rejection, manual override, `~/stop`, and E-STOP;
- how the replanner knows whether the current plan is still authoritative;
- whether a restart resumes, stops, or requires a new operator start.

The exact mechanism and response semantics must come from the checked-out Control implementation. These are design questions, not permission to add a competing path server or change Control.

### 17.12 Safety envelope and operator authority

The existing safety rules remain controlling. Add an explicit safety table to `docs/DESIGN.md` that lists each trigger, the required replanner response, and the evidence / operator decision behind it. Include at least:

- no autonomous mission start;
- path changes confined to the operator-defined mission areas and margins;
- no path change when the input state is stale or inconsistent;
- no path change after manual override or E-STOP;
- response to executor rejection or unknown acceptance state;
- behaviour when the mission plan is exhausted;
- how an operator-requested stop is acknowledged and logged.

Do not define additional real-boat permissions. In particular, the question in §7.2 and §14 about whether path changes may be autonomous within an operator-started real-water mission remains for the user.

### 17.13 Output schemas, logs, and durable artefacts

For each output in §6.3, Phase 1 should specify the exact fields, units, coordinate frame, timestamp semantics, and schema version. For the JSON topics, define a small documented schema including required fields, optional fields, enum values, and compatibility behaviour when an unknown schema version is received.

For the plan archive and run manifest, define how to record at least the information actually available to the module:

- mission identifier and condition;
- effective configuration and any overrides;
- seed / seed-stream identifiers where applicable;
- initial plan and each accepted path change;
- decision / request identifiers and Control's acceptance or rejection result;
- relevant package / source revisions and software versions;
- frame / datum / `geo_anchor` metadata;
- start / end times and the reason for termination.

Keep the existing no-overwrite / `O_EXCL` requirement. Define how partially written files are handled (for example, a temporary file followed by an atomic finalisation) only after checking the platform and data-root conventions. Never write to files owned by another module.

### 17.14 Numerical behaviour and test properties

In addition to the properties in §13, define mathematical invariants for the core. Candidate properties to formalise in `docs/MATH.md` and tests include:

- all probabilities and posterior weights are finite and within valid bounds;
- normalised distributions sum to one within a stated numerical tolerance;
- expected detection value is non-negative where the objective definition requires it;
- repeated identical looks do not add information when the model says they are redundant;
- adding a genuinely new look cannot reduce the expected probability of at least one detection under the stated independent-look model;
- zero remaining budget yields no positive-duration action;
- infeasible candidates are never selected;
- a rejected path change does not mutate the core's accepted-plan state;
- equivalent inputs in equivalent frames produce equivalent decisions within the declared tolerance.

These are candidate properties, not a replacement for the existing tests. Check each property against the actual equations; do not assert monotonicity where a signed utility term or an explicit penalty makes it false. Document floating-point tolerances and why bitwise determinism is expected on the supported environment.

### 17.15 Evaluation protocol and statistical reporting

The existing paired-seed, equal-time-budget, anytime-curve, and budget-sweep requirements remain. Phase 3 should additionally state:

- the experimental unit (mission / world / target) and what `n` counts;
- how world generation, target placement, detector draws, and policy random streams are paired across conditions;
- whether scenarios are sampled independently and how failed / incomplete runs are treated;
- the primary metrics and their exact denominators;
- how contacts, target identities, duplicate detections, false confirmations, and missed targets are matched to ground truth;
- how mission time is measured and how actual overruns / rejected changes enter the metrics;
- whether paired differences, confidence intervals, effect sizes, or other uncertainty summaries are reported in addition to mean ± sd;
- which ablations are confirmatory versus exploratory and how parameter sweeps are reported.

Do not choose a sample size or claim statistical significance without a stated rationale and analysis. A result from the development loop remains model-conditional and is not E4. Keep the guide's separation between development-loop evidence and the full E4 campaign.

### 17.16 Configuration, manifests, and sensitivity

For every tunable value, record its source and category: user decision, measured value, repository default, literature-motivated value, or temporary placeholder. State units, valid range, default, and whether it is swept. Unknown configuration keys should continue to raise errors as §11 proposes.

The manifest should make a run reproducible without relying on chat history. Record the effective configuration rather than only the override file. Where dependencies or environment versions materially affect results, record them. Keep model-conditionally reported parameters distinct from field-validated measurements.

For parameters that materially affect policy selection (including the values already marked as swept), report sensitivity rather than selecting only the best observed setting. Preserve the serious tuning of C2 and the rule against tuning C4 to win.

## 18. Phase-gate additions

These checks complement §§12–14; they do not replace any existing Phase 0 question or stop gate.

### Phase 0 — evidence register (no code)

In addition to the existing verification list:

1. Record the exact repository revision and working-tree state inspected. Do not alter it.
2. Create an evidence register for every project-dependent statement used in the design: source path, symbol / section, revision, status label, and discrepancy if any.
3. Confirm the interpretation of “belief map only” against CM-9, the interface contracts, and §6.2.
4. Ask the user to resolve the append-on-completion mapping and the C4_rand randomness boundary before either is encoded as policy behaviour.
5. Check whether the cited S-44 edition is intentionally fixed at 6.1.0 or should be reviewed against the adopted 6.2.0 edition; do not change it automatically.
6. Report any reference whose complete bibliographic metadata or full-text support has not been verified. Do not inflate the stated review status.

The existing rule to stop if Control has no verified mid-run path-change mechanism remains absolute.

### Phase 1 — design artefacts (no code)

In addition to the existing deliverables:

- `docs/DESIGN.md` should contain the requirements/evidence traceability, input freshness rules, mission state machine, safety response table, path-change transaction model, and unresolved project questions.
- `docs/MATH.md` should define the full contact posterior, clutter likelihood, aspect dependence, residual background derivation, double-counting prevention, value / cost accounting, and numerical invariants.
- `docs/topics.md` should specify publisher / subscriber direction, exact message type, QoS, frame, timestamp use, and whether the source is online, evaluation-only, or file-based, using evidence from the actual workspace.
- Each design decision should be marked [DECIDED], [CODE], [DOC], [TEST], [DEMO], [ASSUMPTION], or [OPEN] as appropriate.

The existing approval stop gate still applies. No implementation begins before user approval.

### Phases 2–5 — evidence carried forward

- Phase 2 must implement the approved `docs/MATH.md`, not silently fill its gaps in code. Any newly discovered ambiguity returns to the user / design artefact before implementation continues.
- Phase 3 reports the complete effective configuration, seed protocol, scenario pairing, run counts, failures, and metric definitions alongside the summary statistics.
- Phase 4 records the exact verified Control interface, request / response behaviour, acceptance / rejection, progress events, and supervised test evidence. Simulation evidence does not establish real-boat safety.
- Phase 5 keeps demonstrated behaviour, repository documentation, assumptions, and untested claims separate in `docs/REPORT.md`.

## 19. Reference map and bibliographic enrichment

The original §15 list and its project-recorded review statuses are retained. The entries below add bibliographic identifiers and a topic map; **they do not change the original `[full]`, `[abs]`, or `[checked]` status**. Metadata and stable identifiers were cross-checked against available publisher, conference, institutional, or scholarly-index records on 2026-11-03. Recheck each entry against its DOI landing page or library database before final thesis submission. Before using any source to justify a specific equation or parameter, read the relevant full text and record the exact section / equation / page in the design or mathematics document.

### 19.1 Topic map

| Replanner topic | Relevant references | What they can support | What they do not establish for this project |
|---|---|---|---|
| Bayesian search and residual target probability | [R1] | Bayesian search framing and expected search utility for a target under uncertainty | The exact contact/clutter posterior in §8.3 or the BlueBoat belief contract |
| Online SSS coverage planning and angle-dependent evidence fusion | [R2] | Online underwater coverage planning; modelling the dependence of observations using the acute angle between track directions | The exact planner sensor model, the 12-sector orientation prior, or calibrated BlueBoat parameters |
| Adaptive selection of additional sonar views | [R3], [R4], [R5] | Multi-view/aspect-aware target observation and the value of choosing informative views | That this replanner's simpler model will outperform a fixed two-pass baseline |
| Multi-aspect sidescan fusion and classifier limitations | [R6] | Empirical multi-look fusion for SSS classification | That additional looks can compensate for an uncalibrated or systematically poor detector |
| Budgeted informative trajectory planning | [R7], [R8], [R10] | Planning under motion/time budgets; conditions under which greedy/adaptive methods have guarantees or bounded adaptivity gaps | A guarantee for this path-dependent cost, this signed value function, or this finite-K lookahead |
| Information-theoretic convergence / stopping | [R11] | A principled example of an information-based convergence criterion for informative planning | The exact 0.02 posterior-change threshold or three-revisit cap in §8.6 |
| Confidence calibration | [R13] | The importance of calibration and temperature scaling for modern neural-network classifiers | That temperature scaling is appropriate for every detector / emulator confidence output without validation |
| Survey search radius and feature search | [R14] | The context and wording of the IHO charted-object confirmation radius and survey feature-search requirements | A universal geometry rule for replanner segment length, or automatic equivalence between orthogonal passes and S-44 compliance |
| ROS 2 QoS and managed-node concepts | [R15], [R16] | General ROS 2 communication compatibility and lifecycle design concepts | The QoS or lifecycle behaviour of the checked-out BlueBoat nodes; the actual workspace remains authoritative |

### 19.2 Complete references and stable identifiers

**[R1]** F. Bourgault, T. Furukawa, and H. F. Durrant-Whyte, “Optimal Search for a Lost Target in a Bayesian World,” in *Field and Service Robotics*, Springer Tracts in Advanced Robotics, vol. 24, pp. 209–222, 2006. DOI: [10.1007/10991459_21](https://doi.org/10.1007/10991459_21). The proceedings list the chapter on pp. 209–222. This is the 2006 Springer volume cited in the original guide.

**[R2]** L. Paull, S. Saeedi, M. Seto, and H. Li, “Sensor-Driven Online Coverage Planning for Autonomous Underwater Vehicles,” *IEEE/ASME Transactions on Mechatronics*, vol. 18, no. 6, pp. 1827–1838, December 2013. DOI: [10.1109/TMECH.2012.2213607](https://doi.org/10.1109/TMECH.2012.2213607). The paper's multi-insonification-angle model uses the acute angle between track lines: parallel lines are treated as dependent, perpendicular lines as independent, and intermediate angles are interpolated.

**[R3]** V. Myers and D. P. Williams, “Adaptive Multiview Target Classification in Synthetic Aperture Sonar Images Using a Partially Observable Markov Decision Process,” *IEEE Journal of Oceanic Engineering*, vol. 37, no. 1, pp. 45–55, January 2012. DOI: [10.1109/JOE.2011.2175510](https://doi.org/10.1109/JOE.2011.2175510).

**[R4]** A. V. Sethuraman, P. D. Baldoni, K. A. Skinner, and J. McMahon, “Learning Which Side to Scan: Multi-View Informed Active Perception with Side Scan Sonar for Autonomous Underwater Vehicles,” in *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2024, pp. 8348–8354. DOI: [10.1109/ICRA57147.2024.10611077](https://doi.org/10.1109/ICRA57147.2024.10611077). Preprint: [arXiv:2402.01106](https://arxiv.org/abs/2402.01106).

**[R5]** M. S. Wiig, T. R. Krogstad, and Ø. Midtgaard, “Autonomous Identification Planning for Mine Countermeasures,” in *Proceedings of the 2012 IEEE/OES Autonomous Underwater Vehicles Symposium (AUV)*, 2012, pp. 1–8. DOI: [10.1109/AUV.2012.6380733](https://doi.org/10.1109/AUV.2012.6380733).

**[R6]** J. Fawcett, V. Myers, D. Hopkin, A. Crawford, M. Couillard, and B. Zerr, “Multiaspect Classification of Sidescan Sonar Images: Four Different Approaches to Fusing Single-Aspect Information,” *IEEE Journal of Oceanic Engineering*, vol. 35, no. 4, pp. 863–876, October 2010. DOI: [10.1109/JOE.2010.2069832](https://doi.org/10.1109/JOE.2010.2069832).

**[R7]** G. A. Hollinger and G. S. Sukhatme, “Sampling-Based Robotic Information Gathering Algorithms,” *The International Journal of Robotics Research*, vol. 33, no. 9, pp. 1271–1287, August 2014. DOI: [10.1177/0278364914533443](https://doi.org/10.1177/0278364914533443).

**[R8]** G. A. Hollinger, B. Englot, F. S. Hover, U. Mitra, and G. S. Sukhatme, “Active Planning for Underwater Inspection and the Benefit of Adaptivity,” *The International Journal of Robotics Research*, vol. 32, no. 1, pp. 3–18, January 2013. DOI: [10.1177/0278364912467485](https://doi.org/10.1177/0278364912467485).

**[R9]** S. Ji, R. Parr, and L. Carin, “Nonmyopic Multiaspect Sensing with Partially Observable Markov Decision Processes,” *IEEE Transactions on Signal Processing*, vol. 55, no. 6, pp. 2720–2730, June 2007. DOI: [10.1109/TSP.2007.893747](https://doi.org/10.1109/TSP.2007.893747).

**[R10]** D. Golovin and A. Krause, “Adaptive Submodularity: Theory and Applications in Active Learning and Stochastic Optimization,” *Journal of Artificial Intelligence Research*, vol. 42, pp. 427–486, 2011. DOI: [10.1613/jair.3278](https://doi.org/10.1613/jair.3278).

**[R11]** M. Ghaffari Jadidi, J. Valls Miro, and G. Dissanayake, “Sampling-Based Incremental Information Gathering with Applications to Robotic Exploration and Environmental Monitoring,” *The International Journal of Robotics Research*, vol. 38, no. 6, pp. 658–685, May 2019. DOI: [10.1177/0278364919844575](https://doi.org/10.1177/0278364919844575).

**[R12]** V. Yordanova and B. Gips, “Coverage Path Planning with Track Spacing Adaptation for Autonomous Underwater Vehicles,” *IEEE Robotics and Automation Letters*, vol. 5, no. 3, pp. 4774–4780, 2020. DOI: [10.1109/LRA.2020.3003886](https://doi.org/10.1109/LRA.2020.3003886). Preprint: [arXiv:2006.12896](https://arxiv.org/abs/2006.12896).

**[R13]** C. Guo, G. Pleiss, Y. Sun, and K. Q. Weinberger, “On Calibration of Modern Neural Networks,” in *Proceedings of the 34th International Conference on Machine Learning*, Proceedings of Machine Learning Research, vol. 70, pp. 1321–1330, 2017. [PMLR paper page](https://proceedings.mlr.press/v70/guo17a.html).

**[R14]** International Hydrographic Organization, *IHO Standards for Hydrographic Surveys*, Special Publication S-44, Edition 6.1.0, September 2022. [Official 6.1.0 PDF](https://iho.int/uploads/user/pubs/standards/s-44/S-44_Edition_6.1.0.pdf). **Edition note:** IHO Circular Letter 21/2025 records adoption of Edition 6.2.0; the IHO catalogue lists Edition 6.2.0 (October 2024). [Official 6.2.0 publication page](https://iho.int/en/standards-and-specifications), [adoption circular](https://iho.int/uploads/user/circular_letters/eng_2025/CL21_2025_EN_v1.pdf). Do not silently replace the 6.1.0 edition named in the original guide; ask whether the project intended to freeze it.

**[R15]** ROS 2, “Quality of Service policies,” ROS 2 design documentation. [Official design article](https://design.ros2.org/articles/qos). Use the documentation matching the ROS 2 distribution actually checked out; verify the actual publisher / subscriber QoS in code and at runtime.

**[R16]** ROS 2, “Managed nodes,” ROS 2 design documentation. [Official design article](https://design.ros2.org/articles/node_lifecycle.html). This is background only; it does not imply that this module must use lifecycle nodes.

## 20. Important limits on how the references are used

These notes are deliberate safeguards against overclaiming. They do not alter the algorithm or decisions already recorded above.

1. **IHO S-44 §3.7 and the `3σ` revisit geometry.** In Edition 6.1.0, §3.7 is titled “Charted Object Confirmation / Disproval.” It recommends a search radius of at least three times the estimated position uncertainty of a *reported / charted hazard* when confirming or disproving that charted object. It also says no empirical formula covers every situation. The guide's `2·max(3σ_along, L_min/2)` straight-section rule is not the same geometric quantity. Retain that proposed formula as a project design candidate, but do not present S-44 §3.7 as a direct derivation of the whole segment-length formula. In Phase 1, explain the intended geometric interpretation and justify / test it independently.
2. **IHO S-44 edition.** The original guide names Edition 6.1.0. Edition 6.2.0 was subsequently adopted by the IHO in 2025. Whether the module should use 6.1.0 or 6.2.0 is a project decision; preserve 6.1.0 until the user says otherwise. Note also that Exclusive Order's 200% feature-search / bathymetric-coverage requirements are survey-standard requirements, not an automatic definition of the replanner's two-pass baseline.
3. **Paull et al. angle dependence.** The paper models the acute intersection angle between track lines, not a universal physical law for every sonar, target, or detector. Its parallel/dependent and perpendicular/independent endpoints motivate a model choice; they do not calibrate the BlueBoat planner's `A_k`, class parameters, or correlation model.
4. **Hollinger et al. adaptivity result.** The constant-factor adaptivity result is tied to the paper's stated reformulation (variance reduction under a cost constraint). It is useful context for why a non-adaptive baseline can be competitive, but it is not a theorem for this replanner's exact `V(P)`, path-dependent `T(P)`, or two-step lookahead.
5. **Golovin and Krause.** Adaptive-greedy guarantees require adaptive submodularity and the relevant assumptions. Do not imply that those guarantees apply to this path-dependent, signed contact / background utility without proving the required properties.
6. **Guo et al. calibration.** Temperature scaling is a calibration technique studied for neural-network classifiers. It is not a substitute for measuring the calibration of the actual detection emulator / detector and the mapping from confidence to `P(real)`.
7. **Myers / Ji / Sethuraman.** These works motivate multi-view, orientation-aware sensing and active view choice. Their POMDPs, class models, data, or learned policies are not direct validation of the simpler 12-sector model proposed here.
8. **Jadidi et al.** The paper motivates information-theoretic convergence criteria in informative motion planning. It does not derive the specific contact posterior-change threshold (`0.02`) or maximum revisit count (`3`) in §8.6. Those remain project heuristics to justify, sweep, and report.
9. **Paull, Yordanova & Gips, and S-44 are different kinds of evidence.** A coverage-spacing method, a multi-aspect observation model, and a hydrographic survey standard answer different questions. Cite each only for the claim it actually supports.
10. **ROS 2 design documents are not the workspace contract.** They explain general QoS / lifecycle concepts. The checked-out project code, launch configuration, and runtime graph determine what this module must interoperate with.

---

## Addendum change record

- **2026-11-03:** Added clarifications for input scope, evidence labels, unresolved baseline / randomness semantics, mathematical model completeness, stale-input handling, execution state / safety / logging requirements, evaluation reproducibility, and reference traceability. Added DOI / stable links and source-specific caveats. The original §§0–15 were retained verbatim; no cross-module project paths, interface names, topic names, or recorded project decisions were changed.
