# Phase 1 closure — consistency review of DESIGN.md and MATH.md

Status: **2026-11-04, cloud session REPLAN-1. Submitted for the user's Phase 1 approval. No
code yet.**

Scope: issues A–I raised by the user after reading the Phase 1 drafts. Each issue gives the
finding, the derivation, the resolution, the documents changed, and the tests and in-machine items
it creates. Labels and evidence ids follow `DESIGN.md`. The brief and the embedded contract are
unchanged.

**Preserved user decisions:**
- 0.5 m/s;
- one-sided swath with range-dependent spacing;
- inputs as in the brief;
- C3 = append-on-completion, built first, with modular policies;
- C4_rand from a pre-drawn table.

Repository state reviewed: BlueBoat-Replanner `ba97fd7` with untracked `docs/` and `.claude/`,
on branch `claude/nice-johnson-ang1j1`. No other module was changed.

| Issue | Outcome |
|---|---|
| A. Background calibration | **Correction**: the brief's formula reproduces Belief's residual only when every class has s_k = 1. p_eff now solves the class-weighted equation |
| B. Candidate cost | **Correction**: the return to coverage was not charged. Candidates are now valued on the executed sequence over the remaining budget |
| C. Lookahead | **Correction**: the second action is chosen by the combined-sequence value. Identity proven; worked example given |
| D. Stop semantics | **Correction**: separate requested / accepted / holding / rejected states. Override and E-STOP disarm the replanner |
| E. Initial motion | **Correction**: no reliance on turning in place. The first path is a Dubins manoeuvre from the hold pose, charged to the budget |
| F. Ranges | **Clarification**: the coverage band is used for spacing only; the detection band (3–25 m, geometric cap) is used for value |
| G. Timing | **Clarification**: verified Control bounds are separated from our margins; a pre-send check is added; acceptance is never assumed |
| H. Assumptions | Grazing term **off by default** (brief §3 formula). The other items are confirmed as placeholders or [OPEN] |
| I. C2 protocol | **New**: `C2_TUNING_PROTOCOL.md`, to be frozen before any C4 result |

---

## A. Background probability calibration (MATH §9)

### Finding

With A ≡ 1, the planner's residual for an undetected target after n folded-sector looks with
per-look probability p·s_k is a class mixture:

```
M(p) = Σ_k π_k (1 − p s_k)^n
```

The brief sets p_b = 1 − Q^{1/n}, so that (1 − p_b)^n = Q. With 0 < s_k ≤ 1, each term satisfies
1 − p_b s_k ≥ 1 − p_b, with equality iff s_k = 1 (p_b > 0). Hence:

```
M(p_b) ≥ (1 − p_b)^n = Q,   with equality  ⇔  s_k = 1 for every k with π_k > 0   (or n = 0, or Q = 1).
```

So the property claimed in MATH §9 ("the aspect-agnostic case reproduces Q_i") is **false for
the default class table** (s = 1, 1, 0.9, 0.6, 0.3).

### Correction (MATH §9.1)

Choose p_i so that the class mixture reproduces Belief's residual:

```
Σ_k π_k (1 − p_i s_k)^{n_i} = Q_i ,      p_i ∈ [0, 1],  n_i = |S_i| ≥ 1.
```

- **Uniqueness.** dM/dp = −n Σ π_k s_k (1 − p s_k)^{n−1} < 0 on [0, 1), so M is strictly
  decreasing from M(0) = 1 to M(1) = Q_min,i = Σ π_k (1 − s_k)^n. A root exists iff
  Q_i ≥ Q_min,i, and it is unique.
- **Solver.** Vectorised bisection over all cells, with a fixed 60 iterations. The bracket width
  is 2⁻⁶⁰, so |M − Q| ≤ n·2⁻⁶⁰. Deterministic, no data-dependent loop count.
- **Unreachable** (Q_i < Q_min,i): p_i = 1, the cell is counted in `calibration_clamped`, and the
  planner's residual is larger than Belief's there. Reported per decision.
- **Closed forms** (tests):
  - s ≡ 1: p = 1 − Q^{1/n} (the brief's formula);
  - s ≡ s̄: p = (1 − Q^{1/n})/s̄;
  - n = 1: p = (1 − Q)/Σ π_k s_k.
- **Fallback** for S_i = ∅ with Q_i < 1 (impossible if Belief is consistent): one sector-free look
  with A ≡ 1 and n = 1. [FALLBACK], counted.
- **Aspect-aware use.** The same p_i enters
  L_i(k, φ) = Π_g (1 − p_i s_k A_k(β_g, φ)). Since A ≤ 1, the aspect-aware residual is ≥ Q_i.
  This is intended: the aspect-agnostic case reproduces Belief, and aspect windows expose
  orientations Belief's model treats as seen.
- **`p_background` check.** In reachable cells with A ≡ 1, q_i = p₀Q_i / (1 − p₀ + p₀Q_i) for any
  s_k, so the runtime consistency check with Belief's `p_background` is now meaningful whatever
  the class table.

### Numerical demonstration

Uniform prior over the five default classes, s = (1, 1, 0.9, 0.6, 0.3). Computed 2026-11-04,
scratch script.

| Q_i | n_i | Brief p_b | M(p_b) | Calibrated p_i | M(p_i) | Q_min |
|---|---|---|---|---|---|---|
| 0.25 | 2 | 0.500 | 0.403 | 0.709 | 0.250 | 0.132 |
| 0.30 | 1 | 0.700 | 0.468 | 0.921 | 0.300 | 0.240 |
| 0.49 | 2 | 0.300 | 0.603 | 0.406 | 0.490 | 0.132 |
| 0.343 | 3 | 0.300 | 0.476 | 0.420 | 0.343 | 0.082 |
| 0.09 | 2 | 0.700 | 0.256 | 1 (clamped) | 0.132 | 0.132 |

With p₀ = 0.002, Q = 0.25, n = 2:
- Belief's q = 5.008e-4;
- the brief's formula gives 8.07e-4 (61 % high);
- the calibrated value gives 5.008e-4.

**The cost of this choice, stated.** For s = 1 classes the calibrated per-look probability exceeds
Belief's apparent one (0.92 vs 0.70 for one look). The planner explains Belief's average residual
with a mixture in which plates are hard to detect.

**The alternative, recorded.** Interpretation I1 reads Belief's Q as the residual of a unit
(s = 1) target, i.e. the brief's formula. It is not chosen: its mixture never matches Belief's
residual, and the `p_background` check would fail whenever s_k ≠ 1.

Which interpretation matches Belief's actual detection model is checked in-machine (L-B6).

### Tests required (gate section "background")

- A-1: heterogeneous s, A ≡ 1, reachable Q → |M(p_i) − Q_i| ≤ 1e-12, on a grid of
  (Q, n, s, π).
- A-2: the three closed forms.
- A-3: unreachable Q → clamp at p = 1, flagged, and M = Q_min.
- A-4: q_i(A ≡ 1) = p₀Q/(1 − p₀ + p₀Q) for heterogeneous s.
- A-5: per-class ordering — residual weight w_i increases as s_k decreases.

---

## B. Candidate cost versus executed trajectory (MATH §10–§12)

### Finding

MATH §10 valued a candidate "from the change point; transit and entry turn included, **the
return is not**", while §11 executed "a₁* + the route back to the nearest unflown coverage +
the coverage remainder". The return was charged to the mission but not to the candidate. That
biases the comparison in favour of revisits, i.e. the adaptive method benefited from omitted
travel time.

The rate V/T of a detour was also compared with CONTINUE's rate. That ignores **what the detour
displaces**: under an equal-time budget, a detour pushes the end of the plan past the budget.

### Correction (MATH §10–§12 rewritten)

1. **Default continuation K** — the ordered list of unflown coverage legs. For C2, C3 and C4 the
   initial plan is **the C2 plan** (pass 1 then pass 2), so K is the C2 remainder.
   - C4 with no detour is exactly C2, which is what the brief means by "lets the planner
     reproduce two-pass when that is best".
   - C3 inserts its revisits at the pass-1/pass-2 boundary (§13). If its single epoch is missed,
     it degrades to C2 (DESIGN §6).
   - C1 is pass 1 only.
2. **Actions as detours.** At change point c (pose x_c on the mirror):
   - an action a is D_a = transit(x_c → body start) + body(a) + rejoin(body end → start pose of
     K's next leg);
   - CONTINUE is D_cont = the generator's ordinary connection from x_c to that leg start;
   - the executed tail is D_a ⊕ K, truncated at the last segment boundary ≤ t_start + B.
3. **Objective.** The value of the union of new looks along the **whole executed tail** over the
   remaining budget:

   ```
   J(a) = V( looks of D_a ⊕ K_trunc(a) ),     K_trunc(a) = K cut at budget end B_rem − T(D_a)
   ```

   - Every option occupies the same B_rem (budget always binds once K ≥ B_rem; for C2's remainder
     T(K) = B_rem exactly). So comparing J is comparing value rates J/B_rem, which keeps the
     brief's rate rule.
   - CONTINUE is evaluated by the same formula.
   - The value lost from the truncated tail of K is charged automatically.
   - A look of K in a (cell or contact, folded sector) already looked at by D_a has no further
     value (union).
4. **Exactly what is charged.**

   | Quantity | Includes | Used for |
   |---|---|---|
   | Candidate horizon | transit with its entry turn (Dubins from x_c's pose and heading), body (lead-in, straight, lead-out), rejoin with its entry turn, K's legs and connections, truncated at the budget | J(a) |
   | Mission budget charge | τ path seconds of everything flown: the initial manoeuvre, coverage, transits, bodies, rejoins, turns | budget rule; C2 duration = B |
   | Revisit charge | ΔT(a) = T(D_a) − T(D_cont) for each executed REVISIT | the ρ_rev·B cap |

   The candidate horizon and the executed tail are sampled by **the same** sampler, so planned and
   charged times agree by construction. The executed τ is logged against wall time (E-14).
5. **RECOVER(j)** becomes "fly leg j of K out of order": a detour to leg j, then rejoin K \ {j}.
   It is kept for interrupted-line remainders (C4_live) and as a reordering candidate.
6. **Computation.** Valued exactly, in MATH §10.4.
   - K's look keys, with their time along K, are precomputed once per epoch.
   - V_K(t) is K's value step function. Values are additive over cells and contacts.
   - Only the cells and contacts touched by D_a are recomputed:
     J(a) = V_touched(D_a ∪ K_≤t(a)) + V_K(t(a)) − V_K,touched(t(a)).

### Approximations (documented in MATH §10.5)

- **Rejoin** is always at the start of K's next unflown leg; K is not re-sequenced. This is the
  same in every condition, and conservative: it never shortens the adaptive route by
  re-optimising K.
- **Truncation** happens at segment boundaries, both in evaluation and in execution.

### Tests required (gate section "candidates / budget")

- B-1: For random candidates, the planned T of D_a ⊕ K equals the sampled duration of the request
  the executor would send, within 1e-9 s.
- B-2: A detour never yields an executed tail longer than the budget.
- B-3: CONTINUE's J equals the value of K_trunc alone.
- B-4: A revisit whose rejoin is omitted is rejected by plan validation. This is a guard test: the
  tail must end on K's legs or at the budget.
- B-5: With K valued, a revisit to a contact that pass 2 will see from the same folded sector has
  zero incremental contact value.

---

## C. Two-step lookahead (MATH §11)

### The identity behind "no double counting"

For a contact c with posterior (p̃, λ), and two disjoint sets of new folded sectors N₁ then N₂:

```
V_c(N₁ ∪ N₂) = V_c(N₁) + P(miss on N₁) · V_c'(N₂)
```

Here V_c' uses the posterior after a miss on N₁:
- p̃' = p̃ Σλ Π₁ / P(miss);
- λ' ∝ λ Π₁;
- P(miss) = p̃ Σλ Π₁ + (1 − p̃)(1 − f)^{|N₁|};
- Π₁ = Π_{n∈N₁}(1 − p_n).

A detection on N₁ confirms the contact (value 0 afterwards). The background has the same identity,
with q' and w' the posterior after a miss. Both are proven in MATH §11.1 and checked numerically:
the differences are 0.0 and −8.7e-19 on random instances. Consequences:
- for a **fixed** second action, the branch-weighted value equals the value of the union of looks,
  so evaluating sequences by their union never counts a look twice;
- branching changes the result **only** because the second action is chosen after the outcome ω
  is known.

### Correction

- **Decision points.** The end of every coverage leg and of every revisit body. Never a transit,
  turn or rejoin segment.
- **First actions.** CONTINUE, REVISIT(c, g), RECOVER(j), each a detour from the current change
  point (B). The top K₁ = 5 are kept by one-step J (CONTINUE always kept).
- **Outcomes.** For a REVISIT of c: ω ∈ {det, miss} with
  P(det) = p̃ P_det_real + (1 − p̃) P_det_clut.
  - Under det: c is confirmed.
  - Under miss: c's posterior is updated.
  - Every other contact, and the background, receive a₁'s looks as **seen**: union, i.e. their
    expected-miss update.
  - For CONTINUE and RECOVER there is a single branch.
- **Second actions.** a₂ starts at a₁'s body exit, or at the next decision point of K for
  CONTINUE. It is chosen **by the combined-sequence value**:

  ```
  J₂(a₁) = Σ_ω P(ω) · max_{a₂ ∈ A₂(ω)}  J( a₁ ⊕ a₂ ⊕ K'_trunc | ω )
  ```

  - a₂ = CONTINUE means a₁'s own rejoin then K.
  - A₂(ω) is the top K₂ = 5 by one-step J in the post-state, which must satisfy the budget rule.
- **Choice.** a₁* = argmax J₂. Leave CONTINUE only if J₂(a₁*) ≥ J₂(CONTINUE) + ε_J.
  - ε_J = 0.02 expected confirmed targets [ASSUMPTION, swept].
  - It replaces the relative η: J₂ includes the value of the whole remainder, so a relative
    threshold would have grown with the remaining plan.
- **Execution.** Only a₁'s tail (a₁ ⊕ rejoin ⊕ K) is sent. a₂ is never sent. The next decision
  point re-decides with what was actually observed.
- **Pending looks.** At every epoch, the mirror path over [τ − t_lat, c] is treated as seen
  before valuing candidates.
  - The flown-but-maybe-not-yet-in-the-grid part is [τ − t_lat, τ]; t_lat = belief latency,
    10 s [OPEN] (L-B5).
  - The committed part [τ, c] is certain to be flown before any new tail.
  - Under the folded model this is idempotent: a look already in the grid adds nothing again. So
    no look is valued twice, and the planner never re-buys a look it has already paid for.

### Numerical example

Remaining budget 100 s. The remainder K is worth 0.02 per second. Detours are given as
(value, extra time); the second-step options are each reachable from one first action.

| Option | Value | Extra time | Own rate V/T | J (with K over the rest) |
|---|---|---|---|---|
| A | 1.00 | 10 s | 0.100 | 1.00 + 0.02·90 = 2.80 |
| B | 0.85 | 10 s | 0.085 | 0.85 + 0.02·90 = 2.65 |
| CONTINUE | 0 | 0 | — | 2.00 |
| after A: z | 0.50 | 10 s | 0.050 | A ⊕ z = 1.50 + 0.02·80 = **3.10** |
| after B: x | 0.30 | 1 s | 0.300 | B ⊕ x = 1.15 + 0.02·89 = 2.93 |
| after B: y | 1.20 | 5 s | 0.240 | B ⊕ y = 2.05 + 0.02·85 = **3.75** |

| Selection rule | Values compared | First action sent |
|---|---|---|
| Individual action rate | A 0.100 > B 0.085 | A |
| Two-step, a₂ by its own rate (old MATH §11) | J₂(A) = 3.10 vs J₂(B) = 2.93 (picks x) | A |
| **Two-step, a₂ by the combined sequence (new)** | J₂(B) = 3.75 vs J₂(A) = 3.10 | **B** |

Only B's tail is sent; y is re-decided at the next decision point. The example is deterministic.
With uncertain outcomes, the second-step values are averaged over ω as above.

### Approximations (documented in MATH §11.4)

1. Only the revisited contact branches; other contacts and the background are valued in
   expectation. This is exact for a fixed a₂ (identity above), and approximate only in a₂'s
   adaptivity.
2. The outcome ω is assumed known at the next decision point. With belief latency t_lat it may
   not be. The pending-look rule keeps execution consistent, but the lookahead may be optimistic
   about adaptivity.
3. The second step is restricted to the top K₂.
4. K is valued open-loop: it does not account for future decisions.
5. Belief confirms on a same-sector repeat (brief §9.2). In the planner's model such a repeat has
   no value (brief §8.2). The planner therefore never buys a same-aspect confirmation. This is
   conservative and deliberate.

### Tests required (gate section "lookahead", Phase 2b)

- C-1: the identity (contacts and background) on random instances, within 1e-12.
- C-2: the worked example reproduces A / A / B under the three rules.
- C-3: CONTINUE scored by the same function.
- C-4: only a₁ is in the emitted request.
- C-5: pending looks — a candidate whose looks are all pending has zero value.

---

## D. Stop and refusal semantics (DESIGN §5.3, §7)

### Finding

DESIGN §5.3 reported `stopped` even when the truncation was refused, while Control kept
following the old path. That is misleading. The draft also resumed decisions after an override,
which the brief forbids: "stops on `~/stop`, manual override or E-STOP" (§7.2).

### Correction

| Situation | Status | What the boat does | Evidence |
|---|---|---|---|
| `~/stop` called | `stop_requested` | follows the current path | — |
| Truncation accepted (answer `ok`, or progress `active_id` = the stop id) | `stop_accepted` (the change point c and its pose are reported) | follows the old path until τ = c, then holds | E-04, E-17 |
| progress `end_reached` with `active_id` = the stop id | `stopped_holding` | holds the stop pose (Control's station-keeping) | E-17 |
| Truncation refused | `stop_rejected`: "STOP NOT IN EFFECT — the boat continues the previous path; use E-STOP, manual target or run stop" | follows the old path | E-09 |
| Run stopped or cleared by the operator (`cleared`, `no_run`) | `run_ended` (not "holding") | per Control: idle, zero thrust | E-18 |

- **Truncation construction**, checked against Control's code: two samples at the mirror pose
  (x_c, y_c, yaw_c), at t = c and t = c + 1 s.
  - Zero displacement, so the segment is not "moving" and has no coherence check
    (`validate_samples`).
  - Position gap 0.
  - Heading equal to the old path's at c, so it passes the 10° check (E-07).
- **Retries.** A refused stop is retried at most 3 times.
  - `lead_too_short`: a fresh c from the latest `min_change_point`.
  - `discontinuous_*`: re-read the mirror once.
  - State codes (`operator_override`, `not_ready`, `no_run`): the operator already has
    authority. The retry waits until progress is `following` again; a stop is an operator
    command, not autonomous replanning.
  - After 3 refusals: `stop_rejected` stays, with the reasons.
- **Override and E-STOP.** Progress `override` (manual target or arrows) or `not_ready`
  (E-STOP) during a mission → **`disarmed`**. No decision and no request follows. Control keeps
  its own behaviour: on release, it resumes the loaded path, and only the operator's release
  does that (E-16).
- **Re-arming** requires all of:
  1. an explicit `~/start`;
  2. progress `following`;
  3. progress `active_id` equal to the mirror's last accepted id;
  4. fresh grid and progress;
  5. no stop pending.
- **Mission start** requires `~/start` and `awaiting_path` (unchanged).
- **Status wording.** Statuses never say "stopped" without the qualifier `_holding`, and only
  `stopped_holding` asserts that the boat holds.

### Tests required (gate, executor against `control_stub`)

- D-1: stop accepted → `stop_accepted`, then `stopped_holding` only after the stub's
  `end_reached`.
- D-2: stop refused 3 times → `stop_rejected`, and the stub's path is unchanged.
- D-3: override during running → `disarmed`; no request after release until `~/start`.
- D-4: E-STOP → same.
- D-5: re-arm refused while `active_id` ≠ mirror.

---

## E. Initial motion model (DESIGN §5.1, MATH §3.5)

### Evidence

- **Control's wording [DOC].** The first path "may have any yaw, since the boat turns in place",
  and the heading check is skipped at a stationary hold. Sources: `PATH_UPDATE_INTERFACE.md`
  §5 ("There the boat turns in place") and §10.4 item 2.
- **Code [CODE].** The awaiting hold uses the keep-location law (`_awaiting_hold_active`,
  `manual_keep_location`, "yaw untouched"). The controllers shape surge by max(0, cos ψ_err) and
  turn the boat round (Control `src/CLAUDE.md` §4).
- **Gap.** No measurement of in-place turning time or radius exists anywhere (E-41).
- **Field evidence.** Missions with turn radii of 1.5–6 m were flown in the field [DOC]
  (interface §10.4 item 6).

### Correction

The design **does not rely on turning in place**:
- The first path is a **Dubins path** of radius r_turn from the hold:
  - position exactly `hold.(x, y)` (Control's 0.20 m check);
  - heading = the boat's yaw (fresh odometry, else `hold.yaw`, which Control captured);
  - ending at the start pose of the first coverage leg.
- A first-path heading that differs from the boat's would be accepted by Control but would rest on
  untested behaviour, so it is not used.
- **The initial manoeuvre is part of every plan, and of B** (the C2 plan from the hold), so every
  condition pays it identically.
- **Mission errors at `~/start`:**
  - the hold outside F_margin ("move the boat inside the area");
  - no Dubins / transit route inside F_margin ("not enough room to turn at the start").

  The replanner never leaves the area to turn.
- **In-machine (L-C5):** characterise Control's response to a first path whose yaw differs by
  90° and 180°. This is for information only; nothing depends on it.

---

## F. Detection range versus usable coverage

**Finding.** MATH §5 capped the look predictor at the coverage band's r_far, including the NOAA
grazing term. That conflated a coverage-planning guarantee with a detection model.

**Resolution.** Two quantities, stated separately in MATH §2 and §5:

| Quantity | Range | Used for | Source |
|---|---|---|---|
| Coverage band [r_near, r_far_cov] | 6 m … √(R² − h²) (grazing term optional) | line spacing only — a conservative geometric guarantee that every point lies in the good part of some line | brief §3; user (one side) |
| Detection band PD_r | 0.7 on [3, 25] m, capped at the geometric limit √(R² − h²) | predicted looks: contact value, background value, the look predictor | brief §8.2 ("default = Belief's") |

- **Looks at 3–6 m ground range do count** for confirmation and for background value. They count
  in Belief's Q under the brief's statement of Belief's default, and the planner must use the
  same window for its predictions to stay consistent with Q (A).
- They are not credited in the spacing guarantee, which only makes the coverage conservative, for
  every condition alike.
- The revisit range r* = 10 m (brief Q7) is checked against the detection band and the geometric
  limit, no longer against the coverage band.
- Belief's actual window is reconciled in-machine (L-B6). Until then the planner's window is a
  placeholder.

---

## G. Timing and Control acceptance

| Quantity | Value | Kind | Evidence |
|---|---|---|---|
| lead_min | max(path_time + 0.1 s, 3 m / U_d); 6.0 s for PID/LoS at 0.5 m/s | **Control, verified** | E-06 |
| `min_change_point` | τ + lead_min, published at 10 Hz and after every answer | **Control, verified** | E-11 |
| τ rate | ≤ 1 path-s per wall-s (governor), so path-second margins are conservative in wall time | **Control, verified** | E-14 |
| Handling ≤ 2.2 ms, round trip ≤ 2.9 ms | Gazebo measurement | **Control, [DOC]** (not re-run) | E-24 report |
| T_decide | 2 s budget for one decision | ours, configurable; a budget, not a guarantee | brief §9.1 |
| latency_margin | 2 path-s | ours, configurable | — |
| answer_timeout | 5 s | ours, configurable | — |
| reconcile_timeout | 10 s after the answer timeout | ours, configurable | — |

**Rules (DESIGN §5.2):**
1. **Trigger.** A decision is triggered when τ ≥ t_dp − (lead_min + T_decide + latency_margin),
   with t_dp the decision point.
2. **Pre-send check.** Immediately before sending, the latest progress (age ≤ 0.2 s) must satisfy
   c ≥ `min_change_point` + latency_margin. Otherwise the request is **not sent**: the epoch is
   recorded as `missed_lead`, and the policy re-decides at the next decision point.
3. **Acceptance is never assumed.** Only an `ok` answer, or progress `active_id` equal to the
   request id, updates the mirror. Every refusal is handled per DESIGN §5.2.
4. **No safety from margins alone.** The margins make acceptance likely; Control's check decides.

Test G-1 (stub): a decision whose simulated compute time exceeds the window is never sent and is
logged `missed_lead`.

---

## H. Remaining assumptions, rechecked

| Item | Decision now | Label | Note |
|---|---|---|---|
| NOAA grazing term (altitude ≥ 8 % of the range in use) | **Off by default** | [DECIDED: brief §3 + user "brief overrides"]; the option itself [ASSUMPTION] | The brief defines the usable swath as "≈ 6 m … √(20² − depth²)". NOAA HSSD §8.3 is a towed-sonar rule; applying it to a hull-mounted sonar was my reading. Kept as `grazing_floor: true`, a sensitivity option (it halves the spacing at h ≤ 1 m). **This reverses the Phase 1 draft.** |
| 0.5 m/s | kept | [DECIDED] user | Above Control's ≤ 0.45 m/s simulation advice (E-23). In-machine L-K3: if τ is throttled, the user decides before E4 |
| r_turn 3 m, t_settle 2 s | placeholders | [ASSUMPTION] | Inside the field-flown 1.5–6 m. L-K1, L-K4 |
| Clutter re-detection f = 0.2 | placeholder, swept 0.05 / 0.2 / 0.5 | [OPEN] O-5 | No project source; C4 only |
| One area per mission | kept | [OPEN] O-8 | Routes between disjoint areas would leave the areas |
| S-44 edition | 6.1.0 kept; context for "≥ 3σ" only | [DECIDED default] Q-17 | 6.2.0 adoption noted |
| Belief latency t_lat 10 s | placeholder | [OPEN] | L-B5 |
| ε_J 0.02 | placeholder, swept | [ASSUMPTION] | C |
| C3 = append-on-completion; C4_rand table | preserved | [DECIDED] user | — |

---

## I. C2 tuning protocol

Specified in `docs/C2_TUNING_PROTOCOL.md`: tunable and fixed parameters, objective, seeds,
metrics, selection, freeze, fairness safeguards. **Not run.** It must be frozen (a hash
recorded) before any C3/C4 result on evaluation seeds exists. The runner enforces it.

---

## Tests required by the closure (added to MATH §14, gate)

A-1…A-5, B-1…B-5, C-1…C-5, D-1…D-5, G-1, plus:
- E-1: every plan starts at hold with the boat's yaw and stays inside F_margin.
- F-1: the look predictor credits looks at 3–6 m.
- F-2: spacing ignores the band below r_near.

Phase 2 covers B-1, B-2, B-4, E-1, F-2, and the C1–C3 parts of D and G. The rest belongs to
Phase 2b (C4).

## Items requiring REPLAN-2-LOCAL (added to `LOCAL_CHECKLIST.md`)

| Item | Purpose |
|---|---|
| L-B5 | Belief latency: time from a contact passing abeam to its update in `/belief_map/belief_grid` (sets t_lat) |
| L-B6 | Belief's actual detection model and range window vs the planner's PD_r; which interpretation of Q (A) matches |
| L-C5 | Control's response to a first path whose yaw differs from the boat's by 90° and 180° (information only) |
| L-C6 | Stop truncation accepted then holding; refused under manual target; status sequence as in D |
| L-K3, L-K4 | 0.5 m/s achieved without τ throttling; 3 m arcs tracked |

## Open questions for the user (not blocking Phase 2)

- O-5 clutter f;
- O-8 multiple areas;
- Q-9 Belief's evidence topic;
- the grazing default (now off, per brief §3 — reverse it if you prefer the NOAA-derived
  conservative spacing);
- E-STOP / override handling (now: disarm until `~/start`, per brief §7.2).
