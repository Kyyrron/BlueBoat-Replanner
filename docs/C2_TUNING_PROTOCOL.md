# C2 tuning protocol (two-pass orthogonal baseline)

Status: specified 2026-11-04 (Phase 1 closure I).

- **Run once in the dev loop** (preliminary, reference belief) and frozen before any C3/C4
  evaluation: `docs/C2_TUNING_RESULT.md`, `config/c2_tuned.yaml`.
- **The E4 freeze is still to be done in Gazebo**, in-machine.

**Purpose.** Synthesis §8.6 ⚠ and CM-12 require the two-pass baseline to be tuned seriously, with
the same coverage generator, speed, turns and execution path as the adaptive conditions. The
baseline's duration also sets every condition's budget (brief Q2).

## 1. Fixed for every condition (never tuned here)

| Parameter | Value | Source |
|---|---|---|
| Speed U | 0.5 m/s | user decision |
| Range R | 20 m | synthesis §8.5 |
| Coverage-band model | r_near 6 m; r_far_cov = √(R² − h²) (grazing option off) | MATH §2; brief §3 |
| r_turn, t_settle, margin | 3 m, 2 s, 2 m, or the measured values from L-K1 / L-K2 once available; then frozen for all conditions | MATH §15 |
| Controller | one, the same for all conditions (PID default) | DESIGN decision 11 |
| Sampling, executor, Control stub (dev loop), emulator, belief | identical | DESIGN §5 |
| Start-corner rule | the coverage start nearest the hold, by route length | MATH §3.2 |
| Planning depth | per scenario, identical across conditions | MATH §2.3 |

## 2. Tuned (C2 only; the tuned values become the shared coverage settings of C2, C3 and C4)

| Parameter | Grid |
|---|---|
| Spacing factor f | {0.6, 0.7, 0.8, 0.9, 1.0}, each verified for single coverage per pass (MATH §2.2) |
| Pass-1 direction | {area long axis, long axis + 90°}; pass 2 is always orthogonal to pass 1 |
| Interlace | {auto (k = ⌈2 r_turn / s⌉), off (k = 1, when turns allow)} |

## 3. Objective and constraint

- **Score** of a configuration on one scenario: confirmed true targets − κ · false confirmations at
  the end of the C2 plan, with κ = 1. This is E4's own primary quantity (synthesis §8.4) and the
  quantity C4 optimises (MATH §8), so the baseline is tuned for what it is judged on.
- **Constraint:** duration T(C2) ≤ B_cap, with B_cap = T(C2 at f = 0.8, long axis) for that
  scenario (synthesis §8.5's ~0.8 × usable swath sets the reference effort). This stops "tuning"
  from simply buying more time.
- **Secondary metrics,** reported and not used for selection: recall of true targets (≥ 1
  detection), false positives per mission, coverage fraction per pass, duration.

## 4. Seeds and scenarios

- Scenario generator: the same as for evaluation (world, targets, clutter, detection draws).
- **Tuning seeds** `S_tune` = 1000–1019 (20 scenarios). **Evaluation seeds** `S_eval` = 2000–…,
  disjoint.
- Each seed is a `numpy.random.SeedSequence`, spawned into independent streams for world, detection
  and policy table. C2 has no policy randomness.

## 5. Selection

1. For each configuration in the grid, compute the mean score over `S_tune` and its standard error.
2. Select the maximum mean score. Ties within one standard error of the best go to the shorter
   mean duration, then to f closest to 0.8.
3. Report the full grid (mean ± sd for every configuration), not only the winner.

## 6. Freeze

- Write `config/c2_tuned.yaml` (the chosen values) and `docs/C2_TUNING_RESULT.md` (grid, seeds,
  code revision, manifest). Record the SHA-256 of `c2_tuned.yaml` in the run manifest.
- **Enforcement.** The `eval/` runner refuses to run C3, C4 or any ablation on `S_eval` unless
  `c2_tuned.yaml` exists, and records its hash in every result row.
- The dev-loop freeze is **preliminary** (reference-belief-conditional). The E4 freeze is done
  in-machine, in Gazebo, on tuning worlds disjoint from the evaluation worlds (D7).

## 7. Fairness safeguards

- C3 and C4 inherit the frozen coverage settings unchanged. No C4 parameter is tuned on `S_eval`.
- A C4 parameter change after any `S_eval` result → re-run **every** condition on `S_eval` and
  report both runs.
- κ, λ, ε_J and the rule thresholds are swept and reported as sensitivity. They are never chosen by
  looking at the C4 − C2 difference on `S_eval`.
- The budget is always the frozen C2 duration from the same hold.
- Any deviation from this protocol is recorded in `C2_TUNING_RESULT.md`.
