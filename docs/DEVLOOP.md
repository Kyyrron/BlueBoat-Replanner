# Development loop — method and results (Phase 3)

> **Reference-belief-conditional, preliminary. Development evidence only, never E4.**
>
> - The belief is a **stand-in written from the briefs**, not BlueBoat-Belief.
> - The world's detection model consists of **placeholders matched to the planner's**. It is not
>   fitted from Gazebo, so per synthesis §6.2 ⚠ no number here may enter the thesis as a result.
> - Everything must be re-run with the real Belief core (`LOCAL_CHECKLIST` L-B7) and, for E4, in
>   Gazebo.

## 1. What runs

```
world (eval/world.py) ─ looks along the executed path ─► reference belief (eval/belief_reference.py)
        ▲                                                        │ belief_grid/0 dict (to_grid_dict), t_lat delay
        │ emulator: one draw per object per pass                ▼
        └──────────── executed path ◄── Control stub ◄── replanner policy (core/policy/*)
```

**Execution model** (`eval/devloop.py`):
- **Tracking.** The boat flies the accepted path exactly at nominal speed (τ = time), simulated in
  1 s steps.
- **First path.** Every condition sends the same first path (the C1 or C2 plan from the hold)
  through the same stub.
- **Decisions.** Decisions follow DESIGN §5.2: trigger at t_dp − (lead_min + T_decide + margin),
  the pre-send check, then the stub's acceptance rules.
- **Belief latency.** The belief snapshot a decision sees is the one from τ − t_lat (t_lat = 10 s).

## 2. Reference belief (stand-in; the semantics of the brief §9.2 and contract C-3/C-4)

| Item | Reference choice |
|---|---|
| Grid | 0.5 m cells over the area + 2 m; cells outside the area are NaN |
| Look | (cell, directed sector, pass), one per pass and sector; pings are not looks |
| Miss update | Q ← Q (1 − PD_b)^ρ, with ρ = 1 for a sector new to the cell and 0.3 otherwise. PD_b = 0.7 on [3, min(25, √(R² − h²))] m |
| Contacts | 3σ gating to the nearest contact, else a new uid (never reused) |
| Sector sets | detected and missed directed sectors; a miss is recorded at the end of its pass. A detection from a previously missed sector adds no evidence |
| Confirmation | ≥ 2 detections on different passes (brief §9.2) |
| Pass | one plan segment: a coverage line, a transit, a connection or a revisit body ([ASSUMPTION]; Belief's own segmentation is L-B6) |
| p_exists | odds update: × PD_b/f_ref per new detected sector, × (1 − PD_b)/(1 − f_ref) per miss; pruned below 0.01 |
| Exports | `first_confidence`, `first_sector`, `coverage` = 1 − Q, `aspect_counts`, `p_background`, `depth_m` |

## 3. World and emulator

**World:**
- 100 × 60 m area at 2 m depth; hold pose (12, 12, 0);
- 6 targets: classes uniform over block, pipe, cage, tyre, plate; continuous orientation uniform
  on [0, 2π);
- 6 clutter objects;
- every object at least 8 m from the edge.

**World detection probability:** PD = 0.7 · s_k · A_k(b − φ) on the detection band, with the
planner's placeholder windows. Clutter is detected with probability 0.2 per pass.

**Draws:**
- One draw per object per pass, when its cell is looked at.
- Detection position noise σ = 0.5 m.
- Raw score: Beta(4, 2) for a target, Beta(2, 4) for clutter.

**Common random numbers.** Every draw comes from `SeedSequence([seed, 2, object, look number])`.
So paired conditions share the world (`SeedSequence([seed, 1])`), and the k-th look at an object
gets the same draw whatever path produced it.

**Calibration** (brief §9.4). Raw scores are mapped to confidences by Platt scaling, fitted on C2
runs of the calibration seeds 900–919 with the frozen C2 settings. The ECE of the evaluation
runs is reported.

## 4. Protocol

- **Unit:** one mission (one world).
- **n:** the number of evaluation seeds (paired across conditions).
- **Seeds:** tuning 1000–1019; calibration 900–919; evaluation 2000 onwards. All disjoint.
- **C2 first.** C2 is tuned and frozen per `C2_TUNING_PROTOCOL.md` (dev-loop freeze,
  preliminary), and every condition uses the frozen coverage settings. The runner refuses C3/C4
  without the freeze, and every result row carries the freeze's sha256.
- **Budget.** B = the duration of the frozen C2 plan from the hold. Every plan is truncated at
  f·B (budget sweep f ∈ {1.0, 0.75, 0.5}). C1 ends earlier, and its metrics are held to the end.
- **Metrics,** against world truth, eval only:
  - confirmed contacts are matched one-to-one to true targets, greedy nearest within 2 m;
  - **score** = confirmed true − κ·false confirmations (κ = 1, the E4 primary quantity);
  - **detected** = targets matched by any contact;
  - **anytime** = the score at 25/50/75/100 % of the budget;
  - **targets confirmed only here / only by C2**, per seed;
  - aspects per target;
  - decision time (wall, eval only);
  - refusals and missed leads.
- **Paired statistics vs C2:** mean ± sd of the difference, ≈ 95 % CI (mean ± 2 sd/√n), and
  better / equal / worse counts.
- **Exploratory.** Every ablation is exploratory. No sample size was chosen for power, and no
  significance is claimed.
- **Failures.** A mission that raises is reported, not dropped. There were none (see the results).

## 5. Results — reference-belief-conditional, preliminary

Run on 2026-11-04 by `eval/runner.py`. The numbers are copied from
`results/devloop_2026-11-04/main/summary.md` and `summary.json`; the plots are `anytime.png`
there and `budget_sweep.png` in `results/devloop_2026-11-04/budget_merged/`.
- Module revision `ba97fd7+uncommitted`, python 3.13.18, numpy 2.4.4.
- Frozen C2: `config/c2_tuned.yaml`, sha256 `abff9534…`: f = 1.0, pass 1 along the long axis,
  interlace auto. B(C2) = 2111 s from the hold. The C2 tuning is in `C2_TUNING_RESULT.md`.
- Calibration: Platt fitted on 371 C2 detections of seeds 900–919, a = 2.136, b = 0.478.
- 240 missions (8 conditions × 30 seeds 2000–2029, paired), plus the budget sweep (§5.3).
  **No mission failed.**
- Over every mission: **0 refusals, 0 missed leads.** Every result row carries the freeze's sha256.

### 5.1 Full budget (f = 1.0, n = 30)

Score = confirmed true − false confirmations (κ = 1). Δ = paired difference vs C2 on the same
seed; CI ≈ mean ± 2 sd/√n.

| Condition | score | confirmed true | false conf. | detected | Δ vs C2 | ≈95 % CI | better / equal / worse | only here / only C2 | aspects per target | revisit time s | decision s mean / max |
|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 | 1.03 ± 0.89 | 1.30 | 0.27 | 3.80 | −0.97 ± 1.27 | [−1.43, −0.50] | 4 / 7 / 19 | 0 / 51 | 1.31 | 0 | — |
| **C2** | **2.00 ± 1.41** | 3.00 | 1.00 | 4.83 | — | — | — | — | 2.43 | 0 | — |
| C3 | 1.90 ± 1.65 | 3.10 | 1.20 | 4.97 | −0.10 ± 1.03 | [−0.48, +0.28] | 8 / 10 / 12 | 15 / 12 | 2.77 | 410 | 0.06 / 0.19 |
| C4 | 2.27 ± 1.55 | 3.57 | 1.30 | 4.77 | +0.27 ± 1.62 | [−0.32, +0.86] | 12 / 9 / 9 | 29 / 12 | 3.74 | 890 | 1.74 / 8.10 |
| C4_iso | 2.20 ± 1.61 | 3.50 | 1.30 | 4.60 | +0.20 ± 1.35 | [−0.29, +0.69] | 12 / 9 / 9 | 23 / 8 | 3.61 | 871 | 1.96 / 8.10 |
| C4_rand | 2.27 ± 1.36 | 3.77 | 1.50 | 5.03 | +0.27 ± 1.51 | [−0.28, +0.82] | 13 / 7 / 10 | 33 / 10 | 3.66 | 912 | 1.89 / 8.06 |
| C4_depth1 | 2.57 ± 1.55 | 3.83 | 1.27 | 4.83 | +0.57 ± 1.87 | [−0.12, +1.25] | 17 / 5 / 8 | 35 / 10 | 3.67 | 897 | 0.57 / 2.01 |
| C4_degraded | 2.40 ± 1.43 | 3.67 | 1.27 | 4.80 | +0.40 ± 1.16 | [−0.02, +0.82] | 17 / 3 / 10 | 29 / 9 | 3.66 | 877 | 1.63 / 5.66 |

Each world holds 6 targets and 6 clutter objects. ECE of the calibrated confidences on the
evaluation runs: 0.037–0.068.

**Anytime** (mean score at a fraction of the budget; paired Δ vs C2 at 50 % in the last column):

| Condition | 25 % | 50 % | 75 % | 100 % | Δ at 50 %, ≈95 % CI, b / e / w |
|---|---|---|---|---|---|
| C1 | 0.53 | 1.03 | 1.03 | 1.03 | — |
| C2 | 0.53 | 1.03 | 1.63 | 2.00 | — |
| C3 | 0.53 | 1.03 | 1.73 | 1.90 | 0.00 (identical to C2 up to the pass boundary) |
| C4 | 0.63 | 1.77 | 2.03 | 2.27 | +0.73 [+0.25, +1.22], 18 / 7 / 5 |
| C4_iso | 0.60 | 1.77 | 2.03 | 2.20 | +0.73 [+0.33, +1.14], 17 / 9 / 4 |
| C4_rand | 0.67 | 1.73 | 2.40 | 2.27 | +0.70 [+0.23, +1.17], 17 / 8 / 5 |
| C4_depth1 | 0.57 | 1.83 | 2.37 | 2.57 | +0.80 [+0.26, +1.34], 17 / 8 / 5 |
| C4_degraded | 0.53 | 1.67 | 1.97 | 2.40 | +0.63 [+0.23, +1.03], 15 / 11 / 4 |

The 50 % column was computed from `results.jsonl` with the same paired formula as the runner.

### 5.2 What these numbers say, and what they do not

All of the following holds **in this dev loop only**: stand-in belief, a world whose detection
model *is* the planner's model, perfect tracking, and decisions that take no mission time.

1. **Single pass is clearly worse than C2** (Δ −0.97, CI [−1.43, −0.50]). The C2 baseline is not a
   strawman here: it is tuned and frozen first (CM-12).
2. **No adaptive condition beats C2 at the full budget by more than the noise.** Every C4 CI
   contains 0; C4_degraded's upper half is the closest ([−0.02, +0.82]). n = 30 was not chosen
   for power, and no significance is claimed (§4).
3. **C4 confirms more true targets than C2 (3.57 vs 3.00), but also makes more false
   confirmations (1.30 vs 1.00).** Revisits confirm clutter as well as targets.
4. **The aspect-aware choice of bearing shows no effect here.** Paired with C4 on the same
   seeds:
   - C4 − C4_rand (C4's revisits, random bearings) = +0.00 ± 1.11, CI [−0.41, +0.41];
   - C4 − C4_iso (aspect-agnostic planner) = +0.07 ± 1.31, CI [−0.41, +0.55];
   - C4_degraded (planner windows ×2, PD −20 %) − C4 = +0.13 ± 1.04, CI [−0.25, +0.51].

   In this dev loop the gain, if any, comes from **revisiting at all, and early**. It does not
   come from the bearing. In the reference belief a confirmation needs two detections on different
   passes (brief §9.2), and **every plan segment is a pass**: each coverage line, transit and
   revisit body (`eval/devloop.py`). So any revisit of a detected contact is a chance to confirm
   it, whatever its bearing. This is a property of the setup, not evidence about aspect
   dependence. The world's aspect windows are the planner's moderate placeholders (block and cage
   only: a = 0.4 / 0.3 over 4 sectors). Whether a measured aspect response (E3), or Belief's own
   pass definition (L-B6), changes this is open.
5. **The one-step lookahead does at least as well as the two-step one.** C4_depth1 − C4 =
   +0.30 ± 1.53, CI [−0.26, +0.86], better / equal / worse 10 / 13 / 7. It decides in a third of
   the time.
6. **The adaptive conditions are clearly ahead mid-mission.** At 50 % of the budget every C4
   variant leads C2 by about +0.7, with a CI excluding 0.
   - By then C2 has flown about pass 1, so its only confirmations come from objects seen from two
     adjacent lines (C2 at 50 % = C1 at its end, 1.03).
   - C4 revisits contacts during pass 1 and confirms them there.

   This is what a budget cut exploits (§5.3).
7. **C3 ≈ C2** (Δ −0.10). Appending revisits at the pass boundary spends 410 s of the budget, and
   loses about as much pass-2 coverage as it gains in confirmations.
8. **Decision time.** C4 decisions took 1.74 s on average and up to 8.1 s (wall, 4 missions in
   parallel on 4 cores). The configured allowance `t_decide_s` is 2.0 s. The dev loop does not
   charge decision time to the mission clock (§1), so a live decision slower than its allowance
   would surface as `missed_lead` (the pre-send check), never as an unsafe request. Measure it
   in-machine (L-G4) and set `t_decide_s` from it, or use C4_depth1 (max 2.0 s here). The
   uncontended timing is in §5.4.
9. **Execution.** 0 refusals and 0 missed leads over every mission. The Control stub reproduces
   `PathUpdater`'s answers ([21]); a stub cannot show Gazebo tracking errors.

### 5.3 Budget sweep (f ∈ {1.0, 0.75, 0.5}, n = 20 paired seeds 2000–2019)

`results/devloop_2026-11-04/budget_sweep/` ran f = 0.75 and 0.5. `budget_merged/` joins them with
the f = 1.0 rows of `main/` on the common seeds (`runner.py --from-results`; it refuses runs whose
C2 freeze, calibration or scenario differ). Final score, Δ vs C2 at the same f, ≈95 % CI,
better / equal / worse:

| Condition | f = 1.0 | f = 0.75 | f = 0.5 |
|---|---|---|---|
| C2 | 1.90 ± 1.55 | 1.40 ± 1.10 | 0.85 ± 1.04 |
| C3 | 1.85; −0.05 [−0.56, +0.46]; 6 / 6 / 8 | 1.65; +0.25 [−0.29, +0.79]; 9 / 5 / 6 | 1.35; **+0.50 [+0.13, +0.87]**; 10 / 8 / 2 |
| C4 | 2.15; +0.25 [−0.35, +0.85]; 7 / 8 / 5 | 2.40; **+1.00 [+0.44, +1.56]**; 13 / 6 / 1 | 1.65; **+0.80 [+0.35, +1.25]**; 11 / 8 / 1 |
| C4_iso | 2.25; +0.35 [−0.20, +0.90]; 8 / 8 / 4 | 2.35; **+0.95 [+0.46, +1.44]**; 14 / 5 / 1 | 1.50; **+0.65 [+0.26, +1.04]**; 10 / 9 / 1 |
| C4_rand | 2.15; +0.25 [−0.44, +0.94]; 9 / 4 / 7 | 2.40; **+1.00 [+0.38, +1.62]**; 13 / 5 / 2 | 2.05; **+1.20 [+0.73, +1.67]**; 14 / 6 / 0 |

Bold: CI excludes 0. Revisit time at f = 0.75 / 0.5: C4 664 / 385 s, C3 367 / 148 s. No refusal,
no missed lead.

**Reading, and its limit:**
- Under a budget cut, the adaptive conditions are clearly ahead of C2. C4_rand is again as good as
  C4 (better at f = 0.5), so the same caveat as §5.2 item 4 applies: revisiting matters, the
  bearing does not, in this setup.
- The score is not monotonic in the budget for the C4 family: C4 scores 2.40 at f = 0.75 and
  2.15 at f = 1.0, with false confirmations rising from 0.90 to 1.30. Extra time also confirms
  clutter, and κ = 1 charges it. The clutter re-detection probability f = 0.2 (O-5) and κ are the
  parameters to sweep for sensitivity, never to tune (C2_TUNING_PROTOCOL §7).
- **The sweep truncates the frozen full-budget C2 plan.** It models a mission stopped early: C2
  at f = 0.5 has flown about pass 1. A C2 *planned* for 0.5·B, for example wider spacing so that
  both passes fit, is **not tested**. CM-12 says a weakened baseline sinks the comparison, so the
  sweep supports "C4 degrades more gracefully when a C2-length mission is cut". It does not
  support "C4 beats a two-pass survey designed for a shorter budget". Re-tuning C2 per budget
  (C2_TUNING_PROTOCOL §3 with B_cap = f·B) is required before the second statement
  (`.claude/TODO.md`).

### 5.4 Decision time without contention, and determinism

`results/devloop_2026-11-04/timing_uncontended/`: C4 and C4_depth1, seeds 2000–2004, one process
(`--jobs 1`), nothing else running. It reuses the main run's manifest and calibration.

| Policy | decisions per mission | decision time mean | max |
|---|---|---|---|
| C4 | 12.0 | 1.54 s | 5.60 s |
| C4_depth1 | 12.2 | 0.53 s | 1.51 s |

- **Determinism:** every one of these 10 missions is identical to its main-run row (final metrics
  and requests). Results do not depend on the job count.
- **Allowance:** C4's worst decision exceeds `t_decide_s` = 2.0 s even uncontended; C4_depth1's
  does not.
  - Live, a late decision becomes `missed_lead` and is not sent. This is safe, but it is a lost
    decision the dev loop did not charge.
  - Choosing a larger `t_decide_s` (or C4_depth1 as the deployed C4) changes C4's decision
    timing. Under C2_TUNING_PROTOCOL §7 every condition is then re-run on `S_eval`, and both runs
    are reported. **User decision** (`HANDOVER_CLOUD.md`, open decisions).
- The CPU here is the cloud container's (4 cores). The laptop's timing is L-G4.
