# C2 tuning result (dev loop) — PRELIMINARY

Reference-belief-conditional, preliminary. Protocol: `C2_TUNING_PROTOCOL.md`. Tuning seeds 1000-1019 (n = 20), disjoint from the evaluation seeds. Scenario: {'width_m': 100.0, 'height_m': 60.0, 'depth_m': 2.0}. B_cap = T(C2 at f = 0.8, long axis) = 2499 s.

**Chosen:** spacing_factor 1.0, pass-1 direction long_axis, interlace auto (frozen in `config/c2_tuned.yaml`, sha256 abff953478c186c5…).

| f | direction | interlace | n | score mean ± sd | confirmed true | false conf. | detected | duration s | feasible |
|---|---|---|---|---|---|---|---|---|---|
| 0.6 | long_axis | auto | 20 | 1.95 ± 1.76 | 3.70 | 1.75 | 4.85 | 2888 | no |
| 0.6 | long_axis | off | 20 | 1.95 ± 1.76 | 3.70 | 1.75 | 4.85 | 2888 | no |
| 0.6 | long_axis_plus_90 | auto | 20 | 1.55 ± 2.09 | 3.70 | 2.15 | 4.75 | 2871 | no |
| 0.6 | long_axis_plus_90 | off | 20 | 1.55 ± 2.09 | 3.70 | 2.15 | 4.75 | 2871 | no |
| 0.7 | long_axis | auto | 20 | 1.75 ± 1.97 | 3.60 | 1.85 | 4.90 | 2785 | no |
| 0.7 | long_axis | off | 20 | 1.75 ± 1.97 | 3.60 | 1.85 | 4.90 | 2785 | no |
| 0.7 | long_axis_plus_90 | auto | 20 | 1.55 ± 2.31 | 3.55 | 2.00 | 4.75 | 2769 | no |
| 0.7 | long_axis_plus_90 | off | 20 | 1.55 ± 2.31 | 3.55 | 2.00 | 4.75 | 2769 | no |
| 0.8 | long_axis | auto | 20 | 1.85 ± 1.66 | 3.50 | 1.65 | 4.80 | 2499 | yes |
| 0.8 | long_axis | off | 20 | 1.85 ± 1.66 | 3.50 | 1.65 | 4.80 | 2499 | yes |
| 0.8 | long_axis_plus_90 | auto | 20 | 1.70 ± 1.98 | 3.30 | 1.60 | 4.75 | 2483 | yes |
| 0.8 | long_axis_plus_90 | off | 20 | 1.70 ± 1.98 | 3.30 | 1.60 | 4.75 | 2483 | yes |
| 0.9 | long_axis | auto | 20 | 1.80 ± 1.44 | 3.10 | 1.30 | 4.70 | 2214 | yes |
| 0.9 | long_axis | off | 20 | 1.80 ± 1.44 | 3.10 | 1.30 | 4.70 | 2214 | yes |
| 0.9 | long_axis_plus_90 | auto | 20 | 1.70 ± 1.75 | 3.15 | 1.45 | 4.70 | 2197 | yes |
| 0.9 | long_axis_plus_90 | off | 20 | 1.70 ± 1.75 | 3.15 | 1.45 | 4.70 | 2197 | yes |
| 1.0 | long_axis | auto | 20 | 1.80 ± 1.85 | 3.25 | 1.45 | 4.80 | 2111 | yes |
| 1.0 | long_axis | off | 20 | 1.80 ± 1.85 | 3.25 | 1.45 | 4.80 | 2111 | yes |
| 1.0 | long_axis_plus_90 | auto | 20 | 1.15 ± 1.79 | 2.95 | 1.80 | 4.80 | 2094 | yes |
| 1.0 | long_axis_plus_90 | off | 20 | 1.15 ± 1.79 | 2.95 | 1.80 | 4.80 | 2094 | yes |

Selection: maximum mean score among feasible configurations; ties within one standard error of the best go to the shorter duration, then to f closest to 0.8. The whole grid is reported.

## Notes added at the freeze (2026-11-04)

- **The protocol was followed as pre-registered.** The best feasible mean was f = 0.8 (1.85).
  f = 1.0 (1.80) lies within one standard error (SE ≈ 0.37). The tie rule therefore chose the
  shorter plan: f = 1.0, long axis, B = 2111 s instead of 2499 s.
- **The budget is shorter for every condition.** C3 and C4 get the same, shorter B. Nothing in
  this choice looked at C3/C4.
- **Interlace has no effect here.** s ≈ 13.9 m ≥ 2 r_turn = 6 m, so `auto` and `off` produce
  identical plans.
- **Caveat.** The dev loop flies the path perfectly. With real or Gazebo tracking error, f = 1.0
  (no overlap margin) may leave holidays that f = 0.8 would not. This is why the E4 freeze must
  be redone in Gazebo, in-machine (protocol §6; `LOCAL_CHECKLIST` L-G). The dev-loop freeze is
  **preliminary**.
- Frozen file: `blueboat_replanner/config/c2_tuned.yaml`, sha256 `abff953478c186c5…`. The full
  hash is recorded in every result row of `results/devloop_2026-11-04/*/results.jsonl`.
