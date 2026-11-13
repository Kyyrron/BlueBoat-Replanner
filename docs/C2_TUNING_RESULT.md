# C2 tuning result (dev loop) — PRELIMINARY

Real-Belief dev loop, preliminary: BlueBoat-Belief's core (revision `093841a`, core sha256 `fcd9531597e593b3…`), misses credited `records`, emulated detector, no class information. Protocol: `C2_TUNING_PROTOCOL.md`. Tuning seeds 1000-1019 (n = 20), disjoint from the evaluation seeds. Scenario: {'width_m': 100.0, 'height_m': 60.0, 'depth_m': 2.0}. B_cap = T(C2 at f = 0.8, long axis) = 2499 s.

**Candidates of the tie rule.** best mean: f = 0.9, long_axis, interlace auto: 1.65 ± 1.81 (SE 0.41), 2214 s. Within one standard error of it: f = 0.8 long_axis auto (1.35, 2499 s); f = 0.8 long_axis off (1.35, 2499 s); f = 0.8 long_axis_plus_90 auto (1.30, 2483 s); f = 0.8 long_axis_plus_90 off (1.30, 2483 s); f = 0.9 long_axis auto (1.65, 2214 s); f = 0.9 long_axis off (1.65, 2214 s); f = 0.9 long_axis_plus_90 auto (1.55, 2197 s); f = 0.9 long_axis_plus_90 off (1.55, 2197 s); f = 1.0 long_axis auto (1.50, 2111 s); f = 1.0 long_axis off (1.50, 2111 s).

**Chosen:** spacing_factor 1.0, pass-1 direction long_axis, interlace auto (frozen in `config/c2_tuned.yaml`, sha256 f0bba92ffd4168bb…).

| f | direction | interlace | n | score mean ± sd | confirmed true | false conf. | detected | duration s | feasible |
|---|---|---|---|---|---|---|---|---|---|
| 0.6 | long_axis | auto | 20 | 1.00 ± 2.03 | 3.70 | 2.70 | 5.15 | 2888 | no |
| 0.6 | long_axis | off | 20 | 1.00 ± 2.03 | 3.70 | 2.70 | 5.15 | 2888 | no |
| 0.6 | long_axis_plus_90 | auto | 20 | 1.10 ± 2.05 | 3.75 | 2.65 | 5.15 | 2871 | no |
| 0.6 | long_axis_plus_90 | off | 20 | 1.10 ± 2.05 | 3.75 | 2.65 | 5.15 | 2871 | no |
| 0.7 | long_axis | auto | 20 | 1.10 ± 1.68 | 3.60 | 2.50 | 5.15 | 2785 | no |
| 0.7 | long_axis | off | 20 | 1.10 ± 1.68 | 3.60 | 2.50 | 5.15 | 2785 | no |
| 0.7 | long_axis_plus_90 | auto | 20 | 1.05 ± 1.93 | 3.60 | 2.55 | 5.15 | 2769 | no |
| 0.7 | long_axis_plus_90 | off | 20 | 1.05 ± 1.93 | 3.60 | 2.55 | 5.15 | 2769 | no |
| 0.8 | long_axis | auto | 20 | 1.35 ± 1.50 | 3.60 | 2.25 | 4.90 | 2499 | yes |
| 0.8 | long_axis | off | 20 | 1.35 ± 1.50 | 3.60 | 2.25 | 4.90 | 2499 | yes |
| 0.8 | long_axis_plus_90 | auto | 20 | 1.30 ± 1.75 | 3.55 | 2.25 | 4.85 | 2483 | yes |
| 0.8 | long_axis_plus_90 | off | 20 | 1.30 ± 1.75 | 3.55 | 2.25 | 4.85 | 2483 | yes |
| 0.9 | long_axis | auto | 20 | 1.65 ± 1.81 | 3.35 | 1.70 | 4.95 | 2214 | yes |
| 0.9 | long_axis | off | 20 | 1.65 ± 1.81 | 3.35 | 1.70 | 4.95 | 2214 | yes |
| 0.9 | long_axis_plus_90 | auto | 20 | 1.55 ± 1.88 | 3.30 | 1.75 | 4.95 | 2197 | yes |
| 0.9 | long_axis_plus_90 | off | 20 | 1.55 ± 1.88 | 3.30 | 1.75 | 4.95 | 2197 | yes |
| 1.0 | long_axis | auto | 20 | 1.50 ± 1.79 | 3.30 | 1.80 | 4.85 | 2111 | yes |
| 1.0 | long_axis | off | 20 | 1.50 ± 1.79 | 3.30 | 1.80 | 4.85 | 2111 | yes |
| 1.0 | long_axis_plus_90 | auto | 20 | 1.05 ± 1.93 | 3.05 | 2.00 | 4.80 | 2094 | yes |
| 1.0 | long_axis_plus_90 | off | 20 | 1.05 ± 1.93 | 3.05 | 2.00 | 4.80 | 2094 | yes |

Selection: maximum mean score among feasible configurations; ties within one standard error of the best go to the shorter duration, then to f closest to 0.8. The whole grid is reported.
