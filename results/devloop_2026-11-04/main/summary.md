# Dev-loop results — main

**reference-belief-conditional, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Reference belief = a stand-in written from the briefs, NOT BlueBoat-Belief. World detection model = placeholders matched to the planner (not fitted from Gazebo).

Seeds: 2000-2029 (n = 30), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.136, b = 0.478, n = 371).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 | 30 | 1.03 ± 0.89 | 1.30 ± 0.79 | 0.27 ± 0.52 | 3.80 | -0.97 ± 1.27 | [-1.43, -0.50] | 4/7/19 | 0 / 51 | 0 | 0.00/0.00 | 0.068 |
| C2 | 30 | 2.00 ± 1.41 | 3.00 ± 1.26 | 1.00 ± 1.05 | 4.83 | — | — | — | — | 0 | 0.00/0.00 | 0.045 |
| C3 | 30 | 1.90 ± 1.65 | 3.10 ± 1.27 | 1.20 ± 1.00 | 4.97 | -0.10 ± 1.03 | [-0.48, +0.28] | 8/10/12 | 15 / 12 | 410 | 0.06/0.19 | 0.042 |
| C4 | 30 | 2.27 ± 1.55 | 3.57 ± 1.19 | 1.30 ± 0.79 | 4.77 | +0.27 ± 1.62 | [-0.32, +0.86] | 12/9/9 | 29 / 12 | 890 | 1.74/8.10 | 0.037 |
| C4_iso | 30 | 2.20 ± 1.61 | 3.50 ± 1.20 | 1.30 ± 0.95 | 4.60 | +0.20 ± 1.35 | [-0.29, +0.69] | 12/9/9 | 23 / 8 | 871 | 1.96/8.10 | 0.050 |
| C4_rand | 30 | 2.27 ± 1.36 | 3.77 ± 1.17 | 1.50 ± 0.86 | 5.03 | +0.27 ± 1.51 | [-0.28, +0.82] | 13/7/10 | 33 / 10 | 912 | 1.89/8.06 | 0.044 |
| C4_depth1 | 30 | 2.57 ± 1.55 | 3.83 ± 1.18 | 1.27 ± 0.98 | 4.83 | +0.57 ± 1.87 | [-0.12, +1.25] | 17/5/8 | 35 / 10 | 897 | 0.57/2.01 | 0.050 |
| C4_degraded | 30 | 2.40 ± 1.43 | 3.67 ± 1.09 | 1.27 ± 0.91 | 4.80 | +0.40 ± 1.16 | [-0.02, +0.82] | 17/3/10 | 29 / 9 | 877 | 1.63/5.66 | 0.049 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C1 | 0.53 | 1.03 | 1.03 | 1.03 |
| C2 | 0.53 | 1.03 | 1.63 | 2.00 |
| C3 | 0.53 | 1.03 | 1.73 | 1.90 |
| C4 | 0.63 | 1.77 | 2.03 | 2.27 |
| C4_iso | 0.60 | 1.77 | 2.03 | 2.20 |
| C4_rand | 0.67 | 1.73 | 2.40 | 2.27 |
| C4_depth1 | 0.57 | 1.83 | 2.37 | 2.57 |
| C4_degraded | 0.53 | 1.67 | 1.97 | 2.40 |

