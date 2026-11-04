# Dev-loop results — budget_merged

**reference-belief-conditional, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Reference belief = a stand-in written from the briefs, NOT BlueBoat-Belief. World detection model = placeholders matched to the planner (not fitted from Gazebo).

Seeds: 2000-2019 (n = 20), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.136, b = 0.478, n = 371).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 1.90 ± 1.55 | 3.00 ± 1.38 | 1.10 ± 1.21 | 4.75 | — | — | — | — | 0 | 0.00/0.00 | 0.052 |
| C3 | 20 | 1.85 ± 1.95 | 3.05 ± 1.39 | 1.20 ± 1.15 | 5.00 | -0.05 ± 1.15 | [-0.56, +0.46] | 6/6/8 | 9 / 8 | 406 | 0.05/0.12 | 0.055 |
| C4 | 20 | 2.15 ± 1.39 | 3.45 ± 1.10 | 1.30 ± 0.80 | 4.65 | +0.25 ± 1.33 | [-0.35, +0.85] | 7/8/5 | 18 / 9 | 868 | 1.73/8.10 | 0.045 |
| C4_iso | 20 | 2.25 ± 1.52 | 3.50 ± 1.05 | 1.25 ± 1.07 | 4.60 | +0.35 ± 1.23 | [-0.20, +0.90] | 8/8/4 | 14 / 4 | 836 | 1.86/7.14 | 0.062 |
| C4_rand | 20 | 2.15 ± 1.35 | 3.70 ± 0.98 | 1.55 ± 0.94 | 4.90 | +0.25 ± 1.55 | [-0.44, +0.94] | 9/4/7 | 22 / 8 | 894 | 1.98/8.06 | 0.058 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.55 | 1.00 | 1.55 | 1.90 |
| C3 | 0.55 | 1.00 | 1.65 | 1.85 |
| C4 | 0.65 | 1.85 | 2.05 | 2.15 |
| C4_iso | 0.60 | 1.90 | 2.15 | 2.25 |
| C4_rand | 0.70 | 1.80 | 2.45 | 2.15 |

## Budget fraction 0.75 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 1.40 ± 1.10 | 1.95 ± 1.00 | 0.55 ± 0.76 | 4.15 | — | — | — | — | 0 | 0.00/0.00 | 0.070 |
| C3 | 20 | 1.65 ± 1.46 | 2.50 ± 1.05 | 0.85 ± 1.04 | 4.25 | +0.25 ± 1.21 | [-0.29, +0.79] | 9/5/6 | 15 / 4 | 367 | 0.06/0.16 | 0.063 |
| C4 | 20 | 2.40 ± 1.39 | 3.30 ± 0.98 | 0.90 ± 0.97 | 4.60 | +1.00 ± 1.26 | [+0.44, +1.56] | 13/6/1 | 30 / 3 | 664 | 1.28/4.63 | 0.063 |
| C4_iso | 20 | 2.35 ± 1.46 | 3.15 ± 1.14 | 0.80 ± 0.89 | 4.35 | +0.95 ± 1.10 | [+0.46, +1.44] | 14/5/1 | 25 / 1 | 660 | 1.29/4.40 | 0.067 |
| C4_rand | 20 | 2.40 ± 1.23 | 3.20 ± 0.83 | 0.80 ± 0.83 | 4.60 | +1.00 ± 1.38 | [+0.38, +1.62] | 13/5/2 | 28 / 3 | 662 | 1.30/4.59 | 0.065 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.20 | 0.80 | 1.15 | 1.40 |
| C3 | 0.20 | 0.80 | 1.10 | 1.65 |
| C4 | 0.25 | 1.50 | 2.05 | 2.40 |
| C4_iso | 0.25 | 1.35 | 2.15 | 2.35 |
| C4_rand | 0.20 | 1.45 | 2.00 | 2.40 |

## Budget fraction 0.5 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 0.85 ± 1.04 | 1.10 ± 0.79 | 0.25 ± 0.55 | 3.45 | — | — | — | — | 0 | 0.00/0.00 | 0.085 |
| C3 | 20 | 1.35 ± 1.31 | 1.70 ± 0.98 | 0.35 ± 0.67 | 3.70 | +0.50 ± 0.83 | [+0.13, +0.87] | 10/8/2 | 12 / 0 | 148 | 0.06/0.15 | 0.087 |
| C4 | 20 | 1.65 ± 1.14 | 2.05 ± 0.89 | 0.40 ± 0.60 | 3.95 | +0.80 ± 1.01 | [+0.35, +1.25] | 11/8/1 | 21 / 2 | 385 | 1.03/4.15 | 0.093 |
| C4_iso | 20 | 1.50 ± 1.24 | 1.85 ± 1.09 | 0.35 ± 0.59 | 3.75 | +0.65 ± 0.88 | [+0.26, +1.04] | 10/9/1 | 18 / 3 | 375 | 1.11/4.06 | 0.092 |
| C4_rand | 20 | 2.05 ± 1.28 | 2.30 ± 1.08 | 0.25 ± 0.55 | 3.75 | +1.20 ± 1.06 | [+0.73, +1.67] | 14/6/0 | 25 / 1 | 407 | 1.00/4.07 | 0.104 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.00 | 0.55 | 0.80 | 0.85 |
| C3 | 0.00 | 0.55 | 0.80 | 1.35 |
| C4 | 0.00 | 0.60 | 1.30 | 1.65 |
| C4_iso | 0.00 | 0.50 | 1.35 | 1.50 |
| C4_rand | 0.00 | 0.65 | 1.45 | 2.05 |

