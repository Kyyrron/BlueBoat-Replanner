# Dev-loop results — budget_merged

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `fcd9531597e593b3…`), through its offline interface. Misses credited: `records`. Class reported by the emulated detector: `none`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2019 (n = 20), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 1.65 ± 1.84 | 3.15 ± 1.35 | 1.50 ± 1.10 | 4.50 | — | — | — | — | 0 | 0.00/0.00 | 0.048 |
| C3 | 20 | 1.70 ± 1.84 | 3.55 ± 1.23 | 1.85 ± 1.18 | 4.90 | +0.05 ± 1.05 | [-0.42, +0.52] | 5/10/5 | 10 / 2 | 417 | 0.03/0.06 | 0.045 |
| C4 | 20 | 2.35 ± 1.09 | 3.95 ± 1.05 | 1.60 ± 0.99 | 4.85 | +0.70 ± 1.53 | [+0.02, +1.38] | 10/7/3 | 18 / 2 | 796 | 0.61/2.27 | 0.060 |
| C4_iso | 20 | 2.20 ± 1.67 | 3.90 ± 1.25 | 1.70 ± 0.98 | 4.80 | +0.55 ± 1.57 | [-0.15, +1.25] | 9/6/5 | 17 / 2 | 716 | 0.66/2.24 | 0.051 |
| C4_rand | 20 | 1.90 ± 1.21 | 3.95 ± 1.10 | 2.05 ± 1.05 | 5.05 | +0.25 ± 1.16 | [-0.27, +0.77] | 6/10/4 | 19 / 3 | 825 | 0.64/2.27 | 0.052 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.30 | 0.95 | 1.10 | 1.65 |
| C3 | 0.30 | 0.95 | 1.75 | 1.70 |
| C4 | 0.60 | 1.80 | 2.15 | 2.35 |
| C4_iso | 0.75 | 1.75 | 2.20 | 2.20 |
| C4_rand | 0.45 | 1.70 | 2.05 | 1.90 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C2 | — | 1633 | 0 | 14 | 16.8 | 0 |
| C3 | 6.8 / 6.8 | 1794 | 0 | 18 | 18.3 | 0 |
| C4 | 6.8 / 8.7 | 1839 | 0 | 27 | 19.8 | 0 |
| C4_iso | 6.8 / 8.7 | 1818 | 0 | 25 | 19.4 | 0 |
| C4_rand | 6.6 / 8.7 | 1914 | 0 | 34 | 20.8 | 0 |

## Budget fraction 0.75 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 0.90 ± 1.33 | 1.90 ± 1.07 | 1.00 ± 0.79 | 4.00 | — | — | — | — | 0 | 0.00/0.00 | 0.060 |
| C3 | 20 | 1.55 ± 1.47 | 2.70 ± 1.08 | 1.15 ± 1.14 | 4.25 | +0.65 ± 1.18 | [+0.12, +1.18] | 13/3/4 | 18 / 2 | 363 | 0.03/0.06 | 0.051 |
| C4 | 20 | 2.10 ± 1.29 | 3.25 ± 1.25 | 1.15 ± 0.99 | 4.40 | +1.20 ± 1.32 | [+0.61, +1.79] | 14/4/2 | 32 / 5 | 583 | 0.67/2.28 | 0.063 |
| C4_iso | 20 | 2.45 ± 1.32 | 3.35 ± 1.14 | 0.90 ± 0.91 | 4.35 | +1.55 ± 1.32 | [+0.96, +2.14] | 15/5/0 | 31 / 2 | 579 | 0.64/2.40 | 0.066 |
| C4_rand | 20 | 2.05 ± 1.28 | 3.15 ± 1.14 | 1.10 ± 0.91 | 4.50 | +1.15 ± 1.27 | [+0.58, +1.72] | 12/7/1 | 28 / 3 | 591 | 0.59/2.11 | 0.068 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.30 | 0.65 | 0.90 | 0.90 |
| C3 | 0.30 | 0.65 | 1.00 | 1.55 |
| C4 | 0.40 | 1.35 | 1.80 | 2.10 |
| C4_iso | 0.30 | 0.95 | 2.00 | 2.45 |
| C4_rand | 0.20 | 1.35 | 1.75 | 2.05 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C2 | — | 1115 | 0 | 8 | 11.1 | 0 |
| C3 | 6.8 / 6.8 | 1265 | 0 | 14 | 13.1 | 0 |
| C4 | 6.7 / 8.7 | 1473 | 0 | 30 | 15.9 | 0 |
| C4_iso | 6.6 / 8.7 | 1438 | 0 | 21 | 14.8 | 0 |
| C4_rand | 6.7 / 8.7 | 1469 | 0 | 26 | 15.5 | 0 |

## Budget fraction 0.5 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 20 | 0.75 ± 1.29 | 1.15 ± 0.99 | 0.40 ± 0.68 | 3.30 | — | — | — | — | 0 | 0.00/0.00 | 0.081 |
| C3 | 20 | 1.15 ± 1.63 | 1.90 ± 1.07 | 0.75 ± 0.79 | 3.65 | +0.40 ± 1.23 | [-0.15, +0.95] | 11/3/6 | 15 / 0 | 146 | 0.03/0.06 | 0.067 |
| C4 | 20 | 1.65 ± 1.23 | 2.20 ± 1.06 | 0.55 ± 0.76 | 3.65 | +0.90 ± 1.17 | [+0.38, +1.42] | 14/4/2 | 23 / 2 | 381 | 0.43/1.84 | 0.100 |
| C4_iso | 20 | 1.55 ± 1.43 | 2.15 ± 1.27 | 0.60 ± 0.75 | 3.75 | +0.80 ± 1.20 | [+0.26, +1.34] | 14/3/3 | 23 / 3 | 378 | 0.46/1.63 | 0.077 |
| C4_rand | 20 | 1.65 ± 1.27 | 2.30 ± 0.98 | 0.65 ± 0.81 | 3.75 | +0.90 ± 0.97 | [+0.47, +1.33] | 14/4/2 | 23 / 0 | 400 | 0.41/1.63 | 0.068 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.10 | 0.30 | 0.65 | 0.75 |
| C3 | 0.10 | 0.30 | 0.65 | 1.15 |
| C4 | 0.10 | 0.50 | 1.15 | 1.65 |
| C4_iso | 0.10 | 0.70 | 1.30 | 1.55 |
| C4_rand | 0.10 | 0.65 | 1.20 | 1.65 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C2 | — | 729 | 0 | 6 | 7.1 | 0 |
| C3 | 4.6 / 4.6 | 902 | 0 | 13 | 9.3 | 0 |
| C4 | 6.9 / 8.7 | 949 | 0 | 22 | 9.7 | 0 |
| C4_iso | 7.0 / 8.7 | 945 | 0 | 17 | 9.9 | 0 |
| C4_rand | 6.7 / 8.7 | 925 | 0 | 20 | 10.2 | 0 |

