# Dev-loop results — main

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `fcd9531597e593b3…`), through its offline interface. Misses credited: `records`. Class reported by the emulated detector: `none`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2029 (n = 30), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C1 | 30 | 1.03 ± 1.03 | 1.37 ± 0.85 | 0.33 ± 0.61 | 3.73 | -0.83 ± 1.74 | [-1.47, -0.20] | 6/7/17 | 0 / 56 | 0 | 0.00/0.00 | 0.057 |
| C2 | 30 | 1.87 ± 1.70 | 3.23 ± 1.30 | 1.37 ± 0.96 | 4.70 | — | — | — | — | 0 | 0.00/0.00 | 0.045 |
| C3 | 30 | 1.73 ± 1.66 | 3.53 ± 1.20 | 1.80 ± 1.03 | 4.93 | -0.13 ± 1.07 | [-0.53, +0.26] | 7/13/10 | 16 / 7 | 427 | 0.04/0.14 | 0.040 |
| C4 | 30 | 2.27 ± 1.17 | 3.97 ± 1.03 | 1.70 ± 0.92 | 4.93 | +0.40 ± 1.54 | [-0.16, +0.96] | 13/11/6 | 27 / 5 | 865 | 0.66/2.54 | 0.047 |
| C4_iso | 30 | 2.23 ± 1.59 | 4.00 ± 1.23 | 1.77 ± 0.90 | 4.93 | +0.37 ± 1.47 | [-0.17, +0.90] | 13/8/9 | 25 / 2 | 739 | 0.70/2.57 | 0.043 |
| C4_rand | 30 | 1.87 ± 1.31 | 3.97 ± 1.22 | 2.10 ± 0.99 | 5.10 | +0.00 ± 1.44 | [-0.53, +0.53] | 8/12/10 | 29 / 7 | 879 | 0.69/2.76 | 0.039 |
| C4_depth1 | 30 | 2.30 ± 1.29 | 4.10 ± 1.24 | 1.80 ± 0.81 | 5.03 | +0.43 ± 1.52 | [-0.12, +0.99] | 13/9/8 | 31 / 5 | 844 | 0.23/0.85 | 0.045 |
| C4_degraded | 30 | 2.20 ± 1.49 | 3.80 ± 1.30 | 1.60 ± 0.77 | 4.80 | +0.33 ± 1.30 | [-0.14, +0.81] | 14/7/9 | 22 / 5 | 809 | 0.68/2.62 | 0.041 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C1 | 0.37 | 1.03 | 1.03 | 1.03 |
| C2 | 0.37 | 1.03 | 1.33 | 1.87 |
| C3 | 0.37 | 1.03 | 1.83 | 1.73 |
| C4 | 0.67 | 1.90 | 2.20 | 2.27 |
| C4_iso | 0.80 | 1.83 | 2.13 | 2.23 |
| C4_rand | 0.50 | 1.73 | 1.93 | 1.87 |
| C4_depth1 | 0.73 | 2.00 | 2.27 | 2.30 |
| C4_degraded | 0.80 | 1.80 | 2.10 | 2.20 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C1 | — | 1237 | 0 | 11 | 8.2 | 0 |
| C2 | — | 2459 | 0 | 22 | 17.2 | 0 |
| C3 | 6.8 / 6.8 | 2709 | 0 | 39 | 18.6 | 0 |
| C4 | 6.7 / 8.7 | 2832 | 0 | 46 | 20.7 | 0 |
| C4_iso | 6.8 / 8.7 | 2775 | 0 | 39 | 20.0 | 0 |
| C4_rand | 6.6 / 8.7 | 2899 | 0 | 55 | 21.0 | 0 |
| C4_depth1 | 6.7 / 8.7 | 2905 | 0 | 46 | 21.1 | 0 |
| C4_degraded | 6.7 / 8.7 | 2762 | 0 | 36 | 20.1 | 0 |

