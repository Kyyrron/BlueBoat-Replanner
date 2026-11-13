# Dev-loop results — class_true

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `fcd9531597e593b3…`), through its offline interface. Misses credited: `records`. Class reported by the emulated detector: `true`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2029 (n = 30), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 30 | 1.87 ± 1.70 | 3.23 ± 1.30 | 1.37 ± 0.96 | 4.70 | — | — | — | — | 0 | 0.00/0.00 | 0.045 |
| C4 | 30 | 2.40 ± 1.22 | 4.00 ± 1.17 | 1.60 ± 0.86 | 4.90 | +0.53 ± 1.74 | [-0.10, +1.17] | 14/7/9 | 27 / 4 | 830 | 0.67/2.71 | 0.046 |
| C4_iso | 30 | 2.23 ± 1.68 | 3.93 ± 1.26 | 1.70 ± 0.92 | 4.90 | +0.37 ± 1.30 | [-0.11, +0.84] | 12/11/7 | 23 / 2 | 739 | 0.78/3.34 | 0.041 |
| C4_rand | 30 | 1.83 ± 1.21 | 3.93 ± 1.17 | 2.10 ± 0.99 | 5.13 | -0.03 ± 1.50 | [-0.58, +0.51] | 9/10/11 | 29 / 8 | 897 | 0.71/2.58 | 0.045 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.37 | 1.03 | 1.33 | 1.87 |
| C4 | 0.80 | 1.87 | 2.07 | 2.40 |
| C4_iso | 0.80 | 1.83 | 2.13 | 2.23 |
| C4_rand | 0.50 | 1.63 | 1.83 | 1.83 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C2 | — | 2459 | 0 | 22 | 17.2 | 0 |
| C4 | 6.7 / 8.7 | 2791 | 0 | 44 | 20.4 | 0 |
| C4_iso | 6.8 / 8.7 | 2752 | 0 | 39 | 19.7 | 0 |
| C4_rand | 6.7 / 8.7 | 2901 | 0 | 55 | 20.9 | 0 |

