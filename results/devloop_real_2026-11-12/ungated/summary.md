# Dev-loop results — ungated

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `fcd9531597e593b3…`), through its offline interface. Misses credited: `ungated`. Class reported by the emulated detector: `none`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2029 (n = 30), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 30 | 1.87 ± 1.70 | 3.23 ± 1.30 | 1.37 ± 0.96 | 4.70 | — | — | — | — | 0 | 0.00/0.00 | 0.045 |
| C3 | 30 | 1.70 ± 1.64 | 3.50 ± 1.17 | 1.80 ± 1.03 | 4.93 | -0.17 ± 1.12 | [-0.57, +0.24] | 7/13/10 | 15 / 7 | 429 | 0.04/0.14 | 0.041 |
| C4 | 30 | 2.17 ± 1.23 | 4.00 ± 1.14 | 1.83 ± 0.83 | 4.93 | +0.30 ± 1.73 | [-0.33, +0.93] | 12/9/9 | 28 / 5 | 868 | 0.68/2.55 | 0.052 |
| C4_rand | 30 | 2.00 ± 1.23 | 3.93 ± 1.11 | 1.93 ± 0.91 | 5.10 | +0.13 ± 1.38 | [-0.37, +0.64] | 12/8/10 | 28 / 7 | 875 | 0.71/2.61 | 0.050 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.47 | 1.03 | 1.33 | 1.87 |
| C3 | 0.47 | 1.03 | 1.83 | 1.70 |
| C4 | 0.73 | 1.83 | 2.13 | 2.17 |
| C4_rand | 0.67 | 1.93 | 2.00 | 2.00 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C2 | — | 2459 | 0 | 22 | 17.2 | 0 |
| C3 | 0.0 / 0.0 | 2707 | 0 | 39 | 18.7 | 0 |
| C4 | 0.0 / 0.0 | 2903 | 0 | 53 | 21.2 | 0 |
| C4_rand | 0.0 / 0.0 | 2929 | 0 | 65 | 21.0 | 0 |

