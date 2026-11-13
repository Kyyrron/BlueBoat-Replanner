# Dev-loop results — timing_uncontended

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `fcd9531597e593b3…`), through its offline interface. Misses credited: `records`. Class reported by the emulated detector: `none`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2004 (n = 5), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C4 | 5 | 2.40 ± 1.14 | 4.20 ± 0.84 | 1.80 ± 1.64 | 5.00 | — | — | — | — | 733 | 0.52/1.79 | 0.097 |
| C4_depth1 | 5 | 2.40 ± 0.55 | 4.40 ± 0.89 | 2.00 ± 1.00 | 4.60 | — | — | — | — | 780 | 0.19/0.49 | 0.089 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C4 | 1.00 | 2.00 | 2.20 | 2.40 |
| C4_depth1 | 0.80 | 1.80 | 2.20 | 2.40 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops.

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops |
|---|---|---|---|---|---|---|
| C4 | 6.7 / 8.7 | 463 | 0 | 7 | 20.2 | 0 |
| C4_depth1 | 6.6 / 8.7 | 483 | 0 | 9 | 21.4 | 0 |

