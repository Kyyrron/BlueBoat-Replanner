# Dev-loop results — distinct_sectors

**real-Belief dev loop, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Belief = BlueBoat-Belief's own core (revision `093841a`, uncommitted changes: True, core sha256 `ac686bf69cf07f6a…`), through its offline interface. Misses credited: `records`. Class reported by the emulated detector: `none`. Belief settings changed for this run: `{'contact': {'confirm_distinct_sectors': True}}`. Belief's PD, rho and the planner's aspect windows are assumed; the detector is emulated; the world's detection model is the planner's placeholders (not fitted from Gazebo); tracking is perfect.

Seeds: 2000-2029 (n = 30), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.135, b = 0.519, n = 355).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C2 | 30 | 2.03 ± 1.73 | 3.20 ± 1.32 | 1.17 ± 0.87 | 4.70 | — | — | — | — | 0 | 0.00/0.00 | 0.045 |
| C3 | 30 | 1.80 ± 1.81 | 3.40 ± 1.25 | 1.60 ± 1.13 | 4.90 | -0.23 ± 1.07 | [-0.63, +0.16] | 6/13/11 | 12 / 6 | 436 | 0.05/0.18 | 0.040 |
| C4 | 30 | 2.23 ± 1.10 | 3.87 ± 0.97 | 1.63 ± 0.85 | 4.97 | +0.20 ± 1.58 | [-0.38, +0.78] | 11/11/8 | 29 / 9 | 871 | 0.97/3.96 | 0.040 |
| C4_iso | 30 | 2.10 ± 1.63 | 3.87 ± 1.25 | 1.77 ± 0.90 | 4.90 | +0.07 ± 1.55 | [-0.50, +0.63] | 12/7/11 | 25 / 5 | 754 | 1.10/4.95 | 0.044 |
| C4_rand | 30 | 1.93 ± 1.14 | 3.83 ± 1.15 | 1.90 ± 0.76 | 5.07 | -0.10 ± 1.75 | [-0.74, +0.54] | 10/7/13 | 30 / 11 | 881 | 0.98/4.25 | 0.039 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C2 | 0.37 | 0.97 | 1.40 | 2.03 |
| C3 | 0.37 | 0.97 | 1.83 | 1.80 |
| C4 | 0.63 | 1.83 | 2.20 | 2.23 |
| C4_iso | 0.77 | 1.77 | 2.20 | 2.10 |
| C4_rand | 0.40 | 1.60 | 2.03 | 1.93 |

Belief side (published output and inputs only): the grid's lag behind the boat at the decisions; looks the emulator tracked at the objects and how many differ from Belief's `aspect_counts` there; draws on one object less than a look time-out apart (two sectors in one sweep); mission stops; inputs Belief reports as lost (its counters `coverage_expired`, `stale_detection`, `future_detection`, `no_pose_at_stamp`; — = not recorded in these rows).

| Condition | lag s (mean / max) | looks tracked | differing | close draws | detections per mission | mission stops | inputs lost |
|---|---|---|---|---|---|---|---|
| C2 | — | 2459 | 0 | 22 | 17.2 | 0 | 0 |
| C3 | 6.8 / 6.8 | 2698 | 0 | 41 | 18.7 | 0 | 0 |
| C4 | 6.7 / 8.7 | 2866 | 0 | 43 | 20.9 | 0 | 0 |
| C4_iso | 6.8 / 8.7 | 2781 | 0 | 44 | 20.2 | 0 | 0 |
| C4_rand | 6.7 / 8.7 | 2907 | 0 | 62 | 21.0 | 0 | 0 |

