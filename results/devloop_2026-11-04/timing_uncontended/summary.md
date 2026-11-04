# Dev-loop results — timing_uncontended

**reference-belief-conditional, preliminary.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).
Reference belief = a stand-in written from the briefs, NOT BlueBoat-Belief. World detection model = placeholders matched to the planner (not fitted from Gazebo).

Seeds: 2000-2004 (n = 5), paired across conditions. Calibration: Platt on C2 detections, seeds 900-919 (a = 2.136, b = 0.478, n = 371).

Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.

## Budget fraction 1 × B(C2)

| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 (mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | decision s (mean/max) | ECE |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| C4 | 5 | 2.00 ± 1.22 | 3.40 ± 0.55 | 1.40 ± 0.89 | 4.60 | — | — | — | — | 882 | 1.54/5.60 | 0.059 |
| C4_depth1 | 5 | 2.40 ± 1.14 | 4.00 ± 0.71 | 1.60 ± 1.34 | 4.60 | — | — | — | — | 864 | 0.53/1.51 | 0.071 |

Anytime (mean score at 25/50/75/100 % of this budget):

| Condition | 25 % | 50 % | 75 % | 100 % |
|---|---|---|---|---|
| C4 | 0.80 | 1.80 | 2.00 | 2.00 |
| C4_depth1 | 0.60 | 2.20 | 2.40 | 2.40 |

