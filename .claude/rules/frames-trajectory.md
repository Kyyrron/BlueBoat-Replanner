---
paths:
  - "blueboat_replanner/blueboat_replanner/core/plan.py"
  - "blueboat_replanner/blueboat_replanner/core/coverage.py"
  - "blueboat_replanner/blueboat_replanner/core/routing.py"
  - "blueboat_replanner/blueboat_replanner/core/geo.py"
  - "blueboat_replanner/blueboat_replanner/core/validate.py"
---
# Frames and the trajectory contract (DESIGN §4; E-08, E-30)

- Plans are made in Belief's map frame M. Requests go out in Control's `world` frame W through
  `odom_to_map` (x_M = R(yaw) x_W + t). Never assume the rotation is zero.
- Junctions come from the mirror (W). The executor re-expresses its plan when the transform
  changes (`transform_segments`).
- `geo.py`'s two functions are verbatim MCS copies; the gate checks bit-identity. Never edit
  them.
- Requests follow Control's rules: t strictly increasing in >= 1 ms steps, t >= 0, speed <= 2 m/s,
  yaw = tangent (45 deg coherence), <= 20 000 samples. The change point must be >= the progress
  `min_change_point` + margin, and continuity at c is 0.20 m / 10 deg.
- No stationary corners: turns are Dubins arcs of radius `r_turn_m`. No in-place turn is ever
  assumed (MATH §3.5).
- Every sample lies inside F_margin of the single mission area.
