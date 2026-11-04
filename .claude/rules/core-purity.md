---
paths:
  - "blueboat_replanner/blueboat_replanner/core/**"
  - "blueboat_replanner/blueboat_replanner/io/**"
  - "blueboat_replanner/blueboat_replanner/ros/adapters.py"
  - "blueboat_replanner/blueboat_replanner/ros/executor.py"
---
# Core purity (DESIGN §3; gate section [14])

- `core/` is ROS-free and deterministic: numpy + stdlib only. No `rclpy`, Qt, OpenCV, torch,
  `random`, `time`, `datetime` or `numpy.random`. PyYAML only in `io/`.
- `Policy.on_epoch` is pure. Clock and randomness never enter a decision.
- `ros/executor.py` takes the clock as an argument (`now`). It is the ONLY code that knows
  Control's path update (`/blueboat/update_path`, refusal codes, progress fields).
- No online code names a detection, sonar or simulation topic (`/sss_ai`, `/side_scan_sonar`,
  `/sss_processor`, `ground_truth`, `/sim/`): CM-9.
- A mathematical ambiguity found while coding goes to `docs/MATH.md` FIRST (brief §18).
