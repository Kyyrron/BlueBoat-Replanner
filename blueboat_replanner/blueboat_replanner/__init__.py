"""BlueBoat-Replanner (thesis deliverable D6).

Layers (DESIGN.md §3):
  core/  ROS-free, deterministic: numpy + stdlib only.
  io/    files (YAML, JSON); the only place PyYAML is imported.
  ros/   duck-typed adapters, the executor (the only code that knows Control's path
         update) and the thin rclpy node (the only rclpy code).
  eval/  development loop on BlueBoat-Belief's core, emulator; the only place with randomness.
"""

__version__ = "0.1.0"
