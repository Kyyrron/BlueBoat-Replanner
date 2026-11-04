# BlueBoat-Replanner

The aspect-aware replanner for the BlueBoat side-scan survey (thesis deliverable D6). It reads the
belief map as its only target evidence and changes the path being followed only through Control's
live path update (`/blueboat/update_path`). All survey conditions go through one coverage
generator and one executor:
- C1, single pass;
- C2, two-pass orthogonal (the baseline);
- C3, append-on-completion;
- C4, informative, with its ablations.

## Status (2026-11-04, cloud session REPLAN-1)

- **Done:** the ROS-free core, the dev loop and its results, the ROS node, the emulator node and
  the launch files.
- **Gate:** `cd blueboat_replanner && python3 -m test.smoke_test` gives **188 passed, 0 failed,
  1 skipped**. The skip is the check against the real Belief core.
- **Not run under ROS:**
  - no `colcon build`;
  - no live node;
  - no Gazebo mission.

  The node and the launch files have only run through the stand-in modules of gate [24].
- **Dev-loop numbers** come from a stand-in belief. They are reference-belief-conditional and
  preliminary, never E4.

## Start here

| Document | For |
|---|---|
| `docs/REPORT.md` | what was built, and how much weight each claim carries |
| `docs/HANDOVER_CLOUD.md` | the state per phase and the open user decisions |
| `docs/LOCAL_CHECKLIST.md` | **the in-machine ROS 2 session:** every check, with its command and expected result, in order |
| `docs/DESIGN.md`, `docs/MATH.md`, `docs/topics.md` | design, mathematics, interfaces |
| `docs/DEVLOOP.md`, `results/devloop_2026-11-04/` | dev-loop method and results |
| `.claude/CLAUDE.md` | rules for working in this module |

## First steps on the machine

Clone this repository as `BlueBoat-Replanner/` beside the other modules, in
`~/ros2_ws/src/BlueBoat-SideScanSonar/`. Then:

```bash
cd ~/ros2_ws && colcon build --packages-select blueboat_interfaces blueboat_replanner
```
```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Replanner/blueboat_replanner && /usr/bin/python3 -m test.smoke_test
```

Then continue with `docs/LOCAL_CHECKLIST.md` (L-0, L-R, L-C, L-B, …).
