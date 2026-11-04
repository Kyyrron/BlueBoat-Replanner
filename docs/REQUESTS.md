# Requests to other modules (for the user to route)

This module edits nothing outside `BlueBoat-Replanner/`. Each item says what is needed, why, and
what it unblocks.

## RQ-1 — BlueBoat-SSS-Sim: `path_source` in `full_mission_launch.py` (= Control R-3)

- **Need:** a launch argument `path_source` (default `mission`), passed as a parameter to the
  `master_control` node that `full_mission_launch.py` starts (its l.305–323).
- **Why:** built-world simulations cannot run replanner-owned runs today (E-20). E4 runs in
  built worlds.
- **Until then:** for development only, compose `Sim_launch.py path_source:=replanner` with
  `sss_sim_launch.py world_dir:=<world>` (Q-12 b). Gazebo then has no walls and no limits.
- **Unblocks:** the Phase 4 C2/C4 Gazebo missions in a built world (L-G1).

## RQ-2 — BlueBoat-Belief session: mission file, datum and contract fields

1. **Mission file.** The Replanner reads a mission YAML (brief §6.4:
   `mission: {name, condition, areas, config}`, with WGS-84 `areas`).
   - Synthesis §5.1 says Belief takes the survey area and anchor "from the mission". The request
     is that Belief read the **same file** (the same `areas`), so the two modules cannot
     disagree on the area.
   - If Belief takes its datum from this file, a `geo_anchor: {lat0, lon0}` key has to be added
     to the schema by agreement. The Replanner does not need it: it uses the published datum.
2. **`first_confidence`, `first_sector`.** Please provide both (contract C-4). Without them, the
   Replanner degrades to `p_exists` and to averaging over the first sector, and its contact value
   is marked [FALLBACK].
3. **Topic, QoS, datum and transform form.** Please confirm, and fill `CONTRACT_PROVIDED.md`.
4. **Detection model and latency** (Phase 1 closure A, C, F). Please state:
   - Belief's per-look PD(range) window and plateau;
   - whether it depends on class;
   - its ρ;
   - the typical delay from a look to its appearance in the grid.

   The Replanner calibrates its background against `coverage` as a class-mixture residual and
   predicts looks on [3, 25] m. Both choices must be reconciled with Belief's actual model
   (`LOCAL_CHECKLIST.md` L-B5, L-B6).

## RQ-3 — Belief or user: which evidence topic Belief consumes (Q-9)

- **The conflict:** `/sss_ai/image_records` (root CLAUDE.md, CM-9, synthesis §5.1) versus
  `/sss_ai/detections` (contract C-1).
- **What depends on it:** the dev-only emulator publishes exactly that topic and type, in place
  of Perception.

## RQ-4 — Root `.claude/CLAUDE.md` (Phase 5, for the user to apply)

- Interface-table rows for `/replanner/*` and this module's use of `/blueboat/update_path`.
- An amended CM-9 / synthesis §5.1 ⚠ wording to match the brief's input list (user decision
  2026-11-04: "the brief overrides anything else"). The proposed lines will be written in Phase 5.
