# Requests to other modules and to the owner

This module edits nothing outside `BlueBoat-Replanner/` (CM-3). Each open item says what is
needed, why, and what it unblocks; the owner routes it.

## Open

### RQ-1 — BlueBoat-SSS-Sim: `path_source` in `full_mission_launch.py` (= Control R-3)

- **Need:** a launch argument `path_source` (default `mission`), passed as a parameter to the
  `master_control` node that `full_mission_launch.py` starts.
- **Why:** built-world simulations cannot run replanner-owned runs today (E-20), and E4 runs in
  built worlds.
- **Until then:** for development only, compose `Sim_launch.py path_source:=replanner` with
  `sss_sim_launch.py world_dir:=<world>` (`LOCAL_CHECKLIST.md` L-G). Gazebo then has no walls
  and no limits.
- **Unblocks:** C2/C4 Gazebo missions in a built world (L-G5) and every E4 run.

### RQ-6 — owner: the class → aspect family map

- **Need:** the content of `class_families` in `config/replanner.yaml`: for each class name a
  detector will report (the 29 World Builder types once a model is trained on them), the aspect
  family it belongs to (block, pipe, cage, tyre, plate) or none. Perception's `yolo.py` holds no
  class list: a detector's class names come from its weights.
- **Why:** with no entry a class is isotropic for the planner, and the aspect-aware choice is
  inert for it (C4 = C4_iso by construction). Belief also has to list those classes in its
  `detector.class_names`, or they all arrive as `other`.
- **Existing assumption, for the emulator's world only** (`eval/emulator_node.py` `CLASS_MAP`,
  DESIGN decision 17): `concrete_block` → block, `pipe_pvc` → pipe, `mesh_crate` → cage,
  `tire_car` → tyre. It is not applied to the planner.

### RQ-8 — BlueBoat-Belief: the confirmation rule in `model_params_json` (low priority)

- **Need:** `contact.confirm_looks` and `contact.confirm_distinct_sectors` among the keys of
  `model_params_json`.
- **Why:** the planner values a look by what it does to a contact. It assumes Belief's default:
  two detected looks confirm, whatever their sectors. With `confirm_distinct_sectors: true` a
  second detection from the same directed sector no longer confirms, and the planner cannot
  see that from the grid.
- **Evidence:** a dev-loop run with the switch on (`DEVLOOP.md` §5.9, 150 missions): the rule
  changes few outcomes, because a second look almost always comes from another directed sector.
- **Until then:** nothing is blocked. The switch is off by default, and this module never sets
  it outside that one labelled run.

## Answered (kept for their numbers)

| Id | Asked of | Answer |
|---|---|---|
| RQ-2 | BlueBoat-Belief: mission file, datum, contract fields, detection model | Belief, 2026-11-12 (`topics.md` §1–§2, §10–§11; `MATH.md`): Belief's area is made from the mission file (`blueboat-belief-make-area --from-mission`), the datum is the area's anchor; `first_confidence` and `first_sector` are published (raw while uncalibrated); PD is a trapezoid with no class dependence, ρ is Paull's pairwise rule, the header stamp is the evidence horizon |
| RQ-3 | Belief / owner: which evidence topic Belief reads | `/sss_ai/detections` (required), `/sss_ai/image_records` (optional). The emulator publishes detections, one array per image |
| RQ-4, RQ-7 | owner: the root `.claude/CLAUDE.md` and `project_synthesis.md` lines about this module | applied on 2026-11-13 by the owner's order (root §1, the modules table, CM-9, `interface-contract.md` §4.1, the skills) |
| RQ-5 | BlueBoat-Belief: counters and statistics in the offline interface | Belief, 2026-11-12: `Belief.counters`, `stats`, `summary()`, `health()` are in its `OFFLINE_CORE_API.md`; the dev loop reads them (`eval/belief_real.RealBelief.run_health`) and the gate checks that no input was lost |
