---
paths:
  - "blueboat_replanner/blueboat_replanner/eval/**"
  - "blueboat_replanner/blueboat_replanner/core/model.py"
  - "blueboat_replanner/blueboat_replanner/core/value.py"
---
# World, Belief and planner models stay apart (brief §8.1)

- **World:** `eval/world.py` (true targets, continuous orientation, detection draws,
  calibrated confidences) with `eval/emulator.py` (looks at objects, seabed images). All
  randomness is in `eval/` (these two and the C4_rand table of `eval/devloop.py`): seeded
  `numpy.random.Generator`s from `SeedSequence`s keyed by (seed, purpose, object, look).
- **Belief:** BlueBoat-Belief's own core, through `eval/belief_real.py` and nothing but the
  names of Belief's `docs/OFFLINE_CORE_API.md`. Never copy Belief code, never read its objects
  (only `to_grid_dict`), never write into its tree, never call `finish()` mid-mission. If it
  cannot be imported the dev loop stops with the reason: there is no other belief here.
  Belief's settings stay Belief's defaults except what the binding needs, and each change is
  written into the manifest. Label every result "real-Belief dev loop, preliminary".
- **Planner:** `core/model.py` plus configuration. Ablations change only the planner block
  (C4_iso, C4_degraded, C4_depth1), never the world.
- Never tune a planner parameter to make C4 win. C2 settings are frozen first
  (`docs/C2_TUNING_PROTOCOL.md`); the runner refuses C3/C4 without the freeze.
- Dev-loop results are development evidence only, never E4 (synthesis §6.2 ⚠).
