---
paths:
  - "blueboat_replanner/blueboat_replanner/eval/**"
  - "blueboat_replanner/blueboat_replanner/core/model.py"
  - "blueboat_replanner/blueboat_replanner/core/value.py"
---
# World, Belief and planner models stay apart (brief §8.1)

- **World:** `eval/world.py` only (true targets, continuous orientation, detection draws,
  calibrated confidences). It is the only place with randomness: a seeded
  `numpy.random.Generator` from a `SeedSequence` keyed by (seed, object, look).
- **Belief:** `eval/belief_reference.py` is a STAND-IN written from the briefs, NOT
  BlueBoat-Belief. Label every result from it "reference-belief-conditional, preliminary".
- **Planner:** `core/model.py` plus configuration. Ablations change only the planner block
  (C4_iso, C4_degraded, C4_depth1), never the world.
- Never tune a planner parameter to make C4 win. C2 settings are frozen first
  (`docs/C2_TUNING_PROTOCOL.md`); the runner refuses C3/C4 without the freeze.
- Dev-loop results are development evidence only, never E4 (synthesis §6.2 ⚠).
