# TODO — BlueBoat-Replanner

Open items only. Settled design: `docs/DESIGN.md`, `docs/MATH.md`, `docs/PHASE1_CLOSURE.md`.
Local session commands: `docs/LOCAL_CHECKLIST.md`.

## [P0] blocking a thesis result
- [ ] **Belief binding** (L-B1…L-B7): find the topic, QoS, datum and transform form; diff
  `CONTRACT_PROVIDED.md` against `CONTRACT_CONSUMED.md`; write `eval/belief_real.py`; re-run gate
  [20] against the real core; re-run the dev loop with the real core.
- [ ] **Belief's evidence topic** (Q-9, RQ-3): `image_records` or `detections`. The emulator
  implements `detections` only.
- [ ] **Gazebo missions** L-G1/L-G2 (C2 and C4 with mid-run updates), first through the
  composition, then through `full_mission_launch` once R-3 lands (RQ-1).
- [ ] **0.5 m/s achieved in Gazebo** without τ throttling (L-K3), plus the turn tracking of 3 m
  arcs (L-K4).
- [ ] **Real-water authority** (Q-13 / O-7): a supervised dry run after Control's own on-water
  path-update test.
- [ ] **First build and first ROS run** (L-R1…L-R6): `package.xml`, `setup.py`, both nodes and
  both launch files have only run through the stand-ins of gate [24].

## [P1] model and protocol
- [ ] Measure t_lat (L-B5) and Belief's detection window and model (L-B6). Then revisit the
  calibration interpretation (MATH §9.1).
- [ ] Clutter re-detection f (O-5): a value from data, or keep the sweep.
- [ ] The E4 C2 freeze in Gazebo, on tuning worlds disjoint from evaluation worlds
  (`C2_TUNING_PROTOCOL.md` §6). The dev-loop freeze is preliminary.
- [ ] A Tier-2 world detection model fitted from Gazebo (synthesis §6.2 ⚠) before any reported
  sweep. Owner: D7.
- [ ] Measure the turn radius and settle time (L-K1, L-K2), then freeze them for every condition.
- [ ] Refit the emulator's confidence calibrator (Platt a, b) on Gazebo C2 runs of the
  calibration worlds; the dev-loop values (a = 2.136, b = 0.478) belong to the dev-loop world.
- [ ] Sensitivity sweeps of κ, λ, ε_J and the rule thresholds (C2_TUNING_PROTOCOL §7): not run.
  Report them as sensitivity, never as a choice made by looking at C4 − C2 on `S_eval`.

## [P2] scope and refinements
- [ ] Multiple areas per mission (O-8).
- [ ] RECOVER candidates and mid-line interruption (C4_live): today only a flag; not run.
- [ ] The emulator's `image_records` output (if Belief reads records).
- [ ] Paull-style partial angle dependence as a planner option (MATH §16; folded step today).
- [ ] Root CLAUDE.md rows and CM-9 wording (`docs/INTEGRATION.md`): the user applies them.
