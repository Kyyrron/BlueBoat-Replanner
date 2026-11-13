# TODO — BlueBoat-Replanner

Open items only. Where the work stands and the owner's decisions: `docs/HANDOVER.md`. ROS,
Gazebo and boat checks with their commands: `docs/LOCAL_CHECKLIST.md`.

## [P0] the owner's
- [ ] **Commit** the working tree (the 2026-11-12 and 2026-11-13 work is uncommitted; HEAD
  `c53fe2a` still holds the removed imitation of Belief), then the superproject gitlink.
- [ ] **Go-ahead for the ROS check on the new node code** (`.claude/tools/ros_check.sh`, isolated
  domain, made-up inputs, no Gazebo, about 28 min): the node's changes of 2026-11-13 (C2 freeze
  applied, pre-send area and junction checks, readiness, answer arrival time) have run offline
  only (`REPORT.md` §F.1).
- [ ] **The open decisions** of `docs/HANDOVER.md` §4, in particular the class → aspect family
  map (RQ-6): until it exists, the bearing choice is inert for every detector class.

## [P0] blocking a thesis result
- [ ] **Control's real path update, live** (L-C1…L-C4). Go-ahead needed.
- [ ] **Gazebo missions** L-G1…L-G4 (C2, C4, stop test), through the composition until R-3
  lands (RQ-1), then L-G5. They give the decision time under load (decision 1) from
  `end.json` and the decision records. Settle the clock first (L-G intro: no `use_sim_time`
  in `replanner_launch.py`; the node uses wall time).
- [ ] **0.5 m/s achieved in Gazebo** without τ throttling (L-K3), and 3 m arcs tracked (L-K4).
- [ ] **Real-water authority** (DESIGN §12 O-7): a supervised dry run after Control's own
  on-water path-update test.
- Not this module's: a trained, calibrated detector (Perception); Control R-3; MCS R-1; the
  Evaluation module.

## [P1] model and protocol
- [ ] The clocks under ROS: Control's `ros_time` and the sonar stamps must be one clock in
  simulation for the evidence horizon (MATH §11.3); the record names the fallback otherwise.
- [ ] The node decides inside its timer callback: while it computes, no progress, grid or
  odometry message is handled. The time is counted against the lead (DESIGN §5.2 rule 2), so
  the cost is a lost decision, never a late request. If decisions are lost in Gazebo: the
  one-step lookahead (decision 1) or a second thread (a design change, to be specified first).
- [ ] The trigger fires on the first tick after its threshold (tick 5 Hz, progress 10 Hz), so up
  to about 0.3 s of the 2.0 s allowance is used before the policy starts. Stated; not changed.
- [ ] The boat's real lag behind the path parameter (`tracking_lag_m` = 3 m, Control's governor
  bound): measure it with L-K3 and tighten it.
- [ ] Clutter re-detection f (decision 5): a value from data, or keep the sweep.
- [ ] The E4 C2 freeze in Gazebo, on tuning worlds disjoint from evaluation worlds
  (`C2_TUNING_PROTOCOL.md` §6). The dev-loop freeze is preliminary.
- [ ] A world detection model fitted from Gazebo (synthesis §6.2 ⚠) before any reported sweep.
- [ ] Measure the turn radius and settle time (L-K1, L-K2), then freeze them for every condition.
- [ ] Refit the emulator's confidence calibrator on Gazebo C2 runs of the calibration worlds.
- [ ] Sensitivity sweeps of κ, λ, ε_J, ρ_floor and the rule thresholds (C2_TUNING_PROTOCOL §7):
  not run. Report them as sensitivity, never as a choice made by looking at C4 − C2.

## [P1] code quality
- [ ] A `use_sim_time` argument in both launch files, once L-G settles the clock question.

## [P2] scope and refinements
- [ ] Multiple areas per mission (decision 6).
- [ ] RECOVER candidates and mid-line interruption (C4_live): only a flag that nothing reads.
- [ ] Paull-style partial angle dependence as a planner option (MATH §16; folded step today).
- [ ] Denser sampling of arcs in the look predictor (MATH §5: conservative today).
- [ ] The emulator node's image records (Belief works without them).
- [ ] The emulator node in a built world, in Gazebo (L-E); compare its looks with Belief's
  `aspect_counts` there, as the dev loop does offline.
- [ ] Belief does not publish its confirmation rule in `model_params_json` (RQ-8).
