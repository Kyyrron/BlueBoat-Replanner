# REAL_BELIEF_BINDING — how the Replanner reads and drives BlueBoat-Belief

Bound on 2026-11-12 to Belief revision `093841a` with uncommitted changes. Because Belief's
working tree is uncommitted, it is identified by a hash of its core sources, recomputed by
`eval/belief_real.belief_info()` and written into every dev-loop manifest.

| When | Belief core sha256 | Belief's own gate | Used for |
|---|---|---|---|
| 2026-11-12, until 20:19 | `fcd9531597e593b3…` | 240 checks, 0 skipped | the binding, the gate, every dev-loop result (19:27–20:12). **It is also the core of the installed package in `~/ros2_ws/install`** (built at 15:34), so of the Belief node the ROS check ran beside (`REPORT.md` §F) |
| 2026-11-12, from 20:19 | `ac686bf69cf07f6a…` | 241 checks, 0 skipped | the gate (262 passed at the end of the day); 40 missions of the main run re-run on it are identical to their stored rows; the extra comparison of `DEVLOOP.md` §5.9 |

**Which copy the dev loop loads:** the checkout beside this repository, always first; the
installed copy only when there is no checkout (an installed copy is older than the checkout
whenever Belief was edited since its last build). The hash and the location written into every
manifest are those of the copy that was loaded; every stored result was made on the checkout.

Belief's session edited three core files at 20:19 (`alignment.py`, `config.py`, `contract.py`):
comments and document references, as read. The 40 identical missions are the evidence that the
behaviour is the same. If the hash changes again: run the gate, then re-run a sample the same
way, and the whole dev loop if a mission differs.

Levels of evidence used below: **verified in code** (read in Belief's or this module's source),
**tested offline** (this module's gate), **real-Belief dev loop** (results on Belief's core,
`DEVLOOP.md` §5), **assumed**, **not run**.

## 1. Two separate things

| | Online (the node) | Dev loop (`eval/`) |
|---|---|---|
| What is read | the `BeliefGrid` message on `/belief_map/belief_grid` | the same fields, from Belief's `to_grid_dict` |
| Belief code imported | none | Belief's core, through `eval/belief_real.py` only |
| Status | gated through fake ROS modules; **run under ROS on 2026-11-12 on an isolated graph beside Belief's installed node, with made-up inputs** (`REPORT.md` §F). Never in Gazebo, never with a real detector | **tested offline** on Belief's core |

Both go through the same adapter (`ros/adapters.grid_to_dict`), the same validator
(`core/evidence.validate_grid`) and the same planner code.

## 2. Reading the grid by meaning (verified in code, tested offline)

The field-by-field table is `CONTRACT_CONSUMED.md`. The changes, with where they are specified:

| # | What Belief means | What the Replanner does now | MATH | Gate |
|---|---|---|---|---|
| 1 | a sector was searched iff `aspect_log_miss < 0`; `aspect_counts` also counts looks with no detector coverage | searched folded sectors S_i from `aspect_log_miss`; `aspect_counts` only when that field is absent, recorded | §9 | [11], [16], [20] |
| 2 | a contact's miss is booked when its look closes (up to 20 s later); the cell record has it at once, at its strength, and has the looks made before the contact existed | a contact's misses = the sectors searched at its cell (and `sectors_missed`), minus the detected ones, at the recorded strength | §7 | [17], [20] |
| 3 | the header stamp is the evidence horizon | pending looks = the mirror path after the horizon; online the stamp is converted through Control's progress clock, minus the tracking margin; five named cases | §11.3 | [20], [23] |
| 4 | `grid_seq` restarts at 1 with Belief | a decrease is a restart: accepted, counted, logged, per-contact memory reset, later records labelled | §11.3 | [23] |
| 5 | scores are raw while `detector_calibrated` is false | every decision record carries `score-conditional`; the flag and the other model keys are recorded | §7 | [22], [23] |
| 6 | `class_names` is Belief's detector class list | classes are mapped to aspect families through `class_families` (empty); unmapped ones are isotropic and recorded | §6 | [1], [17] |
| 7 | Belief has its own survey area | the mission polygon must lie inside it, else the mission stops before the first path | DESIGN §4 | [11], [20], [23] |
| 8 | samples sit at their offset from the centre line (lever arm 0.285 m); the inner side of a turn beyond r* counts for nothing; a bearing on a sector edge goes to the upper sector | the look predictor does the same | §1, §5 | [11], [20] |
| 9 | Belief's PD is a trapezoid with 3 m ramps | kept as a stated difference: the planner's box is its own model; misses in the ramps arrive at their strength | §6 | [20] |

**Predictor against Belief's core** (gate [20], on the 120 × 60 m test area, depth 2 m):

| Path | Predicted pairs | Searched in Belief | Belief's plateau pairs predicted |
|---|---|---|---|
| one coverage line | 8463 | 100.00 % | 100.00 % (Belief has 2604 more pairs, all in its range ramps) |
| line, turn, line, turn, line | 21 064 | 99.93 % | — |
| the same without the turn gate | 22 688 | 92.89 % | — |

On the path with turns Belief holds 6.7 % more pairs than predicted: the outer side of the
turns, where the predictor's poses are too sparse (conservative, MATH §5), and the range ramps.

## 3. Driving Belief's core offline (`eval/belief_real.py`)

- **Interface.** Only the names of Belief's `docs/OFFLINE_CORE_API.md`: `Belief`,
  `BeliefConfig`, `PingObs`, `Det`, `DetArray`, `ImageRec`, `make_area_dict`, `area_from_dict`,
  `processor_side_offsets`, `to_grid_dict`. Results are read only through `to_grid_dict`.
  `finish()` is called once, at the end of a mission.
- **Import.** From the checkout beside this repository (`BLUEBOAT_BELIEF_DIR` overrides), or an
  installed package. Bytecode writing is switched off first. If the import fails,
  `BeliefUnavailable` says why; nothing else is used in its place.
- **Time.** Belief's times must be positive: Belief time = 1000 s + path time.
- **Frames.** `alignment.origin_lat/lon` = the datum with `override_sigma_m` 0: the dev loop's
  poses are the map frame.
- **Pings.** One `PingObs` every 0.1 s (0.05 m), both sides, with the processor's own sample
  offsets (`processor_side_offsets`), `water_depth` = altitude + 0.22 m.
- **Detections.** `Det.t_row` is the time of a fed ping; the covariance is σ_det² + (1 m)², as
  Perception's (Belief removes the 1 m as common-mode); ids are unique per mission.
- **How a miss is credited** (in every manifest):
  - `records`: one `DetArray` (also when empty) and then one `ImageRec` per emulated seabed
    image (stride 2.13 m, span 4.27 m of track). Belief credits pings as it does live; the grid
    lags by one to two strides; a last short image is reported before `finish()`.
  - `ungated`: Belief's development mode, every ping credited at once.
- **Settings.** Belief's defaults, except: the alignment override; `detector.class_names` = the
  five planner families (no `other`); `detector.calibrated` / `base_rate` from the emulator's
  calibration; the coverage mode. `belief_config_changed` in each result lists them.
- **Run health.** Belief's counters and statistics (`Belief.counters`, `Belief.stats`,
  `Belief.health()`), in its documented interface since the evening of 2026-11-12
  (`REQUESTS.md` RQ-5, answered). Every mission reports them; the gate checks that no input was
  lost (`coverage_expired`, `stale_detection`, `future_detection`, `no_pose_at_stamp` all 0),
  that every ping was applied and covered, and that unclassified detections are counted as such.

## 4. Raw scores

- Belief publishes `first_confidence` as the creating detection's score and says in
  `model_params_json.detector_calibrated` whether it is a probability. Today, with Perception,
  it is not.
- The planner still uses it as the prior P(real) of a contact (MATH §7): there is nothing else.
  It reads the flag and labels every decision record `score-conditional` while it is not true.
- In the dev loop the emulated scores are calibrated against the dev-loop world (Platt scaling
  on the calibration seeds), so the binding sets `detector.calibrated: true` and the base rate
  for the evaluation runs, and leaves them false for the tuning and calibration runs.
- Belief's `p_exists` is used only as the fallback when `first_confidence` is absent.

## 5. The emulated detector (`eval/emulator.py`)

- **One draw per look at a world object.** The look is defined as Belief's contract defines it
  (Belief `topics.md` §4): hits of the object's cell from one directed sector with gaps ≤ 20 s,
  after the turn gate. It is the world side's own bookkeeping from the ping poses; it reads
  nothing of Belief's state. Its turn estimate follows the contract text and the windowing of
  Belief's `core/turn.py` as read on 2026-11-12, written anew here.
- **Check, published output only:** after every mission the looks counted at each object are
  compared with Belief's `aspect_counts` at that cell. In every gate mission the difference is 0.
- **Why the unit matters.** A coarser unit than Belief's look leaves Belief with looks that had
  no draw: they close as misses the world never tested. A finer one puts two draws into one
  Belief look and hides world misses.
- **Known artefact, counted:** two neighbouring sectors swept in one turn are two looks, each
  with its own draw (`draws_within_look_timeout`).
- **Class information:** `none` → `unclassified`, uniform class probabilities in Belief; `true`
  → the target's class, one fixed class per clutter object (`DEVLOOP.md` §3).
- **The ROS node** (`eval/emulator_node.py`) uses the same tracker and image cutter and
  publishes one array per image, empty ones included. Run on the isolated ROS graph beside
  Belief's node (`LOCAL_CHECKLIST.md` L-X [R6]–[R7]); not yet in a built world (L-E).

## 6. Commands

```bash
cd blueboat_replanner
/usr/bin/python3 -B -m test.smoke_test                 # SMOKE: 278 passed, 0 failed, 0 skipped (2026-11-13)
~/ros2_ws/.venv/bin/python3 -B -m test.smoke_test      # SMOKE: 278 passed, 0 failed, 0 skipped
cd .. && .claude/tools/devloop_batches.sh devloop_real_2026-11-12 4      # 19:27–20:12, exit 0
.claude/tools/ros_check.sh                                               # 21:27–21:52 (by hand), 42 checks, 0 failed
cd blueboat_replanner && /usr/bin/python3 -B -m blueboat_replanner.eval.compare \
    ../results/devloop_real_2026-11-12/main --pairs C4:C4_rand,C4:C4_iso,C4_depth1:C4
```

- Gate: about 60 s on one core. Belief, Control and MCS working trees are unchanged by it
  (`git status` identical; no bytecode written).
- Results of the dev loop on Belief's core: `DEVLOOP.md` §5 (the main copy) and their weight
  in `REPORT.md` §C. The ROS check: `LOCAL_CHECKLIST.md` L-X.

## 7. What remains

1. The owner's decisions (`HANDOVER.md` §4).
2. In Gazebo: the two clocks of the evidence horizon (Control's `ros_time` and the sonar
   stamps must be one sim-time clock), Belief beside a real sonar stream, the emulator in a
   built world. (The QoS match, a Belief restart and the emulator beside Belief's node were seen
   on the isolated graph, L-X.)
3. The class → aspect family map (`REQUESTS.md` RQ-6); a trained, calibrated detector.
