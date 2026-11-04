# CLAUDE.md — BlueBoat-Replanner (thesis deliverable D6)

Aspect-aware replanner for the BlueBoat side-scan survey. At decision points during a run it
continues the survey or revisits a contact from a chosen bearing, and it makes the boat fly that
within the same run through Control's live path update. It also generates every survey condition
through **one** coverage generator and **one** executor:
- C1 single pass;
- C2 two-pass orthogonal (the baseline);
- C3 append-on-completion;
- C4 informative, with its ablations.

The brief is `docs/REPLANNER_BRIEF.md` (read-only). The contract with Belief is `belief_grid/0`
(prompt §B; `docs/CONTRACT_CONSUMED.md`).

## 1. Authority and documents
- Project rules: root `.claude/CLAUDE.md` and `project_synthesis.md` (there is no context pack).
- Design `docs/DESIGN.md`; mathematics `docs/MATH.md`; interfaces `docs/topics.md`.
- Phase records: `PHASE0_VERIFICATION.md`, `PHASE1_CLOSURE.md`.
- Evidence ids: `docs/EVIDENCE_REGISTER.md` (E-nn, L-nn).
- Open questions: `docs/QUESTIONS_PHASE0.md` (answers at the top) and `.claude/TODO.md`.
- Local work: `docs/LOCAL_CHECKLIST.md`; handover `docs/HANDOVER_CLOUD.md`.
- Where a document and the code disagree, the code wins: fix the document.

## 2. Layout (ament_python, one level below the git root)
`blueboat_replanner/`:
- `blueboat_replanner/core/`: ROS-free, deterministic.
- `io/`: YAML and records.
- `ros/`:
  - `adapters.py`, the one field-name table;
  - `executor.py`, the ONLY code that knows `/blueboat/update_path`;
  - `replanner_node.py`, the only rclpy code.
- `eval/`: dev loop, reference belief, world/emulator, Control stub, runner, emulator node; the
  only randomness.
- Also `config/`, `launch/`, and `test/smoke_test.py`.

Full tree: DESIGN §3.

## 3. Interfaces (details: `docs/topics.md`)
| Name | Dir | Type / QoS |
|---|---|---|
| `/belief_map/belief_grid` (provisional) | in | `blueboat_belief_interfaces/BeliefGrid`, RELIABLE TRANSIENT_LOCAL depth 1 — the ONLY target evidence |
| `/blueboat/path_progress`, `/blueboat/path_update_event` | in | String JSON (Control `blueboat_path_progress/1`, `…_event/1`) |
| `/blueboat/odom` | in | Odometry; freshness by arrival time (real stamps are 0) |
| `/blueboat/update_path` | client | `blueboat_interfaces/srv/UpdatePath`, frame `world` |
| `/replanner/plan` · `/decision` · `/status` | out | Path · JSON `blueboat_replanner_decision/1` · `blueboat_replanner_status/1` |
| `/replanner/start`, `/replanner/stop` | srv | `std_srvs/Trigger` |
| mission file | file | brief §6.4, one WGS-84 polygon, `planning_depth_m` required |

## 4. Non-negotiables
1. **Never `git commit` or `git push`** (root §2.1). The user commits. Deliverables stay in this
   repo; no archives.
2. Edit only inside `BlueBoat-Replanner/`. Other modules are read-only (CM-3). Needs go to
   `docs/REQUESTS.md`.
3. No new message types. JSON goes on `std_msgs/String` with a `"schema"` field (CM-1).
4. The path changes **only** through `/blueboat/update_path`. Never write MCS files, never start a
   `/path_request` server.
5. The belief map is the only online source of target evidence. Never read detections, sonar,
   ground truth or `/sim/*` online (CM-9 as decided; INTEGRATION §4).
6. Core purity: no rclpy, clock or randomness in `core/`. Randomness lives in `eval/` only, from
   seeded `SeedSequence`s.
7. Planner model ≠ world model: configuration, not code. Ablations change only the planner block.
8. **C2 is tuned seriously and frozen before any C3/C4 result** (`docs/C2_TUNING_PROTOCOL.md`; the
   runner enforces it). Never tune to make C4 win (CM-12).
9. Results are model-conditional (CM-13). Every dev-loop number is "reference-belief-conditional,
   preliminary" and never E4 (synthesis §6.2 ⚠).
10. Math first: an ambiguity found while coding goes to `docs/MATH.md` before code (brief §18).
11. Safety:
    - no autonomous start;
    - override or E-STOP → `disarmed`, re-armed only by `/replanner/start` plus checks;
    - "stopped" is reported only as `stopped_holding`; a refused stop is "not in effect".
12. Cite nothing from memory. Literature statuses are in MATH §16, separating what was read from
    what was not.

## 5. Commands
```bash
cd blueboat_replanner && python3 -m test.smoke_test                 # the gate (no ROS)
python3 -m blueboat_replanner.eval.runner --out ../results/<dir>/c2_tuning --tune-c2 --jobs 4
python3 -m blueboat_replanner.eval.runner --out ../results/<dir> \
    --c2-tuned ../results/<dir>/c2_tuning/c2_tuned.yaml --seeds 30 --jobs 4
colcon build --packages-select blueboat_replanner                   # in-machine (L-R1)
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs yaml>
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```
- **Gate.** The Stop hook in `.claude/settings.json` runs `.claude/tools/smoke_gate.sh`, only when
  a package `.py` file changed, and exits 2 on failure.
- **Foreign checks.** The gate cross-checks MCS `geo.py` (bit-identity) and Control's ROS-free
  `path_update.PathUpdater` when they sit beside this repo. It imports them with bytecode writing
  off, so nothing is written into their trees.

## 6. Status
- **2026-11-04, cloud session REPLAN-1: Phases 0–5 done.**
  - The gate: **188 passed, 0 failed, 1 skipped** (the real-Belief check).
  - The dev loop: C2 frozen, then 450 missions with 0 failures, 0 refusals and 0 missed leads.
    Results are in `docs/DEVLOOP.md` §5 and `results/devloop_2026-11-04/`.
  - What each claim is worth: `docs/REPORT.md`. Next steps: `docs/HANDOVER_CLOUD.md` and
    `docs/LOCAL_CHECKLIST.md`.
  - **Not run under ROS:** the node, the emulator node and the launch files. They run only
    through the stand-in modules of gate [24]. No Gazebo mission and no `colcon build` has run.
  - **Not available here:** the real Belief core.
  - **Dev-loop finding to keep in view:** the aspect-aware bearing choice shows no effect
    (C4 = C4_rand). The gain comes from revisiting early (`DEVLOOP.md` §5.2).
