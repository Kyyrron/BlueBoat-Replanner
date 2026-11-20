# CLAUDE.md — BlueBoat-Replanner (thesis deliverable D6)

Aspect-aware replanner for the BlueBoat side-scan survey. At decision points during a run it
continues the survey or revisits a contact from a chosen bearing, and it makes the boat fly that
within the same run through Control's live path update. It also generates every survey condition
through **one** coverage generator and **one** executor:
- C1 single pass;
- C2 two-pass orthogonal (the baseline);
- C3 append-on-completion;
- C4 informative, with its ablations.

The brief is `docs/REPLANNER_BRIEF.md` (read-only). The contract with Belief is `belief_grid/0`:
Belief's `docs/topics.md` is its text, `docs/CONTRACT_CONSUMED.md` is how this module reads it.

Simulated results follow `../SIM_VALIDITY_CONDITIONS.md` (superproject root; simulator frozen 2026-11-18,
literal verdict NOT READY, a model-conditional tool). A time difference between conditions inside **±3 % plus
0.6 s per turn** is not a result (simulated MPC turns run up to 20 % long, which works against the adaptive
conditions); simulated position error is up to about **1.1 m right after a turn** (0.25 m on straight legs).
Compare conditions paired (same world, seeds, sea schedule) and at both R1 ends. The Replanner has run in
Gazebo once (one C2 development run in a built world, 2026-11-18, `docs/LOCAL_CHECKLIST.md` L-G1); the dev-loop numbers are preliminary and predate Belief's fore/aft arm (CR-REP-1).

## 1. Authority and documents
- Project rules: root `.claude/CLAUDE.md` and `project_synthesis.md` (there is no context pack).
- **Where the work stands, the owner's decisions, what comes next: `docs/HANDOVER.md`** (read
  it first). Open items in short: `.claude/TODO.md`.
- What the module is, the science behind it, how it works: `README.md`.
- Design `docs/DESIGN.md`; mathematics `docs/MATH.md`; interfaces `docs/topics.md`.
- Belief binding: `docs/REAL_BELIEF_BINDING.md`, `docs/CONTRACT_CONSUMED.md`. Dev loop:
  `docs/DEVLOOP.md` (the main copy of its results). Weight of every claim: `docs/REPORT.md`.
- Evidence ids: `docs/EVIDENCE_REGISTER.md` (E-nn, L-nn, A-nn). Requests: `docs/REQUESTS.md`.
- ROS, Gazebo and boat checks: `docs/LOCAL_CHECKLIST.md` (L-X is the main copy of the ROS step).
- Where a document and the code disagree, the code wins: fix the document.
- **Only current documents** (owner, 2026-11-13): no session records, no change logs; one main
  copy of each fact, other documents point to it. Git keeps the history.

## 2. Layout (ament_python, one level below the git root)
`blueboat_replanner/`:
- `blueboat_replanner/core/`: ROS-free, deterministic.
- `io/`: YAML and records.
- `ros/`:
  - `adapters.py`, the one field-name table;
  - `executor.py`, the ONLY code that knows `/blueboat/update_path`;
  - `replanner_node.py`, the only rclpy code.
- `eval/`: dev loop and runner, `belief_real.py` (the binding to BlueBoat-Belief's core),
  `emulator.py` (looks at world objects, seabed images), world, Control stub, emulator node; the
  only randomness.
- Also `config/`, `launch/`, `test/smoke_test.py` (the gate's runner; one file per section in
  `test/gate/`; `test/test_gate.py` runs it for `colcon test`) and `test/ros_graph_check.py` (the
  ROS runtime check, run by hand through `.claude/tools/ros_check.sh`, never on domain 0).

Full tree: DESIGN §3.

## 3. Interfaces (details: `docs/topics.md`)
| Name | Dir | Type / QoS |
|---|---|---|
| `/belief_map/belief_grid` | in | `blueboat_belief_interfaces/BeliefGrid`, RELIABLE TRANSIENT_LOCAL depth 1 — the ONLY target evidence. Its header stamp is Belief's evidence horizon |
| `/blueboat/path_progress`, `/blueboat/path_update_event` | in | String JSON (Control `blueboat_path_progress/1`, `…_event/1`); `ros_time` + `tau` convert the grid stamp to path time |
| `/blueboat/odom` | in | Odometry; freshness by arrival time (real stamps are 0) |
| `/blueboat/update_path` | client | `blueboat_interfaces/srv/UpdatePath`, frame `world` |
| `/replanner/plan` · `/decision` · `/status` | out | Path · JSON `blueboat_replanner_decision/1` · `blueboat_replanner_status/1` |
| `/replanner/start`, `/replanner/stop` | srv | `std_srvs/Trigger` |
| mission file | file | brief §6.4, one WGS-84 polygon, `planning_depth_m` required; it must lie inside Belief's area |

## 4. Non-negotiables
1. **Never `git commit`, `git push`, or stage** (root §2.1). The user does. Deliverables stay in
   this repo; no archives.
2. Edit only inside `BlueBoat-Replanner/`. Other modules are read-only (CM-3), **Belief
   included**. Needs go to `docs/REQUESTS.md`. One exception (owner, 2026-11-13): the
   superproject's root lines that describe this module (root `.claude/CLAUDE.md`,
   `interface-contract.md`, `synthesis-discrepancies.md`, the skills, `README.md`,
   `project_synthesis.md`) are kept current by this module's sessions; change only those
   lines, and re-read each file first (other sessions edit it). Lines in other modules that
   describe this module are corrected only by an explicit owner's order (as on 2026-11-13).
3. No new message types. JSON goes on `std_msgs/String` with a `"schema"` field (CM-1).
4. The path changes **only** through `/blueboat/update_path`. Never write MCS files, never start a
   `/path_request` server.
5. The belief map is the only online source of target evidence. Never read detections, sonar,
   ground truth or `/sim/*` online (root CM-9).
6. Core purity: no rclpy, clock or randomness in `core/`. Randomness lives in `eval/` only, from
   seeded `SeedSequence`s.
7. Planner model ≠ world model ≠ Belief's model: configuration, not code. Ablations change only
   the planner block.
8. **C2 is tuned seriously and frozen before any C3/C4 result** (`docs/C2_TUNING_PROTOCOL.md`; the
   runner enforces it and never overwrites a freeze). Never tune to make C4 win (CM-12).
9. Results are model-conditional (CM-13). Every dev-loop number is "real-Belief dev loop,
   preliminary" and never E4 (synthesis §6.2 ⚠). While Belief's `detector_calibrated` is false,
   everything built on a detection score is `score-conditional`.
10. Math first: an ambiguity found while coding goes to `docs/MATH.md` before code (brief §18).
11. Safety:
    - no autonomous start;
    - override or E-STOP → `disarmed`, re-armed only by `/replanner/start` plus checks;
    - "stopped" is reported only as `stopped_holding`; a refused stop is "not in effect".
12. Cite nothing from memory. Literature statuses are in MATH §16, separating what was read from
    what was not.
13. **The belief map is BlueBoat-Belief's, everywhere** (owner, 2026-11-12). The dev loop drives
    Belief's own core through its documented offline interface (`eval/belief_real.py`), and
    nothing here imitates Belief. If the core cannot be imported, the dev loop stops.
14. **Read Belief by meaning.** Searched sectors are `aspect_log_miss < 0`, never
    `aspect_counts`; the grid is complete up to its header stamp; a lower `grid_seq` is a restart
    (`docs/CONTRACT_CONSUMED.md`).
15. **Do not load the machine while a simulation is recorded:** before a dev-loop run, a
    `colcon`, `ros2` or `gz` command, check `pgrep -af 'record_ru[n]\.py'`. That tool also finds
    other runs by scanning process command lines, so keep simulator and node names out of shell
    commands while it runs.

## 5. Commands
```bash
cd blueboat_replanner && python3 -B -m test.smoke_test              # the gate (no ROS, ~90 s)
.claude/tools/devloop_batches.sh                                    # dev loop on Belief's core: C2 freeze, then every batch
.claude/tools/ros_check.sh                                          # scratch build + the nodes on an isolated ROS domain beside Belief's node (made-up inputs, no Gazebo; ~28 min)
colcon build --packages-select blueboat_replanner                   # needs blueboat_interfaces, blueboat_belief_interfaces (L-R1)
ros2 launch blueboat_replanner replanner_launch.py mission_file:=<abs yaml>   # Gazebo: + use_sim_time:=true time_source:=ros
ros2 service call /replanner/start std_srvs/srv/Trigger {}
```
- **Gate.** The Stop hook in `.claude/settings.json` runs `.claude/tools/smoke_gate.sh`, only when
  a package `.py` file changed, and exits 2 on failure.
- **Foreign checks.** The gate cross-checks MCS `geo.py` (bit-identity), Control's ROS-free
  `path_update.PathUpdater` and BlueBoat-Belief's core when they sit beside this repo. It imports
  them with bytecode writing off, so nothing is written into their trees.

## 6. Status (2026-11-18; details in `docs/HANDOVER.md`)
- Built: core, executor, node, emulator node, launch files. Gate **294 passed, 0 failed,
  0 skipped** (2026-11-18); `colcon test` runs it.
- The node's clock (2026-11-18, RQ-1): parameter `time_source` `wall` (default, byte-identical
  records, gate [26]) | `ros` (node clock); launch arguments `use_sim_time`, `time_source`. In
  Gazebo `use_sim_time:=true time_source:=ros` (MATH §11.3, `LOCAL_CHECKLIST.md` L-G).
- Dev loop on BlueBoat-Belief's own core, its missions executed by the node's executor
  (40 stored missions re-run on it are identical): C2 frozen, 690 missions + 150
  (`docs/DEVLOOP.md` §5).
  In this loop only: at the full budget no adaptive condition is distinguishable from C2;
  mid-mission and under a budget cut they are ahead; the aspect model shows no effect
  (C4 = C4_iso).
- Under ROS: only on an isolated graph beside Belief's node with made-up inputs, on the code
  of 2026-11-12 (`docs/LOCAL_CHECKLIST.md` L-X). Built into `~/ros2_ws/install` (2026-11-18).
- In Gazebo once, 2026-11-18 (L-G1): a C2 development run in a built world beside Control's
  node, Belief, Perception (dummy) and the GCS, `use_sim_time:=true time_source:=ros`; first path
  accepted, plan to `done`, no stale input through the render holds. Never on the boat; C4 and
  the stop test in Gazebo not yet (L-G2, L-G3).
- Uncommitted on 2026-11-18 (session 13, for the owner to commit): `time_source`, its launch
  arguments, gate [26], the documents. Package code before it: `2f84a46`.
