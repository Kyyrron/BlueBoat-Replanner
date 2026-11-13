# Handover — where the work stands and what comes next

**Read this first when work resumes.** Updated 2026-11-13 (evening). What each claim is worth:
`REPORT.md`. Every ROS, Gazebo and boat check with its command: `LOCAL_CHECKLIST.md`. Open
items in short: `.claude/TODO.md`.

## 1. Where the work stands

| Part | State |
|---|---|
| ROS-free core (coverage, look predictor, value model, C1–C4 policies), executor, node, emulator node, launch files | built |
| Offline gate `test/smoke_test.py` | **278 passed, 0 failed, 0 skipped** (2026-11-13; system python, venv python, and a shell with the ROS workspace sourced). One file per section in `test/gate/`; `colcon test` runs it (`test/test_gate.py`, 1 test) |
| Reading of Belief's grid | by meaning (`CONTRACT_CONSUMED.md`), checked against BlueBoat-Belief's own core (gate [20]) |
| Dev loop | on BlueBoat-Belief's own core (`REAL_BELIEF_BINDING.md`); **its missions are executed by the node's own executor** since 2026-11-13 (DESIGN §5.4): 40 stored missions re-run on it are identical to their rows, so the results stand (`DEVLOOP.md` §1). C2 frozen (f = 1.0, B = 2111 s); 690 missions + 150 in a labelled extra comparison, none failed (`DEVLOOP.md` §5). **Real-Belief dev loop, preliminary**, never E4 |
| Under ROS | the node, the emulator node and both launch files ran on an isolated ROS graph (domain 47) beside Belief's installed node, with made-up sonar, navigation and Control: 42 checks, 0 failed (`LOCAL_CHECKLIST.md` L-X), **on the code of 2026-11-12**: the node changes of 2026-11-13 have not run under ROS |
| Workspace | **built into `~/ros2_ws/install`** (2026-11-13, plain install, system python; `LOCAL_CHECKLIST.md` L-R1). No node was started from it |
| Not done | anything with Control's real node, Gazebo, a sonar image, a detector, or the boat |
| Git | **nothing of the 2026-11-12 and 2026-11-13 work is committed.** HEAD (`c53fe2a`) still contains the removed imitation of Belief and its results; the working tree does not. The owner commits |

**The code fixes of 2026-11-13** (the review's [P0] code, [P1] code quality, [P2] dead code):
- the executor checks the area (F_(margin − 0.5 m)) and the junction at c before every send,
  and refuses to send otherwise (DESIGN §5.2 rule 2; MATH §3.6);
- an error in the executor's handling of an answer is no longer hidden as a failed call;
- answers are handled at their arrival time;
- no first request before the service is ready;
- C4_live and C4_degraded are refused in a mission file;
- the node applies the C2 freeze (`c2_tuned_file`) to every mission and refuses a mission that
  contradicts it;
- the dev loop runs the executor;
- C4_rand's table is drawn over the grid's sectors, and the look-key stride is checked in
  `validate_grid`;
- the archived plan uses the configured sampling;
- the gate is split into section files;
- dead code is removed, the admission and next-line helpers exist once, and the remaining
  constants are configuration or node parameters.

**Belief is identified by its core hash**, because its working tree is uncommitted: the results
were made on `fcd9531597e593b3…`; 40 missions re-run on `ac686bf69cf07f6a…` (comments only
changed) equal their stored rows. If the hash differs again: the gate re-checks the binding;
re-run a sample of stored missions (the runner refuses to add rows to a folder made with
another hash); re-run `.claude/tools/devloop_batches.sh` into a new folder (45 minutes on 4
workers) only if a mission differs.

## 2. The owner's decisions in force (do not ask again)

1. **Only the real BlueBoat-Belief.** No imitation of Belief exists in this module and none is
   reintroduced; if Belief's core cannot be imported, the dev loop stops.
2. **Class information of the emulated detector: both, kept apart.** Main runs with none; a
   smaller run with the true class.
3. **Look predictor:** follow Belief's turn gate and its 0.285 m lever arm.
4. **Documented behaviours the thesis needs are implemented**, consistent with the superproject.
5. **Dev-loop runs only when no simulation recording is running, on 4 workers.**
6. **C2 tie rule: the paired yardstick** (`C2_TUNING_PROTOCOL.md` §5, `eval/runner.select_c2`).
7. **The lookahead of C4 is decided after timing under ROS:** C4 (two-step) and C4_depth1
   (one-step) both stay until decision times are measured in a Gazebo run.
8. **Each ROS step needs its own go-ahead.** The first (isolated graph, no Gazebo) was given
   and done; Control's real node and Gazebo each need a new one, and must not disturb a
   simulator session.
9. **Root documents (2026-11-13):** the superproject's lines about this module are kept current
   by this module's sessions, `project_synthesis.md` included.
10. **Documents (2026-11-13):** only current documents are kept; no session records.

How the owner wants questions: every term explained in plain words in the question itself,
with what each option changes.

## 3. Next steps, in order

1. **Commit** (the owner): the module's working tree, then the superproject's gitlink.
2. **Re-run the ROS check on the new node code** (`.claude/tools/ros_check.sh`, about 28 min,
   isolated domain, made-up inputs, no Gazebo): the node's changes of 2026-11-13 have only run
   offline. The same kind of step as the first one; it needs a go-ahead (decision 8).
3. **Control's real `/blueboat/update_path`** in Control's own simulation launch
   (`LOCAL_CHECKLIST.md` L-C1…L-C4). Go-ahead needed.
4. **Gazebo missions** through the composition until R-3 lands (L-G1…L-G4): C2, then C4, then the
   stop test. They give the decision time under load (decision 1 of §4), the 0.5 m/s and
   3 m-turn tracking (L-K3, L-K4) and the clock check (L-G intro). Go-ahead needed.
5. **The emulator in a built world**, its calibrator refitted on Gazebo C2 runs (L-E).
6. Then: the E4 C2 freeze in Gazebo on tuning worlds (`C2_TUNING_PROTOCOL.md` §6), a world
   detection model fitted from Gazebo, sensitivity sweeps, real water after Control's own
   on-water path-update test.

## 4. Decisions open for the owner

Numbers are from the dev loop on Belief's core (`DEVLOOP.md` §5), so they are indications, not
results.

| # | Decision, in plain words | What is known |
|---|---|---|
| 1 | **Which lookahead the main adaptive strategy (C4) uses.** C4 looks two decisions ahead; C4_depth1 looks one ahead. A decision must be computed within 2.0 s | Same score (C4_depth1 − C4 = +0.03 [−0.31, +0.37]). Slowest decision alone on the laptop: C4 1.79 s, C4_depth1 0.49 s; with four missions in parallel C4 2.5–2.8 s. Inside the node on the isolated graph: C4 mean 0.46 s, slowest 1.74 s. A late decision is dropped, never sent. Owner: decide after the Gazebo timing |
| 2 | **What the budget comparison may claim.** The other strategies are compared with C2 cut to the same shorter time; C2 is not re-planned for that time | Leaning recorded: "missions cut short" only. Cut at 0.75 B: C4 +1.20 [+0.61, +1.79] |
| 3 | **The thesis argument.** H1: return visits help. H2: choosing the bearing of a return visit helps | Leaning recorded: H1 primary, H2 unproven. In the loop H1 shows only mid-mission and under a cut budget; H2 shows no effect (C4 = C4_iso) |
| 4 | **Survey speed**: 0.5 m/s here (owner, 2026-11-04) against 1.0 m/s in `project_synthesis.md` §8.5 | At 0.5 m/s B = 2111 s, the grid lags 4–9 s; at 1.0 m/s B ≈ 1055 s. Gazebo tops out near 0.78 m/s and Control advises ≤ 0.45 m/s in simulation. Recorded as a discrepancy in the superproject |
| 5 | **How often a clutter object is re-detected** on a second look (f = 0.2, a placeholder) | Clutter confirmations decide the sign of C4 − C2 at the full budget: Belief confirms any contact on two detected looks |
| 6 | **Missions with more than one area** | not built; one area per mission |
| 7 | **When the replanner may change the path on real water** without an operator approving each change | Today: autonomous within an operator-started run, first only after Control's on-water test and a supervised dry run (DESIGN §9) |
| 8 | **Belief's assumed target density** in the dev loop: its default 5 per hectare, while the scenario has 10 | not varied |
| 9 | **Which detector class belongs to which aspect family** (`REQUESTS.md` RQ-6) | Without it the bearing choice is inert for every class |
| 10 | Optional: a stricter test of why H2 shows nothing, with Belief confirming only on looks along different lines (a request to Belief) | The test with different directed sectors changed little (`DEVLOOP.md` §5.9) |

## 5. Before any run, and rules that bite

```bash
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-Replanner
pgrep -af 'record_ru[n]\.py'                    # must print nothing before a long or ROS command
cd blueboat_replanner && /usr/bin/python3 -B -m test.smoke_test | tail -1
#   expected: SMOKE: 278 passed, 0 failed, 0 skipped   (about 90 s, one core)
/usr/bin/python3 -B -c "import sys; sys.dont_write_bytecode=True; sys.path.insert(0,'.'); \
from blueboat_replanner.eval.belief_real import belief_info as b; i=b(); print(i['core_sha256'][:16], i['gate'])"
#   expected: ac686bf69cf07f6a {'checks': 241, 'skipped': 0, ...}
```

- A simulation recording may be running: check before every dev-loop run, `colcon`, `ros2` or
  `gz` command, and keep simulator and node names out of shell commands (the recording tool
  scans process command lines). With `pgrep -f`, keep the searched names out of the rest of the
  command line, or the shell matches itself.
- Never stage, commit or push.
- The Stop hook runs the gate when a package `.py` file changed (about 90 s).
- Do not edit a package `.py` file while `devloop_batches.sh` runs: every batch starts a new
  process.
- The C4 mission of the ROS check runs at real speed (`--c4-speedup 1`, the default): the node
  reserves real seconds for a decision, so a faster path clock makes Control refuse decisions.
- Run the gate once from a shell with the ROS workspace sourced as well: it is the only place
  where an installed copy of a neighbour is on the path.
- The ROS check's scratch build lives in a temporary folder: after a reboot,
  `.claude/tools/ros_check.sh` re-creates it (about 28 minutes).

## 6. Outside this repository

- This module's code and tools change nothing in another module; documents of other modules
  are edited only by the owner's order (the one of 2026-11-13 is below).
- The superproject's root files describe this module as of 2026-11-13 (root `.claude/CLAUDE.md`,
  `interface-contract.md`, `synthesis-discrepancies.md`, the skills, `README.md`,
  `project_synthesis.md`).
- The neighbours' lines about this module were corrected on 2026-11-13 by the owner's order:
  Belief (`.claude/CLAUDE.md`, `README.md`, `topics.md` D-C4, `VALIDATION_REPORT.md`), Control
  (`docs/REQUESTS.md` R-4 and R-5, `docs/PATH_UPDATE_DESIGN.md`), SSS-Sim (`.claude/CLAUDE.md`,
  `docs/roadmap.md`, `docs/engineering/raycast_assessment.md` §11), the GCS
  (`blueboat_gcs/docs/HANDOVER.md`), and the root `git-history` skill. Dated snapshots stay as
  they were (`BlueBoat-Control_report_2026-11-03.md`, `hub/IMPLEMENTATION_PROMPT.md`). The hub's
  Replanner tile replaced its "planned" ghost on 2026-11-13 by the owner's order (`hub/apps.toml`,
  group Operate: it starts the node idle with a mission file from `~/worlds/<project>/<world>/`;
  arming stays a separate `/replanner/start` call; hub tests 40 passed, clippy clean).
