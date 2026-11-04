# Contract deviations — consumer side (contract `belief_grid/0`, C-9)

Every place where the repository, the briefs or a user answer forced or may force a change to the
contract. The embedded contract text (prompt §B) is never edited. A change made by a user answer
is recorded here and in `HANDOVER_CLOUD.md`.

**Status 2026-11-04 (Phase 0): no deviation adopted.** The consumer will be built to
`belief_grid/0` as written. Candidates raised by Phase 0, pending user answers:

| Id | Contract item | Observation | Evidence | Pending |
|---|---|---|---|---|
| CD-1 | C-1 "Belief subscribes `/sss_ai/detections`" | Root CLAUDE.md §1, CM-9 and synthesis §5.1 say Belief reads `/sss_ai/image_records` | root CLAUDE.md l.53–57, l.476–480; synthesis l.216 | Q-9. Affects only the `eval/` emulator, not the `BeliefGrid` fields |
| CD-2 | C-3 (no survey-area polygon field) | Synthesis §5.1 ⚠ says Belief carries the survey area for the Replanner | synthesis l.224–226 | **Settled 2026-11-04 (user: "the brief overrides anything else"): the Replanner reads the mission file. No contract change.** `REQUESTS.md` RQ-2 asks Belief to read the same file |
| CD-3 | C-5 look geometry | Perception's per-row look directions match C-5 (port = ψ + π/2) | Perception `core/geometry.py:90` | none (consistency noted) |
| CD-4 | C-8 "Belief's configuration YAML … the Replanner does not read it in the cloud" | Brief §6.2 lists Belief's configuration file as a read-only Replanner input | brief §6.2 | none: the contract rule is followed; shared values come from the message |
