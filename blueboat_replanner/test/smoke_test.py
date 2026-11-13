"""Offline gate (no ROS): `python3 -m test.smoke_test` from the package root.

Numbered sections, one line per check, exit status 1 on any failure (pattern of
BlueBoat-SSS-Sim's smoke test). Each section is one file, `test/gate/sNN_*.py`; `gate/common.py`
holds the imports, `check` / `skip` / `raises` / `sibling` and the shared fixtures. This runner
executes `common.py`, then every section in order, in ONE shared namespace (later sections use
names made by earlier ones). A section that raises counts as one failure, with its traceback,
and the next section still runs.

Checks that need a neighbour module (MCS geo, Control's path_update, BlueBoat-Belief's core)
SKIP visibly when it is absent. Foreign modules are imported with bytecode writing disabled so
that no file is created in another module's tree.

The belief map of sections [20] and [22] is BlueBoat-Belief's own core, driven through its
documented offline interface (`eval/belief_real.py`). Hand-built grids (`make_grid`) are
fixtures of the contract's shape for the validator and the value model, nothing more.

`test/test_gate.py` runs this gate in a subprocess for pytest (`colcon test`).
"""

from __future__ import annotations

import sys
import traceback
from pathlib import Path

sys.dont_write_bytecode = True
GATE = Path(__file__).resolve().parent / "gate"


def _exec(path: Path, ns: dict) -> None:
    ns["__file__"] = str(path)
    exec(compile(path.read_text(), str(path), "exec"), ns)


def main() -> int:
    ns = {"__name__": "blueboat_replanner_gate", "__builtins__": __builtins__}
    _exec(GATE / "common.py", ns)
    for path in sorted(GATE.glob("s[0-9][0-9]_*.py")):
        try:
            _exec(path, ns)
        except Exception as exc:  # noqa: BLE001 - one failure, the gate goes on
            ns["FAIL"] += 1
            print(f"  FAIL  section {path.name} raised {type(exc).__name__}: {exc}")
            for line in traceback.format_exc().rstrip().splitlines()[-6:]:
                print(f"        {line}")
    print(f"\nSMOKE: {ns['PASS']} passed, {ns['FAIL']} failed, {ns['SKIP']} skipped")
    return 1 if ns["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main())
