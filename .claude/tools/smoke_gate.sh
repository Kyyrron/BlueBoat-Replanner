#!/usr/bin/env bash
# Stop hook: run the offline gate when a Python file of the package changed since the last
# green run; exit 2 (block the stop) when the gate fails. Pattern of BlueBoat-SSS-Sim.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PKG="$ROOT/blueboat_replanner"
STAMP="${TMPDIR:-/tmp}/blueboat_replanner_gate.sha"
SUM="$(find "$PKG" -name '*.py' -not -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -d' ' -f1)"
if [ -f "$STAMP" ] && [ "$(cat "$STAMP")" = "$SUM" ]; then
  exit 0
fi
cd "$PKG" || exit 0
OUT="$(PYTHONDONTWRITEBYTECODE=1 timeout 900 /usr/bin/python3 -m test.smoke_test 2>&1)"
RC=$?
if [ $RC -ne 0 ]; then
  echo "$OUT" | grep -E "FAIL|SMOKE|Traceback|Error" | tail -20 >&2
  echo "blueboat_replanner smoke gate FAILED (exit $RC): fix before stopping" >&2
  exit 2
fi
echo "$SUM" > "$STAMP"
exit 0
