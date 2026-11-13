#!/usr/bin/env bash
# The dev loop on BlueBoat-Belief's core, batch after batch (docs/DEVLOOP.md §4).
#
#   .claude/tools/devloop_batches.sh [results folder name] [workers]
#   default: devloop_real_<today>, 4 workers
#
# - Before every batch it stops if a simulation recording is running on this machine: a
#   simulation slowed by CPU contention can miss its wall-clock limit and lose its result.
# - It never overwrites: an existing C2 freeze is kept, and every runner call resumes from its
#   own results.jsonl. Re-running the script after a stop continues where it stopped.
# - Order: tune and freeze C2 first (docs/C2_TUNING_PROTOCOL.md), then everything else.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
PKG="$ROOT/blueboat_replanner"
NAME="${1:-devloop_real_$(date +%Y-%m-%d)}"
JOBS="${2:-4}"
R="../results/$NAME"
PY="nice -n 10 /usr/bin/python3 -B -m blueboat_replanner.eval.runner"
cd "$PKG" || exit 1
export PYTHONDONTWRITEBYTECODE=1

guard() {
  if pgrep -f 'record_ru[n]\.py' > /dev/null; then
    echo "$(date +%T) STOPPED before '$1': a simulation recording is running"; exit 3
  fi
  echo "$(date +%T) batch: $1"
}
run() {  # name, then runner arguments
  local name="$1"; shift
  guard "$name"
  if ! $PY "$@" > "$R/$name.log" 2>&1; then
    echo "$(date +%T) FAILED: $name (see $R/$name.log)"; tail -5 "$R/$name.log"; exit 4
  fi
  tail -1 "$R/$name.log"
}

mkdir -p "$R"
# 1. C2 first: tuned and frozen before any C3 / C4 run
if [ ! -f "$R/c2_tuning/c2_tuned.yaml" ]; then
  run c2_tuning --out "$R/c2_tuning" --tune-c2 --jobs "$JOBS"
fi
if [ ! -f config/c2_tuned.yaml ]; then
  cp "$R/c2_tuning/c2_tuned.yaml" config/c2_tuned.yaml
  cp "$R/c2_tuning/C2_TUNING_RESULT.md" ../docs/C2_TUNING_RESULT.md
fi
cmp -s "$R/c2_tuning/c2_tuned.yaml" config/c2_tuned.yaml || {
  echo "config/c2_tuned.yaml is not the freeze of $R/c2_tuning: stop"; exit 5; }
sha256sum config/c2_tuned.yaml

# 2. main run: every condition, 30 paired seeds, no class information
run main --out "$R/main" --name main --c2-tuned config/c2_tuned.yaml --seeds 30 --jobs "$JOBS"
# 3. missions cut short (the frozen C2 plan truncated; C2 is not re-tuned per budget)
run budget_sweep --out "$R/budget_sweep" --name budget_sweep --c2-tuned config/c2_tuned.yaml \
    --seeds 20 --fractions 0.75,0.5 --conditions C2,C3,C4,C4_iso,C4_rand \
    --calibration-from "$R/main/manifest.json" --jobs "$JOBS"
run budget_merged --out "$R/budget_merged" --name budget_merged \
    --from-results "$R/main,$R/budget_sweep" --conditions C2,C3,C4,C4_iso,C4_rand \
    --fractions 1.0,0.75,0.5
# 4. the emulated detector reports the true class (an upper bound on what class knowledge buys)
run class_true --out "$R/class_true" --name class_true --class-info true \
    --c2-tuned config/c2_tuned.yaml --seeds 30 --conditions C2,C4,C4_iso,C4_rand \
    --calibration-from "$R/main/manifest.json" --jobs "$JOBS"
# 5. sensitivity: misses credited at once (no image delay)
run ungated --out "$R/ungated" --name ungated --miss-credit ungated \
    --c2-tuned config/c2_tuned.yaml --seeds 30 --conditions C2,C3,C4,C4_rand --jobs "$JOBS"
# 6. decision time with nothing else running on the machine
run timing_uncontended --out "$R/timing_uncontended" --name timing_uncontended \
    --c2-tuned config/c2_tuned.yaml --seeds 5 --conditions C4,C4_depth1 \
    --calibration-from "$R/main/manifest.json" --jobs 1
echo "$(date +%T) ALL BATCHES DONE: $R"
