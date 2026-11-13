"""Paired comparisons between two conditions of a finished dev-loop run (eval/ only).

    python3 -m blueboat_replanner.eval.compare DIR --pairs C4:C4_rand,C4:C4_iso,C4_depth1:C4

Reads `DIR/results.jsonl` (written by `eval/runner.py`) and prints, for each pair A:B and each
budget fraction, the paired difference A - B on the same seeds: mean ± sd, ≈ 95 % interval
(mean ± 2 sd / sqrt(n), the runner's formula), and how often A was better / equal / worse.
`--at 0.5` compares the anytime score at that fraction of the budget instead of the final one.

It adds nothing to the results: it is the runner's own paired statistic, for pairs other than
"against C2". Every number stays "real-Belief dev loop, preliminary".
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

METRICS = ("score", "confirmed_true", "false_confirmations", "recall_detected")


def load(path: str) -> dict:
    by = {}
    with open(os.path.join(path, "results.jsonl")) as fh:
        for line in fh:
            r = json.loads(line)
            by.setdefault((r["condition"], r["budget_fraction"]), {})[r["seed"]] = r
    return by


def value(row: dict, metric: str, at: float | None) -> float:
    if at is None:
        return float(row["final"][metric])
    for a in row["anytime"]:
        if abs(a["t"] / row["B"] - at) < 1e-6:
            return float(a[metric])
    raise KeyError(f"no anytime mark at {at} of the budget")


def paired(by: dict, a: str, b: str, frac: float, metric: str = "score",
           at: float | None = None) -> dict | None:
    ga, gb = by.get((a, frac)), by.get((b, frac))
    if not ga or not gb:
        return None
    seeds = sorted(set(ga) & set(gb))
    if not seeds:
        return None
    d = np.array([value(ga[s], metric, at) - value(gb[s], metric, at) for s in seeds])
    n = len(d)
    sd = float(d.std(ddof=1)) if n > 1 else 0.0
    half = 2.0 * sd / math.sqrt(n) if n > 1 else float("nan")
    return {"a": a, "b": b, "fraction": frac, "metric": metric, "at": at, "n": n,
            "mean": float(d.mean()), "sd": sd, "ci": [float(d.mean() - half),
                                                     float(d.mean() + half)],
            "better": int((d > 0).sum()), "equal": int((d == 0).sum()),
            "worse": int((d < 0).sum()),
            "mean_a": float(np.mean([value(ga[s], metric, at) for s in seeds])),
            "mean_b": float(np.mean([value(gb[s], metric, at) for s in seeds]))}


def table(rows: list) -> str:
    L = ["| A − B | budget | metric | n | A | B | difference (mean ± sd) | ≈95 % interval | "
         "better / equal / worse |", "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        what = r["metric"] + ("" if r["at"] is None else f" at {r['at']:g} of the budget")
        L.append(f"| {r['a']} − {r['b']} | {r['fraction']:g} | {what} | {r['n']} | "
                 f"{r['mean_a']:.2f} | {r['mean_b']:.2f} | {r['mean']:+.2f} ± {r['sd']:.2f} | "
                 f"[{r['ci'][0]:+.2f}, {r['ci'][1]:+.2f}] | "
                 f"{r['better']} / {r['equal']} / {r['worse']} |")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("dir")
    ap.add_argument("--pairs", required=True, help="comma-separated A:B pairs")
    ap.add_argument("--fractions", default=None, help="default: every fraction in the run")
    ap.add_argument("--metrics", default="score")
    ap.add_argument("--at", type=float, default=None,
                    help="compare the anytime value at this fraction of the budget")
    a = ap.parse_args(argv)
    by = load(a.dir)
    fracs = ([float(v) for v in a.fractions.split(",")] if a.fractions else
             sorted({f for _, f in by}, reverse=True))
    rows = []
    for pair in a.pairs.split(","):
        x, y = pair.split(":")
        for f in fracs:
            for m in a.metrics.split(","):
                if m not in METRICS:
                    raise SystemExit(f"metric {m!r} not in {METRICS}")
                r = paired(by, x, y, f, m, a.at)
                if r is not None:
                    rows.append(r)
    print(table(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main())
