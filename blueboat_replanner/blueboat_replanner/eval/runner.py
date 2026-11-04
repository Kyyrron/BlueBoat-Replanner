"""Dev-loop runner: paired seeds, resumable JSONL, summary tables and plots, manifest.

    python3 -m blueboat_replanner.eval.runner --out DIR --seeds 30 \
        --conditions C1,C2,C3,C4,C4_iso,C4_rand,C4_depth1,C4_degraded --fractions 1.0,0.75,0.5

Every number it writes is "reference-belief-conditional, preliminary" (DEVELOPMENT EVIDENCE
ONLY, never E4). Seeds: calibration 900-919 (C2 only), evaluation from --seed0 (default 2000).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import subprocess
import sys
import time
from multiprocessing import Pool

import numpy as np

from .devloop import LABEL, Scenario, degraded_overrides, run_mission
from .world import Calibrator

CAL_SEEDS = list(range(900, 920))


def rand_table(seed: int, n: int = 64, G: int = 6) -> list:
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), 3]))
    return [int(v) for v in rng.integers(0, G, n)]


def _job(args):
    seed, cond, frac, cal, sc_kw, shared = args
    sc = Scenario(**sc_kw)
    over, pover = dict(shared or {}), None
    run_cond = cond
    if cond == "C4_rand":
        over["c4_rand"] = {"bearing_table": rand_table(seed)}
    if cond == "C4_degraded":
        from ..core.config import Config
        pover = degraded_overrides(Config.from_dict({"planning_depth_m": sc.depth_m}))
    calib = Calibrator(cal["a"], cal["b"], cal["note"])
    t0 = time.perf_counter()
    r = run_mission(seed, run_cond, sc, overrides=over, planner_overrides=pover,
                    calibrator=calib, budget_fraction=frac)
    r["wall_s"] = time.perf_counter() - t0
    # reliability of the calibrated confidences on this run (detections only)
    det = [(calib(raw), lab) for raw, lab, ok in r.pop("raw_scores") if ok]
    r["calib_pairs"] = det
    r["shared_overrides"] = dict(shared or {})
    r.pop("stub_log", None)
    return r


def calibrate(sc_kw, jobs: int, shared: dict) -> dict:
    with Pool(jobs) as pool:
        rows = pool.map(_cal_job, [(s, sc_kw, shared) for s in CAL_SEEDS])
    scores, labels = [], []
    for r in rows:
        for raw, lab, ok in r:
            if ok:
                scores.append(raw)
                labels.append(1.0 if lab else 0.0)
    cal = Calibrator.fit(scores, labels, f"Platt on C2 detections, seeds {CAL_SEEDS[0]}-"
                                         f"{CAL_SEEDS[-1]}")
    return {"a": cal.a, "b": cal.b, "note": cal.fitted_on, "n": len(scores),
            "real_fraction": float(np.mean(labels)) if labels else float("nan")}


def _cal_job(args):
    seed, sc_kw, shared = args
    r = run_mission(seed, "C2", Scenario(**sc_kw), overrides=dict(shared or {}),
                    calibrator=Calibrator())
    return r["raw_scores"]


# ---------------------------------------------------------------- C2 tuning (protocol §2-§6)
TUNE_SEEDS = list(range(1000, 1020))
TUNE_GRID = [{"spacing_factor": f, "pass1_direction": d, "interlace": i}
             for f in (0.6, 0.7, 0.8, 0.9, 1.0) for d in ("long_axis", "long_axis_plus_90")
             for i in ("auto", "off")]


def _tune_job(args):
    seed, k, over, sc_kw = args
    r = run_mission(seed, "C2", Scenario(**sc_kw), overrides=over, calibrator=Calibrator())
    return {"k": k, "seed": seed, "score": r["final"]["score"],
            "confirmed_true": r["final"]["confirmed_true"],
            "false_confirmations": r["final"]["false_confirmations"],
            "recall_detected": r["final"]["recall_detected"], "duration": r["duration"]}


def tune_c2(sc_kw, jobs: int, out_yaml: str, out_md: str) -> dict:
    """C2_TUNING_PROTOCOL.md §2-§6 in the dev loop (PRELIMINARY: reference belief)."""
    import hashlib

    import yaml
    with Pool(jobs) as pool:
        rows = pool.map(_tune_job, [(s, k, cfg_, sc_kw) for k, cfg_ in enumerate(TUNE_GRID)
                                    for s in TUNE_SEEDS])
    ref = TUNE_GRID.index({"spacing_factor": 0.8, "pass1_direction": "long_axis",
                           "interlace": "auto"})
    B_cap = max(r["duration"] for r in rows if r["k"] == ref)
    stats = []
    for k, cfg_ in enumerate(TUNE_GRID):
        rr = [r for r in rows if r["k"] == k]
        sc = np.array([r["score"] for r in rr], float)
        dur = float(np.mean([r["duration"] for r in rr]))
        stats.append({**cfg_, "n": len(rr), "score_mean": float(sc.mean()),
                      "score_sd": float(sc.std(ddof=1)),
                      "se": float(sc.std(ddof=1) / math.sqrt(len(sc))),
                      "confirmed_true": float(np.mean([r["confirmed_true"] for r in rr])),
                      "false_confirmations": float(np.mean([r["false_confirmations"]
                                                            for r in rr])),
                      "recall_detected": float(np.mean([r["recall_detected"] for r in rr])),
                      "duration": dur, "feasible": dur <= B_cap + 1e-6})
    feas = [st for st in stats if st["feasible"]]
    best = max(feas, key=lambda st: st["score_mean"])
    tied = [st for st in feas if st["score_mean"] >= best["score_mean"] - best["se"]]
    chosen = min(tied, key=lambda st: (round(st["duration"], 6),
                                       abs(st["spacing_factor"] - 0.8),
                                       st["pass1_direction"], st["interlace"]))
    doc = {"c2_tuned": {k: chosen[k] for k in ("spacing_factor", "pass1_direction",
                                               "interlace")},
           "protocol": "docs/C2_TUNING_PROTOCOL.md", "label": "PRELIMINARY (dev loop, "
           "reference belief); the E4 freeze is done in Gazebo in-machine",
           "seeds": [TUNE_SEEDS[0], TUNE_SEEDS[-1]], "scenario": sc_kw, "B_cap_s": B_cap}
    text = yaml.safe_dump(doc, sort_keys=False)
    with open(out_yaml, "w") as fh:
        fh.write(text)
    sha = hashlib.sha256(text.encode()).hexdigest()
    L = ["# C2 tuning result (dev loop) — PRELIMINARY", "",
         "Reference-belief-conditional, preliminary. Protocol: `C2_TUNING_PROTOCOL.md`. Tuning "
         f"seeds {TUNE_SEEDS[0]}-{TUNE_SEEDS[-1]} (n = {len(TUNE_SEEDS)}), disjoint from the "
         f"evaluation seeds. Scenario: {sc_kw}. B_cap = T(C2 at f = 0.8, long axis) = "
         f"{B_cap:.0f} s.", "",
         f"**Chosen:** spacing_factor {chosen['spacing_factor']}, pass-1 direction "
         f"{chosen['pass1_direction']}, interlace {chosen['interlace']} (frozen in "
         f"`config/c2_tuned.yaml`, sha256 {sha[:16]}…).", "",
         "| f | direction | interlace | n | score mean ± sd | confirmed true | false conf. | "
         "detected | duration s | feasible |", "|---|---|---|---|---|---|---|---|---|---|"]
    for st in stats:
        L.append(f"| {st['spacing_factor']} | {st['pass1_direction']} | {st['interlace']} | "
                 f"{st['n']} | {st['score_mean']:.2f} ± {st['score_sd']:.2f} | "
                 f"{st['confirmed_true']:.2f} | {st['false_confirmations']:.2f} | "
                 f"{st['recall_detected']:.2f} | {st['duration']:.0f} | "
                 f"{'yes' if st['feasible'] else 'no'} |")
    L += ["", "Selection: maximum mean score among feasible configurations; ties within one "
          "standard error of the best go to the shorter duration, then to f closest to 0.8. "
          "The whole grid is reported."]
    with open(out_md, "w") as fh:
        fh.write("\n".join(L) + "\n")
    return {"sha256": sha, **doc}


def ece(pairs, bins: int = 10) -> float:
    if not pairs:
        return float("nan")
    q = np.array([p for p, _ in pairs])
    y = np.array([1.0 if lab else 0.0 for _, lab in pairs])
    idx = np.minimum((q * bins).astype(int), bins - 1)
    e = 0.0
    for b in range(bins):
        m = idx == b
        if m.any():
            e += m.mean() * abs(q[m].mean() - y[m].mean())
    return float(e)


def summarise(rows, conditions, fractions) -> dict:
    by = {}
    for r in rows:
        by.setdefault((r["condition"], r["budget_fraction"]), {})[r["seed"]] = r
    out = {"label": LABEL, "groups": []}
    for f in fractions:
        base = by.get(("C2", f), {})
        for c in conditions:
            g = by.get((c, f), {})
            if not g:
                continue
            seeds = sorted(g)
            sc = np.array([g[s]["final"]["score"] for s in seeds], float)
            ct = np.array([g[s]["final"]["confirmed_true"] for s in seeds], float)
            fc = np.array([g[s]["final"]["false_confirmations"] for s in seeds], float)
            rc = np.array([g[s]["final"]["recall_detected"] for s in seeds], float)
            ent = {"condition": c, "fraction": f, "n": len(seeds),
                   "score": [float(sc.mean()), float(sc.std(ddof=1)) if len(sc) > 1 else 0.0],
                   "confirmed_true": [float(ct.mean()), float(ct.std(ddof=1)) if len(ct) > 1
                                      else 0.0],
                   "false_confirmations": [float(fc.mean()), float(fc.std(ddof=1))
                                           if len(fc) > 1 else 0.0],
                   "recall_detected": [float(rc.mean()), float(rc.std(ddof=1)) if len(rc) > 1
                                       else 0.0],
                   "revisit_time_s": float(np.mean([g[s]["revisit_used"] for s in seeds])),
                   "decision_time_mean_s": float(np.mean([g[s]["decision_time_mean_s"]
                                                          for s in seeds])),
                   "decision_time_max_s": float(np.max([g[s]["decision_time_max_s"]
                                                        for s in seeds])),
                   "refusals": int(sum(len(g[s]["refusals"]) for s in seeds)),
                   "missed_lead": int(sum(g[s]["missed_lead"] for s in seeds)),
                   "aspects_mean": float(np.mean([g[s]["target_aspects_mean"] for s in seeds])),
                   "ece": ece([p for s in seeds for p in g[s].get("calib_pairs", [])])}
            any_m = {}
            for s in seeds:
                for a in g[s]["anytime"]:
                    any_m.setdefault(round(a["t"] / g[s]["B"], 2), []).append(a)
            ent["anytime"] = {str(k): {"confirmed_true": float(np.mean([a["confirmed_true"]
                                                                        for a in v])),
                                       "score": float(np.mean([a["score"] for a in v]))}
                              for k, v in sorted(any_m.items())}
            paired = [s for s in seeds if s in base]
            if c != "C2" and paired:
                d = np.array([g[s]["final"]["score"] - base[s]["final"]["score"] for s in paired])
                n = len(d)
                sd = float(d.std(ddof=1)) if n > 1 else 0.0
                half = 2.0 * sd / math.sqrt(n) if n > 1 else float("nan")   # ~95 % (t ~ 2)
                only_c = sum(len(set(g[s]["final"]["confirmed_ids"]) -
                                 set(base[s]["final"]["confirmed_ids"])) for s in paired)
                only_b = sum(len(set(base[s]["final"]["confirmed_ids"]) -
                                 set(g[s]["final"]["confirmed_ids"])) for s in paired)
                ent["paired_vs_C2"] = {"n": n, "mean_diff": float(d.mean()), "sd": sd,
                                       "ci95_approx": [float(d.mean() - half),
                                                       float(d.mean() + half)],
                                       "better": int((d > 0).sum()), "equal": int((d == 0).sum()),
                                       "worse": int((d < 0).sum()),
                                       "targets_confirmed_only_here": only_c,
                                       "targets_confirmed_only_by_C2": only_b}
            out["groups"].append(ent)
    return out


def write_markdown(summary, manifest, path):
    L = [f"# Dev-loop results — {manifest['name']}", "",
         f"**{LABEL}.** Development evidence only: never E4 (synthesis §6.2 ⚠, §4).",
         "Reference belief = a stand-in written from the briefs, NOT BlueBoat-Belief. World "
         "detection model = placeholders matched to the planner (not fitted from Gazebo).", "",
         f"Seeds: {manifest['seeds'][0]}-{manifest['seeds'][-1]} (n = {len(manifest['seeds'])}), "
         f"paired across conditions. Calibration: {manifest['calibration']['note']} "
         f"(a = {manifest['calibration']['a']:.3f}, b = {manifest['calibration']['b']:.3f}, "
         f"n = {manifest['calibration']['n']}).", "",
         "Score = confirmed true targets − κ · false confirmations (κ = 1), at the end of the "
         "budget. Paired differences vs C2 on the same seed; CI ≈ mean ± 2 sd/√n.", ""]
    fracs = sorted({g["fraction"] for g in summary["groups"]}, reverse=True)
    for f in fracs:
        L += [f"## Budget fraction {f:g} × B(C2)", "",
              "| Condition | n | score | confirmed true | false conf. | detected | Δ score vs C2 "
              "(mean ± sd) | ≈95 % CI | better/equal/worse | only here / only C2 | revisit s | "
              "decision s (mean/max) | ECE |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for g in summary["groups"]:
            if g["fraction"] != f:
                continue
            p = g.get("paired_vs_C2")
            pd_ = (f"{p['mean_diff']:+.2f} ± {p['sd']:.2f}" if p else "—")
            ci = (f"[{p['ci95_approx'][0]:+.2f}, {p['ci95_approx'][1]:+.2f}]" if p else "—")
            bew = (f"{p['better']}/{p['equal']}/{p['worse']}" if p else "—")
            oo = (f"{p['targets_confirmed_only_here']} / {p['targets_confirmed_only_by_C2']}"
                  if p else "—")
            L.append(f"| {g['condition']} | {g['n']} | {g['score'][0]:.2f} ± {g['score'][1]:.2f}"
                     f" | {g['confirmed_true'][0]:.2f} ± {g['confirmed_true'][1]:.2f}"
                     f" | {g['false_confirmations'][0]:.2f} ± {g['false_confirmations'][1]:.2f}"
                     f" | {g['recall_detected'][0]:.2f} | {pd_} | {ci} | {bew} | {oo}"
                     f" | {g['revisit_time_s']:.0f} | {g['decision_time_mean_s']:.2f}/"
                     f"{g['decision_time_max_s']:.2f} | {g['ece']:.3f} |")
        L.append("")
        L += ["Anytime (mean score at 25/50/75/100 % of this budget):", "",
              "| Condition | 25 % | 50 % | 75 % | 100 % |", "|---|---|---|---|---|"]
        for g in summary["groups"]:
            if g["fraction"] != f:
                continue
            a = g["anytime"]
            vals = [a.get(k, {}).get("score", float("nan")) for k in ("0.25", "0.5", "0.75", "1.0")]
            L.append(f"| {g['condition']} | " + " | ".join(f"{v:.2f}" for v in vals) + " |")
        L.append("")
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")


def plots(summary, outdir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    files = []
    fracs = sorted({g["fraction"] for g in summary["groups"]}, reverse=True)
    f0 = fracs[0]
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for g in summary["groups"]:
        if g["fraction"] != f0:
            continue
        ks = sorted(g["anytime"], key=float)
        ax.plot([float(k) * 100 for k in ks], [g["anytime"][k]["score"] for k in ks],
                marker="o", label=g["condition"])
    ax.set_xlabel("% of the budget used")
    ax.set_ylabel("mean score (confirmed true − κ false)")
    ax.set_title(f"Anytime score — {LABEL}", fontsize=9)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    p = os.path.join(outdir, "anytime.png")
    fig.tight_layout()
    fig.savefig(p, dpi=130)
    plt.close(fig)
    files.append(p)
    if len(fracs) > 1:
        fig, ax = plt.subplots(figsize=(7, 4.5))
        conds = sorted({g["condition"] for g in summary["groups"]})
        for c in conds:
            pts = sorted((g["fraction"], g["score"][0]) for g in summary["groups"]
                         if g["condition"] == c)
            ax.plot([a for a, _ in pts], [b for _, b in pts], marker="o", label=c)
        ax.set_xlabel("budget fraction of B(C2)")
        ax.set_ylabel("mean final score")
        ax.set_title(f"Budget sweep — {LABEL}", fontsize=9)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        p = os.path.join(outdir, "budget_sweep.png")
        fig.tight_layout()
        fig.savefig(p, dpi=130)
        plt.close(fig)
        files.append(p)
    return files


def merge_results(dirs, out, name, conds, fracs) -> int:
    """Summarise existing runs together (e.g. the f = 1.0 groups of one run with a budget
    sweep). Refuses runs whose frozen C2 settings or calibration differ. Only the seeds present
    in every requested (condition, fraction) group are kept, so every comparison stays paired."""
    mans, rows = [], []
    for d in dirs:
        with open(os.path.join(d, "manifest.json")) as fh:
            mans.append(json.load(fh))
        with open(os.path.join(d, "results.jsonl")) as fh:
            rows += [json.loads(line) for line in fh]
    keys = {(m["c2_tuned"]["sha256"], m["calibration"]["a"], m["calibration"]["b"],
             json.dumps(m["scenario"], sort_keys=True)) for m in mans}
    if len(keys) != 1:
        raise SystemExit(f"--from-results: runs differ in C2 freeze, calibration or scenario: "
                         f"{sorted(keys)}")
    rows = [r for r in rows if r["condition"] in conds and r["budget_fraction"] in fracs]
    groups = {}
    for r in rows:
        groups.setdefault((r["condition"], r["budget_fraction"]), set()).add(r["seed"])
    missing = [(c, f) for c in conds for f in fracs if (c, f) not in groups]
    if missing:
        raise SystemExit(f"--from-results: no rows for {missing}")
    seeds = sorted(set.intersection(*groups.values()))
    rows = [r for r in rows if r["seed"] in seeds]
    manifest = dict(mans[0], name=name, seeds=seeds, conditions=conds, fractions=fracs,
                    merged_from=[os.path.abspath(d) for d in dirs])
    with open(os.path.join(out, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=1)
    summary = summarise(rows, conds, fracs)
    with open(os.path.join(out, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    write_markdown(summary, manifest, os.path.join(out, "summary.md"))
    plots(summary, out)
    print(f"wrote {out}/summary.md (merged, n = {len(seeds)} paired seeds)")
    return 0


def git_rev(path):
    try:
        rev = subprocess.run(["git", "-C", path, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=10).stdout.strip()
        dirty = subprocess.run(["git", "-C", path, "status", "--porcelain"], capture_output=True,
                               text=True, timeout=10).stdout.strip()
        return rev + ("+uncommitted" if dirty else "")
    except Exception:  # noqa: BLE001 - provenance only
        return "unknown"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="devloop")
    ap.add_argument("--seeds", type=int, default=30)
    ap.add_argument("--seed0", type=int, default=2000)
    ap.add_argument("--conditions", default="C1,C2,C3,C4,C4_iso,C4_rand,C4_depth1,C4_degraded")
    ap.add_argument("--fractions", default="1.0")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1)
    ap.add_argument("--width", type=float, default=100.0)
    ap.add_argument("--height", type=float, default=60.0)
    ap.add_argument("--depth", type=float, default=2.0)
    ap.add_argument("--tune-c2", action="store_true",
                    help="run the C2 tuning protocol and freeze config/c2_tuned.yaml")
    ap.add_argument("--c2-tuned", default=None, help="frozen C2 settings (required for C3/C4)")
    ap.add_argument("--from-results", default=None,
                    help="comma-separated result dirs: summarise their rows (no new mission) "
                         "over --conditions, --fractions and the seeds common to every group")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    if a.from_results:
        return merge_results(a.from_results.split(","), a.out, a.name,
                             a.conditions.split(","), [float(v) for v in a.fractions.split(",")])
    sc_kw0 = {"width_m": a.width, "height_m": a.height, "depth_m": a.depth}
    if a.tune_c2:
        res = tune_c2(sc_kw0, a.jobs, os.path.join(a.out, "c2_tuned.yaml"),
                      os.path.join(a.out, "C2_TUNING_RESULT.md"))
        print(json.dumps(res, indent=1))
        return 0
    shared, c2_sha = {}, None
    if a.c2_tuned:
        import hashlib

        import yaml
        txt = open(a.c2_tuned).read()
        c2_sha = hashlib.sha256(txt.encode()).hexdigest()
        shared = dict(yaml.safe_load(txt)["c2_tuned"])
    conds = a.conditions.split(",")
    if any(c not in ("C1", "C2") for c in conds) and not a.c2_tuned:
        raise SystemExit("C2_TUNING_PROTOCOL §6: C3/C4 runs need the frozen C2 settings "
                         "(--c2-tuned config/c2_tuned.yaml)")
    fracs = [float(v) for v in a.fractions.split(",")]
    seeds = list(range(a.seed0, a.seed0 + a.seeds))
    sc_kw = {"width_m": a.width, "height_m": a.height, "depth_m": a.depth}
    man_path = os.path.join(a.out, "manifest.json")
    if os.path.exists(man_path):
        manifest = json.load(open(man_path))
        cal = manifest["calibration"]
    else:
        cal = calibrate(sc_kw, a.jobs, shared)
        manifest = {"name": a.name, "label": LABEL, "seeds": seeds, "conditions": conds,
                    "fractions": fracs, "scenario": sc_kw, "calibration": cal,
                    "calibration_seeds": CAL_SEEDS,
                    "module_revision": git_rev(os.path.dirname(os.path.abspath(__file__))),
                    "python": platform.python_version(), "numpy": np.__version__,
                    "belief_backend": "reference (NOT BlueBoat-Belief)",
                    "world": "eval/world.py WORLD_DEFAULTS (matched to planner placeholders)",
                    "c2_tuned": {"file": a.c2_tuned, "sha256": c2_sha, "settings": shared}}
        json.dump(manifest, open(man_path, "w"), indent=1)
    res_path = os.path.join(a.out, "results.jsonl")
    done = set()
    rows = []
    if os.path.exists(res_path):
        for line in open(res_path):
            r = json.loads(line)
            rows.append(r)
            done.add((r["seed"], r["condition"], r["budget_fraction"]))
    todo = [(s, c, f, cal, sc_kw, shared) for f in fracs for c in conds for s in seeds
            if (s, c, f) not in done]
    with Pool(a.jobs) as pool, open(res_path, "a") as fh:
        for r in pool.imap_unordered(_job, todo):
            r["c2_tuned_sha256"] = c2_sha
            fh.write(json.dumps(r) + "\n")
            fh.flush()
            rows.append(r)
            print(f"{r['condition']:12s} seed {r['seed']} f={r['budget_fraction']:g} "
                  f"score {r['final']['score']:+.0f} wall {r['wall_s']:.1f}s", flush=True)
    summary = summarise(rows, conds, fracs)
    json.dump(summary, open(os.path.join(a.out, "summary.json"), "w"), indent=1)
    write_markdown(summary, manifest, os.path.join(a.out, "summary.md"))
    plots(summary, a.out)
    print(f"wrote {a.out}/summary.md")


if __name__ == "__main__":
    sys.exit(main())
