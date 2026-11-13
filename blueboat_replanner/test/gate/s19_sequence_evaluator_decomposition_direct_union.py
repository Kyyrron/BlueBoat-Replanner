"""Gate section [19]: sequence evaluator: decomposition = direct union value (MATH §10.4).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [19]
print("[19] sequence evaluator: decomposition = direct union value (MATH §10.4)")
from blueboat_replanner.core.sequence import Evaluator  # noqa: E402
from blueboat_replanner.core.looks import (Looks, plan_looks, union,  # noqa: E402
                                           looks_from_poses, plan_poses)
gS = make_grid([contact(1, 40, 30, first_sector=0), contact(2, 80, 40, first_sector=4)])
rngS = np.random.default_rng(19)
gS["coverage"] = np.clip(rngS.uniform(0, 0.9, gS["coverage"].size), 0, 1)
set_searched(gS, mask=rngS.uniform(size=(gS["coverage"].size, 12)) < 0.15)
gS["p_background"] = np.full(gS["coverage"].size, 0.001)
snapS = validate_grid(gS)
idx_dp = last_pass1_line(cov.c2)
kS = Plan(cov.c2.segments[idx_dp + 1:], 0.0, CFG.speed_mps)
Brem = kS.duration * 0.8
evS = Evaluator(CFG, snapS, RECT, kS, Looks.empty(), Brem)
worst = 0.0
for k in range(6):
    st = cov.c2.segments[idx_dp].end_pose()
    gl = (float(rngS.uniform(20, 100)), float(rngS.uniform(15, 45)), float(rngS.uniform(-3, 3)))
    pr = connect(st, gl, RECT, CFG)
    if not pr:
        continue
    from blueboat_replanner.core.plan import Segment as _Seg  # noqa: E402
    dp_ = Plan([_Seg.make("transit", pr)], 0.0, CFG.speed_mps)
    pl = evS.piece_looks(dp_, 0.0)
    Tp = dp_.duration
    fast = evS.sequence_value(pl, Tp)["J"]
    cut = evS.cut_of(evS.tail(0)["plan"], Brem - Tp)
    Kl = evS.tail(0)["looks"]
    m = Kl.t <= cut + 1e-9
    direct = evS.union_value(union(pl, Looks(Kl.cell[m], Kl.g[m], Kl.pd[m], Kl.t[m] + Tp)))
    worst = max(worst, abs(fast - direct))
check("J by decomposition = direct union value (6 random detours)", worst < 1e-9, f"{worst}")
cont0 = evS.sequence_value(Looks.empty(), 0.0)["J"]
Kl = evS.tail(0)["looks"]
cut0 = evS.cut_of(evS.tail(0)["plan"], Brem)
m0 = Kl.t <= cut0 + 1e-9
check("B-3: CONTINUE with no detour = value of K truncated alone",
      abs(cont0 - evS.union_value(Looks(Kl.cell[m0], Kl.g[m0], Kl.pd[m0], Kl.t[m0]))) < 1e-9)
evP = Evaluator(CFG, snapS, RECT, kS, pl, Brem)
check("C-5: looks that are all pending have zero value",
      abs(evP.union_value(pl)) < 1e-15 and evS.union_value(pl) > 0)

