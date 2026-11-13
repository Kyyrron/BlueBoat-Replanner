"""Gate section [12]: C1 / C2 policies.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [12]
print("[12] C1 / C2 policies")
for cond in ("C1", "C2"):
    p = policy_for(cond)
    pl = p.initial_plan(cov)
    check(f"{cond}: no decision points (one request, never changed)", p.decision_points(pl) == [])
check("C2 initial plan is the two-pass plan", policy_for("C2").initial_plan(cov) is cov.c2)

