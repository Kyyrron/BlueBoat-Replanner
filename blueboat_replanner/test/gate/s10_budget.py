"""Gate section [10]: budget.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [10]
print("[10] budget")
setup = setup_mission(CFG, "C2", [[(34.5, 133.5), (34.5, 133.5013), (34.50054, 133.5013),
                                  (34.50054, 133.5)]], 34.5, 133.5, (10.0, 10.0, 0.0))
check("B = C2 plan duration (incl. the initial manoeuvre)",
      setup.budget.B == setup.coverage.c2.duration and setup.coverage.c2.segments[0].kind ==
      "initial")
check("C1 shorter than B", setup.coverage.c1.duration < setup.budget.B)
check("area built from WGS-84 through the datum", abs(setup.area.area - 120.3 * 60.0) /
      (120 * 60) < 0.01, f"{setup.area.area}")

