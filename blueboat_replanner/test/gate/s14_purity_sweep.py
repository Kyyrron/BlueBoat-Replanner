"""Gate section [14]: purity sweep.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [14]
print("[14] purity sweep")
pk = PKG_ROOT / "blueboat_replanner"
bad_imports = []
for f in list((pk / "core").rglob("*.py")) + list((pk / "io").rglob("*.py")) + \
        [pk / "ros" / "adapters.py"]:
    txt = f.read_text()
    for pat in ("import rclpy", "from rclpy", "numpy.random", "np.random", "import random",
                "import time", "from time", "datetime"):
        if pat in txt:
            bad_imports.append(f"{f.name}:{pat}")
    if f.parent.name in ("core", "policy") and ("import yaml" in txt):
        bad_imports.append(f"{f.name}:yaml")
check("core/io/adapters: no rclpy, randomness, clock; core: no yaml", not bad_imports,
      f"{bad_imports}")
forbidden = ("/sss_ai", "ground_truth", "/sim/", "/side_scan_sonar", "/sss_processor")
hits = []
for f in list((pk / "core").rglob("*.py")) + list((pk / "ros").rglob("*.py")):
    txt = f.read_text()
    hits += [f"{f.name}:{p}" for p in forbidden if p in txt]
check("online code names no detection, sonar or simulation topic (CM-9)", not hits, f"{hits}")
check("importing the core loaded no rclpy", "rclpy" not in sys.modules)

