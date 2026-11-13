"""Gate section [21]: Control stub vs Control's PathUpdater.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [21]
print("[21] Control stub vs Control's PathUpdater")
from blueboat_replanner.eval.control_stub import ControlStub  # noqa: E402
stub = ControlStub((float(x[0]), float(y[0]), 0.0))
a1 = stub.handle("r1-0000", float("nan"), t, x, y, w, 0.0)
a2 = stub.handle("r1-0001", c, t2, x2, y2, w2, c - 20.0)
a3 = stub.handle("r1-0002", c + 30.0, t2, x2 + 0.5, y2, w2, c - 20.0)
a4 = stub.handle("r1-0003", c + 1.0, t2, x2, y2, w2, c - 1.0)
a5 = stub.handle("r1-0001", c + 40.0, t2, x2, y2, w2, c - 20.0)
check("stub: first path, tail ok; offset -> discontinuous_position; lead_too_short; duplicate",
      a1["accepted"] and a2["accepted"] and a3["reason"] == "discontinuous_position" and
      a4["reason"] == "lead_too_short" and a5["reason"] == "duplicate_id")
mir_before = list(stub.mirror.segments)
stub.handle("r1-0009", c + 30.0, t2, x2 + 0.5, y2, w2, c - 20.0)
check("stub: a refusal leaves the path unchanged", stub.mirror.segments == mir_before)
if lib is not None and (lib / "path_update.py").is_file():
    import path_update as pu  # noqa: E402
    upd = pu.PathUpdater()
    ctx0 = pu.Context(ready=True, path_source="replanner", tau=0.0)
    upd.reset(ctx0, "launch")
    upd.set_hold((float(x[0]), float(y[0]), 0.0))
    reqs = [("r1-0000", float("nan"), (t, x, y, w), 0.0),
            ("r1-0001", c, (t2, x2, y2, w2), c - 20.0),
            ("r1-0002", c + 30.0, (t2, x2 + 0.5, y2, w2), c - 20.0),
            ("r1-0003", c + 1.0, (t2, x2, y2, w2), c - 1.0),
            ("r1-0001", c + 40.0, (t2, x2, y2, w2), c - 20.0)]
    stub2 = ControlStub((float(x[0]), float(y[0]), 0.0))
    agree = True
    for rid, cc, arr, tau_ in reqs:
        theirs, _, _ = upd.handle(pu.Request(rid, cc, "world", *arr),
                                  pu.Context(ready=True, path_source="replanner", tau=tau_))
        ours = stub2.handle(rid, cc, *arr, tau_)
        agree &= theirs["accepted"] == ours["accepted"] and theirs["reason"] == ours["reason"]
    check("stub == Control PathUpdater on the same 5 requests (accept / refusal codes)", agree)
else:
    skip("stub vs Control PathUpdater", "BlueBoat-Control not found")

