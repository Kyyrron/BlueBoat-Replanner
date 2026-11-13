"""Gate section [15]: mission file and trajectory archive.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [15]
print("[15] mission file and trajectory archive")
M = {"mission": {"name": "t1", "condition": "C3",
                 "areas": [[[34.5, 133.5], [34.5, 133.5013], [34.50054, 133.5013],
                            [34.50054, 133.5]]], "config": {"planning_depth_m": 2.0}}}
check("valid mission parses", parse_mission(M).condition == "C3")


def mut_mission(fn):
    import copy
    m = copy.deepcopy(M)
    fn(m["mission"])
    return lambda: parse_mission(m)


check("unknown key raises", raises(mut_mission(lambda m: m.update(extra=1)), MissionError))
check("two areas -> [OPEN] O-8", raises(mut_mission(lambda m: m["areas"].append(
    m["areas"][0])), MissionError, "O-8"))
check("missing planning depth raises", raises(mut_mission(lambda m: m.update(config={})),
                                              MissionError, "planning_depth_m"))
check("bad condition raises", raises(mut_mission(lambda m: m.update(condition="C9")),
                                     MissionError))
check("C4_rand without table raises", raises(mut_mission(lambda m: m.update(condition="C4_rand")),
                                             MissionError, "bearing_table"))
check("unknown config key raises", raises(mut_mission(lambda m: m["config"].update(foo=1)),
                                          MissionError, "foo"))
with tempfile.TemporaryDirectory() as td:
    doc = trajectory_doc("p", t, x, y, w, 34.5, 133.5)
    p1 = write_exclusive(os.path.join(td, "plan.yaml"), doc)
    p2 = write_exclusive(os.path.join(td, "plan.yaml"), doc)
    import yaml
    back = yaml.safe_load(open(p1))
    pts = np.asarray(back["points"])
    check("archive: never overwrites (-2 suffix)", p2.endswith("plan-2.yaml"))
    check("archive: Control loader's format (format key, N x 4, increasing t)",
          back["format"] == "blueboat_trajectory/1" and pts.shape[1] == 4 and
          bool(np.all(np.diff(pts[:, 0]) > 0)))
from blueboat_replanner.io.mission import (MISSION_FILE_CONDITIONS,  # noqa: E402
                                           load_c2_freeze)
for c_ in ("C4_live", "C4_degraded"):
    check(f"{c_}: refused in a mission file (a dev-loop condition; it would run as plain C4)",
          raises(mut_mission(lambda m, c_=c_: m.update(condition=c_)), MissionError,
                 "dev-loop condition"))
check("the online conditions parse, C4_depth1 included (decision 7: timed in Gazebo)",
      all(parse_mission({"mission": {**M["mission"], "condition": c_,
                                     "config": dict(M["mission"]["config"],
                                                    c4_rand={"bearing_table": [0, 1]})}}
                        ).condition == c_ for c_ in MISSION_FILE_CONDITIONS))
FZ = load_c2_freeze(str(PKG_ROOT / "config" / "c2_tuned.yaml"))
import hashlib as _hl  # noqa: E402
check("the C2 freeze file: settings and sha256 of config/c2_tuned.yaml",
      FZ["settings"] == _yaml.safe_load((PKG_ROOT / "config" / "c2_tuned.yaml").read_text()
                                        )["c2_tuned"] and
      FZ["sha256"] == _hl.sha256((PKG_ROOT / "config" / "c2_tuned.yaml").read_bytes()
                                 ).hexdigest())
mF = parse_mission(M, {"spacing_factor": 0.6, "r_near_m": 5.0}, FZ)
check("the freeze is applied over the node's configuration; other keys stay",
      mF.config.spacing_factor == FZ["settings"]["spacing_factor"] and
      mF.config.r_near_m == 5.0 and mF.freeze is FZ)
check("a mission setting a frozen key to another value is refused, naming it",
      raises(mut_mission(lambda m: m["config"].update(spacing_factor=0.8)), MissionError)
      is False and  # without a freeze it is an ordinary override
      raises(lambda: parse_mission({"mission": {**M["mission"],
                                                "config": dict(M["mission"]["config"],
                                                               spacing_factor=0.8)}}, None, FZ),
             MissionError, "spacing_factor"))
check("the same value as the freeze is accepted",
      parse_mission({"mission": {**M["mission"], "config": dict(
          M["mission"]["config"], spacing_factor=FZ["settings"]["spacing_factor"])}}, None,
          FZ).config.spacing_factor == FZ["settings"]["spacing_factor"])
check("config/mission_example.yaml parses with the shipped freeze",
      parse_mission(_yaml.safe_load((PKG_ROOT / "config" / "mission_example.yaml").read_text()),
                    None, FZ).config.spacing_factor == FZ["settings"]["spacing_factor"])

