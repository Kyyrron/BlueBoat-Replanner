"""Gate section [1]: configuration.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [1]
print("[1] configuration")
check("defaults load", CFG.speed_mps == 0.5 and CFG.range_m == 20.0 and CFG.grazing_floor is False)
check("every parameter has a category and unit",
      all(s.category in "URLPD" and len(s.category) == 1 and s.unit for s in PARAMS.values()))
check("unknown key raises, naming it",
      raises(lambda: Config.from_dict({"speed": 1.0}), ConfigError, "speed"))
check("wrong type raises", raises(lambda: Config.from_dict({"n_max": 2.5}), ConfigError, "n_max"))
check("bool is not an int", raises(lambda: Config.from_dict({"k1": True}), ConfigError, "k1"))
check("out of range raises", raises(lambda: Config.from_dict({"spacing_factor": 1.5}),
                                    ConfigError, "spacing_factor"))
check("non-finite raises", raises(lambda: Config.from_dict({"margin_m": float("nan")}),
                                  ConfigError, "margin_m"))
check("choices enforced", raises(lambda: Config.from_dict({"interlace": "x"}), ConfigError))
import yaml as _yaml  # noqa: E402 - io/ dependency, here only to read the shipped defaults
_ycfg = _yaml.safe_load((PKG_ROOT / "config" / "replanner.yaml").read_text())["replanner"]
check("config/replanner.yaml lists every parameter with its default",
      {k: v for k, v in _ycfg.items() if k not in ("classes", "class_families")} ==
      {k: sp.default for k, sp in PARAMS.items()} and
      Config.from_dict(_ycfg).as_dict() == Config.from_dict({}).as_dict())
check("class_families: empty by default; a family must be a key of classes",
      dict(CFG.class_families) == {} and
      Config.from_dict({"class_families": {"pipe_pvc": "pipe"}}).class_families["pipe_pvc"] ==
      "pipe" and raises(lambda: Config.from_dict({"class_families": {"pipe_pvc": "nope"}}),
                        ConfigError, "class_families"))
check("class s must be in (0, 1]",
      raises(lambda: Config.from_dict({"classes": {"x": {"m": 2, "w_deg": 10, "a": 0.1,
                                                         "s": 1.2}}}), ConfigError, "classes.x"))
check("class table override", Config.from_dict(
    {"classes": {"x": {"m": 0, "s": 0.5}}}).classes["x"].s == 0.5)
check("c4_rand table parsed", Config.from_dict({"c4_rand": {"bearing_table": [1, 0, 5]}}
                                               ).bearing_table == (1, 0, 5))
check("immutable", raises(lambda: setattr(CFG, "speed_mps", 1.0), AttributeError))
check("as_dict round trip", Config.from_dict({k: v for k, v in CFG.as_dict().items()}).as_dict()
      == CFG.as_dict())

