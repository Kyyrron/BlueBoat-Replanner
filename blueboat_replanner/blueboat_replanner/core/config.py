"""Validated configuration (MATH.md §15).

Every parameter has a unit, a category (MATH §15), a swept flag and a source. Unknown keys
raise, wrong types raise, and out-of-range values raise, each naming the key. The planner
class table (MATH §6) is a separate, validated block.

Categories: U user decision · R repository default or fact · L literature-motivated ·
P placeholder · D design choice.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping


class ConfigError(ValueError):
    """Invalid configuration; the message names the key."""


@dataclass(frozen=True)
class ParamSpec:
    default: Any
    kind: type
    unit: str
    category: str
    source: str
    lo: float | None = None
    hi: float | None = None
    swept: bool = False
    choices: tuple | None = None
    optional: bool = False          # None allowed


def _p(default, kind, unit, category, source, lo=None, hi=None, swept=False,
       choices=None, optional=False):
    return ParamSpec(default, kind, unit, category, source, lo, hi, swept, choices, optional)


# name -> spec. Order follows MATH §15.
PARAMS: Mapping[str, ParamSpec] = MappingProxyType({
    # vehicle and sonar
    "speed_mps": _p(0.5, float, "m/s", "U", "user 2026-11-04", 0.05, 2.0),
    "range_m": _p(20.0, float, "m", "U", "synthesis §8.5; brief Q8", 1.0, 100.0),
    "sensor_depth_m": _p(0.22, float, "m", "R", "root CLAUDE.md §4.3", 0.0, 2.0),
    "planning_depth_m": _p(None, float, "m", "U", "per mission (MATH §2.3)", 0.0, 200.0,
                           optional=True),
    # coverage band and spacing (MATH §2)
    "grazing_floor": _p(False, bool, "-", "U", "brief §3 (off); NOAA HSSD 2025.0.01 §8.3 (option)",
                        swept=True),
    "grazing_ratio": _p(0.08, float, "-", "L", "NOAA HSSD 2025.0.01 §8.3", 0.01, 0.5),
    "r_near_m": _p(6.0, float, "m", "P", "brief §3", 0.0, 50.0, swept=True),
    "spacing_factor": _p(0.8, float, "-", "R", "synthesis §8.5", 0.1, 1.0, swept=True),
    "spacing_check_step_m": _p(0.05, float, "m", "D", "MATH §2.2", 1e-3, 1.0),
    "spacing_bisect_iterations": _p(40, int, "-", "D", "MATH §2.2", 1, 200),
    # geometry (MATH §3)
    "margin_m": _p(2.0, float, "m", "P", "brief Q9", 0.0, 50.0),
    "area_check_slack_m": _p(0.5, float, "m", "U", "owner 2026-11-13; MATH §3.6", 0.0, 50.0),
    "r_turn_m": _p(3.0, float, "m", "P", "E-23 (field 1.5-6 m)", 0.5, 50.0, swept=True),
    "t_settle_s": _p(2.0, float, "s", "P", "brief §8.5", 0.0, 60.0),
    "geom_step_m": _p(0.05, float, "m", "D", "MATH §3", 1e-3, 1.0),
    "pass1_direction": _p("long_axis", str, "-", "D", "MATH §3.2; C2 protocol",
                          choices=("long_axis", "long_axis_plus_90")),
    "interlace": _p("auto", str, "-", "D", "MATH §3.2; C2 protocol", choices=("auto", "off")),
    "connection_retries": _p(10, int, "-", "D", "MATH §3.2", 0, 100),
    "connection_shorten_m": _p(0.5, float, "m", "D", "MATH §3.2", 0.01, 10.0),
    "min_leg_m": _p(1.0, float, "m", "D", "MATH §3.2", 0.0, 100.0),
    # sampling (MATH §4)
    "sample_ds_m": _p(0.25, float, "m", "R", "MCS designer sampling.py l.66", 0.01, 5.0),
    "max_samples_per_request": _p(19000, int, "-", "D", "Control cap 20 000 (E-08)", 2, 20000),
    # planner sensor model (MATH §6)
    "pd_plateau": _p(0.7, float, "-", "P", "brief §8.2 (Belief's default)", 0.0, 1.0, swept=True),
    "pd_r_lo_m": _p(3.0, float, "m", "P", "brief §8.2", 0.0, 100.0),
    "pd_r_hi_m": _p(25.0, float, "m", "P", "brief §8.2", 0.0, 200.0),
    "aspect_agnostic": _p(False, bool, "-", "D", "brief §4.5 (C4_iso)"),
    # look geometry shared with Belief (MATH §5)
    "lever_arm_m": _p(0.285, float, "m", "R", "Belief topics.md §4 item 2 (processor offset)",
                      0.0, 2.0),
    "turn_gate": _p(True, bool, "-", "R", "Belief MATH §4 item 6 (fallback when not published)"),
    "turn_min_yaw_rate_rad_s": _p(0.01, float, "rad/s", "R",
                                  "Belief turn.min_yaw_rate_rad_s (fallback when not published)",
                                  1e-6, 10.0),
    # contact and value (MATH §7-§9)
    "clutter_f": _p(0.2, float, "-", "P", "[OPEN] O-5", 0.0, 1.0, swept=True),
    "kappa": _p(1.0, float, "-", "U", "brief §5 Q1", 0.0, 100.0, swept=True),
    "lam": _p(0.5, float, "-", "U", "brief §5 Q1", 0.0, 100.0, swept=True),
    "contact_exclusion_sigma": _p(3.0, float, "-", "D", "MATH §9", 0.0, 10.0),
    "calibration_iterations": _p(60, int, "-", "D", "MATH §9.1", 10, 200),
    # candidates (MATH §10)
    "r_star_m": _p(10.0, float, "m", "U", "brief §5 Q7 (placeholder)", 0.5, 100.0, swept=True),
    "l_min_m": _p(10.0, float, "m", "P", "MATH §10.2", 0.0, 200.0, swept=True),
    "revisit_depth_radius_m": _p(10.0, float, "m", "D", "MATH §2.3", 0.0, 100.0),
    # selection (MATH §11)
    "k1": _p(5, int, "-", "D", "MATH §11.2", 1, 100),
    "k2": _p(5, int, "-", "D", "MATH §11.4", 0, 100),
    "sector_prefilter_k": _p(3, int, "-", "D", "MATH §11.2", 1, 64),
    "epsilon_j": _p(0.02, float, "exp. targets", "P", "MATH §11", 0.0, 10.0, swept=True),
    "lookahead_depth": _p(2, int, "-", "D", "brief §8.5 (C4_depth1 = 1)", 1, 2),
    "t_lat_s": _p(10.0, float, "s", "P", "MATH §11.3 (fallback when the stamp cannot be used)",
                  0.0, 120.0),
    "contact_guard_m": _p(1.0, float, "m", "D", "MATH §11.3 (two cells)", 0.0, 20.0),
    "tracking_lag_m": _p(3.0, float, "m", "R", "Control gov_Lmax (E-08); MATH §11.3", 0.0, 50.0),
    "clock_tolerance_s": _p(1.0, float, "s", "D", "MATH §11.3", 0.0, 60.0),
    "horizon_max_lag_s": _p(120.0, float, "s", "D", "MATH §11.3", 1.0, 3600.0),
    "area_tolerance_m": _p(0.05, float, "m", "R", "Belief area.CORNER_TOL_M; DESIGN §4",
                           0.0, 5.0),
    # rules (MATH §12)
    "p_res": _p(0.05, float, "-", "P", "brief §8.6", 0.0, 1.0, swept=True),
    "n_max": _p(3, int, "-", "P", "brief §8.6", 0, 100, swept=True),
    "delta": _p(0.02, float, "-", "P", "brief §8.6", 0.0, 1.0, swept=True),
    "rho_rev": _p(0.5, float, "-", "P", "brief §8.6", 0.0, 1.0, swept=True),
    "rho_floor": _p(0.0, float, "1/s", "P", "MATH §12", 0.0, 10.0),
    # timing and execution (MATH §15, DESIGN §5.2)
    "t_decide_s": _p(2.0, float, "s", "D", "brief §9.1", 0.0, 60.0),
    "latency_margin_s": _p(2.0, float, "path s", "D", "DESIGN §5.2", 0.0, 60.0),
    "answer_timeout_s": _p(5.0, float, "s", "D", "DESIGN §5.2", 0.1, 120.0),
    "reconcile_timeout_s": _p(10.0, float, "s", "D", "DESIGN §5.2", 0.1, 300.0),
    "belief_max_age_s": _p(5.0, float, "s", "D", "contract C-2 (~1 Hz)", 0.1, 600.0),
    "progress_max_age_s": _p(1.0, float, "s", "D", "E-11 (10 Hz)", 0.05, 60.0),
    "progress_send_max_age_s": _p(0.2, float, "s", "D", "DESIGN §5.2", 0.01, 10.0),
    "odom_max_age_s": _p(1.0, float, "s", "D", "DESIGN §8", 0.05, 60.0),
    "transform_jump_m": _p(0.5, float, "m", "D", "DESIGN §4", 0.0, 100.0),
    "start_timeout_s": _p(60.0, float, "s", "D", "DESIGN §7", 1.0, 3600.0),
    "stop_retries": _p(3, int, "-", "D", "DESIGN §5.3", 0, 20),
    "stop_hold_s": _p(1.0, float, "path s", "D", "DESIGN §5.3", 0.01, 60.0),
})

# Control's request rules (E-08), mirrored for offline validation. These are facts of the
# checked-out Control (`path_update.py` l.46-55, Config l.119-128), not tunable here.
CONTROL_RULES = MappingProxyType({
    "max_points": 20000,
    "min_sample_dt": 1e-3,
    "max_parameter": 1_000_000.0,
    "max_speed": 2.0,
    "hold_speed": 0.05,
    "min_segment_length": 1e-3,
    "heading_coherence": math.radians(45.0),
    "pos_tol": 0.20,
    "heading_tol": math.radians(10.0),
    "gov_Lmax": 3.0,
    "first_path_t0_tol": 1e-6,
    # governor window spacing h = path_time / (path_steps - 1) at Control's defaults (0.05 s,
    # 2 steps; E-06, E-07): the heading check is skipped where the old path is stationary over h
    "gov_h": 0.05,
})


@dataclass(frozen=True)
class ClassModel:
    """Planner aspect model of one class (MATH §6): m-fold, half-width w, floor a, scale s."""

    m: int
    w_rad: float
    a: float
    s: float

    def validate(self, name: str) -> None:
        if self.m < 0:
            raise ConfigError(f"classes.{name}.m must be >= 0, got {self.m}")
        if not (0.0 < self.s <= 1.0):
            raise ConfigError(f"classes.{name}.s must be in (0, 1], got {self.s}")
        if not (0.0 <= self.a <= 1.0):
            raise ConfigError(f"classes.{name}.a must be in [0, 1], got {self.a}")
        if self.m > 0 and not (0.0 < self.w_rad <= math.pi / self.m):
            raise ConfigError(f"classes.{name}.w_deg must be in (0, 180/m], got "
                              f"{math.degrees(self.w_rad):.3g}")


DEFAULT_CLASSES: Mapping[str, ClassModel] = MappingProxyType({
    # brief §8.2 placeholders (category P)
    "block": ClassModel(4, math.radians(20.0), 0.4, 1.0),
    "pipe": ClassModel(2, math.radians(10.0), 0.1, 1.0),
    "cage": ClassModel(4, math.radians(25.0), 0.3, 0.9),
    "tyre": ClassModel(0, 0.0, 1.0, 0.6),
    "plate": ClassModel(0, 0.0, 1.0, 0.3),
})


def _coerce(name: str, spec: ParamSpec, value: Any) -> Any:
    if value is None:
        if spec.optional:
            return None
        raise ConfigError(f"{name} must not be null")
    if spec.kind is bool:
        if not isinstance(value, bool):
            raise ConfigError(f"{name} must be a bool, got {value!r}")
        return value
    if spec.kind is int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigError(f"{name} must be an integer, got {value!r}")
    elif spec.kind is float:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ConfigError(f"{name} must be a number, got {value!r}")
        value = float(value)
        if not math.isfinite(value):
            raise ConfigError(f"{name} must be finite, got {value!r}")
    elif spec.kind is str:
        if not isinstance(value, str):
            raise ConfigError(f"{name} must be a string, got {value!r}")
        if spec.choices and value not in spec.choices:
            raise ConfigError(f"{name} must be one of {spec.choices}, got {value!r}")
        return value
    if spec.lo is not None and value < spec.lo:
        raise ConfigError(f"{name} must be >= {spec.lo}, got {value}")
    if spec.hi is not None and value > spec.hi:
        raise ConfigError(f"{name} must be <= {spec.hi}, got {value}")
    return value


def _parse_classes(raw: Any) -> dict:
    if not isinstance(raw, Mapping):
        raise ConfigError("classes must be a mapping name -> {m, w_deg, a, s}")
    out = {}
    for name, entry in raw.items():
        if not isinstance(entry, Mapping):
            raise ConfigError(f"classes.{name} must be a mapping")
        unknown = set(entry) - {"m", "w_deg", "a", "s"}
        if unknown:
            raise ConfigError(f"classes.{name}: unknown keys {sorted(unknown)}")
        try:
            cm = ClassModel(int(entry.get("m", 0)), math.radians(float(entry.get("w_deg", 0.0))),
                            float(entry.get("a", 1.0)), float(entry.get("s", 1.0)))
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"classes.{name}: {exc}") from None
        cm.validate(name)
        out[str(name)] = cm
    return out


def _parse_class_families(raw: Any, classes: Mapping) -> dict:
    """`class_families`: Belief class name -> aspect family (a key of `classes`). MATH §6."""
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise ConfigError("class_families must be a mapping: Belief class name -> family")
    out = {}
    for name, fam in raw.items():
        if not isinstance(name, str) or not isinstance(fam, str):
            raise ConfigError(f"class_families.{name}: name and family must be strings")
        if fam not in classes:
            raise ConfigError(f"class_families.{name}: family {fam!r} is not in classes "
                              f"{sorted(classes)}")
        out[name] = fam
    return out


def _parse_bearing_table(raw: Any) -> tuple:
    if raw is None:
        return ()
    if not isinstance(raw, (list, tuple)) or not all(
            isinstance(v, int) and not isinstance(v, bool) and v >= 0 for v in raw):
        raise ConfigError("c4_rand.bearing_table must be a list of non-negative integers "
                          "(folded sector indices)")
    return tuple(raw)


class Config:
    """Immutable, validated parameter set. Attribute access by parameter name."""

    __slots__ = ("_values", "classes", "class_families", "bearing_table", "_overrides")

    def __init__(self, values: Mapping[str, Any], classes: Mapping[str, ClassModel],
                 bearing_table: tuple, overrides: Mapping[str, Any],
                 class_families: Mapping[str, str] | None = None):
        object.__setattr__(self, "_values", MappingProxyType(dict(values)))
        object.__setattr__(self, "classes", MappingProxyType(dict(classes)))
        object.__setattr__(self, "class_families", MappingProxyType(dict(class_families or {})))
        object.__setattr__(self, "bearing_table", tuple(bearing_table))
        object.__setattr__(self, "_overrides", MappingProxyType(dict(overrides)))

    def __getattr__(self, name):
        try:
            return self._values[name]
        except KeyError:
            raise AttributeError(name) from None

    def __setattr__(self, name, value):
        raise AttributeError("Config is immutable; use Config.with_overrides")

    @classmethod
    def from_dict(cls, overrides: Mapping[str, Any] | None = None) -> "Config":
        overrides = dict(overrides or {})
        values = {k: spec.default for k, spec in PARAMS.items()}
        classes = dict(DEFAULT_CLASSES)
        table = ()
        families_raw = None
        for key, val in overrides.items():
            if key == "classes":
                classes = _parse_classes(val)
            elif key == "class_families":
                families_raw = val
            elif key == "c4_rand":
                if not isinstance(val, Mapping) or set(val) - {"bearing_table"}:
                    raise ConfigError("c4_rand must be a mapping with only 'bearing_table'")
                table = _parse_bearing_table(val.get("bearing_table"))
            elif key in PARAMS:
                values[key] = _coerce(key, PARAMS[key], val)
            else:
                raise ConfigError(f"unknown configuration key {key!r}")
        if values["pd_r_hi_m"] < values["pd_r_lo_m"]:
            raise ConfigError("pd_r_hi_m must be >= pd_r_lo_m")
        families = _parse_class_families(families_raw, classes)
        return cls(values, classes, table, overrides, families)

    def with_overrides(self, more: Mapping[str, Any]) -> "Config":
        merged = dict(self._overrides)
        merged.update(more)
        return Config.from_dict(merged)

    def as_dict(self) -> dict:
        """Effective configuration (for manifests), JSON-friendly."""
        out = dict(self._values)
        out["classes"] = {k: {"m": v.m, "w_deg": math.degrees(v.w_rad), "a": v.a, "s": v.s}
                          for k, v in self.classes.items()}
        out["class_families"] = dict(self.class_families)
        out["c4_rand"] = {"bearing_table": list(self.bearing_table)}
        return out

    @staticmethod
    def categories() -> dict:
        return {k: s.category for k, s in PARAMS.items()}
