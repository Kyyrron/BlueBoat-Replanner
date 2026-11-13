"""Mission file (brief §6.4). Unknown keys raise; the config block is validated by Config.

    mission:
      name: beach_2026_10_12
      condition: C3                     # one of MISSION_FILE_CONDITIONS
      areas: [[[lat, lon], ...]]        # exactly one simple polygon (O-8), WGS-84
      config: {planning_depth_m: 2.0}   # overrides of config/replanner.yaml, same keys

The configuration is layered: `config/replanner.yaml` (the node's `config_file`) < the C2 freeze
(`config/c2_tuned.yaml`, C2_TUNING_PROTOCOL §6) < the mission's `config`. A mission that sets a
frozen key to another value is refused: every condition flies the frozen coverage plan, as in
the dev loop (DESIGN §5.1).
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass

from ..core.areas import MissionError
from ..core.config import Config

CONDITIONS = ("C1", "C2", "C3", "C4", "C4_iso", "C4_rand", "C4_live", "C4_degraded",
              "C4_depth1")
# what a mission file may name. C4_live (interruption: a flag nothing reads) and C4_degraded
# (its planner block is set by eval/ only) would run online as plain C4 (DESIGN §6)
MISSION_FILE_CONDITIONS = ("C1", "C2", "C3", "C4", "C4_iso", "C4_rand", "C4_depth1")
NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


@dataclass(frozen=True)
class Mission:
    name: str
    condition: str
    areas: tuple            # tuple of tuples of (lat, lon)
    config: Config
    raw_config: dict
    freeze: dict | None = None             # the C2 freeze applied: {file, sha256, settings}
    # dev loop only (eval/devloop.py); a mission file cannot set them
    planner_config: Config | None = None   # the policy's configuration (C4_degraded's block)
    budget_fraction: float = 1.0           # a mission cut short: B = fraction * B_C2


def load_c2_freeze(path: str) -> dict:
    """The C2 freeze file (`config/c2_tuned.yaml`, written by `eval/runner.py --tune-c2`):
    {file, sha256, settings}. Its settings must be valid configuration keys."""
    import yaml  # lazily: io/ only
    with open(path, "rb") as fh:
        raw = fh.read()
    doc = yaml.safe_load(raw.decode())
    settings = doc.get("c2_tuned") if isinstance(doc, dict) else None
    if not isinstance(settings, dict) or not settings:
        raise MissionError(f"{path}: no 'c2_tuned' settings")
    try:
        Config.from_dict(dict(settings))
    except ValueError as exc:
        raise MissionError(f"{path}: {exc}") from None
    return {"file": str(path), "sha256": hashlib.sha256(raw).hexdigest(),
            "settings": dict(settings)}


def parse_mission(doc, base_overrides: dict | None = None,
                  freeze: dict | None = None) -> Mission:
    """`freeze`: the C2 freeze of `load_c2_freeze` (the node always passes one)."""
    if not isinstance(doc, dict) or set(doc) != {"mission"}:
        raise MissionError("mission file: top level must be exactly {'mission': {...}}")
    m = doc["mission"]
    if not isinstance(m, dict):
        raise MissionError("mission: must be a mapping")
    unknown = set(m) - {"name", "condition", "areas", "config"}
    if unknown:
        raise MissionError(f"mission: unknown keys {sorted(unknown)}")
    name = m.get("name")
    if not isinstance(name, str) or not NAME_RE.match(name):
        raise MissionError(f"mission.name must match {NAME_RE.pattern}, got {name!r}")
    cond = m.get("condition")
    if cond in CONDITIONS and cond not in MISSION_FILE_CONDITIONS:
        raise MissionError(f"mission.condition {cond} is a dev-loop condition (eval/ only); a "
                           f"mission file takes one of {MISSION_FILE_CONDITIONS}")
    if cond not in MISSION_FILE_CONDITIONS:
        raise MissionError(f"mission.condition must be one of {MISSION_FILE_CONDITIONS}, "
                           f"got {cond!r}")
    areas = m.get("areas")
    if not isinstance(areas, list) or len(areas) == 0:
        raise MissionError("mission.areas must be a non-empty list of polygons")
    if len(areas) != 1:
        raise MissionError("mission.areas: exactly one polygon is supported ([OPEN] O-8)")
    polys = []
    for k, poly in enumerate(areas):
        if not isinstance(poly, list) or len(poly) < 3:
            raise MissionError(f"mission.areas[{k}] needs >= 3 [lat, lon] vertices")
        pts = []
        for i, v in enumerate(poly):
            if (not isinstance(v, (list, tuple)) or len(v) != 2 or
                    not all(isinstance(c, (int, float)) and not isinstance(c, bool) for c in v)):
                raise MissionError(f"mission.areas[{k}][{i}] must be [lat, lon]")
            lat, lon = float(v[0]), float(v[1])
            if not (-90 < lat < 90 and -180 <= lon <= 180):
                raise MissionError(f"mission.areas[{k}][{i}] out of range: {v}")
            pts.append((lat, lon))
        polys.append(tuple(pts))
    raw = m.get("config") or {}
    if not isinstance(raw, dict):
        raise MissionError("mission.config must be a mapping")
    merged = dict(base_overrides or {})
    frozen = dict((freeze or {}).get("settings") or {})
    for k, v in raw.items():
        if k in frozen and v != frozen[k]:
            raise MissionError(f"mission.config.{k} = {v!r} differs from the C2 freeze "
                               f"({(freeze or {}).get('file')}: {frozen[k]!r})")
    merged.update(frozen)
    merged.update(raw)
    try:
        cfg = Config.from_dict(merged)
    except ValueError as exc:
        raise MissionError(f"mission.config: {exc}") from None
    if cfg.planning_depth_m is None:
        raise MissionError("mission.config.planning_depth_m is required (MATH §2.3)")
    if cond == "C4_rand" and not cfg.bearing_table:
        raise MissionError("C4_rand needs config.c4_rand.bearing_table (pre-drawn in eval/)")
    return Mission(name, cond, tuple(polys), cfg, dict(raw), freeze=freeze)


def load_mission(path: str, base_overrides: dict | None = None,
                 freeze: dict | None = None) -> Mission:
    import yaml  # lazily: io/ only
    with open(path, "r") as fh:
        doc = yaml.safe_load(fh)
    return parse_mission(doc, base_overrides, freeze)
