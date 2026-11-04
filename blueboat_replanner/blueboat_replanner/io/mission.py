"""Mission file (brief §6.4). Unknown keys raise; the config block is validated by Config.

    mission:
      name: beach_2026_10_12
      condition: C3
      areas: [[[lat, lon], ...]]        # exactly one simple polygon (O-8), WGS-84
      config: {planning_depth_m: 2.0}   # overrides of config/replanner.yaml, same keys
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from ..core.areas import MissionError
from ..core.config import Config

CONDITIONS = ("C1", "C2", "C3", "C4", "C4_iso", "C4_rand", "C4_live", "C4_degraded",
              "C4_depth1")
NAME_RE = re.compile(r"^[A-Za-z0-9._-]{1,64}$")


@dataclass(frozen=True)
class Mission:
    name: str
    condition: str
    areas: tuple            # tuple of tuples of (lat, lon)
    config: Config
    raw_config: dict


def parse_mission(doc, base_overrides: dict | None = None) -> Mission:
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
    if cond not in CONDITIONS:
        raise MissionError(f"mission.condition must be one of {CONDITIONS}, got {cond!r}")
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
    merged.update(raw)
    try:
        cfg = Config.from_dict(merged)
    except ValueError as exc:
        raise MissionError(f"mission.config: {exc}") from None
    if cfg.planning_depth_m is None:
        raise MissionError("mission.config.planning_depth_m is required (MATH §2.3)")
    if cond == "C4_rand" and not cfg.bearing_table:
        raise MissionError("C4_rand needs config.c4_rand.bearing_table (pre-drawn in eval/)")
    return Mission(name, cond, tuple(polys), cfg, dict(raw))


def load_mission(path: str, base_overrides: dict | None = None) -> Mission:
    import yaml  # lazily: io/ only
    with open(path, "r") as fh:
        doc = yaml.safe_load(fh)
    return parse_mission(doc, base_overrides)
