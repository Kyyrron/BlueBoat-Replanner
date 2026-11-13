"""Duck-typed adapters (contract C-7). The ONE field-name table of this module.

`grid_to_dict(obj)` reads a `blueboat_belief_interfaces/BeliefGrid` message OR Belief's
`to_grid_dict()` output (attribute or key access) and returns a dict with the contract's
names, which `core.evidence.validate_grid` validates. A rename on Belief's side costs one line
in `GRID_FIELDS` / `CONTACT_FIELDS`. No rclpy import here: testable without ROS.
"""

from __future__ import annotations

import json
import math

import numpy as np

# contract name -> name on the source object. Belief's names are the contract's
# (BlueBoat-Belief core/contract.py GRID_FIELDS / CONTACT_FIELDS, read 2026-11-12).
GRID_FIELDS = {
    "contract_version": "contract_version",
    "grid_seq": "grid_seq",
    "width": "width",
    "height": "height",
    "resolution": "resolution",
    "origin_x": "origin_x",
    "origin_y": "origin_y",
    "datum_lat": "datum_lat",
    "datum_lon": "datum_lon",
    "odom_to_map_x": "odom_to_map_x",
    "odom_to_map_y": "odom_to_map_y",
    "odom_to_map_yaw": "odom_to_map_yaw",
    "odom_to_map_sigma_m": "odom_to_map_sigma_m",
    "odom_to_map_valid": "odom_to_map_valid",
    "aspect_sector_count": "aspect_sector_count",
    "aspect_sector_origin_deg": "aspect_sector_origin_deg",
    "class_names": "class_names",
    "prior_density_per_ha": "prior_density_per_ha",
    "model_params_json": "model_params_json",
    "p_background": "p_background",
    "p_target": "p_target",
    "coverage": "coverage",
    "n_looks": "n_looks",
    "n_pings": "n_pings",
    "n_aspects": "n_aspects",
    "depth_m": "depth_m",
    "last_intensity_db": "last_intensity_db",
    "max_confidence": "max_confidence",
    "aspect_counts": "aspect_counts",
    "aspect_log_miss": "aspect_log_miss",
    "area_x": "area_x",
    "area_y": "area_y",
    "contacts": "contacts",
}

CONTACT_FIELDS = {
    "uid": "uid",
    "x": "x",
    "y": "y",
    "covariance": "covariance",
    "p_exists": "p_exists",
    "state": "state",
    "class_probabilities": "class_probabilities",
    "sectors_detected": "sectors_detected",
    "sectors_missed": "sectors_missed",
    "n_looks_detected": "n_looks_detected",
    "first_confidence": "first_confidence",
    "first_sector": "first_sector",
    "max_confidence": "max_confidence",
    "shadow_known": "shadow_known",
    "has_shadow": "has_shadow",
}

BELIEF_TOPIC = "/belief_map/belief_grid"          # contract C-2 (Belief topics.md §1)

_MISSING = object()


def _get(obj, name):
    if isinstance(obj, dict):
        return obj.get(name, _MISSING)
    return getattr(obj, name, _MISSING)


def _stamp(obj):
    """The grid's header stamp in seconds: Belief's evidence horizon (MATH §11.3). Reads a ROS
    message (header.stamp.sec / nanosec) and Belief's `to_grid_dict` (header["stamp"], a
    float). None when there is none."""
    hdr = _get(obj, "header")
    if hdr is _MISSING or hdr is None:
        st = _get(obj, "stamp")
    else:
        st = _get(hdr, "stamp")
    if st is _MISSING or st is None:
        return None
    if isinstance(st, (int, float)) and not isinstance(st, bool):
        return float(st)
    sec, nsec = _get(st, "sec"), _get(st, "nanosec")
    if sec is _MISSING:
        return None
    return float(sec) + (0.0 if nsec is _MISSING else float(nsec) * 1e-9)


def _value(v):
    """Arrays stay arrays; a numpy scalar (Belief's `to_grid_dict` uses message dtypes, for
    example `np.uint32` for `grid_seq`) becomes the Python scalar a ROS message would carry."""
    if isinstance(v, np.generic):
        return v.item()
    if isinstance(v, (list, tuple)) or hasattr(v, "__array__"):
        a = np.asarray(v)
        return a.item() if a.ndim == 0 else a
    return v


def contact_to_dict(c) -> dict:
    out = {}
    for name, src in CONTACT_FIELDS.items():
        v = _get(c, src)
        if v is not _MISSING:
            out[name] = _value(v)
    return out


def grid_to_dict(obj) -> dict:
    out = {}
    for name, src in GRID_FIELDS.items():
        v = _get(obj, src)
        if v is _MISSING:
            continue
        if name == "contacts":
            out[name] = [contact_to_dict(c) for c in v]
        elif name == "class_names":
            out[name] = [str(s) for s in v]
        else:
            out[name] = _value(v)
    out["stamp"] = _stamp(obj)
    return out


# ---------------------------------------------------------------- Control JSON
PROGRESS_SCHEMA = "blueboat_path_progress/1"
EVENT_SCHEMA = "blueboat_path_update_event/1"


def parse_progress(text: str) -> dict | None:
    """/blueboat/path_progress payload (E-11) -> dict, or None if not that schema."""
    try:
        d = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(d, dict) or d.get("schema") != PROGRESS_SCHEMA:
        return None
    return d


def parse_event(text: str) -> dict | None:
    try:
        d = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(d, dict) or d.get("schema") != EVENT_SCHEMA:
        return None
    return d


def nan_to_none(v):
    if v is None:
        return None
    v = float(v)
    return v if math.isfinite(v) else None
