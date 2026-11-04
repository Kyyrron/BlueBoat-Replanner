"""Offline validation of requests and plans (DESIGN.md §5, §11).

`request_check` mirrors Control's request checks (E-08; `path_update.validate_samples`, rev
1fadbb2) so that a request is validated before it is sent. Control remains the authority:
passing this check does not mean Control will accept (closure G).
"""

from __future__ import annotations

import math
import re

import numpy as np

from .config import CONTROL_RULES
from .geo import wrap_pi

ID_PATTERN = re.compile(r"^[A-Za-z0-9._:-]{1,64}$")


def request_check(req_id: str, t, x, y, yaw, rules=CONTROL_RULES) -> tuple:
    """(code, message): 'ok' or one of Control's request refusal codes."""
    if not isinstance(req_id, str) or not ID_PATTERN.match(req_id):
        return "bad_request", f"id {req_id!r} must match [A-Za-z0-9._:-]{{1,64}}"
    n = {len(t), len(x), len(y), len(yaw)}
    if len(n) != 1:
        return "bad_request", "t, x, y, yaw lengths differ"
    n = n.pop()
    if n < 2:
        return "too_few_points", f"{n} samples"
    if n > rules["max_points"]:
        return "too_many_points", f"{n} samples"
    pts = np.column_stack([np.asarray(t, float), np.asarray(x, float),
                           np.asarray(y, float), np.asarray(yaw, float)])
    if not np.all(np.isfinite(pts)):
        return "not_finite", "non-finite sample"
    t, x, y, yaw = pts.T
    dts = np.diff(t)
    if np.any(dts < rules["min_sample_dt"]):
        k = int(np.argmax(dts < rules["min_sample_dt"]))
        return "time_not_increasing", f"t[{k}]={t[k]} -> t[{k + 1}]={t[k + 1]}"
    if t[0] < 0.0 or t[-1] >= rules["max_parameter"]:
        return "time_out_of_range", f"[{t[0]}, {t[-1]}]"
    dist = np.hypot(np.diff(x), np.diff(y))
    speed = dist / dts
    if np.any(speed > rules["max_speed"]):
        return "too_fast", f"{float(speed.max()):.3g} m/s"
    moving = (dist >= rules["min_segment_length"]) & (speed >= rules["hold_speed"])
    if np.any(moving):
        course = np.arctan2(np.diff(y), np.diff(x))
        err = np.minimum(np.abs(wrap_pi(yaw[:-1] - course)), np.abs(wrap_pi(yaw[1:] - course)))
        bad = moving & (err > rules["heading_coherence"])
        if np.any(bad):
            k = int(np.argmax(bad))
            return "heading_incoherent", f"segment {k}: {math.degrees(err[k]):.1f} deg"
    return "ok", ""


def continuity_check(mirror, change_point: float, t, x, y, yaw, h: float,
                     rules=CONTROL_RULES) -> tuple:
    """Junction check at c against the mirror (E-07): position 0.20 m, heading 10 deg unless the
    current path is stationary at c. `h` is the governor window spacing (Control: path_time /
    (path_steps - 1))."""
    from .plan import control_eval
    nx, ny, nyaw = control_eval(t, x, y, yaw, change_point)
    rx, ry, ryaw = mirror.pose(change_point)
    gap = math.hypot(nx - rx, ny - ry)
    if gap > rules["pos_tol"]:
        return "discontinuous_position", f"{gap:.3f} m"
    stationary = (mirror.speed(change_point - h, h) < rules["hold_speed"]
                  and mirror.speed(change_point, h) < rules["hold_speed"])
    hgap = abs(math.atan2(math.sin(nyaw - ryaw), math.cos(nyaw - ryaw)))
    if not stationary and hgap > rules["heading_tol"]:
        return "discontinuous_heading", f"{math.degrees(hgap):.2f} deg"
    return "ok", ""


def plan_in_area(plan, area, margin: float, step: float) -> tuple:
    """(ok, worst_point) — every sample of every segment in F_margin."""
    from .dubins import sample_prims
    worst = None
    for seg in plan.segments:
        x, y, _ = sample_prims(seg.prims, step)
        if len(x) == 0:
            continue
        pts = np.column_stack([x, y])
        ok = area.feasible(pts, margin)
        if not np.all(ok):
            k = int(np.argmin(ok))
            worst = (float(pts[k, 0]), float(pts[k, 1]), seg.kind)
            return False, worst
    return True, None
