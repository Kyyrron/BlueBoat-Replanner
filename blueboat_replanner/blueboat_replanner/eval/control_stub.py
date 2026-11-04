"""Offline stub of Control's live path update (eval/ and tests only).

Written from BlueBoat-Control `docs/PATH_UPDATE_INTERFACE.md` (rev 1fadbb2) only, like Control's
own demo client: state gate (replanner run assumed), request validity, change-point range, lead
(lead_min = max(path_time + 2 dt, gov_Lmax / U_d)), continuity (0.20 m, 10 deg unless
stationary), splice. A refusal changes nothing. The gate cross-checks this stub against
Control's ROS-free `path_update.PathUpdater` when it is importable.
"""

from __future__ import annotations

import math

import numpy as np

from ..core.config import CONTROL_RULES
from ..core.plan import ControlPathMirror
from ..core.validate import continuity_check, request_check


class ControlStub:
    def __init__(self, hold_pose, path_time: float = 0.05, dt: float = 0.05,
                 path_steps: int = 2, rules=CONTROL_RULES):
        self.hold = tuple(hold_pose)
        self.path_time = path_time
        self.dt = dt
        self.h = path_time / max(1, path_steps - 1)
        self.rules = rules
        self.mirror = ControlPathMirror()
        self.used_ids = set()
        self.awaiting = True
        self.state = "awaiting_path"       # or following / end_reached / override / not_ready
        self.log = []

    # ------------------------------------------------------------- progress
    def lead_min(self, tau: float) -> float:
        floor = self.path_time + 2.0 * self.dt
        if not self.mirror:
            return floor
        u = self.mirror.speed(tau, self.h)
        if u >= self.rules["hold_speed"] and u > 0:
            return max(floor, self.rules["gov_Lmax"] / u)
        return floor

    def progress(self, tau: float) -> dict:
        accepting = self.state not in ("override", "not_ready")
        end = self.mirror.t_end if self.mirror else None
        return {"schema": "blueboat_path_progress/1", "state":
                ("awaiting_path" if self.awaiting else
                 ("end_reached" if end is not None and tau >= end else self.state)),
                "tau": tau, "accepting": accepting,
                "lead_min": self.lead_min(tau) if self.mirror else None,
                "min_change_point": (tau + self.lead_min(tau)) if (self.mirror and accepting
                                                                    and not self.awaiting) else None,
                "active_id": self.mirror.active_id, "t_end": end,
                "hold": None if not self.awaiting else {"x": self.hold[0], "y": self.hold[1],
                                                        "yaw": self.hold[2]},
                "end_reached": (tau >= end) if end is not None else None}

    # ------------------------------------------------------------- request
    def handle(self, req_id, change_point, t, x, y, yaw, tau: float) -> dict:
        ans = self._handle(req_id, change_point, t, x, y, yaw, tau)
        self.log.append({"id": req_id, "tau": tau, "change_point": change_point, **ans})
        return ans

    def _refuse(self, code, msg, tau):
        mcp = tau + self.lead_min(tau) if (self.mirror and not self.awaiting) else float("nan")
        return {"accepted": False, "reason": code, "message": msg,
                "active_id": self.mirror.active_id or "", "tau": tau, "min_change_point": mcp}

    def _handle(self, req_id, c, t, x, y, yaw, tau):
        if self.state in ("override",):
            return self._refuse("operator_override", "operator control engaged", tau)
        if self.state == "not_ready":
            return self._refuse("not_ready", "E-STOP or not ready", tau)
        code, msg = request_check(req_id, t, x, y, yaw, self.rules)
        if code != "ok":
            return self._refuse(code, msg, tau)
        if req_id in self.used_ids:
            return self._refuse("duplicate_id", req_id, tau)
        t = np.asarray(t, float)
        t0, tl = float(t[0]), float(t[-1])
        if self.awaiting:
            if not (c is None or (isinstance(c, float) and math.isnan(c)) or
                    abs(c - t0) <= self.rules["first_path_t0_tol"]):
                return self._refuse("change_point_outside", "first path: NaN or t[0]", tau)
            gap = math.hypot(x[0] - self.hold[0], y[0] - self.hold[1])
            if gap > self.rules["pos_tol"]:
                return self._refuse("discontinuous_position", f"{gap:.3f} m from hold", tau)
            self.mirror.splice(t0, t, x, y, yaw, req_id)
            self.awaiting = False
            self.state = "following"
            self.used_ids.add(req_id)
            return {"accepted": True, "reason": "ok", "message": "first path loaded",
                    "active_id": req_id, "tau": t0, "min_change_point": float("nan")}
        if c is None or (isinstance(c, float) and math.isnan(c)):
            return self._refuse("change_point_outside", "change point required", tau)
        if not (t0 <= c < tl):
            return self._refuse("change_point_outside", f"{c} not in [{t0}, {tl})", tau)
        lead = self.lead_min(tau)
        if c < tau + lead - 1e-9:
            return self._refuse("lead_too_short", f"c={c} < tau+lead={tau + lead}", tau)
        code, msg = continuity_check(self.mirror, c, t, x, y, yaw, self.h, self.rules)
        if code != "ok":
            return self._refuse(code, msg, tau)
        self.mirror.splice(c, t, x, y, yaw, req_id)
        self.used_ids.add(req_id)
        return {"accepted": True, "reason": "ok", "message": "accepted", "active_id": req_id,
                "tau": tau, "min_change_point": tau + self.lead_min(tau)}
