"""The dev loop's binding to BlueBoat-Belief (eval/ only): the belief map of the dev loop IS
Belief's own core, driven through the offline interface Belief documents
(`BlueBoat-Belief/blueboat_belief/docs/OFFLINE_CORE_API.md`).

Rules of this file:
- only the names that document lists are used (`Belief`, `BeliefConfig`, `PingObs`, `Det`,
  `DetArray`, `ImageRec`, `make_area_dict`, `area_from_dict`, `processor_side_offsets`,
  `to_grid_dict`, and the run-health read-outs `Belief.counters`, `Belief.stats`,
  `Belief.health()` of its §5); results are read only through `to_grid_dict`;
- Belief is imported from its checkout beside this repository (or from an installed package),
  with bytecode writing off, so nothing is written into its tree;
- if it cannot be imported, `BeliefUnavailable` is raised with the reason. There is no other
  belief implementation in this module;
- `finish()` is called once, at the end of a mission;
- the core is never imported outside `eval/` (the online node reads only the `BeliefGrid`
  message).

How a miss is credited (`mode`, recorded in every manifest):
- "records": as in a live run. Pings are cut into seabed images (`eval/emulator.ImageCutter`);
  for every image one `DetArray` (empty ones included) and then one `ImageRec` are fed. Belief
  credits a ping only once the images containing it have been reported, so the grid lags the
  boat by one to two image strides, and its header stamp is the evidence horizon.
- "ungated": Belief's development mode (`coverage.require_detector: false`): every ping counts
  as searched at once and a detection is applied when it is fed. No lag. A sensitivity check.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from .emulator import ImageCutter, image_stride_m

T0 = 1000.0          # Belief's times must be > 0: Belief time = T0 + path time
MODES = ("records", "ungated")
# Belief's counters that mean an input was lost (OFFLINE_CORE_API.md §5: "A mission that lost
# nothing has coverage_expired, stale_detection, future_detection and no_pose_at_stamp at 0")
LOSSES = ("coverage_expired", "stale_detection", "future_detection", "no_pose_at_stamp")
_CORE = None


class BeliefUnavailable(RuntimeError):
    """The real Belief core cannot be imported; the message says why."""


def find_checkout():
    """BlueBoat-Belief's package root (`.../BlueBoat-Belief/blueboat_belief`), or None."""
    env = os.environ.get("BLUEBOAT_BELIEF_DIR")
    cands = [Path(env)] if env else []
    here = Path(__file__).resolve()
    for up in (4, 5):
        if len(here.parents) > up:
            cands.append(here.parents[up] / "BlueBoat-Belief" / "blueboat_belief")
    for c in cands:
        if (c / "blueboat_belief" / "core" / "belief.py").is_file():
            return c
    return None


def load_core():
    """Import Belief's supported offline surface. Bytecode writing is switched off for the
    rest of the process: Belief also imports lazily at call time."""
    global _CORE
    if _CORE is not None:
        return _CORE
    sys.dont_write_bytecode = True
    root = find_checkout()
    if root is not None:
        # The checkout beside this repository comes FIRST. A shell with the ROS workspace
        # sourced also holds an installed copy (`~/ros2_ws/install/blueboat_belief`), which is
        # older than the checkout whenever Belief was edited since its last build. Without a
        # checkout the installed copy is used. Either way `belief_info()` hashes what was
        # actually loaded, and the manifests record its location.
        if str(root) in sys.path:
            sys.path.remove(str(root))
        sys.path.insert(0, str(root))
    try:
        import blueboat_belief
        from blueboat_belief.core.area import area_from_dict, make_area_dict
        from blueboat_belief.core.belief import Belief, Det, DetArray, ImageRec, PingObs
        from blueboat_belief.core.config import BeliefConfig
        from blueboat_belief.core.contract import to_grid_dict
        from blueboat_belief.core.geometry import processor_side_offsets
    except ImportError as exc:
        where = (f"looked in {root}" if root is not None else
                 "no checkout found beside this repository (set BLUEBOAT_BELIEF_DIR) and no "
                 "installed package")
        raise BeliefUnavailable(f"BlueBoat-Belief core not importable: {exc} ({where})") from None
    pkg = Path(blueboat_belief.__file__).resolve().parent
    _CORE = SimpleNamespace(
        Belief=Belief, BeliefConfig=BeliefConfig, PingObs=PingObs, Det=Det, DetArray=DetArray,
        ImageRec=ImageRec, make_area_dict=make_area_dict, area_from_dict=area_from_dict,
        processor_side_offsets=processor_side_offsets, to_grid_dict=to_grid_dict,
        package_dir=pkg, version=getattr(blueboat_belief, "__version__", "unknown"))
    return _CORE


def belief_info() -> dict:
    """What a manifest records about the Belief the results were made with: its revision, a
    hash of its core sources (its working tree may hold uncommitted changes), and the size of
    the gate its own report states."""
    core = load_core()
    pkg = core.package_dir
    h = hashlib.sha256()
    files = sorted((pkg / "core").glob("*.py"))
    for f in files:
        h.update(f.name.encode())
        h.update(f.read_bytes())
    rev, dirty = "unknown", None
    try:
        rev = subprocess.run(["git", "-C", str(pkg), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=10).stdout.strip() or rev
        dirty = bool(subprocess.run(["git", "-C", str(pkg), "status", "--porcelain"],
                                    capture_output=True, text=True, timeout=10).stdout.strip())
    except Exception:  # noqa: BLE001 - provenance only
        pass
    gate = None
    report = pkg.parent / "docs" / "VALIDATION_REPORT.md"
    if report.is_file():
        m = re.search(r"ALL CHECKS PASSED \((\d+) checks, (\d+) skipped\)", report.read_text())
        if m:
            gate = {"checks": int(m.group(1)), "skipped": int(m.group(2)),
                    "source": "BlueBoat-Belief docs/VALIDATION_REPORT.md"}
    return {"package": "blueboat_belief", "version": core.version, "revision": rev,
            "uncommitted_changes": dirty, "core_sha256": h.hexdigest(),
            "core_files": len(files), "gate": gate, "location": str(pkg),
            "interface": "docs/OFFLINE_CORE_API.md"}


@dataclass
class EmDet:
    """One emulated detection: row time and position in the map frame (path time axis)."""

    t: float
    x: float
    y: float
    score: float
    class_name: str
    psi: float
    uid: str                    # unique per run


class RealBelief:
    """BlueBoat-Belief's core for one dev-loop mission."""

    backend = "BlueBoat-Belief core (offline interface)"

    def __init__(self, x_min: float, y_min: float, x_max: float, y_max: float, datum,
                 depth_m: float, cfg, class_names, mode: str = "records",
                 calibrated: bool = False, base_rate: float = 0.5, det_sigma_m: float = 0.5,
                 overrides: dict | None = None, detector_runs: bool = True):
        """`detector_runs=False`: every image is reported as not searched (a detector that
        never ran on pixels). Belief then counts looks and credits no search at all."""
        if mode not in MODES:
            raise ValueError(f"miss-credit mode {mode!r} not in {MODES}")
        core = load_core()
        self.core, self.mode = core, mode
        h = float(depth_m) - cfg.sensor_depth_m
        if h <= 0.05:
            # Belief reads water_depth == sensor depth as "altitude unknown" and drops the ping
            raise ValueError(f"depth {depth_m} m gives a transducer altitude of {h:.2f} m")
        self.h = h
        self.water_depth = h + cfg.sensor_depth_m
        area = core.area_from_dict(core.make_area_dict(float(datum[0]), float(datum[1]),
                                                       float(x_min), float(y_min), float(x_max),
                                                       float(y_max), name="devloop"))
        sections = {
            # the dev loop's poses are already in the map frame: odom (0, 0) is the datum
            "alignment": {"origin_lat": float(datum[0]), "origin_lon": float(datum[1]),
                          "override_sigma_m": 0.0},
            "detector": {"class_names": list(class_names), "calibrated": bool(calibrated),
                         "base_rate": float(base_rate)},
            "coverage": ({"source": "records"} if mode == "records"
                         else {"require_detector": False}),
        }
        for sec, vals in (overrides or {}).items():
            sections.setdefault(sec, {}).update(vals)
        default = core.BeliefConfig()
        self.bcfg = default.with_values(**sections)
        flat_d, flat = default.to_flat(), self.bcfg.to_flat()
        self.changed = {k: v for k, v in flat.items() if flat_d.get(k) != v}
        self.b = core.Belief(self.bcfg, area)
        self.port = core.processor_side_offsets(h, +1, range_m=cfg.range_m)
        self.stbd = core.processor_side_offsets(h, -1, range_m=cfg.range_m)
        self.db_p = np.full(self.port.size, -20.0, dtype=np.float32)     # display-only
        self.db_s = np.full(self.stbd.size, -20.0, dtype=np.float32)
        # Perception's detection covariance contains its assumed navigation sigma, which
        # Belief removes as common-mode (OFFLINE_CORE_API.md §2 rule 5)
        self.det_cov = np.eye(2) * (det_sigma_m ** 2 + self.bcfg.contact.common_mode_sigma_m ** 2)
        self.stride = image_stride_m(cfg.range_m)
        self.cutter = ImageCutter(self.stride) if mode == "records" else None
        self.detector_runs = bool(detector_runs)
        self.queue = []              # emulated detections not yet reported (records mode)
        self.n_pings = self.n_images = self.n_dets = 0
        self.last_t = -math.inf
        self.seq = 0
        self.closed = False

    # ------------------------------------------------------------- what the world side needs
    @property
    def look_timeout_s(self) -> float:
        return self.bcfg.look.timeout_s

    @property
    def lever_arm_m(self) -> float:
        return self.bcfg.pd.lever_arm_m

    @property
    def offsets(self) -> np.ndarray:
        """Ground offsets of one side's samples from the centre line (the processor's)."""
        return np.abs(self.port.astype(np.float64))

    def turn_params(self) -> dict:
        t = self.bcfg.turn
        return {"window_s": t.window_s, "min_yaw_rate": t.min_yaw_rate_rad_s,
                "min_speed": t.min_speed_mps, "enabled": t.enabled}

    # ------------------------------------------------------------- inputs
    def feed(self, t, x, y, psi, dets=()) -> None:
        """Pings at path times `t` (increasing) and the emulated detections whose rows are
        among them, in time order."""
        if self.closed:
            raise RuntimeError("the mission is finished: no further input")
        core = self.core
        dets = sorted(dets, key=lambda d: (d.t, d.uid))
        j = 0
        for i in range(len(t)):
            ti = float(t[i])
            if ti <= self.last_t:
                continue                      # a pose repeated at a segment join
            self.last_t = ti
            self.n_pings += 1
            self.b.ingest_ping(core.PingObs(
                t=T0 + ti, x=float(x[i]), y=float(y[i]), yaw=float(psi[i]),
                water_depth=self.water_depth, port_ping_number=self.n_pings,
                starboard_ping_number=self.n_pings, port_y=self.port, port_db=self.db_p,
                starboard_y=self.stbd, starboard_db=self.db_s))
            now = []
            while j < len(dets) and dets[j].t <= ti + 1e-9:
                now.append(dets[j])
                j += 1
            if self.mode == "ungated":
                for d in now:
                    self.b.ingest_detections(core.DetArray(t=T0 + d.t,
                                                           detections=(self._det(d),)))
                    self.n_dets += 1
                continue
            self.queue.extend(now)
            for img in self.cutter.add_ping(ti, float(x[i]), float(y[i])):
                self._report(img)
        if j < len(dets):
            raise ValueError("a detection row lies after the last ping fed")

    def _det(self, d: EmDet):
        return self.core.Det(id=d.uid, t_row=T0 + d.t, x=d.x, y=d.y, theta=d.psi,
                             class_name=d.class_name, score=d.score, cov=self.det_cov)

    def _report(self, img) -> None:
        """One seabed image: its detection array (also when empty), then its record."""
        core = self.core
        mine = [d for d in self.queue if d.t <= img.t_max + 1e-9]
        self.queue = [d for d in self.queue if d.t > img.t_max + 1e-9]
        self.b.ingest_detections(core.DetArray(t=T0 + img.t_max,
                                               detections=tuple(self._det(d) for d in mine)))
        self.b.ingest_record(core.ImageRec(
            key=f"devloop/{img.index}", t_min=T0 + img.t_min, t_max=T0 + img.t_max,
            searched=self.detector_runs, dummy=False,
            calibrated=bool(self.bcfg.detector.calibrated),
            detector="replanner_dev_loop_emulator"))
        self.n_images += 1
        self.n_dets += len(mine)

    def finish(self) -> None:
        """End of the mission: the last short image, then Belief's `finish()` (once)."""
        if self.closed:
            return
        if self.cutter is not None:
            img = self.cutter.flush()
            if img is not None:
                self._report(img)
        self.b.finish()
        self.closed = True

    # ------------------------------------------------------------- output
    def grid(self) -> dict:
        """Belief's `to_grid_dict`: every `belief_grid/0` field under its contract name."""
        self.seq += 1
        return self.core.to_grid_dict(self.b, grid_seq=self.seq)

    @staticmethod
    def horizon(grid: dict):
        """The grid's evidence horizon as a path time, or None when nothing was applied yet."""
        st = float(grid["header"]["stamp"])
        return st - T0 if st > 0.0 else None

    def run_health(self) -> dict:
        """What Belief dropped, applied and released (its documented run-health read-outs,
        OFFLINE_CORE_API.md §5): the non-zero rejection counters, the statistics, the counters
        that mean a lost input, and its health level."""
        counters = {k: int(v) for k, v in dict(self.b.counters).items()}
        level, message = self.b.health()
        return {"counters": {k: v for k, v in counters.items() if v},
                "stats": {k: int(v) for k, v in dict(self.b.stats).items()},
                "losses": {k: counters.get(k, 0) for k in LOSSES},
                "health": [str(level), str(message)]}

    def settings(self) -> dict:
        return {"miss_credit": self.mode, "time_offset_s": T0,
                "image_stride_m": self.stride if self.mode == "records" else None,
                "belief_config_changed": {k: (list(v) if isinstance(v, tuple) else v)
                                          for k, v in sorted(self.changed.items())},
                "detection_covariance_m2": float(self.det_cov[0, 0])}
