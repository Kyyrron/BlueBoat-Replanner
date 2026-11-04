"""Coverage band and line spacing as a function of range (MATH.md §2).

The coverage band is used ONLY for line spacing (closure F). Predicted detections use the
detection band of MATH §6 (`looks.py`).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .areas import MissionError


def r_geo(range_m: float, h: float) -> float:
    """Geometric far limit: ground range of slant range R at altitude h (brief §3)."""
    if h >= range_m:
        return 0.0
    return math.sqrt(range_m * range_m - h * h)


def altitude(depth_m: float, sensor_depth_m: float) -> float:
    return max(0.0, depth_m - sensor_depth_m)


@dataclass(frozen=True)
class CoverageBand:
    r_near: float
    r_far: float
    range_m: float
    h: float
    grazing_floor: bool

    @property
    def width(self) -> float:
        return self.r_far - self.r_near


def coverage_band(cfg, depth_m: float) -> CoverageBand:
    h = altitude(depth_m, cfg.sensor_depth_m)
    if cfg.grazing_floor:
        slant = min(cfg.range_m, h / cfg.grazing_ratio) if h > 0 else 0.0
        far = math.sqrt(max(0.0, slant * slant - h * h))
    else:
        far = r_geo(cfg.range_m, h)
    return CoverageBand(cfg.r_near_m, far, cfg.range_m, h, bool(cfg.grazing_floor))


def even_offsets(width: float, e: float, s: float) -> np.ndarray:
    """Line cross-track offsets in [0, width]: first and last at e and width - e, spread
    evenly with spacing <= s (MATH §2.2). One central line if width <= 2e."""
    span = width - 2.0 * e
    if span < 0:
        return np.array([width / 2.0])
    if span == 0:
        return np.array([e])
    n = int(math.ceil(span / s - 1e-12)) + 1
    return e + np.linspace(0.0, span, n)


def multiplicity(offsets: np.ndarray, width: float, band: CoverageBand, step: float) -> np.ndarray:
    """For x sampled every `step` over [0, width], the number of lines j with
    r_near <= |x - y_j| <= r_far."""
    n = max(2, int(math.ceil(width / step)) + 1)
    x = np.linspace(0.0, width, n)
    d = np.abs(x[:, None] - np.asarray(offsets)[None, :])
    ok = (d >= band.r_near - 1e-9) & (d <= band.r_far + 1e-9)
    return ok.sum(axis=1)


@dataclass(frozen=True)
class SpacingResult:
    s0: float
    s: float
    offsets: np.ndarray
    min_multiplicity: int
    mean_multiplicity: float
    bisected: bool


def spacing_for_width(width: float, band: CoverageBand, e: float, cfg) -> SpacingResult:
    """Verified spacing for a pass over a cross-track extent `width` (MATH §2.2)."""
    if band.width <= 0:
        raise MissionError(
            f"spacing: no usable band (R={band.range_m}, h={band.h:.3f}, r_near={band.r_near}, "
            f"r_far={band.r_far:.3f})")
    s0 = cfg.spacing_factor * band.width

    def check(s):
        offs = even_offsets(width, e, s)
        m = multiplicity(offs, width, band, cfg.spacing_check_step_m)
        return offs, m

    offs, m = check(s0)
    if m.min() >= 1:
        return SpacingResult(s0, s0, offs, int(m.min()), float(m.mean()), False)
    lo, hi = 0.0, s0
    best = None
    for _ in range(cfg.spacing_bisect_iterations):
        mid = 0.5 * (lo + hi)
        if mid <= 1e-6:
            break
        o2, m2 = check(mid)
        if m2.min() >= 1:
            lo, best = mid, (o2, m2)
        else:
            hi = mid
    if best is None:
        raise MissionError(
            f"spacing: no spacing gives single coverage over width {width:.2f} m "
            f"(R={band.range_m}, h={band.h:.3f}, r_near={band.r_near}, r_far={band.r_far:.3f}, "
            f"edge offset e={e:.2f})")
    o2, m2 = best
    return SpacingResult(s0, lo, o2, int(m2.min()), float(m2.mean()), True)


def nominal_spacing(band: CoverageBand, e: float, cfg, n_gaps: int = 10) -> SpacingResult:
    """Area-independent verified spacing (the MATH §2.4 table): a strip of n_gaps gaps."""
    width = 2.0 * e + n_gaps * cfg.spacing_factor * band.width
    return spacing_for_width(width, band, e, cfg)
