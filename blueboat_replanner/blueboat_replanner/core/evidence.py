"""BeliefGrid (contract `belief_grid/0`) -> validated arrays (CONTRACT_CONSUMED.md).

Input: a dict with the contract's field names (C-3, C-4), produced by `ros.adapters` from a
ROS message or by `eval` from Belief's `to_grid_dict`. Every violation raises `ContractError`
naming the field; a missing required field is never replaced by a default (C-6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .geo import Frame2D

CONTRACT_VERSION = "belief_grid/0"
UNKNOWN_SECTOR = 255


class ContractError(ValueError):
    def __init__(self, fieldname: str, message: str):
        super().__init__(f"{fieldname}: {message}")
        self.field = fieldname


def _req(d, name):
    if name not in d or d[name] is None:
        raise ContractError(name, "required field missing")
    return d[name]


def _num(d, name, finite=True):
    v = _req(d, name)
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ContractError(name, f"not a number: {v!r}") from None
    if finite and not math.isfinite(v):
        raise ContractError(name, "not finite")
    return v


def _int(d, name, lo=None):
    v = _req(d, name)
    if isinstance(v, bool) or not isinstance(v, (int, np.integer)):
        raise ContractError(name, f"not an integer: {v!r}")
    v = int(v)
    if lo is not None and v < lo:
        raise ContractError(name, f"must be >= {lo}, got {v}")
    return v


def _array(d, name, n, dtype=float, optional=False):
    if name not in d or d[name] is None:
        if optional:
            return None
        raise ContractError(name, "required field missing")
    a = np.asarray(d[name], dtype=dtype).ravel()
    if optional and a.size == 0:
        return None
    if a.size != n:
        raise ContractError(name, f"length {a.size} != {n}")
    return a


@dataclass
class Contact:
    uid: int
    x: float
    y: float
    cov: np.ndarray
    p_exists: float
    state: int
    class_probabilities: np.ndarray
    sectors_detected: tuple
    sectors_missed: tuple
    first_confidence: float
    first_sector: int
    max_confidence: float
    fallbacks: tuple = ()

    @property
    def confirmed(self) -> bool:
        return self.state == 1


@dataclass
class BeliefSnapshot:
    grid_seq: int
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    datum_lat: float
    datum_lon: float
    frame: Frame2D
    transform_sigma_m: float
    sector_count: int
    sector_origin_deg: float
    class_names: tuple
    prior_density_per_ha: float
    coverage: np.ndarray          # (height, width), NaN outside Belief's polygon
    aspect_counts: np.ndarray     # (height, width, S)
    p_background: np.ndarray | None
    depth_m: np.ndarray | None
    contacts: list
    stamp: float | None = None
    fallbacks: tuple = field(default_factory=tuple)

    # ------------------------------------------------------------- geometry
    def sector_of(self, bearing):
        deg = np.degrees(np.asarray(bearing, float))
        return (np.floor(np.mod(deg - self.sector_origin_deg, 360.0) /
                         (360.0 / self.sector_count))).astype(int) % self.sector_count

    def sector_centre(self, k) -> float:
        return math.radians(self.sector_origin_deg + (k + 0.5) * 360.0 / self.sector_count)

    def cell_of(self, x, y):
        col = np.floor((np.asarray(x, float) - self.origin_x) / self.resolution).astype(int)
        row = np.floor((np.asarray(y, float) - self.origin_y) / self.resolution).astype(int)
        return row, col

    def cell_centre(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def depth_near(self, x: float, y: float, radius: float):
        if self.depth_m is None:
            return None
        r = int(math.ceil(radius / self.resolution))
        row, col = self.cell_of(x, y)
        r0, r1 = max(0, row - r), min(self.height, row + r + 1)
        c0, c1 = max(0, col - r), min(self.width, col + r + 1)
        if r0 >= r1 or c0 >= c1:
            return None
        rr, cc = np.mgrid[r0:r1, c0:c1]
        cx, cy = self.cell_centre(rr, cc)
        m = np.hypot(cx - x, cy - y) <= radius
        vals = self.depth_m[r0:r1, c0:c1][m]
        vals = vals[np.isfinite(vals)]
        return float(np.median(vals)) if vals.size else None


def _contact(c, S, n_classes, i) -> Contact:
    pre = f"contacts[{i}]."
    uid = _int(c, "uid", 0) if "uid" in c else None
    if uid is None:
        raise ContractError(pre + "uid", "required field missing")
    x = _num(c, "x")
    y = _num(c, "y")
    cov = np.asarray(_req(c, "covariance"), float).ravel()
    if cov.size != 4 or not np.all(np.isfinite(cov)):
        raise ContractError(pre + "covariance", "must be 4 finite values")
    cov = cov.reshape(2, 2)
    scale = max(1.0, float(np.abs(cov).max()))
    if abs(cov[0, 1] - cov[1, 0]) > 1e-9 * scale:
        raise ContractError(pre + "covariance", "not symmetric")
    if np.linalg.eigvalsh(cov).min() < -1e-12 * scale:
        raise ContractError(pre + "covariance", "not positive semi-definite")
    pe = _num(c, "p_exists")
    if not (0.0 <= pe <= 1.0):
        raise ContractError(pre + "p_exists", f"{pe} not in [0, 1]")
    state = _int(c, "state")
    if state not in (0, 1):
        raise ContractError(pre + "state", f"{state} not in {{0, 1}}")
    fallbacks = []
    cp = np.asarray(_req(c, "class_probabilities"), float).ravel()
    if cp.size == 0:
        cp = np.full(n_classes, 1.0 / n_classes)
        fallbacks.append("class_probabilities_uniform")
    elif cp.size != n_classes or not np.all(np.isfinite(cp)) or np.any(cp < 0):
        raise ContractError(pre + "class_probabilities", "length or values invalid")
    else:
        tot = float(cp.sum())
        if tot <= 0:
            raise ContractError(pre + "class_probabilities", "sum is zero")
        if abs(tot - 1.0) > 1e-6:
            fallbacks.append("class_probabilities_renormalised")
        cp = cp / tot

    def sectors(name):
        v = [int(s) for s in np.asarray(_req(c, name)).ravel()]
        if v != sorted(set(v)) or any(s < 0 or s >= S for s in v):
            raise ContractError(pre + name, f"must be sorted unique sectors < {S}: {v}")
        return tuple(v)

    det = sectors("sectors_detected")
    mis = sectors("sectors_missed")
    mc = _num(c, "max_confidence", finite=False)
    fc = c.get("first_confidence")
    fc = float("nan") if fc is None else float(fc)
    if math.isnan(fc):
        fallbacks.append("first_confidence_missing")
    elif not (0.0 <= fc <= 1.0):
        raise ContractError(pre + "first_confidence", f"{fc} not in [0, 1]")
    fs = c.get("first_sector")
    fs = UNKNOWN_SECTOR if fs is None else int(fs)
    if fs == UNKNOWN_SECTOR:
        fallbacks.append("first_sector_unknown")
    elif not (0 <= fs < S):
        raise ContractError(pre + "first_sector", f"{fs} not < {S} nor 255")
    return Contact(uid, x, y, cov, pe, state, cp, det, mis, fc, fs, mc, tuple(fallbacks))


def validate_grid(d: dict, require_transform: bool = True) -> BeliefSnapshot:
    """Validate a contract dict (C-3, C-4, C-6). Raises ContractError naming the field."""
    v = _req(d, "contract_version")
    if v != CONTRACT_VERSION:
        raise ContractError("contract_version", f"{v!r} != {CONTRACT_VERSION!r}")
    seq = _int(d, "grid_seq", 0)
    w = _int(d, "width", 1)
    h = _int(d, "height", 1)
    res = _num(d, "resolution")
    if res <= 0:
        raise ContractError("resolution", "must be > 0")
    ox, oy = _num(d, "origin_x"), _num(d, "origin_y")
    lat, lon = _num(d, "datum_lat"), _num(d, "datum_lon")
    if abs(lat) >= 90.0:
        raise ContractError("datum_lat", "out of range")
    fx, fy, fyaw = _num(d, "odom_to_map_x"), _num(d, "odom_to_map_y"), _num(d, "odom_to_map_yaw")
    valid = _req(d, "odom_to_map_valid")
    if not isinstance(valid, (bool, np.bool_)):
        raise ContractError("odom_to_map_valid", "not a bool")
    if require_transform and not bool(valid):
        raise ContractError("odom_to_map_valid", "false: alignment not converged, no plan")
    sigma = d.get("odom_to_map_sigma_m")
    sigma = float("nan") if sigma is None else float(sigma)
    S = _int(d, "aspect_sector_count", 1)
    so = _num(d, "aspect_sector_origin_deg")
    names = tuple(str(n) for n in _req(d, "class_names"))
    if not names or len(set(names)) != len(names):
        raise ContractError("class_names", "must be non-empty and unique")
    dens = _num(d, "prior_density_per_ha")
    if dens < 0:
        raise ContractError("prior_density_per_ha", "must be >= 0")
    n = w * h
    cov = _array(d, "coverage", n)
    finite = np.isfinite(cov)
    if np.any((cov[finite] < 0) | (cov[finite] > 1)):
        raise ContractError("coverage", "values outside [0, 1]")
    ac = _array(d, "aspect_counts", n * S, dtype=np.int64)
    if np.any(ac < 0):
        raise ContractError("aspect_counts", "negative count")
    fallbacks = []
    pb = _array(d, "p_background", n, optional=True)
    if pb is None:
        fallbacks.append("p_background_missing")
    else:
        if not np.array_equal(np.isnan(pb), ~finite):
            raise ContractError("p_background", "NaN mask differs from coverage's")
    depth = _array(d, "depth_m", n, optional=True)
    contacts_raw = d.get("contacts")
    if contacts_raw is None:
        raise ContractError("contacts", "required field missing")
    contacts = [_contact(c, S, len(names), i) for i, c in enumerate(contacts_raw)]
    uids = [c.uid for c in contacts]
    if len(set(uids)) != len(uids):
        raise ContractError("contacts", "duplicate uid")
    stamp = d.get("stamp")
    return BeliefSnapshot(
        seq, w, h, res, ox, oy, lat, lon, Frame2D(fx, fy, fyaw), sigma, S, so, names, dens,
        cov.reshape(h, w), ac.reshape(h, w, S),
        None if pb is None else pb.reshape(h, w),
        None if depth is None else depth.reshape(h, w),
        contacts, None if stamp is None else float(stamp), tuple(fallbacks))


def geometry_signature(s: BeliefSnapshot) -> tuple:
    """Fields whose change between snapshots stops a mission (DESIGN §4)."""
    return (s.width, s.height, s.resolution, s.origin_x, s.origin_y, s.datum_lat, s.datum_lon,
            s.sector_count, s.sector_origin_deg, s.class_names)
