"""World model and detection emulator (eval/ only).

Randomness lives only in eval/: here, in `emulator.py` and in the C4_rand table of `runner.py`.

World ≠ planner ≠ Belief: the world uses continuous target orientations and its own parameter
block (matched to the planner's placeholders by default; ablations change only the PLANNER
block). Belief's model is Belief's own and is never set from here.
Detection draws are keyed by (seed, object id, look number) through `SeedSequence`, so two
conditions on the same seed share the world and the k-th look at an object gets the same draw
whatever path led to it (common random numbers, DESIGN §6 / C2 protocol §4).

Confidences are calibrated against this world (brief §9.4): a raw score s ~ Beta(real) or
Beta(clutter) is mapped by a Platt calibrator fitted on separate calibration seeds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

WORLD_DEFAULTS = {
    "pd_plateau": 0.7, "pd_r_lo_m": 3.0, "pd_r_hi_m": 25.0,
    # class -> (m, w_deg, a, s): matched to the planner placeholders (MATH §6)
    "classes": {"block": (4, 20.0, 0.4, 1.0), "pipe": (2, 10.0, 0.1, 1.0),
                "cage": (4, 25.0, 0.3, 0.9), "tyre": (0, 0.0, 1.0, 0.6),
                "plate": (0, 0.0, 1.0, 0.3)},
    "clutter_pd": 0.2,
    "det_sigma_m": 0.5,
    "score_real": (4.0, 2.0),       # Beta parameters of the raw score
    "score_clutter": (2.0, 4.0),
    "n_targets": 6,
    "n_clutter": 6,
    "edge_m": 8.0,                  # objects at least this far inside the area
}


@dataclass
class WorldObject:
    oid: int
    kind: str           # "target" or "clutter"
    cls: str            # planner class name for targets, "clutter" otherwise
    x: float
    y: float
    phi: float          # orientation (rad), continuous
    label: str = ""     # the name a classifying detector would report (a built world's object
                        # type); "" = the class itself for a target, a drawn one for clutter


@dataclass
class World:
    seed: int
    objects: list
    params: dict
    looks: dict = field(default_factory=dict)       # oid -> number of looks so far
    aspects: dict = field(default_factory=dict)     # oid -> set of folded sectors looked

    @property
    def targets(self):
        return [o for o in self.objects if o.kind == "target"]


def make_world(seed: int, area, params: dict | None = None) -> World:
    p = dict(WORLD_DEFAULTS)
    p.update(params or {})
    ss = np.random.SeedSequence([int(seed), 1])
    rng = np.random.default_rng(ss)
    vx = area.vertices
    x0, y0 = vx.min(axis=0)
    x1, y1 = vx.max(axis=0)
    objs = []
    oid = 1
    names = sorted(p["classes"])
    for kind, n in (("target", p["n_targets"]), ("clutter", p["n_clutter"])):
        for _ in range(n):
            for _try in range(1000):
                x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
                if area.boundary_distance([[x, y]])[0] >= p["edge_m"] and \
                        bool(area.contains([[x, y]])[0]):
                    break
            cls = names[int(rng.integers(len(names)))] if kind == "target" else "clutter"
            objs.append(WorldObject(oid, kind, cls, float(x), float(y),
                                    float(rng.uniform(0, 2 * math.pi))))
            oid += 1
    return World(int(seed), objs, p)


def world_pd(world: World, obj: WorldObject, r: float, b: float, r_hi_geo: float) -> float:
    """World detection probability of one look at `obj` from bearing b, at ground range r from
    the transducer (the world's own model: a box in range, the class's aspect window)."""
    p = world.params
    if not (p["pd_r_lo_m"] <= r <= min(p["pd_r_hi_m"], r_hi_geo)):
        return 0.0
    if obj.kind == "clutter":
        return p["clutter_pd"]
    m, w_deg, a, s = p["classes"][obj.cls]
    A = 1.0
    if m > 0:
        P = 2 * math.pi / m
        d = (b - obj.phi) - P * round((b - obj.phi) / P)
        A = 1.0 if abs(d) <= math.radians(w_deg) else a
    return p["pd_plateau"] * s * A


def look_rng(world: World, obj: WorldObject, look_no: int):
    return np.random.default_rng(np.random.SeedSequence([world.seed, 2, obj.oid, look_no]))


@dataclass
class Calibrator:
    """Platt scaling on logit(score): q = sigmoid(a * logit(s) + b) (eval only)."""

    a: float = 1.0
    b: float = 0.0
    fitted_on: str = "identity"

    def __call__(self, s: float) -> float:
        s = min(max(s, 1e-6), 1 - 1e-6)
        z = self.a * math.log(s / (1 - s)) + self.b
        return 1.0 / (1.0 + math.exp(-z))

    @staticmethod
    def fit(scores, labels, note: str) -> "Calibrator":
        s = np.clip(np.asarray(scores, float), 1e-6, 1 - 1e-6)
        y = np.asarray(labels, float)
        X = np.column_stack([np.log(s / (1 - s)), np.ones_like(s)])
        w = np.zeros(2)
        for _ in range(50):
            z = X @ w
            pz = 1 / (1 + np.exp(-z))
            g = X.T @ (pz - y)
            Hm = X.T @ (X * (pz * (1 - pz))[:, None]) + 1e-9 * np.eye(2)
            w -= np.linalg.solve(Hm, g)
        return Calibrator(float(w[0]), float(w[1]), note)


def draw_detection(world: World, obj: WorldObject, r: float, b: float, r_hi_geo: float,
                   calibrator: Calibrator):
    """One draw per object per look (brief §10; `eval/emulator.py` defines the look). Returns
    (detected, x, y, q, raw_score)."""
    n = world.looks.get(obj.oid, 0) + 1
    world.looks[obj.oid] = n
    rng = look_rng(world, obj, n)
    u = rng.uniform()
    pd = world_pd(world, obj, r, b, r_hi_geo)
    a_b = world.params["score_real"] if obj.kind == "target" else world.params["score_clutter"]
    raw = float(rng.beta(*a_b))
    nx, ny = rng.normal(0.0, world.params["det_sigma_m"], 2)
    if u >= pd:
        return False, None, None, None, raw
    return True, obj.x + float(nx), obj.y + float(ny), calibrator(raw), raw
