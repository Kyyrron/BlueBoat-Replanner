"""Plan archive as `blueboat_trajectory/1` (E-30, E-32): frame M, geo_anchor = Belief's datum.

Written once, with an exclusive create (CM-7). The format is the one Control's loader reads
(`format`, N×4 `points`, optional `name`, `loop`); extra keys are ignored by that loader.
"""

from __future__ import annotations

import os

FORMAT = "blueboat_trajectory/1"


def trajectory_doc(name: str, t, x, y, yaw, lat0: float, lon0: float, **extra) -> dict:
    pts = [[round(float(a), 3), round(float(b), 4), round(float(c), 4), round(float(d), 6)]
           for a, b, c, d in zip(t, x, y, yaw)]
    doc = {"format": FORMAT, "name": name, "generator": "blueboat_replanner", "frame": "map",
           "loop": False, "geo_anchor": {"lat0": float(lat0), "lon0": float(lon0),
                                         "theta_deg": 0.0},
           "points": pts}
    doc.update(extra)
    return doc


def write_exclusive(path: str, doc: dict) -> str:
    """Create `path` exclusively; on collision append -2, -3, ... (never overwrite)."""
    import yaml  # lazily: io/ only
    base, ext = os.path.splitext(path)
    k = 1
    while True:
        cand = path if k == 1 else f"{base}-{k}{ext}"
        try:
            fd = os.open(cand, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        except FileExistsError:
            k += 1
            continue
        with os.fdopen(fd, "w") as fh:
            yaml.safe_dump(doc, fh, sort_keys=False, default_flow_style=None)
        return cand
