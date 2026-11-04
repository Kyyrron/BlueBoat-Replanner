"""BeliefPort — the dev loop's only coupling to a belief implementation (prompt §B.2).

    feed_swath(events)       look events (row, col, sector, r, b, pass, t) of the boat's path
    feed_detections(dets)    (x, y, confidence, sector, pass) from the emulator
    end_passes(current)      close passes that are over (misses become final)
    snapshot(stamp) -> dict  a contract `belief_grid/0` dict (to_grid_dict semantics, C-7)

`ReferencePort` wraps the reference belief (a stand-in, NOT BlueBoat-Belief). `RealPort` is
written in the in-machine session after reading Belief's ingestion API (CONTRACT C-7: that API
is Belief's own); importing the real core is allowed in eval/ only (DESIGN decision 14).
"""

from __future__ import annotations

from typing import Protocol

from .belief_reference import ReferenceBelief


class BeliefPort(Protocol):
    def feed_swath(self, events) -> None: ...
    def feed_detections(self, dets) -> None: ...
    def end_passes(self, current_pass) -> None: ...
    def snapshot(self, stamp: float) -> dict: ...


class ReferencePort:
    backend = "reference"

    def __init__(self, belief: ReferenceBelief):
        self.b = belief

    def look_events(self, t, x, y, psi, pass_ids):
        ev = []
        for i in range(len(t)):
            ev += self.b.look_events(t[i:i + 1], x[i:i + 1], y[i:i + 1], psi[i:i + 1],
                                     int(pass_ids[i]))
        return ev

    def feed_swath(self, events) -> None:
        self.b.feed_looks(events)

    def feed_detections(self, dets) -> None:
        self.b.feed_detections(dets)

    def end_passes(self, current_pass) -> None:
        self.b.finalize_passes(current_pass)

    def snapshot(self, stamp: float) -> dict:
        return self.b.to_grid_dict(stamp)


def real_port(*args, **kwargs):
    """Binding to the real BlueBoat-Belief core — [NOT AVAILABLE in the cloud session].
    To be written in REPLAN-2-LOCAL (LOCAL_CHECKLIST L-B7)."""
    raise NotImplementedError("belief_backend 'real' needs the BlueBoat-Belief core "
                              "(in-machine session, LOCAL_CHECKLIST L-B7)")
