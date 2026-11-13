"""Policy interface (DESIGN.md §6).

`Policy.on_epoch(ctx) -> Decision` is pure: no clock, no randomness, no I/O. The executor (the
only code that knows Control's mechanism) turns a REPLACE decision into a request.
"""

from __future__ import annotations

from dataclasses import dataclass, field

CONTINUE = "CONTINUE"
REPLACE = "REPLACE"
STOP = "STOP"


@dataclass(frozen=True)
class Decision:
    action: str
    change_point: float
    tail: object = None              # core.plan.Plan starting at change_point (REPLACE)
    rules_fired: tuple = ()
    scored: tuple = ()
    info: dict = field(default_factory=dict)


@dataclass
class Budget:
    """Mission budget in path seconds (MATH §4, §12)."""

    t_start: float
    B: float
    revisit_used: float = 0.0

    @property
    def t_limit(self) -> float:
        return self.t_start + self.B

    def revisit_cap(self, rho_rev: float) -> float:
        return rho_rev * self.B

    def admission(self, elapsed: float, T: float, dT: float, cap_left: float) -> str:
        """Why an action is not admitted (MATH §12): "budget" when `elapsed` + its duration T
        passes B, "revisit_cap" when its added revisit time dT passes what is left of the cap;
        "" when it is admitted."""
        if elapsed + T > self.B + 1e-9:
            return "budget"
        if dT > cap_left + 1e-9:
            return "revisit_cap"
        return ""


@dataclass
class EpochContext:
    cfg: object
    area: object
    plan: object              # current full plan (core.plan.Plan, frame M, Control's t axis)
    c: float                  # change point = the decision point's end time
    dp_index: int             # segment index of the decision point
    budget: Budget
    belief: object = None     # core.evidence.BeliefSnapshot or None (stale / not received)
    coverage: object = None   # core.coverage.CoveragePlans
    state: dict = field(default_factory=dict)   # per-mission policy memory (counters)
    # MATH §11.3: the path time up to which `belief` is complete (None: not known, the
    # policy falls back to tau - t_lat) and how it was obtained
    tau_horizon: float | None = None
    horizon_source: str = ""


def horizon_of(ctx, cfg, tau: float) -> tuple:
    """(tau_h, source) of MATH §11.3 for this epoch."""
    if ctx.tau_horizon is not None:
        return float(ctx.tau_horizon), ctx.horizon_source or "stamp"
    return tau - cfg.t_lat_s, ctx.horizon_source or "t_lat:no_stamp"


class Policy:
    name = "base"
    may_interrupt = False

    def initial_plan(self, coverage):
        raise NotImplementedError

    def decision_points(self, plan) -> list:
        """(t_end, segment index) at which this policy decides; [] for fixed plans."""
        return []

    def on_epoch(self, ctx: EpochContext) -> Decision:
        return Decision(CONTINUE, ctx.c)
