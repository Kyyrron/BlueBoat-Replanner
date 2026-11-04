"""C1 (single pass) and C2 (two-pass orthogonal): fixed plans, no decisions (MATH §10.1)."""

from __future__ import annotations

from .base import Policy


class CoveragePolicy(Policy):
    def __init__(self, passes: int):
        if passes not in (1, 2):
            raise ValueError("passes must be 1 or 2")
        self.passes = passes
        self.name = "C1" if passes == 1 else "C2"

    def initial_plan(self, coverage):
        return coverage.c1 if self.passes == 1 else coverage.c2

    def decision_points(self, plan) -> list:
        return []
