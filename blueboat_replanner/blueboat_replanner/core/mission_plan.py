"""Mission set-up shared by the node and the dev loop: area in M, coverage plans, budget,
policy (DESIGN.md §5.1, MATH.md §3-§4)."""

from __future__ import annotations

from dataclasses import dataclass

from .areas import Area, MissionError
from .coverage import CoveragePlans, build_coverage
from .geo import areas_to_map
from .policy.append import AppendOnCompletionPolicy
from .policy.base import Budget, Policy
from .policy.coverage import CoveragePolicy
from .spacing import CoverageBand, coverage_band


def policy_for(condition: str) -> Policy:
    if condition == "C1":
        return CoveragePolicy(1)
    if condition == "C2":
        return CoveragePolicy(2)
    if condition == "C3":
        return AppendOnCompletionPolicy()
    if condition.startswith("C4"):
        from .policy.informative import informative_policy   # C4 family
        return informative_policy(condition)
    raise MissionError(f"unknown condition {condition!r}")


@dataclass
class MissionSetup:
    area: Area
    band: CoverageBand
    coverage: CoveragePlans
    policy: Policy
    budget: Budget
    initial_plan: object


def setup_mission(cfg, condition: str, areas_latlon, datum_lat: float, datum_lon: float,
                  hold_pose_m, t0: float = 0.0) -> MissionSetup:
    polys = areas_to_map(areas_latlon, datum_lat, datum_lon)
    if len(polys) != 1:
        raise MissionError("exactly one area is supported ([OPEN] O-8)")
    e = cfg.margin_m + cfg.r_turn_m
    area = Area(polys[0], min_area=(2.0 * e) ** 2)
    band = coverage_band(cfg, cfg.planning_depth_m)
    cov = build_coverage(area, hold_pose_m, band, cfg, t0=t0)
    policy = policy_for(condition)
    budget = Budget(t_start=t0, B=cov.c2.duration)
    return MissionSetup(area, band, cov, policy, budget, policy.initial_plan(cov))
