"""Shared part of the offline gate: imports, `check` / `skip` / `raises` / `sibling` and the
fixtures every section uses. Executed first by `test/smoke_test.py`, in the namespace the
sections then share."""

from __future__ import annotations

import ast
import inspect
import math
import os
import sys
import tempfile
import types
from pathlib import Path

import numpy as np

sys.dont_write_bytecode = True
PKG_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG_ROOT))

from blueboat_replanner.core import geo as geo_mod  # noqa: E402
from blueboat_replanner.core.areas import Area, MissionError  # noqa: E402
from blueboat_replanner.core.config import (CONTROL_RULES, PARAMS, Config,  # noqa: E402
                                            ConfigError)
from blueboat_replanner.core.coverage import (build_coverage,  # noqa: E402
                                              coverage_fraction, line_visit_order)
from blueboat_replanner.core.dubins import (Arc, dubins_candidates,  # noqa: E402
                                            sample_prims)
from blueboat_replanner.core.evidence import (ContractError,  # noqa: E402
                                              validate_grid)
from blueboat_replanner.core.geo import Frame2D  # noqa: E402
from blueboat_replanner.core.mission_plan import policy_for, setup_mission  # noqa: E402
from blueboat_replanner.core.plan import (ControlPathMirror, Plan,  # noqa: E402
                                          control_eval, hold_segment)
from blueboat_replanner.core.policy.append import last_pass1_line  # noqa: E402
from blueboat_replanner.core.policy.base import Budget, EpochContext  # noqa: E402
from blueboat_replanner.core.routing import connect, prims_feasible  # noqa: E402
from blueboat_replanner.core.spacing import (coverage_band, multiplicity,  # noqa: E402
                                             nominal_spacing, spacing_for_width)
from blueboat_replanner.core.validate import (continuity_check,  # noqa: E402
                                              plan_in_area, request_check)
from blueboat_replanner.io.mission import parse_mission  # noqa: E402
from blueboat_replanner.io.trajectory_yaml import (trajectory_doc,  # noqa: E402
                                                   write_exclusive)
from blueboat_replanner.ros.adapters import grid_to_dict  # noqa: E402

PASS = FAIL = SKIP = 0


def check(name: str, cond, detail: str = "") -> None:
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok    {name}")
    else:
        FAIL += 1
        print(f"  FAIL  {name}  {detail}")


def skip(name: str, why: str) -> None:
    global SKIP
    SKIP += 1
    print(f"  SKIP  {name}  ({why})")


def raises(fn, exc, field=None) -> bool:
    try:
        fn()
    except exc as e:
        return field is None or field in str(e) or getattr(e, "field", None) == field
    return False


def sibling(name: str):
    """A neighbour repository: env override, sibling of this repo's git root, or the
    superproject layout (sibling submodule)."""
    env = os.environ.get("BLUEBOAT_" + name.upper().replace("-", "_") + "_DIR")
    cands = [Path(env)] if env else []
    root = PKG_ROOT.parent
    cands += [root.parent / name, root.parent.parent / name]
    for c in cands:
        if c.is_dir() and any(c.iterdir()):
            return c
    return None


CFG = Config.from_dict({"planning_depth_m": 2.0})
RECT = Area([(0, 0), (120, 0), (120, 60), (0, 60)])
HOLD = (10.0, 10.0, 0.3)
