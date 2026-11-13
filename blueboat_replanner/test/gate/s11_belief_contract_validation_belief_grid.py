"""Gate section [11]: Belief contract validation (belief_grid/0).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [11]
def make_grid(contacts=(), S=12, W=260, H=140, res=0.5, ox=-5.0, oy=-5.0, cov_val=0.3):
    n = W * H
    cv = np.full(n, cov_val)
    ac = np.zeros(n * S, dtype=np.int64)
    return {
        "contract_version": "belief_grid/0", "grid_seq": 7, "width": W, "height": H,
        "resolution": res, "origin_x": ox, "origin_y": oy, "datum_lat": 34.5,
        "datum_lon": 133.5, "odom_to_map_x": 0.0, "odom_to_map_y": 0.0,
        "odom_to_map_yaw": 0.0, "odom_to_map_sigma_m": 0.5, "odom_to_map_valid": True,
        "aspect_sector_count": S, "aspect_sector_origin_deg": 0.0,
        "class_names": ["block", "pipe", "cage", "tyre", "plate"],
        "prior_density_per_ha": 20.0, "model_params_json": '{"schema": 1}',
        "p_background": np.full(n, 0.001), "coverage": cv, "aspect_counts": ac,
        "aspect_log_miss": np.zeros(n * S),
        "area_x": [ox, ox + W * res, ox + W * res, ox], "area_y": [oy, oy, oy + H * res,
                                                                    oy + H * res],
        "depth_m": np.full(n, 2.0), "contacts": list(contacts)}


def set_searched(g, sectors=None, mask=None, pd=0.7):
    """Mark sectors as searched in a fixture grid: `aspect_counts` and `aspect_log_miss`."""
    S_ = g["aspect_sector_count"]
    ac_ = g["aspect_counts"].reshape(-1, S_)
    alm_ = g["aspect_log_miss"].reshape(-1, S_)
    if mask is None:
        mask = np.zeros(ac_.shape, bool)
        mask[:, list(sectors)] = True
    ac_[mask] = 1
    alm_[mask] = math.log(1.0 - pd)


def contact(uid, x, y, first_sector=0, state=0, fc=0.6):
    return {"uid": uid, "x": x, "y": y, "covariance": [1.0, 0.0, 0.0, 1.0], "p_exists": 0.5,
            "state": state, "class_probabilities": [0.2] * 5, "sectors_detected": [first_sector],
            "sectors_missed": [], "n_looks_detected": 1, "first_confidence": fc,
            "first_sector": first_sector, "max_confidence": fc, "shadow_known": False,
            "has_shadow": False}


print("[11] Belief contract validation (belief_grid/0)")
G = make_grid([contact(1, 40, 30), contact(2, 80, 20, first_sector=3)])
snap = validate_grid(G)
check("valid grid passes", len(snap.contacts) == 2 and snap.coverage.shape == (140, 260))
muts = [("contract_version", lambda g: g.update(contract_version="belief_grid/1")),
        ("coverage", lambda g: g.update(coverage=g["coverage"][:-1])),
        ("aspect_counts", lambda g: g.update(aspect_counts=g["aspect_counts"][:-3])),
        ("coverage", lambda g: g["coverage"].__setitem__(5, 1.5)),
        ("odom_to_map_valid", lambda g: g.update(odom_to_map_valid=False)),
        ("p_background", lambda g: g["p_background"].__setitem__(3, np.nan)),
        ("contacts", lambda g: g.update(contacts=[contact(1, 0, 0), contact(1, 1, 1)])),
        ("covariance", lambda g: g["contacts"][0].update(covariance=[1, 0, 0, -1])),
        ("sectors_detected", lambda g: g["contacts"][0].update(sectors_detected=[3, 1])),
        ("first_sector", lambda g: g["contacts"][0].update(first_sector=12)),
        ("class_probabilities", lambda g: g["contacts"][0].update(class_probabilities=[1, 0])),
        ("datum_lat", lambda g: g.pop("datum_lat")),
        ("class_names", lambda g: g.update(class_names=["a", "a"]))]
for field, mut in muts:
    g = make_grid([contact(1, 40, 30)])
    mut(g)
    check(f"ContractError names '{field}'", raises(lambda: validate_grid(g), ContractError, field))
g = make_grid([contact(1, 40, 30, fc=float("nan"))])
g["contacts"][0]["first_sector"] = 255
g.pop("p_background")
sn = validate_grid(g)
check("degradable fields -> recorded fallbacks", "first_confidence_missing" in
      sn.contacts[0].fallbacks and "first_sector_unknown" in sn.contacts[0].fallbacks and
      "p_background_missing" in sn.fallbacks)
obj = types.SimpleNamespace(**{k: v for k, v in G.items() if k != "contacts"})
obj.contacts = [types.SimpleNamespace(**c) for c in G["contacts"]]
obj.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=12, nanosec=5e8))
sa = validate_grid(grid_to_dict(obj))
sb = validate_grid(grid_to_dict(G))
check("adapter: attribute object == dict", sa.contacts[1].uid == sb.contacts[1].uid and
      np.array_equal(sa.coverage, sb.coverage) and sa.stamp == 12.5)
check("sectors: 29.9 -> 0, 30.1 -> 1, -0.1 -> 11, centre(0) = 15 deg",
      list(snap.sector_of(np.radians([29.9, 30.1, -0.1]))) == [0, 1, 11] and
      abs(math.degrees(snap.sector_centre(0)) - 15.0) < 1e-12)
check("sectors: a bearing on an edge goes to the upper sector (30, 90, 180, 270, 360 deg)",
      list(snap.sector_of(np.radians([30.0, 90.0, 180.0, 270.0, 360.0]))) == [1, 3, 6, 9, 0] and
      list(snap.sector_of([math.pi / 2, -math.pi / 2, math.pi])) == [3, 9, 6])
g = make_grid([contact(1, 40, 30)])
g["aspect_log_miss"][7] = 0.2
check("ContractError names 'aspect_log_miss' (a positive value)",
      raises(lambda: validate_grid(g), ContractError, "aspect_log_miss"))
g = make_grid([contact(1, 40, 30)])
set_searched(g, sectors=[3])
g["aspect_log_miss"][:12] = np.nan
sn = validate_grid(g)
check("H-1: searched = aspect_log_miss < 0 (NaN outside the area is not searched)",
      sn.searched.shape == (140, 260, 12) and bool(sn.searched[5, 5, 3]) and
      not bool(sn.searched[5, 5, 0]) and not sn.searched[0, 0].any() and
      abs(sn.miss_pd[5, 5, 3] - 0.7) < 1e-12)
g = make_grid([contact(1, 40, 30)])
g.pop("aspect_log_miss")
g.pop("area_x")
sn = validate_grid(g)
check("aspect_log_miss / area absent -> recorded fallbacks",
      sn.searched is None and "searched_from_aspect_counts" in sn.fallbacks and
      "area_polygon_missing" in sn.fallbacks and sn.area_xy is None)
g = make_grid([contact(1, 40, 30)])
g["model_params_json"] = ('{"schema": 1, "detector_calibrated": false, "negative_evidence": '
                          '"gated", "coverage_source": "records", "look_timeout_s": 20.0, '
                          '"rho_model": {"ordering": "paull_pairwise", "fold_opposite": true}, '
                          '"turn_gating": {"enabled": true, "min_yaw_rate_rad_s": 0.01}}')
sn = validate_grid(g)
check("model_params_json: the keys read are parsed",
      sn.model["detector_calibrated"] is False and sn.model["negative_evidence"] == "gated" and
      sn.model["coverage_source"] == "records" and sn.model["rho_ordering"] == "paull_pairwise"
      and sn.model["fold_opposite"] is True and sn.model["look_timeout_s"] == 20.0 and
      sn.model["turn_gating"]["enabled"] is True and
      "detector_calibrated_unknown" not in sn.fallbacks)
g["model_params_json"] = "{not json"
check("ContractError names 'model_params_json' (not JSON)",
      raises(lambda: validate_grid(g), ContractError, "model_params_json"))
g = make_grid([])
g["model_params_json"] = ""
check("model_params_json empty -> fallback, every key None",
      "model_params_missing" in validate_grid(g).fallbacks)
g = make_grid([contact(1, 40, 30)])
g["grid_seq"] = np.uint32(9)
g["header"] = {"stamp": 1234.5, "frame_id": "map"}
sn = validate_grid(grid_to_dict(g))
check("adapter: Belief's to_grid_dict form (float header stamp, numpy scalars) is read",
      sn.stamp == 1234.5 and sn.grid_seq == 9 and sn.area_xy.shape == (4, 2))
from blueboat_replanner.core.evidence import mission_area_check  # noqa: E402
okA, whyA = mission_area_check(RECT, snap)
g = make_grid([])
g["area_x"], g["area_y"] = [0.0, 100.0, 100.0, 0.0], [0.0, 0.0, 60.0, 60.0]
okB, whyB = mission_area_check(RECT, validate_grid(g))
g = make_grid([])
g["coverage"].reshape(140, 260)[:, 200:] = np.nan
g["p_background"].reshape(140, 260)[:, 200:] = np.nan
okC, whyC = mission_area_check(RECT, validate_grid(g))
check("H-5: mission polygon inside Belief's area -> ok; outside its polygon or over cells "
      "with no belief data -> refused with the reason",
      okA and not okB and "outside" in whyB and not okC and "no belief data" in whyC,
      f"{whyA} | {whyB} | {whyC}")
from blueboat_replanner.core.model import KEY_STRIDE  # noqa: E402
check("look keys cell*KEY_STRIDE+g: a grid whose folded sector count reaches the stride is "
      "refused, naming aspect_sector_count; one below it is read",
      raises(lambda: validate_grid(make_grid(S=2 * KEY_STRIDE, W=20, H=20)), ContractError,
             "aspect_sector_count") and
      validate_grid(make_grid(S=2 * KEY_STRIDE - 2, W=20, H=20)).sector_count ==
      2 * KEY_STRIDE - 2)

