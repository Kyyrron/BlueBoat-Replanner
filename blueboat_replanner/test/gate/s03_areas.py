"""Gate section [3]: areas.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [3]
print("[3] areas")
cw = Area([(0, 0), (0, 10), (10, 10), (10, 0)])
check("clockwise input normalised to CCW", geo_mod.wrap_scalar(0) == 0 and
      cw.vertices[1][0] == 10.0 and abs(cw.area - 100.0) < 1e-9)
check("closing duplicate removed", len(Area([(0, 0), (10, 0), (10, 10), (0, 0)]).vertices) == 3)
check("self-intersecting raises", raises(lambda: Area([(0, 0), (10, 10), (10, 0), (0, 10)]),
                                         MissionError, "not simple"))
check("< 3 vertices raises", raises(lambda: Area([(0, 0), (1, 1)]), MissionError))
check("zero area raises", raises(lambda: Area([(0, 0), (1, 1), (2, 2)]), MissionError))
check("min area enforced", raises(lambda: Area([(0, 0), (5, 0), (5, 5), (0, 5)], min_area=100),
                                  MissionError, "minimum"))
pts = np.array([[60, 30], [-1, 30], [60, 61], [119.9, 59.9]])
check("contains", list(RECT.contains(pts)) == [True, False, False, True])
check("boundary distance exact", np.allclose(RECT.boundary_distance([[60, 30], [5, 30],
                                                                     [3, 4]]), [30, 5, 3]))
check("F_d threshold", list(RECT.feasible([[2.0, 30], [1.999, 30]], 2.0)) == [True, False])
check("long axis of 120x60 rectangle = 0", abs(RECT.long_axis_direction()) < 1e-12)
rot = math.radians(30)
R2 = np.array([[math.cos(rot), -math.sin(rot)], [math.sin(rot), math.cos(rot)]])
rr = Area((R2 @ RECT.vertices.T).T)
check("long axis follows a 30 deg rotation", abs(rr.long_axis_direction() - rot) < 1e-9)
L_AREA = Area([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])
check("L-shape has one reflex vertex", L_AREA.reflex_vertices() == [3])
off = L_AREA.reflex_offset_points(5.0)
check("reflex offset point at distance >= d", len(off) == 1 and
      L_AREA.boundary_distance(off)[0] >= 5.0 - 1e-9 and bool(L_AREA.contains(off)[0]))

