"""Gate section [25]: detection emulator: looks and seabed images (eval/emulator.py).

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [25]
print("[25] detection emulator: looks and seabed images (eval/emulator.py)")
from blueboat_replanner.eval.emulator import (ImageCutter, LookTracker,  # noqa: E402
                                              TurnRate, image_stride_m, reported_class,
                                              side_offsets)
D_ = image_stride_m(20.0)
cut = ImageCutter(D_)
imgs = []
for i in range(2001):                                   # 100 m at 0.5 m/s, a ping every 0.1 s
    imgs += cut.add_ping(1000.0 + 0.1 * i, 0.05 * i, 0.0)
lastI = cut.flush()
check("image stride = 128 rows of range/1200 m; one image per stride, two strides long",
      abs(D_ - 128 * 20.0 / 1200) < 1e-12 and len(imgs) == int(100.0 // D_) - 1 and
      all(abs((im.t_max - im.t_min) * 0.5 - 2 * D_) < 0.11 for im in imgs) and
      [im.index for im in imgs] == list(range(len(imgs))))
check("images: each overlaps the previous one and starts after the one before that ends",
      all(imgs[m].t_min < imgs[m - 1].t_max for m in range(1, len(imgs))) and
      all(imgs[m].t_min > imgs[m - 2].t_max for m in range(2, len(imgs))))
check("flush: the last, shorter image holds every ping not yet in a full image, once",
      lastI.index == len(imgs) and lastI.t_max == 1000.0 + 200.0 and
      lastI.t_min > imgs[-2].t_max and cut.flush() is None)
offs25 = side_offsets(1.78, 20.0, 0.285)
check("side offsets: from the lever arm out to the geometric limit plus the lever arm",
      0.285 < offs25[0] < 0.6 and abs(offs25[-1] - (math.sqrt(400 - 1.78 ** 2) + 0.285)) < 1e-9
      and bool(np.all(np.diff(offs25) > 0)))


def tracker25(objs_, turn_=None):
    return LookTracker(objs_, 0.0, -20.0, 0.5, 200, 80, 12, 0.0, offs25, 0.285, 20.0,
                       turn_ or TurnRate())


trk = tracker25([types.SimpleNamespace(oid=7, x=20.0, y=8.0)])
hits25 = []
for i in range(400):                                    # East along y = 0, x from 10 to 30
    hits25 += trk.observe(100.0 + 0.1 * i, 10.0 + 0.05 * i, 0.0, 0.0)
h0 = hits25[0][1]
check("tracker: an object abeam to port, heading East = one look from sector 3, at its range "
      "from the transducer", len({id(h.look) for _, h in hits25}) == 1 and h0.look.sector == 3
      and h0.opened and abs(h0.r - (8.0 - 0.285)) < 0.3 and trk.counts[0, 3] == 1 and
      len(hits25) > 5, f"{len(hits25)} hits, r {h0.r}")
t_hit = hits25[-1][1].t
again = trk.observe(t_hit + 19.0, 20.25, 0.0, 0.0)      # abeam again, inside the time-out
check("tracker: within the look time-out the look continues", len(again) == 1 and
      not again[0][1].opened and trk.n_looks[0] == 1)
for i in range(400):                                    # back West, 45 s later
    hits25 += trk.observe(185.0 + 0.1 * i, 30.0 - 0.05 * i, 0.0, math.pi)
check("tracker: the same sector after more than the look time-out is a new look",
      trk.counts[0, 3] == 2 and trk.n_looks[0] == 2 and hits25[-1][1].look.number == 2)


def circle25(trk_):
    out = []
    for i in range(400):                                # a left circle of radius 3 m at 0.5 m/s
        a_ = 0.1 * i * 0.5 / 3.0
        out += trk_.observe(500.0 + 0.1 * i, 20.0 + 3 * math.cos(a_), 3 * math.sin(a_),
                            a_ + math.pi / 2)
    return [h.r + 0.285 for _, h in out]


objC = [types.SimpleNamespace(oid=1, x=20.0, y=-1.0)]   # inside the circle: inner side only
r_gate, r_free = circle25(tracker25(objC)), circle25(tracker25(objC, TurnRate(enabled=False)))
check("tracker: in a turn the inner side beyond the turn radius gives no look (the gate)",
      len(r_gate) > 0 and max(r_gate) <= 3.0 + 0.3 and max(r_free) > 3.6 and
      len(r_free) > len(r_gate), f"{max(r_gate):.2f} {max(r_free):.2f}")
w25 = types.SimpleNamespace(seed=3, params={"classes": {"block": 0, "pipe": 0, "cage": 0}})
tgt = types.SimpleNamespace(oid=1, kind="target", cls="pipe", label="")
clu = types.SimpleNamespace(oid=2, kind="clutter", cls="clutter", label="")
check("reported class: 'none' says nothing; 'true' = the target's class, one fixed class per "
      "clutter object", reported_class(w25, tgt, "none") == "unclassified" and
      reported_class(w25, tgt, "true") == "pipe" and
      reported_class(w25, clu, "true") == reported_class(w25, clu, "true") and
      reported_class(w25, clu, "true") in ("block", "pipe", "cage") and
      raises(lambda: reported_class(w25, tgt, "maybe"), ValueError))

