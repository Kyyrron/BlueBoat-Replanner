"""Gate section [18]: union = sequential identity (MATH §11.1) and the worked example.

Executed by `test/smoke_test.py` in the gate's shared namespace: the names of `common.py` and of the earlier sections are visible."""
# flake8: noqa

# =========================================================================== [18]
print("[18] union = sequential identity (MATH §11.1) and the worked example")
rng = np.random.default_rng(18)
wi = 0.0
for _ in range(100):
    cp = rng.dirichlet(np.ones(5))
    cs_r = contact_posterior(one_contact(cp, det=(int(rng.integers(0, 12)),), fs=None or 0,
                                         q0=float(rng.uniform(.05, .95))), mod, CFG, 0)
    free = [g for g in range(6) if g not in cs_r.seen_g]
    if len(free) < 3:
        continue
    N1 = {free[0]: float(rng.uniform(.2, .7))}
    N2 = {free[1]: float(rng.uniform(.2, .7)), free[2]: float(rng.uniform(.2, .7))}
    U = contact_value(cs_r, mod, CFG, {**N1, **N2})
    pm = 1 - detection_probability(cs_r, mod, CFG, N1)
    Sq = contact_value(cs_r, mod, CFG, N1) + pm * contact_value(after_miss(cs_r, mod, CFG, N1),
                                                                 mod, CFG, N2)
    wi = max(wi, abs(U - Sq))
check("C-1: V(N1 u N2) = V(N1) + P(miss) V'(N2) (contacts, 100 random)", wi < 1e-12, f"{wi}")
B_, r_ = 100.0, 0.02
J = lambda v, t: v + r_ * (B_ - t)   # noqa: E731
rateA, rateB = 1.0 / 10, 0.85 / 10
own_A = J(1.0 + 0.5, 20)
own_B = max([(0.3 / 1, J(0.85 + 0.3, 11)), (1.2 / 5, J(0.85 + 1.2, 15))])[1]
comb_B = max(J(0.85 + 0.3, 11), J(0.85 + 1.2, 15))
check("C-2: worked example: individual rate -> A; a2 by own rate -> A; combined -> B",
      rateA > rateB and own_A > own_B and comb_B > own_A and abs(comb_B - 3.75) < 1e-12 and
      abs(own_A - 3.1) < 1e-12 and abs(own_B - 2.93) < 1e-12)

