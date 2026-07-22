"""ML offline study on DUNE static captures.

Data: static_capture.txt (train era, tags 240@216 / 241@848 sweeps) and
static_verify_capture.txt (held-out session). Phase-C geometry: rect6p3x3p0
layout, A1 dead, anchor_bias.json additive corrections.

Truth available: parked tags (position pseudo-truth = robust median of fixes;
conservative variant uses OLD-solver median) and the physical inter-tag
baseline 0.527 m in static_verify (camera-independent invariant).

Experiments:
  E0  old solver vs new solver replay (accuracy, iterations, wall time)
  E1  supervised per-anchor range-error regression on diagnostics
      (linear-in-fp refit vs gradient boosting), time-split + cross-session
  E2  unsupervised anomaly weighting (Mahalanobis / IsolationForest) vs
      the 3 dB gap heuristic
  E3  solver hyperparameter tuning (huberDelta, TRIMW, gap params) on the
      train era, scored on held-out
  E4  EKF (CV + position update, Huber-gated) replay: default vs tuned
"""
import json, time
import numpy as np
from parse import parse_capture, load_layout, load_bias, sweep_matrix, REPO

rng = np.random.default_rng(0)
BASELINE_TRUE = 0.527
TAGZ = 0.24   # Phase-C plate height

# ---------------------------------------------------------------- solvers ---
def resid_jac(p, A, d, dz2):
    dx = p[0] - A[:, 0]; dy = p[1] - A[:, 1]
    pred = np.maximum(np.sqrt(dx**2 + dy**2 + dz2), 1e-6)
    return pred - d, np.column_stack([dx / pred, dy / pred])

def lm_core(A, d, w, tagZ, x0, max_iter, delta, robust):
    dz2 = (A[:, 2] - tagZ) ** 2
    wb = w**2
    p = np.asarray(x0, float).copy()
    lam = 1e-3
    r, J = resid_jac(p, A, d, dz2)
    def cost(r):
        if not robust or np.isinf(delta):
            return np.sum(wb * r**2)
        return np.sum(wb * delta**2 * (np.sqrt(1 + (r / delta) ** 2) - 1))
    def hw(r):
        if not robust or np.isinf(delta):
            return np.ones_like(r)
        return 1.0 / np.sqrt(1 + (r / delta) ** 2)
    u = hw(r); c = cost(r); it = 0
    for it in range(1, max_iter + 1):
        v = wb * u
        Jw = J * v[:, None]
        H = Jw.T @ J; g = Jw.T @ r
        try:
            step = -np.linalg.solve(H + lam * np.diag(np.diag(H)), g)
        except np.linalg.LinAlgError:
            break
        pn = p + step
        rn, Jn = resid_jac(pn, A, d, dz2)
        cn = cost(rn)
        if cn < c:
            p, r, J, c = pn, rn, Jn, cn
            u = hw(r)
            lam = max(lam / 3, 1e-9)
            if np.linalg.norm(step) < 1e-6:
                break
        else:
            lam *= 5
            if lam > 1e6:
                break
    return p, r, it, u

def srls_seed(A, d, w, tagZ):
    d2 = np.maximum(d**2 - (A[:, 2] - tagZ) ** 2, 0)
    ref = int(np.argmax(w))
    i = np.ones(len(d), bool); i[ref] = False
    G = 2 * (A[i, :2] - A[ref, :2])
    h = (np.sum(A[i, :2] ** 2, 1) - np.sum(A[ref, :2] ** 2)) - (d2[i] - d2[ref])
    Gw = G * w[i][:, None]
    H = Gw.T @ G
    if np.linalg.cond(H) < 1e9:
        return np.linalg.solve(H, Gw.T @ h)
    return (A[:, :2] * w[:, None]).sum(0) / w.sum()

def screen(A, d, slack=0.5):
    Dm = np.linalg.norm(A[:, None, :] - A[None, :, :], axis=2)
    V = (np.abs(d[:, None] - d[None, :]) > Dm + slack) | (d[:, None] + d[None, :] < Dm - slack)
    return V.sum(1) > (len(d) - 1) / 2

def solve_new(A, d, w, tagZ=TAGZ, x0=None, delta=0.15, trimw=0.25):
    ok = np.isfinite(d) & (w > 0)
    A_, d_, w_ = A[ok], d[ok], w[ok]
    if len(d_) < 3:
        return np.array([np.nan, np.nan]), 0
    bad = screen(A_, d_)
    if (~bad).sum() < 3:
        return np.array([np.nan, np.nan]), 0
    A_, d_, w_ = A_[~bad], d_[~bad], w_[~bad]
    sv = np.linalg.svd(A_[:, :2] - A_[:, :2].mean(0), compute_uv=False)
    if sv[1] < 0.3:
        return np.array([np.nan, np.nan]), 0
    x0 = srls_seed(A_, d_, w_, tagZ) if x0 is None or not np.all(np.isfinite(x0)) else x0
    p, r, it, u = lm_core(A_, d_, w_, tagZ, x0, 50, delta, True)
    if np.isfinite(p).all() and (u < trimw).any() and len(d_) - (u < trimw).sum() >= 3:
        keep = u >= trimw
        p, r, it2, u = lm_core(A_[keep], d_[keep], w_[keep], tagZ, p, 50, delta, True)
        it += it2
    return p, it

def solve_old(A, d, w, tagZ=TAGZ, x0=None, gateK=3.0):
    ok = np.isfinite(d) & (w > 0)
    A_, d_, w_ = A[ok], d[ok], w[ok]
    if len(d_) < 3:
        return np.array([np.nan, np.nan]), 0
    sv = np.linalg.svd(A_[:, :2] - A_[:, :2].mean(0), compute_uv=False)
    if sv[1] < 0.3:
        return np.array([np.nan, np.nan]), 0
    if x0 is None or not np.all(np.isfinite(x0)):
        x0 = (A_[:, :2] * w_[:, None]).sum(0) / w_.sum()
    p, r, it, _ = lm_core(A_, d_, w_, tagZ, x0, 50, np.inf, False)
    if gateK > 0 and len(d_) > 3 and np.isfinite(p).all():
        mad = 1.4826 * np.median(np.abs(r - np.median(r)))
        keep = np.abs(r) <= max(gateK * mad, 0.02)
        if (~keep).any() and keep.sum() >= 3:
            p, r, it2, _ = lm_core(A_[keep], d_[keep], w_[keep], tagZ, p, 50, np.inf, False)
            it += it2
    return p, it

def gap_weights(gap, thresh=3.0, floor=0.05):
    w = np.maximum(floor, 10 ** (-np.maximum(0, gap - thresh) / 10))
    w[np.isnan(gap)] = 1.0
    return w

# ------------------------------------------------------------------- data ---
ids, APOS = load_layout("anchors_rect6p3x3p0_2026-07-01.json")
AB, TB = load_bias()
abias = np.array([AB[int(i)] for i in ids])

def prep(capname):
    m = sweep_matrix(parse_capture(f"{REPO}/matlab/results/{capname}"), ids)
    tbias = np.array([TB.get(int(t), 0.0) for t in m["tag"]])
    m["C"] = m["R"] - abias[None, :] - tbias[:, None]   # corrected ranges
    m["GAP"] = m["RX"] - m["FP"]
    return m

def replay(m, solver, weights=None, extracorr=None, **kw):
    n = len(m["t"])
    P = np.full((n, 2), np.nan); iters = np.zeros(n, int)
    prev = {}
    C = m["C"] if extracorr is None else m["C"] - extracorr
    W = gap_weights(m["GAP"]) if weights is None else weights
    t0 = time.perf_counter()
    for i in range(n):
        tg = m["tag"][i]
        p, it = solver(APOS, C[i], np.nan_to_num(W[i], nan=1.0), x0=prev.get(tg), **kw)
        P[i] = p; iters[i] = it
        if np.isfinite(p).all():
            prev[tg] = p
    dt = time.perf_counter() - t0
    return P, iters, dt

def metrics(P, m, truth):
    out = {}
    for tg in (240, 241):
        s = (m["tag"] == tg) & np.isfinite(P[:, 0])
        if s.sum() < 5:
            continue
        e = np.linalg.norm(P[s] - truth[tg], axis=1)
        out[tg] = dict(n=int(s.sum()), rmse=np.sqrt((e**2).mean()) * 1e3,
                       p95=np.percentile(e, 95) * 1e3, mx=e.max() * 1e3)
    return out

def baseline_err(P, m, truth):
    """Per-sweep fix of one tag vs the OTHER tag's pseudo-truth: distance
    should equal 0.527 m (both parked). Camera-independent accuracy metric."""
    errs = []
    for tg, other in ((240, 241), (241, 240)):
        s = (m["tag"] == tg) & np.isfinite(P[:, 0])
        d = np.linalg.norm(P[s] - truth[other], axis=1)
        errs.append(np.abs(d - BASELINE_TRUE))
    e = np.concatenate(errs)
    return dict(rmse=np.sqrt((e**2).mean()) * 1e3, p95=np.percentile(e, 95) * 1e3)

def pseudo_truth(P, m):
    return {tg: np.nanmedian(P[m["tag"] == tg], axis=0) for tg in (240, 241)}

cap = prep("static_capture.txt")
ver = prep("static_verify_capture.txt")

# ---------------------------------------------------------------------- E0 --
print("=" * 72)
print("E0  old vs new solver replay")
for name, mm in (("static_capture", cap), ("static_verify", ver)):
    Po, io, to = replay(mm, solve_old)
    Pn, in_, tn = replay(mm, solve_new)
    truth_o = pseudo_truth(Po, mm)     # conservative: OLD solver's own median
    mo, mn = metrics(Po, mm, truth_o), metrics(Pn, mm, truth_o)
    print(f"-- {name}: old {io.mean():.1f} it/solve {1e3*to/len(io):.2f} ms | "
          f"new {in_.mean():.1f} it {1e3*tn/len(in_):.2f} ms")
    for tg in mo:
        print(f"   tag{tg}  scatter-rmse old {mo[tg]['rmse']:6.1f}  new {mn[tg]['rmse']:6.1f} mm"
              f"   p95 old {mo[tg]['p95']:6.1f} new {mn[tg]['p95']:6.1f}"
              f"   max old {mo[tg]['mx']:7.1f} new {mn[tg]['mx']:7.1f}")
    bo, bn = baseline_err(Po, mm, truth_o), baseline_err(Pn, mm, truth_o)
    print(f"   baseline-err rmse old {bo['rmse']:.1f} new {bn['rmse']:.1f} mm  "
          f"p95 old {bo['p95']:.1f} new {bn['p95']:.1f}")

# Reference positions for the rest of the study: robust median of new fixes.
P_cap, _, _ = replay(cap, solve_new)
P_ver, _, _ = replay(ver, solve_new)
T_cap, T_ver = pseudo_truth(P_cap, cap), pseudo_truth(P_ver, ver)
print("pseudo-truth cap:", {k: np.round(v, 4).tolist() for k, v in T_cap.items()})
print("pseudo-truth ver:", {k: np.round(v, 4).tolist() for k, v in T_ver.items()})

# ---------------------------------------------------------------------- E1 --
print("=" * 72)
print("E1  supervised range-error regression on diagnostics")
from sklearn.ensemble import HistGradientBoostingRegressor
from sklearn.linear_model import LinearRegression

def anchor_residuals(m, truth):
    """Per-measurement signed range error vs pseudo-truth position."""
    n, M = m["C"].shape
    E = np.full((n, M), np.nan)
    for tg in (240, 241):
        s = m["tag"] == tg
        d_true = np.linalg.norm(APOS - np.r_[truth[tg], TAGZ], axis=1)
        E[s] = m["C"][s] - d_true[None, :]
    return E

E_cap, E_ver = anchor_residuals(cap, T_cap), anchor_residuals(ver, T_ver)

# time split within static_capture: first 60% train
n_cap = len(cap["t"]); cut = int(0.6 * n_cap)
corr_cap = np.zeros_like(E_cap); corr_ver = np.zeros_like(E_ver)
print(f"{'anchor':>6} {'set':>12} {'raw':>7} {'lin-fp':>7} {'gbm':>7}  (residual RMSE mm)")
for c in range(len(ids)):
    def feats(m):
        return np.column_stack([m["FP"][:, c], m["RX"][:, c], m["GAP"][:, c], m["Q"][:, c]])
    Xa, ya = feats(cap), E_cap[:, c]
    tr = np.isfinite(ya) & np.all(np.isfinite(Xa), 1) & (np.arange(n_cap) < cut)
    if tr.sum() < 50:
        continue
    lin = LinearRegression().fit(Xa[tr][:, :1], ya[tr])            # current method: linear in fp
    gbm = HistGradientBoostingRegressor(max_iter=200, max_depth=3,
                                        random_state=0).fit(Xa[tr], ya[tr])
    for setname, X, y, mask_extra in (
            ("cap-test", Xa, ya, np.arange(n_cap) >= cut),
            ("verify", feats(ver), E_ver[:, c], np.ones(len(ver["t"]), bool))):
        te = np.isfinite(y) & np.all(np.isfinite(X), 1) & mask_extra
        if te.sum() < 30:
            continue
        raw = np.sqrt(np.mean(y[te] ** 2))
        rl = np.sqrt(np.mean((y[te] - lin.predict(X[te][:, :1])) ** 2))
        rg = np.sqrt(np.mean((y[te] - gbm.predict(X[te])) ** 2))
        print(f"A{ids[c]:<5} {setname:>12} {1e3*raw:7.1f} {1e3*rl:7.1f} {1e3*rg:7.1f}")
    # store predicted corrections for the position-level test
    okc = np.all(np.isfinite(Xa), 1); corr_cap[okc, c] = gbm.predict(Xa[okc])
    Xv = feats(ver); okv = np.all(np.isfinite(Xv), 1); corr_ver[okv, c] = gbm.predict(Xv[okv])

print("-- position-level effect of GBM correction (held-out):")
for name, m, T, corr, mask in (("cap-test", cap, T_cap, corr_cap, np.arange(n_cap) >= cut),
                               ("verify", ver, T_ver, corr_ver, None)):
    P0, _, _ = replay(m, solve_new)
    P1, _, _ = replay(m, solve_new, extracorr=corr)
    if mask is not None:
        P0, P1 = P0.copy(), P1.copy()
        P0[~mask] = np.nan; P1[~mask] = np.nan
    m0, m1 = metrics(P0, m, T), metrics(P1, m, T)
    for tg in m0:
        print(f"   {name} tag{tg}: scatter-rmse {m0[tg]['rmse']:6.1f} -> {m1[tg]['rmse']:6.1f} mm"
              f"   p95 {m0[tg]['p95']:6.1f} -> {m1[tg]['p95']:6.1f}")
    b0, b1 = baseline_err(P0, m, T), baseline_err(P1, m, T)
    print(f"   {name} baseline-err rmse {b0['rmse']:.1f} -> {b1['rmse']:.1f} mm")

# drift predictability check: can diagnostics explain the slow wander?
print("-- drift predictability (tag241, largest anchor set):")
for c in range(len(ids)):
    y = E_cap[cap["tag"] == 241, c]
    X = np.column_stack([cap["FP"][cap["tag"] == 241, c], cap["RX"][cap["tag"] == 241, c],
                         cap["Q"][cap["tag"] == 241, c]])
    ok = np.isfinite(y) & np.all(np.isfinite(X), 1)
    if ok.sum() < 100:
        continue
    yc = y[ok] - y[ok].mean()
    ntr = int(0.6 * ok.sum())
    g = HistGradientBoostingRegressor(max_iter=150, max_depth=3, random_state=0)
    g.fit(X[ok][:ntr], yc[:ntr])
    ss = 1 - np.mean((yc[ntr:] - g.predict(X[ok][ntr:])) ** 2) / np.var(yc[ntr:])
    print(f"   A{ids[c]}: residual std {1e3*np.std(yc):.0f} mm, held-out R^2 of diagnostics {ss:+.2f}")

# ---------------------------------------------------------------------- E2 --
print("=" * 72)
print("E2  unsupervised anomaly weighting vs gap heuristic")
from sklearn.covariance import EmpiricalCovariance
Wg_cap = gap_weights(cap["GAP"]); Wg_ver = gap_weights(ver["GAP"])
Wa_cap = np.ones_like(Wg_cap); Wa_ver = np.ones_like(Wg_ver)
for c in range(len(ids)):
    F = lambda m: np.column_stack([m["FP"][:, c], m["RX"][:, c], m["GAP"][:, c], m["Q"][:, c]])
    Xa = F(cap); ok = np.all(np.isfinite(Xa), 1) & (np.arange(n_cap) < cut)
    if ok.sum() < 50:
        continue
    ec = EmpiricalCovariance().fit(Xa[ok])
    for m, W in ((cap, Wa_cap), (ver, Wa_ver)):
        X = F(m); okx = np.all(np.isfinite(X), 1)
        d2 = ec.mahalanobis(X[okx])
        # weight ~ chi2 tail: unity for typical, decays beyond 95th pct of train
        thr = np.percentile(ec.mahalanobis(Xa[ok]), 95)
        W[okx, c] = np.minimum(1.0, np.maximum(0.05, thr / np.maximum(d2, 1e-9)))
for name, m, T, Wg, Wa in (("cap", cap, T_cap, Wg_cap, Wa_cap),
                           ("verify", ver, T_ver, Wg_ver, Wa_ver)):
    Pg, _, _ = replay(m, solve_new, weights=Wg)
    Pa, _, _ = replay(m, solve_new, weights=Wa)
    Pb, _, _ = replay(m, solve_new, weights=np.sqrt(Wg * Wa))   # blend
    mg, ma, mb = metrics(Pg, m, T), metrics(Pa, m, T), metrics(Pb, m, T)
    for tg in mg:
        print(f"   {name} tag{tg}: rmse gap {mg[tg]['rmse']:6.1f}  maha {ma[tg]['rmse']:6.1f}"
              f"  blend {mb[tg]['rmse']:6.1f} mm | p95 {mg[tg]['p95']:6.1f} /"
              f" {ma[tg]['p95']:6.1f} / {mb[tg]['p95']:6.1f}")

# ---------------------------------------------------------------------- E3 --
print("=" * 72)
print("E3  solver hyperparameter tuning (train: cap first 60% -> test: rest + verify)")
best, results = None, []
for delta in (0.05, 0.10, 0.15, 0.25):
    for trimw in (0.0, 0.15, 0.25, 0.4):
        for gth in (2.0, 3.0, 5.0):
            for gfl in (0.02, 0.05, 0.15):
                W = gap_weights(cap["GAP"], gth, gfl)
                P, _, _ = replay(cap, solve_new, weights=W, delta=delta, trimw=trimw)
                P = P.copy(); P[np.arange(n_cap) >= cut] = np.nan
                mm = metrics(P, cap, T_cap)
                score = np.mean([mm[tg]["rmse"] for tg in mm])
                results.append(((delta, trimw, gth, gfl), score))
                if best is None or score < best[1]:
                    best = ((delta, trimw, gth, gfl), score)
(delta, trimw, gth, gfl), tr_score = best
print(f"   best on train: huberDelta={delta} trimW={trimw} gapThresh={gth} gapFloor={gfl}"
      f"  (train rmse {tr_score:.1f} mm)")
for name, m, T in (("cap-test", cap, T_cap), ("verify", ver, T_ver)):
    Wd = gap_weights(m["GAP"])
    P0, _, _ = replay(m, solve_new, weights=Wd)
    P1, _, _ = replay(m, solve_new, weights=gap_weights(m["GAP"], gth, gfl),
                      delta=delta, trimw=trimw)
    if name == "cap-test":
        P0, P1 = P0.copy(), P1.copy()
        P0[np.arange(n_cap) < cut] = np.nan; P1[np.arange(n_cap) < cut] = np.nan
    m0, m1 = metrics(P0, m, T), metrics(P1, m, T)
    for tg in m0:
        print(f"   {name} tag{tg}: rmse default {m0[tg]['rmse']:6.1f} -> tuned {m1[tg]['rmse']:6.1f} mm"
              f"   p95 {m0[tg]['p95']:6.1f} -> {m1[tg]['p95']:6.1f}")
    b0, b1 = baseline_err(P0, m, T), baseline_err(P1, m, T)
    print(f"   {name} baseline-err {b0['rmse']:.1f} -> {b1['rmse']:.1f} mm")

# ---------------------------------------------------------------------- E4 --
print("=" * 72)
print("E4  EKF replay (CV + gated position update), default vs tuned q")

def ekf_replay(m, P, sigA=0.8, posSig=0.08, still_sigA=None):
    X = {}; Pcov = {}; out = np.full_like(P, np.nan); tprev = {}
    CHI2 = 5.991
    for i in range(len(m["t"])):
        tg = m["tag"][i]; z = P[i]
        if not np.isfinite(z).all():
            continue
        if tg not in X:
            X[tg] = np.r_[z, 0, 0]; Pcov[tg] = np.diag([.25, .25, 1, 1]); tprev[tg] = m["t"][i]
            out[i] = z
            continue
        dt = max(m["t"][i] - tprev[tg], 1e-3); tprev[tg] = m["t"][i]
        sa = still_sigA if still_sigA is not None else sigA
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        Q = np.zeros((4, 4))
        Q[:2, :2] = sa**2 * dt**3 / 3 * np.eye(2)
        Q[:2, 2:] = Q[2:, :2] = sa**2 * dt**2 / 2 * np.eye(2)
        Q[2:, 2:] = sa**2 * dt * np.eye(2)
        x = F @ X[tg]; Pc = F @ Pcov[tg] @ F.T + Q
        H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1
        R = np.eye(2) * posSig**2
        y = z - H @ x
        S = H @ Pc @ H.T + R
        nis = y @ np.linalg.solve(S, y)
        if nis > CHI2 and nis <= 10 * CHI2:
            R = R * (nis / CHI2); S = H @ Pc @ H.T + R
        elif nis > 10 * CHI2:
            X[tg], Pcov[tg] = x, Pc
            continue
        K = Pc @ H.T @ np.linalg.inv(S)
        X[tg] = x + K @ y
        Pcov[tg] = (np.eye(4) - K @ H) @ Pc
        out[i] = X[tg][:2]
    return out

for name, m, T in (("cap", cap, T_cap), ("verify", ver, T_ver)):
    P, _, _ = replay(m, solve_new)
    for lbl, kw in (("raw fixes", None), ("EKF default (sa=0.8)", dict(sigA=0.8)),
                    ("EKF still-tuned (sa=0.05)", dict(sigA=0.05))):
        Pf = P if kw is None else ekf_replay(m, P, **kw)
        mm = metrics(Pf, m, T)
        line = "  ".join(f"tag{tg} rmse {mm[tg]['rmse']:5.1f} p95 {mm[tg]['p95']:5.1f}" for tg in mm)
        bb = baseline_err(Pf, m, T)
        print(f"   {name:7s} {lbl:26s} {line}  baseline {bb['rmse']:5.1f} mm")
print("done")

# ---------------------------------------------------------------------- E5 --
print("=" * 72)
print("E5  learned-sigma weighting + combined stack (held-out verify)")
corr5 = {}; sig5 = {}
for c in range(len(ids)):
    F = lambda m: np.column_stack([m["FP"][:, c], m["RX"][:, c], m["GAP"][:, c], m["Q"][:, c]])
    Xa, ya = F(cap), E_cap[:, c]
    tr = np.isfinite(ya) & np.all(np.isfinite(Xa), 1) & (np.arange(n_cap) < cut)
    if tr.sum() < 50:
        continue
    g = HistGradientBoostingRegressor(max_iter=200, max_depth=3, random_state=0).fit(Xa[tr], ya[tr])
    s = HistGradientBoostingRegressor(max_iter=100, max_depth=3, random_state=0)
    s.fit(Xa[tr], np.abs(ya[tr] - g.predict(Xa[tr])))
    corr5[c] = g; sig5[c] = s

def apply5(m):
    C = np.zeros_like(m["C"]); Wv = np.ones_like(m["C"])
    for c, g in corr5.items():
        X = np.column_stack([m["FP"][:, c], m["RX"][:, c], m["GAP"][:, c], m["Q"][:, c]])
        ok = np.all(np.isfinite(X), 1)
        C[ok, c] = g.predict(X[ok])
        Wv[ok, c] = np.minimum((0.03 / np.maximum(sig5[c].predict(X[ok]), 0.01)) ** 2, 1.0)
    return C, Wv

Cx, Wv = apply5(ver)
for lbl, extr, wts, ekf in (("default", None, None, None),
                            ("learned-sigma weights", None, Wv, None),
                            ("GBM corr", Cx, None, None),
                            ("still-EKF only", None, None, 0.05),
                            ("GBM corr + still-EKF", Cx, None, 0.05)):
    P, _, _ = replay(ver, solve_new, extracorr=extr, weights=wts)
    if ekf:
        P = ekf_replay(ver, P, sigA=ekf)
    mm = metrics(P, ver, T_ver); bb = baseline_err(P, ver, T_ver)
    print(f"   {lbl:24s} tag240 {mm[240]['rmse']:5.1f}/{mm[240]['p95']:5.1f}  "
          f"tag241 {mm[241]['rmse']:5.1f}/{mm[241]['p95']:5.1f}  baseline {bb['rmse']:5.1f} mm")
print("done")
