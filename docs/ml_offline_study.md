# ML offline study — can learning improve the pipeline?

**Date:** 2026-07-22 · **Code:** [`analysis/ml_offline/`](../analysis/ml_offline/)
(Python replay of the solver/EKF, verified to reproduce `dune.multilaterate`
to <2 µm in its plain-LS mode)

Question: of the candidate ML insertion points (learned range-error
regression, learned measurement weighting, unsupervised anomaly weighting,
hyperparameter tuning, bias field, CIR-based NLOS, stillness classifier),
which actually help on the data we already have?

## Data and ground rules

Only two raw captures are committed to the repo — both Phase-C era
(rect6p3x3p0 layout, A1 dead, `anchor_bias.json` corrections, tags parked):

| capture | sweeps | role |
|---|---|---|
| `static_capture.txt` | 1064 (240: 216, 241: 848) | train = first 60 %, test = rest |
| `static_verify_capture.txt` | 628 | held-out **session** (never trained on) |

The 19-spot campaign raw data, the rover-run JSONL, and CIR captures are
**not** in the repo, so three recommendations were untestable (see end).

Truth: parked-tag pseudo-truth = median of solved fixes (circular for
absolute accuracy, fine for scatter/precision), plus one genuinely
camera-independent accuracy invariant: the **0.527 m inter-tag baseline** —
each tag's per-sweep fix must sit 0.527 m from the other tag's position.
"baseline err" below is the RMSE of that violation. All models are trained
on the train split only; every number quoted is held-out.

## Results (held-out verify session, mm)

| variant | tag240 rmse/p95 | tag241 rmse/p95 | baseline err |
|---|---|---|---|
| raw fixes, default pipeline | 38.9 / 73.8 | 41.8 / 74.0 | 28.5 |
| + GBM range correction (E1) | 35.1 / 60.7 | 34.0 / 64.1 | 21.9 |
| + still-EKF only (E4) | 27.6 / 46.3 | 22.4 / 39.4 | 14.9 |
| + GBM correction **and** still-EKF | **23.9 / 44.7** | **18.2 / 30.9** | **11.4** |
| learned-sigma weights (E5) | 46.8 / 88.7 | 47.5 / 105.4 | 33.7 |
| unsupervised Mahalanobis weights (E2) | 45.8 / 87.0 | 51.2 / 102.9 | — |

## Findings per recommendation

**1. Supervised range-error regression — works, but linear already wins.**
Per-anchor residual RMSE vs diagnostics (fp, rx, gap, quality): raw 33–81 mm
drops to ~28–32 mm — and generalises across sessions (verify improved as
much as the in-capture test split). But gradient boosting matched, never
beat, the plain per-anchor linear-in-fp fit (e.g. A3 verify: raw 65.0,
linear 30.1, GBM 32.6). Two consequences: (a) the July-20
`range_correction.json` approach is validated out-of-sample — these
captures pre-date it, so most of the "GBM corr" gain above is really "any
fp correction vs none"; (b) there is **no evidence the ML model adds
anything over the linear fit** on static data. Ship nothing new here;
re-test GBM on rover-run data where fp varies more.

**1b. Drift is NOT predictable from diagnostics.** Regressing each anchor's
slow residual wander on (fp, rx, quality) gives held-out R² of −0.05…−0.12:
the diagnostics carry no signal about the parked drift. This is consistent
with the floor-reflection diagnosis (the blended first path moves without
changing the summary powers) and means **no diagnostics-based ML will fix
the wander** — the fix stays physical (raise the antennas) or CIR-based.

**2. Unsupervised anomaly weighting — worse than the physics heuristic.**
Mahalanobis-distance weights learned on train diagnostics degraded verify
rmse by 18–22 % vs the 3 dB gap rule. Learned per-anchor sigma weighting
(supervised) also lost to gap weights. The gap heuristic encodes real
physics; naive statistical replacements underperform it. Negative result.

**3. Hyperparameter tuning — marginal on static data.** Grid over
(huberDelta, trimW, gapThresh, gapFloor): best train config improved
held-out rmse by ≤ 1 mm. Static captures contain too few outlier events to
exercise the robustness knobs; inconclusive until rover logs are available.

**4. Still-mode EKF — the biggest single lever, and it's already in the
codebase.** A CV-model EKF with process noise capped at σ=0.05 m/s²
(exactly `FusionEkf.sigmaAccelStill`) cuts scatter 29–47 % and baseline err
28.5→14.9 mm. The full stack (fp correction + still-EKF) reaches
**11.4 mm** baseline err, −60 % vs raw. This validates the existing
`stillMode` design offline; the "ML" framing (auto-tuning) merely re-found
the value already chosen by hand.

**Speed.** New robust solver: 5.8 vs 3.6 mean LM iterations (+0.2 ms/solve
in Python) — irrelevant at 7 Hz. Old-vs-new accuracy on this clean static
data is a wash (as designed; the robust layers only engage under
contamination), with slightly better tails (tag240 max 154→110 mm).

## Untestable with committed data — data wishlist

| recommendation | blocked by | what to commit/log |
|---|---|---|
| per-anchor position-dependent bias field (GP) | truth at only 2 positions | 19-spot campaign raw + camera truth, or a rover run |
| CIR-based NLOS / drift correction | no CIR captures in repo | `cir_capture` waterfalls, ideally alongside truth |
| stillness classifier | no motion data | rover-run JSONL (has IMU + truth speed) |

The single highest-value action for any future ML work: **commit one full
rover-run capture (UWB JSONL + vision log)**. It would unlock recs 2/3/5,
give real train/test splits by run, and let E1/E3 be re-scored on moving
data where they have room to matter.
