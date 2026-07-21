# Wobble investigation — Phase A findings (offline forensics)

_2026-07-21. Data: 19-spot campaign (static_accuracy_20260720_195637, 10,093 sweeps,
full parked windows re-segmented against truth) + 3 long parked dwells auto-detected
in the live logs. 44,322 pooled per-anchor observations. Tooling:
`matlab/scripts/phase_a_forensics.m` (reports + figures in
`results/phaseA_forensics_<stamp>/`, gitignored — regenerate by re-running)._

## A1 — cadence hypothesis: REJECTED as the breathing driver (small real effect exists)

Per-anchor realised inter-exchange gap vs dwell-demeaned range residual:

- corr(residual, own inter-success gap): |ρ| ≤ 0.057 across all anchors (A1 +0.046,
  A2 +0.020, A3 −0.008, A4 +0.001, A5 −0.025). corr vs sweep period: |ρ| ≤ 0.04.
  The production cadence wander does **not** modulate the ~50–70 mm breathing.
- A genuine micro-effect exists at the ±5–7 mm level: an exchange preceded by a
  failed slot (+~45–75 ms extra receiver idle) or first-in-sweep shifts the range by
  A2 +5.5…+6.7 mm (p ≤ 1e-4), A5 −5.6 mm (p 3e-4), A3 +1.5 ns, A4 −0.4 ns.
  Anchor-dependent sign — consistent with the burst-vs-sweep anomaly mechanism
  (receiver-state-dependent bias), but two orders below the +60…+500 mm burst
  offset and irrelevant to the breathing.
- A3's first range after a >2 s skip window reads +39 mm long (p 2e-4, n=12) —
  supports RangeHold/bias-memory design; first-sample-after-recovery is biased.

## A2 — breathing decomposition: PER-ANCHOR INDEPENDENT ⇒ link/anchor-side

Cross-anchor correlation of 2 s-median smoothed residuals within 23 dwells:

- Mean off-diagonal correlation **+0.05** (max pair +0.11). PC1 loadings same-sign
  in only 26% of dwells (chance ≈ 12–25%); PC1 variance share 51% ≈ the
  short-series noise baseline.
- Verdict: the slow breathing is **independent per anchor**, not common-mode.
  This eliminates tag-side global causes (tag crystal/temperature drift, Vbat,
  tag antenna) for the dominant component and points at the per-link physics:
  ground-bounce/Fresnel fading modulated by moving bodies (reviewer bet #2),
  anchor-local state, or per-link LDE/first-path drift.

## A3 — stillMode noise-budget inversion: NEGATIVE result (kept behind a flag)

`FusionEkf.stillInvert` (position Q→~0.005 m/s², per-anchor bias RW opened to
0.02 m/√s while still; 3 s engage delay in ekf_replay): campaign replay
RMSE 91→319 mm with 175 reinits (vs pos-mode 91→88 mm). Structural failure, not a
tuning failure: A2 shows the residuals are **spot-specific per anchor**, so biases
learned while parked at one spot are wrong at the next; position+bias share a null
space while frozen, and `reinitFrom` deliberately keeps biases → corruption
cascades across dwells. Confirms the reviewer's "estimator layer is at its floor".
Flag stays available (default off) for genuinely-permanent-parked scenarios.

Baseline numbers on the campaign with truth-based spot windows (ekf_replay updated):
pos-mode EKF 88 mm RMSE (p95 145); wander (max drift of 2 s-median position within
a dwell, median over spots) raw 56 mm → EKF 70 mm — **the EKF does not reduce the
breathing**, it rides it, as expected for a temporally-correlated error.

## Method notes (hard-won, for reuse)

- The campaign `spots.json` `medPos` are **pre-power-correction raw-solve medians**
  (recorded before range_correction.json existed) — up to 0.33 m from truth.
  Any spot clustering on corrected re-solves must anchor on `truth`, not `medPos`
  (fixed in `ekf_replay` and `phase_a_forensics`).
- A warm-start chain through carrying phases can lock the MAD-gated LM into a
  shifted solution for a whole dwell — cold-start when solving for segmentation.
- The scheduler fail/skip state machine is exactly reconstructable offline from
  the RTLS stream + `[SCHED]` lines (697/697 skip windows matched) — realised
  cadence needs no firmware change to analyse historically.

## Implication for the investigation

Phase B (CIR waterfalls) and Phase D2 (raised anchors) are now the discriminating
experiments: A2's independence verdict predicts the CIR leading edge should
**breathe per-link** (channel/LDE cause) rather than stay frozen while range
wanders. Phase C observables (die temp, Vbat, CFO) mainly serve to *close out*
the tag-side hypotheses cheaply and to watch anchor-side drift via per-anchor CFO.
