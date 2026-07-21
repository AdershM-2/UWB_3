# DUNE UWB — Rebuild Plan

_Last updated: 2026-07-21. Paper sidelined — focus is the working system._

## Goal
2D position RMSE ≤ 3 cm with the rover **moving** (EKF-filtered). Dual-tag yaw.
Kinect v2 = ground truth. All host work in MATLAB. Pose output is display/logging only.

## Current status (2026-07-21)
Steps 1–5 done, 7.1–7.2 done. Live MATLAB pipeline at 5–7 Hz over COM with EKF display.
Anchor delays calibrated on the sweep path vs Kinect truth (4 mm at the reference point);
tag 241 own-delay tuned (one step, −507 mm → −2 mm common offset). Tier-2 power correction
wired (floor-wide static per-sweep RMSE 199 → 94 mm). FusionEkf ported + hardened (robust
M-estimation, stillMode, per-anchor bias states, RangeHold) — anchor-dropout jumps down to
6–10 mm. **Remaining error: slow ~5–7 cm per-anchor "breathing" at fixed position — the
active investigation below (expert-reviewed 2026-07-21).**

## ACTIVE: wobble root-cause investigation (before 7.3)
Remaining error: slow ~5–7 cm per-anchor breathing at fixed position. The oracle test
proved it is time-varying — no static map can absorb it; catch the source, don't smooth it.

**Ruled out so far** (details: docs/phase_a_findings.md, docs/wobble_mitigation_summary.md):
- ~~Estimator layer~~ — at its floor (reviewer + A3; nine mitigations already shipped:
  delays, power corr, EKF, ZUPT, stillMode, bias states, RangeHold, robust M-est, NLOS ablation).
- ~~CFO×asymmetry in the TWR formula~~ — audit: Neirynck product form, excluded at 1st order.
- ~~Cadence/receiver-state as breathing driver~~ — A1: gap↔residual |ρ| ≤ 0.06; only a real
  ±5–7 mm receiver-idle micro-effect (anchor-signed) + A3-anchor +39 mm first-sample-after-skip.
- ~~Tag-side common-mode (tag clock/temp/Vbat)~~ — A2: breathing is per-anchor INDEPENDENT
  (off-diag corr +0.05) ⇒ the cause is per-link/anchor-side.
- ~~A3 stillMode noise-budget inversion~~ — negative, 91→319 mm (spot-specific residuals
  poison bias memory); stillInvert kept default-off.
- ~~D1 metronomic-cadence dwell~~ — obsoleted by A1's offline rejection of cadence.

**Pending** (in order):
- **Phase B — CIR snapshots** ◀ NEXT, user-side (tag-240 reflash staged, idle()-fix 1c10187):
  cir_capture quiet dwell / walking dwell / far anchor. Fork: leading edge breathes with
  residual (channel/LDE — A2 predicts this) vs edge frozen while range wanders (timing).
- **Phase C — free observables**: firmware STAGED awaiting user review, then flash tag 240
  (RTLS v4: per-anchor CFO + realised exchange-start ms; DIAG die-temp/Vbat tail; parser +
  JSONL done) → locked-room long dwell (Kinect timestamps intrusions) → correlate vs
  residuals. Role after A2: cheap close-out of tag-side hypotheses + anchor-drift watch.
- **D2 — raised anchors 1.8–2 m** (no firmware; re-click + retune ~30 min): ground-bounce/
  Fresnel share — now the PRIME suspect (z=0.24 m puts the floor inside the first Fresnel
  zone on every link; 2–3 cm bounce excess path is unresolvable at 500 MHz).
- **D3 — channel 2/5 alternation** (all boards + per-channel calibration): only if B/C point
  at carrier-dependent multipath.
- **D4 — continuous CIR tail** (~60 taps/exchange) for CIR-regression error models.

## Steps
1. **Hardware** — ✅ DONE.
2. **Kinect ground truth** — ✅ DONE (h=4.066 m, f=1067.6 px, registration RMSE 13.5 mm;
   click-to-world truth via clickTagTruth/clickAnchorsWorld). _Leftover → 7.4: moving
   trajectory recorder._
3. **MATLAB pipeline** — ✅ DONE (TagSerial → parseRtlsLine → solveSweep → live_tag/
   replay_rtls; protocol-agnostic sweeps; JSONL logs replayable).
4. **Calibration** — ✅ DONE. Sweep-path delay tuning (anchors 2026-07-20, tag 241
   2026-07-20; NVS junk wiped); Tier-2 per-anchor linear-in-fp correction
   (config/range_correction.json, auto-loaded). anchor_bias.json deprecated.
   _Backlog: PINN refit failed at 19 spots (linear 98 vs PINN 108 mm LOSO) — revisit with
   dense 7.4 truth + CIR features._
5. **Static accuracy** — ✅ DONE (baseline): per-sweep RMSE 94 mm corrected; oracle ceiling
   ~72 mm (time-varying residual → see investigation above).
6. **Broadcast-POLL** _(headline contribution)_ — DEFERRED (after 7). Spec first; A/B vs
   round-robin with the step-7 pipeline as control. Investigate burst-vs-sweep anomaly
   BEFORE protocol work (same timing-physics territory).
7. **Fusion + dynamics**:
   - 7.1 ✅ tag-241 delay tuned (tune_tag_delay).
   - 7.2 ✅ FusionEkf ported + validated offline (robust updates, stillMode, bias states);
     IMU accel prediction OFF until 7.3 frame validation.
   - 7.3 ◀ NEXT after investigation: dual-port live app (240=COM12, 241=COM11), yaw from
     the 0.48 m baseline vs BNO085 yaw (validates IMU frame → re-enable useImuAccel);
     also gives the dual-tag parked discriminator for the investigation.
   - 7.4 Kinect moving-truth recorder (AprilTag → world CSV, t_host-synced).
   - 7.5 Moving validation: trajectories vs Kinect truth → RMSE ≤ 3 cm goal + yaw
     accuracy; recalibrate ZUPT/stillness thresholds with labelled motion data.
8. **Run live** — extend live_tag to dual-tag EKF output (largely exists).

## Key notes
- Reject `rx = -2147483648` sentinels (handled in solveSweep).
- Corrections: on-device NVS delays + config/range_correction.json. Serial open resets the
  tag (DTR) → boot banner shows each connect's delay source.
- Firmware changes need discussion before reflash; keep libraries/UwbRtls synced with the
  Arduino-path copy on EVERY library change.
- Transport: serial/COM only (UDP blocked on iitk WiFi).
- Radio: MODE_LONGDATA_RANGE_ACCURACY (110 kb/s, PRF 64, long preamble), CHANNEL_5.
  Driver applies the APS011 power-bias table on every RX timestamp (always on).

## Board → sketch map
| Board | Sketch | ID |
|---|---|---|
| Anchors 1–5 | `sketches/Anchor/Anchor.ino` | `ANCHOR_ID` 0x01–0x05 |
| Tag with IMU | `sketches/TagWrover/TagWrover.ino` | 0xF0 (COM12) |
| Tag without IMU | `sketches/Tag/Tag.ino` | 0xF1 (COM11) |
