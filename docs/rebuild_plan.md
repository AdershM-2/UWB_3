# DUNE UWB — Rebuild Plan

_Last updated: 2026-07-20. Paper is sidelined for now — focus is the working system._

## Goal
2D position RMSE ≤ 3 cm with the rover **moving** (EKF-filtered). Dual-tag yaw.
Kinect v2 = ground truth. All host work in MATLAB. Pose output is display/logging only.

## Current status (2026-07-20)
Steps 1–5 DONE. Live MATLAB pipeline runs at ~5–6 Hz over COM. Anchor delays calibrated
against Kinect click-truth (parked error 4 mm at the calibration point). Tier-2 power-bias
correction fitted on a 19-spot campaign and wired into the pipeline: static per-sweep
2D RMSE 199 → 94 mm (median 69). Remaining error is per-sweep noise tails (oracle ceiling
~72 mm RMSE) — that is step-7 filtering territory, not calibration.

## Steps
1. **Fix hardware** — ✅ DONE. All 5 anchors + 2 tags healthy.
2. **Ground truth (Kinect overhead camera)** — ✅ DONE. h=4.066 m, f=1067.6 px, distortion
   negligible, homography tape-anchored; world registration RMSE 13.5 mm
   (world_registration.mat). The registered Kinect is a click-to-world truth machine
   (clickTagTruth / clickAnchorsWorld). _Leftover moved to step 7: the moving-trajectory
   recorder (Kinect → AprilTag → world CSV, time-synced to UWB)._
3. **MATLAB pipeline** — ✅ DONE. dune.TagSerial (COM ingest) → parseRtlsLine →
   dune.solveSweep (sentinel reject → corrections → NLOS gap weights → weighted LM) →
   live_tag map + JSONL logging (readSessionLog schema, fully replayable);
   replay_rtls exercises the exact live chain on old logs. Ingest is protocol-agnostic
   (a "sweep" = set of (anchor, range, rx, fp)@t) ready for broadcast-POLL.
4. **Calibration** — ✅ DONE (2026-07-20).
   - Anchor antenna delays: NVS junk from the July-1 bad-hardware era found (+4 m on A5!)
     and wiped; final delays tuned on the **sweep path** against Kinect truth
     (tune_delays_sweep): all anchors ±16 mm, parked position error 4 mm.
     Final ticks A1 16501, A2 16515, A3 16488, A4 16483, A5 16552; tag 240 = reference
     (16360, sketch default). anchor_bias.json (July-1) is DEPRECATED.
   - **Open firmware anomaly:** HWCALIB burst ranging vs ring-sweep ranging differ by a
     constant per-anchor offset (+60..+500 mm) though both call the same rangeTo().
     Investigate before broadcast-POLL work. Evidence in config/delay_calibration.json.
   - Tier-2: per-anchor **linear-in-first-path-power** range correction (slopes
     7.7–9.5 mm/dB), fitted on 19 Kinect-truth spots, LOSO-validated, wired as
     config/range_correction.json → dune.solveSweep(rangeCorr=…), auto-loaded by
     live_tag / static_accuracy.
   - _Backlog: refit the power-bias model as a **PINN** (physics-informed NN — APS011
     leading-edge physics as the prior, campaign data as training set) instead of plain
     linear regression; the 19-spot dataset is saved in
     results/static_accuracy_20260720_195637/bakeoff.mat (+ raw serial logs). Revisit
     after step 7._
   - _Tag 241 delay still uncalibrated → step 7.1._
5. **Static accuracy** — ✅ DONE (baseline). 19-spot campaign: per-sweep RMSE 94 mm
   corrected (worst spot 141). Oracle test: even a perfect static correction leaves
   ~72 mm per-sweep RMSE (noise tails) → the ≤3 cm goal rides on step-7 filtering.
   Harness: static_accuracy.m (park → click truth → 20 s sweeps → stats/map/JSON).
6. **Broadcast-POLL** _(headline contribution)_ — DEFERRED (after step 7). Write the
   protocol spec first (no reflash without joint review); A/B vs round-robin with the
   step-7 pipeline as the control.
7. **Fusion + dynamics** — ◀ NEXT.
   - 7.1 Tune tag 241's own delay (SETMYDELAY sweep-path variant of tune_delays_sweep;
     241 on USB, truth via clickTagTruth(0.24, 241)). Anchors must NOT be retuned.
   - 7.2 Port the EKF to MATLAB (reference: D:\UWB_modules_new rigid_body_ekf.py):
     CV model + per-sweep range/position updates, innovation gating, ZUPT;
     develop OFFLINE first on the existing campaign logs (they contain walking segments).
   - 7.3 IMU fusion (tag 240 BNO085: quaternion/gyro already parsed + logged) and
     dual-tag ingest (two TagSerial ports) → rigid-body state incl. dual-tag yaw
     (0.48 m baseline) cross-checked against IMU yaw.
   - 7.4 Kinect moving-truth recorder: AprilTag on the unit → world CSV @ ~15-30 Hz,
     t_host-synced (the step-2 leftover; alignTruth/readVisionLog already exist).
   - 7.5 Moving validation: walked/driven trajectories, EKF output vs Kinect truth →
     moving 2D RMSE (goal ≤ 3 cm) + yaw accuracy.
8. **Run live** — MATLAB live pipeline + display/logging (largely exists via live_tag;
   extend to EKF output + dual tag).

## Key notes
- Data quality: reject `rx = -2147483648` (error sentinel; handled in solveSweep).
- Range corrections live on-device (NVS antenna delays) + host config
  (range_correction.json). anchor_bias.json is historical only.
- Serial open resets the tag (DTR) — boot banner shows its delay source every connect.
- Firmware is proven; protocol changes need discussion before any reflash.
- Keep the vendored library (`libraries/UwbRtls`) in sync with the Arduino-path copy.
- Transport: PC reads tags over **serial/COM** (UDP blocked on `iitk` WiFi).

## Board → sketch map
| Board | Sketch | ID |
|---|---|---|
| Anchors 1–5 | `sketches/Anchor/Anchor.ino` | `ANCHOR_ID` 0x01–0x05 |
| Tag with IMU | `sketches/TagWrover/TagWrover.ino` | 0xF0 |
| Tag without IMU | `sketches/Tag/Tag.ino` | 0xF1 |
