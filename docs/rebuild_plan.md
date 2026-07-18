# DUNE UWB — Rebuild Plan

_Last updated: 2026-07-18. Paper is sidelined for now — focus is the working system._

## Goal
2D position RMSE ≤ 3 cm with the rover **moving**. Dual-tag yaw. Kinect v2 = ground truth.
All host work in MATLAB. Pose output is display/logging only.

## Current status
**Step 1 (hardware) — DONE.** All 5 anchors + 2 tags healthy; both tags sweep 5/5.
A3 revived (was dead), tag 240 (0xF0) stable with no brownouts, all boards reflashed clean.
IMU gyro confirmed working when moving.

## Steps
1. **Fix hardware** — ✅ DONE. (Still worth letting A1 run ~10 min on battery to fully confirm.)
2. **Ground truth (Kinect)** — NEXT. Re-run Kinect world registration (old RMSE 0.259 m is too big).
   Build the trajectory recorder (Kinect → AprilTag → world-frame CSV). Time-sync to UWB.
3. **MATLAB pipeline** — NEXT. Read serial stream → parse `RTLS` line → correct bias → NLOS weights →
   weighted multilateration → live position. Reads **COM** (UDP blocked on `iitk`).
4. **Redo calibration** — anchor geometry (self-survey), per-anchor + per-tag bias, Tier-2 spatial
   error map. (July-1 calibration was taken with bad hardware — redo it.)
5. **Static accuracy** — 2D RMSE at known positions. Goal ≤ 3 cm. Validates the round-robin baseline.
6. **Broadcast-POLL** _(headline contribution)_ — write the protocol spec now (no reflash yet);
   build + A/B test vs round-robin **after** step 5.
7. **Fusion + dynamics** — port FusionEKF to MATLAB, add IMU (0xF0) + dual-tag yaw, validate
   moving-rover RMSE and yaw.
8. **Run live** — MATLAB serial/UDP live pipeline + display/logging.

**Critical path: steps 2 + 3.** Step 2 (hardware/Kinect, user) and step 3 (MATLAB, code) run in parallel.

## Key notes
- Data quality: reject `rx = -2147483648` (error sentinel seen on A3).
- IMU gyro reads 0.0000 at rest — that's normal; it moves when rotated.
- Firmware is proven; protocol changes need discussion before any reflash.
- Keep the vendored library (`libraries/UwbRtls`) in sync with the Arduino-path copy on every change.
- Transport: PC reads the tag over **serial/COM** (UDP fails on `iitk` WiFi).

## Board → sketch map
| Board | Sketch | ID |
|---|---|---|
| Anchors 1–5 | `sketches/Anchor/Anchor.ino` | `ANCHOR_ID` 0x01–0x05 |
| Tag with IMU | `sketches/TagWrover/TagWrover.ino` | 0xF0 |
| Tag without IMU | `sketches/Tag/Tag.ino` | 0xF1 |
