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
2. **Ground truth (Kinect overhead camera)** — NEXT. Full context in
   `docs/camera_calibration_handoff.md`. Tools live in `matlab/vision/` (copied in from
   D:\UWB_modules_new). Kinect is roof-mounted looking straight down; height fixed by parallax
   (h ≈ 3.873 m, provisional ±0.2 m). No valid world registration yet, and `config/anchors.json`
   does **not** match the real ~2.5×2.6 m layout (anchor 5 near centre). Do the accuracy plan **in order**:
   - **2.1 Clean parallax round** — `estimateCameraHeight`: plumb-bob chair/box hops + anchor
     base→antenna-tip clicks → pin h / nadir / tilt. Targets: h ±3–5 cm, per-hop spread < 0.15 m,
     expect h ∈ [3.6, 4.0], implied f ≈ 1020.
   - **2.2 Stage A** — `calibrateOverheadCamera` plumb-line strings → distortion k1/k2. Target: corrected bow < 2 px.
   - **2.3 Stage B** — AprilTag floor sweep (12+ spots) + tape scale bar → floor homography. Target: corner reproj < 1 px, scale-bar error < 0.5%.
   - **2.4 Stage C** — box hops → focal length f_est (supersedes 1050). Cross-check f ≈ 1020.
   - **2.5 `auditFloorScale`** acceptance test — every zone 0.99–1.01.
   - **2.6** Fix `anchors.json` to the real layout (tape or Phase-1.5 self-survey), then re-run
     `registerWorldFrameClicks`. Then build the trajectory recorder (Kinect → AprilTag → world CSV) + time-sync to UWB.
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
