# DUNE UWB — Rebuild Plan

_Last updated: 2026-07-21. Paper sidelined — focus is the working system._

## Goal
_Revised 2026-07-21: accuracy target relaxed 3 → 8 cm; the primary objective is now
**stability** of the received data (no parked wander/jumps, consistent per-anchor ranges),
not chasing absolute accuracy._
2D position RMSE ≤ 8 cm with the rover **moving** (EKF-filtered). Dual-tag yaw.
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

**Phase B — CIR snapshots** ✅ DONE 2026-07-21 (sets cir_20260721_194042 A5 quiet,
194227 A4 quiet, 194423 A4 walking): **channel/LDE branch confirmed.** First path is
~20 dB BELOW the channel peak on both links (fp/peak 0.07–0.11; A4's dominant arrival
at +25 taps ≈ 7.5 m excess path; A5 = dense tail right behind the edge), edge only
8–10 dB above pre-cursor noise — ground-grazing/Fresnel regime, cm-level LDE wander
expected. Quiet room already wanders (A4 p2p 132 mm/81 s) ≈ walking run — body motion
NOT required. Not clock/timing.

**A/B without A5** ✅ DONE 2026-07-21 (parked runs 195325/195625, both re-solved with and
without A5): A5 is the worst breather (slow range p2p ~122 mm, 2–3× the others) but NLOS
weights already suppress it — dropping it wins only ~15–20% wander and costs geometry.
Every anchor breathes 40–92 mm ⇒ constellation-wide edge problem, keep all 5.

**Output pin** ✅ SHIPPED 2026-07-21 (user request, ac5743c + 2e27ec5): live_tag freezes
the REPORTED position while parked and inside a 5 cm deadband (release on motion or
sustained excursion). Fix 2e27ec5: the red dot now follows the pinned value (was drawn
at the raw solve), and the pin also engages on a calm solve (residual < 6 cm AND EKF
speed < 8 cm/s), not only IMU stillness. Display/log-level only — masks the parked
wobble, does not fix ranges. Moving-tag stability still needs the physics fixed.

_Constraint (user, 2026-07-21): anchors CANNOT be raised (~fixed installation).
The tag CAN be raised → D2 becomes a raised-TAG experiment._

**DECISION 2026-07-21 (user):** the wobble investigation is good enough for now — the
static residual is characterised and the parked display is pinned. Pivot to the ACTUAL
goal: moving-tag performance. Run motion tests first; the remaining static experiments
(reflash, raised-tag, A5, locked-room) are DEFERRED until the motion tests say we need
better ranges.

**Pending, in order (what we do next):**
1. **Motion test (NOW).** Drive the tag by hand, live_tag running, and watch/log the
   moving behaviour that actually matters: (a) the pin RELEASES cleanly the moment the
   tag moves and the dot follows; (b) ~50 cm moves in each direction read back as ~50 cm;
   (c) a hand-drawn curve comes out as a smooth curve, not a lagging/overshooting mess;
   (d) it re-pins when the tag stops. No Kinect truth yet — known-distance moves + shape
   are the poor-man's truth. Analyser staged (motion_check.m): segments still/moving,
   per-move displacement, EKF-vs-raw lag, pin behaviour. → tells us if ZUPT/stillness
   thresholds and CV process noise need retuning (7.5 territory) BEFORE building the
   Kinect recorder.
2. **Reflash tag 240** with the Phase-C read fixes (ac5743c: SAR temp/Vbat constant
   −132 °C; CFO read after RX re-arm → ~0). Review `git show ac5743c`, recompile, flash.
   Needed only when we return to the slow-drift diagnostics. 5 min.
3. **Raised-TAG test (replaces D2, deferred).** Same tag at 0.24 m vs ~0.8–1 m at the
   SAME (x,y); cir_capture A4+A5 at both heights. Steeper floor-bounce angle should
   sharpen the edge (fp/peak up) and shrink breathing → permanent fix = short mast on
   the rover plate. ~20 min, no firmware.
4. **A5 link inspection (deferred).** Worst edge at only 2.3 m: check for metal/clutter,
   rotate/move it, or board-swap; re-check with cir_capture A5. ~15 min.
5. **Locked-room long dwell (deferred, needs step 2 first).** Tag parked, room empty,
   30–60 min log → correlate per-anchor range vs die temp, CFO, cadence. Passive.
6. **Later / only if justified:** D3 channel 2/5 alternation (all-board reflash +
   per-channel calibration — only if raised-tag says carrier-dependent multipath);
   D4 continuous CIR tail; E TX-power boost (manual TX_POWER, ~+10 dB, bench-only,
   ~1 h recalibration). Broadcast-POLL (step 6) and E both pushed later.

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
   _Firmware scope (answered 2026-07-21): requires reflashing ALL 5 anchors + the tag —
   the current anchor responder only answers addressed POLLs (immediate POLL_ACK); a
   broadcast POLL needs anchor-side RX timestamping + slotted delayed-TX replies. There
   is no anchor-untouched variant._
7. **Fusion + dynamics**:
   - 7.1 ✅ tag-241 delay tuned (tune_tag_delay).
   - 7.2 ✅ FusionEkf ported + validated offline (robust updates, stillMode, bias states);
     IMU accel prediction OFF until 7.3 frame validation.
   - 7.3 ◀ NEXT after investigation: dual-port live app (240=COM12, 241=COM11), yaw from
     the 0.48 m baseline vs BNO085 yaw (validates IMU frame → re-enable useImuAccel);
     also gives the dual-tag parked discriminator for the investigation.
   - 7.4 Kinect moving-truth recorder (AprilTag → world CSV, t_host-synced).
   - 7.5 Moving validation: trajectories vs Kinect truth → RMSE ≤ 8 cm goal +
     STABILITY metrics first-class (parked wander, subset-jump size, per-anchor range
     consistency) + yaw accuracy; recalibrate ZUPT/stillness thresholds with labelled
     motion data.
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
