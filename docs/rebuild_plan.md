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

**Output pin** ✅ SHIPPED 2026-07-21 (user request, ac5743c): live_tag freezes the
REPORTED position while parked and inside a 5 cm deadband (release on motion or
sustained excursion). Display/log-level only — masks the parked wobble, does not fix
ranges. Moving-tag stability still needs the physics fixed.

_Constraint (user, 2026-07-21): anchors CANNOT be raised (~fixed installation).
The tag CAN be raised → D2 becomes a raised-TAG experiment._

**Pending, in order (what we do next):**
1. **Reflash tag 240** with the two Phase-C read fixes (ac5743c: SAR temp/Vbat was never
   converting → constant −132 °C; CFO integrator read after the receiver re-armed → ~0).
   User reviews `git show ac5743c`, recompiles TagWrover, flashes. 5 min.
2. **Raised-TAG test (replaces D2).** Park the tag at one marked spot at normal height
   (0.24 m) for ~3 min, then put the SAME tag on a box/pole ~0.8–1 m at the SAME (x,y)
   for ~3 min, cir_capture A4 + A5 at both heights. Raising one end steepens the floor-
   bounce angle (weaker reflection) and strengthens the direct ray, so the leading edge
   should sharpen (fp/peak up) and breathing shrink. If it does → permanent fix = short
   mast for the dual-tag plate on the rover (tagZ + truth clicks change, one recalib).
   ~20 min, no firmware.
3. **A5 link inspection.** Its edge is the worst at only 2.3 m: look for metal/clutter
   near A5, try turning/moving it slightly, or swap the board with a spare to rule out
   the board. Re-check with cir_capture A5. ~15 min.
4. **Locked-room long dwell (Phase C payoff).** After step 1: tag parked, room empty,
   30–60 min live_tag log → correlate per-anchor range vs die temp, CFO, cadence over
   a long window. Closes out slow-drift hypotheses with real data. Passive.
5. **Later / only if justified:** D3 channel 2/5 alternation (needs all-board reflash +
   per-channel calibration — only if the raised-tag test says carrier-dependent
   multipath); D4 continuous CIR tail for error models; E TX-power boost (manual
   TX_POWER register, ~+10 dB, bench-only, ~1 h recalibration — only if the edge stays
   noisy after the geometry fixes). Broadcast-POLL (step 6) and E both pushed later.

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
