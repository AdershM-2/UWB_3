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

## PARKED (2026-07-21): wobble root-cause investigation + broadcast-POLL
**Decision (user, 2026-07-21):** the parked wobble is characterised well enough (Phase A/B
done, source = per-link ground-grazing/LDE physics; pinned + smoothed at the output). Set
aside the REMAINING investigation items below (reflash, raised-tag, A5, locked-room, D3/D4/E)
AND broadcast-POLL (step 6) for now — they are documented and resumable. **Active work moves
to step 7.3 (two tags).** 7.4 Kinect accuracy validation stays later (overall comparison).

_Remaining error: slow ~5–7 cm per-anchor breathing at fixed position. The oracle test
proved it is time-varying — no static map can absorb it; catch the source, don't smooth it.
The output pin + EMA + MHE mask it for display/logging; the physics fixes below would
actually reduce it if we return to them._

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
1. **Motion test** ✅ DONE 2026-07-21 (log 202505): pin works (81% while still, 0 mm
   output scatter), ~50 cm moves read 0.44–0.58 m, moving track was jagged. Smoothed via
   an output EMA (opts.smooth 0.6: ~50% less jerk for ~25 mm lag; CV process-noise tuning
   was a poor lever, 13% for 3× lag). motion_check.m analyser committed.
1b. **Experimental: Moving Horizon Estimator (MHE)** ✅ EVALUATED 2026-07-21
   (dune.MheEstimator, mhe_replay.m; CasADi 3.7.0 + IPOPT at
   C:\Users\itisa\Downloads\casadi-3.7.0). Sliding window of N=10 sweeps, decision vars =
   whole [p,v] trajectory, cost = robust pseudo-Huber range fit + CV process + ZUPT
   (still) + arrival cost. **Verdict:** MOTION 40–44% smoother than the EKF (jerk
   0.34–0.38 vs 0.49–0.61) for +8 mm lag, 3 ms/solve median (live-feasible at 5 Hz);
   STATIC a tie (median wander 55 vs 54 mm, RMSE 61 vs 64 mm — MHE marginally better).
   Warts: a couple of static spots (S8) show ~0.23 m slow drift where the breathing
   pulls the window — RMSE stays fine (mean is right), and it is NOT sudden jumps so the
   guard doesn't remove it; a solve-time tail (p95 30 ms, max 90 ms, still < the 200 ms
   live budget). Added a jump-guard (maxJump 0.20 m: reject an output that leaps beyond
   the CV prediction or a non-converged solve → fall back to the raw fix, reseed the
   window; prevents the divergence cascade). **WIRED as opt-in:** live_tag(estimator="mhe",
   horizon=10) — default stays EKF; the EKF keeps running for stillness + the pin's
   velocity gate, MHE just supplies the display fix. Committed e2ab0d5/eefa215/<this>.
   NEXT if pursued: the S8 static-drift (tighter arrival cost / stronger ZUPT) and the
   solve-tail; otherwise it is a validated experimental smoother for motion.
1c. **IMU fusion into the MHE** ◀ STARTED 2026-07-21. IMU probe on log 202505: accel
   bias ~0 ([-0.007 -0.026 +0.013] m/s^2 while still — BNO085 gravity-removal is clean, so
   the old EKF divergence was a FRAME problem, not bias), tag is flat (body gz = world yaw
   rate), but the absolute orientation frame looks rotated (world-accel vs measured dv/dt
   corr -0.39, wrong sign). Per the user's delta idea: added a **gyro coordinated-turn**
   model to the MHE (velocity vector rotates at the world-vertical gyro rate; DELTA-only,
   no absolute heading; omega=0 => exact CV). Wired: MheEstimator OMEGA param + trapezoidal
   position, mhe_replay useImu, live_tag(estimator="mhe", useImu=true default). Offline on
   the hand-carried log it is a WASH (jerk 0.357 CV vs 0.362 turn) — that log has no
   sustained turns to exploit; it should help a rover driving real arcs. Safe (guard
   intact, solve time unchanged). Accel-delta-velocity fusion DEFERRED: the rotated frame
   would inject wrong accelerations, and neither can be validated offline (raw-fix
   double-difference is too noisy) — both need **7.4 Kinect moving truth** to judge, which
   is now the bottleneck for all motion-model tuning.
1d. **Non-holonomic (unicycle) MHE model** ◀ BUILT 2026-07-21 for the RC car (user placed
   the tag on a small front-steering RC car). MheEstimator.model="unicycle": state
   [px py theta speed], gyro yaw rate drives heading (change-in-yaw, NO absolute compass -
   the UWB anchors the absolute heading), velocity FORCED along heading (no sideways slip),
   speed free. Physically-correct model for a car-like vehicle; needs no trusted
   orientation. Wired: mhe_replay/live_tag model="unicycle". Offline on the HOLONOMIC hand
   log it is smoother (jerk 0.331 vs cv 0.362) but laggier (39 vs 28 mm) - expected, the
   no-slip rule fights the hand's sideways motion; on the RC car the lag penalty should
   vanish. Stable + fast (3.3 ms, max 8.5 ms). VALIDATE on an RC-car log (or 7.4 truth).
   Check on the car: if turns lag, the gyro sign (worldYawRate) may need flipping for the
   IMU mounting.
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
   - 7.3 ◀ **IN PROGRESS: two-tag / rigid-body.** Handoff: docs/handoff_two_tags.md.
     - Task 1 TRANSPORT ✅ 2026-07-21: the firmware ALREADY UDP-broadcasts every sweep;
       only the host side was missing. Added dune.TagUdp (Java DatagramSocket — udpport
       needs the Instrument Control Toolbox, not installed) + udp_probe. VERIFIED: BOTH
       tags reach the host over WiFi/UDP (~3 Hz each, 5 anchors; 240=172.29.108.220,
       241=172.29.109.27, host 172.29.108.246). The iitk UDP block did NOT bite — no
       firmware change / hotspot needed.
     - Tasks 2+3 DUAL DISPLAY + YAW ✅ 2026-07-21 (live_dual_tag): both tags demuxed on
       one socket, per-tag solve+EKF, rigid baseline drawn, baseline yaw (atan2 of
       tag_hi−tag_lo) vs IMU yaw + circular-mean offset, logs dual_log/dual_yaw. Validated
       live with BOTH tags — rig behaves rigid (constant inter-tag distance, coupled
       motion). IMU currently silent on tag 240 (fixable connection issue; only the
       TagWrover tag 240 has a BNO085) so yaw-vs-IMU is deferred, baseline yaw works now.
     - MEASURED BASELINE (parked, log dual_log_20260721_222016, 82 nearest-time pairs):
       independent solves give L = 0.317 m mean (median 0.315, std 29 mm, p2p 162 mm).
       Tags were physically ~30 cm apart (NOT the old 0.50 m) → the 0.317 m MATCHES to
       ~2 cm, i.e. no systematic bias; the 29 mm wander is the per-anchor breathing showing
       in the baseline. STILL NEEDED: a precise tape measure (antenna-centre to
       antenna-centre) as the hard constraint L; placement is currently ad-hoc ~30 cm.
     - Task 4 RIGID SOLVER ✅ built 2026-07-21: dune.rigidSolve (hard constraint — L baked
       into the parameterisation p = c ± (L/2)[cosψ,sinψ]; LM over [cx cy psi]; reuses
       solveSweep weights). Offline on the parked log: baseline std 36 mm → exact, centre
       wander −15%, yaw std 7.1→5.6°, range rmse 61→70 mm (expected 3-DOF cost). Wired
       into live_dual_tag (purple centre + heading arrow, yawRig logged). rigid_replay.m
       compares indep vs rigid on any udp_raw log. Rigid-body MHE (temporal, gyro→psi,
       no-slip) NOT started — wait for IMU fix + 7.4 truth.
     - ⚠ END-OF-DAY STATE (2026-07-21 evening): the LAST FULLY RELIABLE config is
       single-tag live_tag over COM (+ MHE unicycle). The dual-tag live run DEGRADED in a
       later session: dots slid apart / away (screenshot ~3.5 m apart at ~30 cm true).
       Diagnosed so far (udp_raw/dual_log_20260721_223629 + diag replay):
       1. A1 & A2 stopped answering mid-run → most sweeps only A3/A4/A5, rx weak
          (−95…−103 dBm). WHY is open: RF/power/obstruction/connection — check tomorrow.
       2. A3/A4/A5 are the three TOP-EDGE anchors (y≈2.1–2.3, near-collinear) →
          multilaterate's degenerate-geometry gate (sv(2)<0.3, multilaterate.m:41)
          correctly refuses a fix (iters=0, mirror ambiguity). NOT a solver bug.
       3. With no fixes the per-tag FusionEkf COASTS on its CV prediction (reached
          (−17,−7)) and live_dual_tag's display FOLLOWS it — my bug: live_tag only moves
          the dot on a valid raw fix; live_dual_tag dropped that guard. FIX FIRST
          TOMORROW (guard display on finite raw p + reinit EKF after a long coast).
       4. The rigid centre stayed CORRECT throughout (re-solves from both tags' ranges
          jointly — 6 measurements, less degenerate). Argues for making the rigid solve
          the primary dual-tag output.
       TOMORROW: fix (3), then investigate (1) — solver vs wifi vs connection — then
       re-validate live; tape-measure L; tag-240 IMU (silent, connection suspected).
   - 7.4 Kinect moving-truth recorder (AprilTag → world CSV, t_host-synced) — LATER, for
     overall accuracy comparison of EKF vs MHE(cv/unicycle) vs rigid-body.
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
- Transport: serial/COM (single tag) OR WiFi/UDP (dune.TagUdp, both tags at
  once). UDP VERIFIED working on the current network 2026-07-21 (the old iitk
  broadcast block did not bite; no firmware change / hotspot needed).
- Radio: MODE_LONGDATA_RANGE_ACCURACY (110 kb/s, PRF 64, long preamble), CHANNEL_5.
  Driver applies the APS011 power-bias table on every RX timestamp (always on).

## Board → sketch map
| Board | Sketch | ID |
|---|---|---|
| Anchors 1–5 | `sketches/Anchor/Anchor.ino` | `ANCHOR_ID` 0x01–0x05 |
| Tag with IMU | `sketches/TagWrover/TagWrover.ino` | 0xF0 (COM12) |
| Tag without IMU | `sketches/Tag/Tag.ino` | 0xF1 (COM11) |
