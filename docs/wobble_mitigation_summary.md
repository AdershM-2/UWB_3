# DUNE UWB — Parked-position wobble: mitigation history & open questions

_Prepared 2026-07-21 for external review. All numbers are measured, not estimated._

## System

- 5× DW1000 anchors (ESP32 hosts), room-scale layout ~5.9 × 3.0 m, antennas at z = 0.24 m;
  2 tags (one with BNO085 IMU). Asymmetric double-sided TWR, addressed round-robin sweeps,
  5–7 Hz per tag over USB serial to a MATLAB host.
- Ground truth: ceiling Kinect v2, click-registered to the world frame (registration RMSE
  13.5 mm); any point at a known height can be measured to ~mm-cm.
- Calibration state before the wobble investigation:
  - Anchor antenna delays tuned on the production ranging path against Kinect truth
    (feedback loop over `SETANTDELAY`); parked reference point solves 4 mm from truth.
  - Per-anchor range correction, linear in first-path power (7.7–9.5 mm/dB, fitted on a
    19-spot truth campaign, leave-one-spot-out validated). Static position RMSE across the
    floor: 199 mm raw → 94–98 mm corrected (per-sweep, unfiltered).

## Symptom

With a parked tag, the solved position shows three distinct behaviours:

1. **Fast scatter**: ~25 mm std per sweep (white-ish).
2. **Slow wander ("breathing")**: the 2-second-median position drifts ~75 mm (median over
   19 parked dwells; worst 146 mm) over seconds-to-minutes.
3. **Discrete jumps** of 15–30 mm whenever the *answering anchor subset* changes
   (scheduler skip-backoff, failed exchanges): different subsets average the residual
   per-anchor biases differently.

## Mitigations tried, in order (measured effect)

| # | Mitigation | Mechanism | Result |
|---|---|---|---|
| 1 | On-device antenna-delay calibration (sweep-path feedback vs Kinect truth) | removes per-board constant bias | metre-scale errors → cm; parked ref point 4 mm. No effect on wobble (bias ≠ noise) |
| 2 | Power-bias correction (per-anchor linear in first-path power) | removes signal-level leading-edge bias | floor-wide static RMSE 199 → 94–98 mm. Reduces per-spot bias, not scatter |
| 3 | CV Kalman filter (6-state, gated position updates; adaptive R = LM covariance × RMSE/NLOS/DOP inflation, clip 50×) | temporal smoothing + outlier gating | static per-sweep RMSE 102 → 93–98 mm (sits on the spatial-bias floor); scatter visually reduced, slow wander passes through |
| 4 | ZUPT (zero-velocity pseudo-measurement). IMU tag: windowed stillness (max\|acc\| < 0.12 m/s², max\|gyro\| < 0.05 rad/s over 8 samples). No-IMU tag: filter-inferred stillness (\|v\| < 0.08 m/s and NIS < 3 for ~1.5 s) | clamp velocity while parked | reduces velocity-driven drift; position still followed measurement noise (see #5) |
| 5 | stillMode: freeze process noise when still (σ_accel 0.8 → 0.05 m/s²) | ZUPT alone left the CV process noise re-injecting ~30 mm/step of position uncertainty, keeping the Kalman gain open | parked dot genuinely pins; no effect on subset-change jumps (a persistent offset defeats smoothing) |
| 6 | Tightly-coupled per-range updates (sequential, χ²(1)-gated) | per-range outlier rejection, works with < 3 anchors | **failed on full sessions**: partial gate acceptance keeps the filter alive at a wrong position (2.2 m → 0.34 m with recovery nets; still worse than loose coupling statically). Needs per-range timestamps + moving truth to justify |
| 7 | Per-anchor range-bias states in the EKF (z = ‖p−a‖ + b_k, random walk 3 mm/√s, biases retained through dropouts) | learn & remember each anchor's residual bias so subset changes stop mattering | subset-change jump 13–30 mm → **6–9 mm** |
| 8 | NLOS soft-weight ablation (rx−fp gap weights off, MAD gate kept) | weight flutter continuously varies the effective anchor mix | scatter 26 → 24 mm, wander unchanged, accuracy slightly better (48 → 39 mm). Weights now optional |
| 9 | RangeHold: last-known-range memory per anchor (held value = median of last 5 samples; full weight while still, ~1 s decay + 3 s cap while moving) | keep the full 5-anchor geometry in every solve — the subset never changes | subset-change jump 13–28 mm → **6–10 mm at the raw-fix level** (all downstream modes benefit) |

**Tried and rejected:**
- IMU-accelerometer-driven prediction: BNO085 world-frame rotation unvalidated → DC accel
  leak → 6–14 m divergence (with gating locking out recovery). Disabled until the dual-tag
  baseline validates the orientation frame.
- Neural-network / physics-informed (monotonicity prior) power-bias models: LOSO on 31.5k
  samples — linear 98 mm vs MLP 107 / PINN 108 mm position RMSE. No gain at 19 spots;
  revisit with dense spatial data.

## Where it stands

With everything enabled (parked tag): fast scatter pinned (~<2 cm), subset jumps ≤ 1 cm,
**remaining wobble = the slow ~5–7 cm multipath "breathing"** — a temporally correlated,
spatially local error that: (a) is invisible to signal-level features (first-path power,
rx−fp gap barely correlate with it), (b) survives temporal filtering (it looks like slow
truth motion to any filter), and (c) an oracle test bounds: even a *perfect* per-spot
constant range correction leaves ~72 mm per-sweep RMSE (tails included).

## Open anomaly (possibly related)

HWCALIB burst ranging (50 back-to-back exchanges to one anchor, 5 ms spacing) and the
production ring-sweep ranging disagree by a **constant per-anchor offset (+60…+500 mm)**
despite calling the identical `rangeTo()` DS-TWR code — reproduced across runs, linear
4.72 mm/tick within each path. Cause unknown (radio warm-up/AGC state? inter-exchange
timing?). Calibration is therefore done on the sweep path.

## Questions for review

1. Is the slow "breathing" consistent with DW1000 experience (multipath fading as people /
   reflections move; clock/temperature drift)? Any known instrumentation to separate the
   two (e.g. temperature logging, CIR capture)?
2. Would channel / PRF / preamble changes plausibly reduce it, or is spatial averaging
   (dense error map) the realistic path?
3. Any known mechanism for the burst-vs-sweep constant offset above?
4. Filtering: better alternatives to hard χ² gating for this regime (robust/Huber updates,
   IMM still/moving)?
5. Does per-range timestamping (ranges within a 125 ms sweep are sequential) materially
   matter at ≤1 m/s, in your experience?
