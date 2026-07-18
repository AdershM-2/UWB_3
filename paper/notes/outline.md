# Living Manuscript Outline

*Status markers: [ ] empty | [~] placeholder text written | [draft] first draft | [rev] revised | [done] camera-ready*
*This is the structural skeleton. Actual section files live in paper/sections/.*

---

## Abstract [~]
- 1 sentence: problem statement (indoor localization without cameras)
- 1-2 sentences: what we built (UWB RTLS, scalable, low-cost, DW1000-based)
- 1 sentence: key contributions (protocol, calibration, NLOS, self-survey)
- 1-2 sentences: key results (TODO: RMSE, update rate, number of anchors demonstrated)
- 1 sentence: open-source availability

TODO: insert localization RMSE once hardware experiments complete.

---

## I. Introduction [ ]
### I-A. Motivation
- Indoor positioning use cases: robot manipulation, warehouse tracking, person-following robots
- Why not cameras: privacy, lighting dependence, infrastructure cost
- Why not BLE/WiFi RSSI: meter-level accuracy, not cm-level
- Why not LiDAR SLAM: cost, requires mapped environment
- UWB: cm-level ToF ranging, works in darkness, 4 nodes, ~$150 total hardware

### I-B. Problem
- N fixed anchors at known or self-surveyed locations
- 1+ mobile tags reporting ranges → host solves position
- Requirements: <5 cm 2D RMSE, >5 Hz update, multi-tag, N-anchor scaling

### I-C. Challenges
- Existing open-source DW1000 libraries: 4-anchor ceiling, RF collisions, 0.6 Hz
- NLOS through walls and furniture: systematic positive range bias
- Antenna delay calibration: uncalibrated → 30-50 cm bias
- Multi-tag: collision avoidance, per-tag state management

### I-D. Paper Contributions
- C1: Scalable addressed-TDMA protocol (unlimited anchors, no RF collision)
- C2: Same-frame NLOS detection + weighted multilateration
- C3: Automatic anchor self-survey via inter-anchor DS-TWR + MDS
- C4: Two-tier calibration (hardware delay + spatial error map)
- C5: Complete open-source system achieving <X cm RMSE (TODO: fill in)

---

## II. Related Work [ ]
### II-A. UWB Indoor Positioning Systems
- Decawave / Qorvo commercial systems (TREK1000, MDEK1001)
- Open-source DW1000 libraries (thotro, Makerfabs, jremington, pizzo00, dw1000-ng)
- Our specific improvements over each

### II-B. NLOS Mitigation in UWB
- Channel impulse response analysis (APS006)
- Machine learning NLOS classifiers
- Robust estimation approaches (Huber, M-estimators)
- Our approach: FP/RX power gap as lightweight score, weighted LM

### II-C. Anchor Placement and Self-Calibration
- Optimal anchor placement (DOP minimization)
- Simultaneous localization and anchor calibration (joint optimization)
- MDS-based coordinate recovery
- Our approach: firmware-integrated inter-anchor survey → MDS → auto config

### II-D. Sensor Fusion for Indoor Positioning
- UWB + IMU loose coupling (EKF position-level)
- UWB + IMU tight coupling (EKF range-level, better under sparse anchors)
- UWB + pedestrian dead reckoning
- Our contribution: Singer-model FusionEKF with adaptive R, ZUPT, NIS gating

### II-E. Research Gap
- No existing open-source system simultaneously addresses: scalable protocol +
  NLOS detection + self-survey + two-tier calibration in a validated hardware system.
- We fill this gap.

---

## III. Problem Formulation [ ]
### III-A. Notation and Coordinate Frames
- World frame: ENU, Z-up, gravity = [0,0,-9.81] m/s²
- Body frame: fixed to tag
- Anchor positions: a_i ∈ R^dim, i=1..N
- Tag position: p(t) ∈ R^dim

### III-B. DS-TWR Measurement Model
- Range measurement: ρ_i = ||p - a_i||₂ + δ_i + ε_i
  - δ_i: bias (NLOS, multipath, antenna delay)
  - ε_i: zero-mean Gaussian noise (σ ≈ 0.05–0.15 m)

### III-C. Localization Objective
- Estimate p(t) given {ρ_i(t)} and {a_i}
- Secondary: estimate orientation from IMU quaternion

---

## IV. System Architecture [ ]
### IV-A. Hardware Platform
- Board: Makerfabs ESP32 UWB Pro (ESP32 + DW1000 + OLED)
- TODO: photograph, hardware spec table
- DW1000 vs. DW3000 note (why DW1000)

### IV-B. Network Topology
- 3+ fixed anchors at known/surveyed positions
- 1+ mobile tags
- Host computer (Python or MATLAB)
- Transport: WiFi UDP (primary) or USB serial

### IV-C. Software Stack Overview
- Firmware (Arduino/ESP32): TwrEngine, UwbScheduler, HostLink
- Host pipeline: FrameParser → Multilaterator → FusionEKF → GUI/Log
- Config: anchors.json (one place for anchor coordinates)

---

## V. UWB Ranging Protocol [ ]
### V-A. Asymmetric DS-TWR
- 4-message exchange (POLL, POLL_ACK, RANGE, RANGE_REPORT)
- Clock-offset-cancelling estimator (ToF formula)
- Delayed transmit for precise TX timestamp

### V-B. Custom Frame Format
- 4-byte header: type, src, dst, seq
- Software address filtering (not 802.15.4 MAC)
- Message types: POLL, POLL_ACK, RANGE, RANGE_REPORT, SURVEY_REQ/RESP, ANT_DELAY/ACK

### V-C. Scalable Round-Robin Scheduler (UwbScheduler)
- Addressed POLL → only targeted anchor replies
- N-anchor operation with no RF collisions
- Dead-anchor skip with exponential backoff
- Sweep rate analysis vs. N anchors

### V-D. Comparison with Existing Libraries
- Table: thotro, Makerfabs, jremington, pizzo00, dw1000-ng vs. ours
- Root cause of 4-anchor ceiling in thotro (17B × 4 = 68B buffer overflow)
- Broadcast POLL collision probability analysis

---

## VI. NLOS Detection and Weighted Multilateration [ ]
### VI-A. Physics of UWB NLOS
- FP power vs. RX power definition (APS006)
- NLOS score: gap = rx_dBm − fp_dBm
- Thresholds: LOS < 3 dB, soft NLOS 3-6 dB, hard NLOS > 6 dB

### VI-B. Firmware NLOS Score (Phase 2.1)
- Same-frame measurement (our fix)
- Why prior implementations were wrong (mixed-link rx/fp)

### VI-C. Weighted Levenberg-Marquardt Multilateration
- Standard LM formulation
- Weight: w_i = 10^(−max(0, gap_i − thresh) / 10)
- Weighted Jacobian and covariance (GLS form)
- MAD outlier gate (independent from soft weighting)
- NLOS range bias correction (d − k·max(0, gap − thresh))

### VI-D. Adaptive Measurement Covariance
- R_eff = R_base × clip(rms_factor × nlos_factor × dop_factor, 1, 50)
- Propagates ranging quality into the EKF

---

## VII. Calibration [ ]
### VII-A. Antenna Delay Calibration (Tier 1)
- Source of error: DW1000 processing time offset (ticks → distance)
- Binary search procedure (HWCALIB)
- Push via UWB relay; NVS storage on anchor
- Effect: ~30-50 cm uncalibrated → <1 cm after calibration

### VII-B. Spatial Error Map (Tier 2)
- Residual error after antenna-delay calibration: position-dependent, non-monotonic
- Grid placement protocol (3×3 grid, 9 points)
- IDW interpolation with ε-smoothing and linear fade
- Subtracted before multilateration

### VII-C. Anchor Self-Survey (Phase 1.5)
- MSG_SURVEY_REQ/RESP protocol
- Classical MDS recovery of anchor coordinates
- Frame convention (anchor 1 at origin, anchor 2 on +X)
- Accuracy: ~1-2 cm (TODO: validate)

---

## VIII. State Estimation [ ]
### VIII-A. Multilateration (Levenberg-Marquardt)
- Robust residual gating (MAD, k=3)
- Covariance approximation: σ² (J'J)⁻¹
- Fall-through to prior position if solve fails

### VIII-B. FusionEKF (UWB + IMU)
- State vector [px, py, vx, vy, bx, by]
- IMU-driven prediction (Singer model)
- CV fallback when IMU absent/unreliable
- Measurement update (H = [I₂ 0 0])
- NIS gate (χ²(2, p=0.95) = 5.991)
- ZUPT detection (sliding window, 4-condition gate)
- Adaptive R_eff

### VIII-C. Coordinate Frame and Gravity Removal
- BNO085 rotation vector → world frame rotation
- Linear acceleration (gravity-removed) → a_world_xy
- Yaw from BNO085 magnetometer fusion (not re-estimated in EKF)

---

## IX. Experimental Setup [ ]
### IX-A. Hardware Configuration
- TODO: room dimensions, anchor heights, anchor coordinates
- TODO: photograph of deployment

### IX-B. Ground Truth
- TODO: what is the reference? (tape, AprilTag, mocap, laser tracker)
- Static accuracy test: tag at K known positions, N samples each
- Dynamic accuracy test: tag carried along known trajectory

### IX-C. Evaluation Metrics
- RMSE (2D and 3D), percentile errors (50th, 90th, 95th, 99th)
- Maximum error
- Update rate (Hz)
- CPU/latency (radio pulse → GUI)

### IX-D. Baselines
- TODO: thotro library (if comparison experiment runs)
- TODO: commercial reference (Pozyx/TREK)

---

## X. Results [ ]
### X-A. Ranging Accuracy
- Static range error per anchor (after calibration)
- NLOS scenario (anchor obstructed): gap values, weighted vs. unweighted RMSE

### X-B. Localization Accuracy (2D)
- Static RMSE at K positions
- Dynamic RMSE along walked path
- CDF of error

### X-C. Scalability
- Sweep rate vs. N anchors (3, 4, 6, 8)
- No collision artifacts with N anchors

### X-D. Calibration Results
- Before/after Tier-1 RMSE
- Before/after Tier-2 RMSE

### X-E. Self-Survey Accuracy
- MDS positions vs. tape-measure ground truth
- Localization accuracy using self-survey anchor map vs. measured anchor map

### X-F. Multi-Tag Operation
- 2-tag tracking accuracy vs. 1-tag (no interference degradation)

---

## XI. Ablation Study [ ]
- NLOS weighting: enabled vs. disabled
- Spatial error map: enabled vs. disabled
- EKF: CV vs. CA vs. FusionEKF (with IMU)
- Median pre-filter: N=1 (raw) vs. N=8 (filtered)
- Anchor count: 3 vs. 4 anchors (DOP, accuracy, Z observable)

---

## XII. Discussion [ ]
- Phase 1.3B failure (6.8 Mbps hardware incompatibility) — lessons
- Chicken-and-egg in spatial error map (position-dependent correction uses prior position)
- Limitations: 2D only (all anchors coplanar), 10 Hz UWB rate, Phase A IMU prediction
- Applicability to DW3000 (techniques transfer; driver change only)

---

## XIII. Limitations [ ]
- 2D only (anchors co-planar)
- ~10 Hz update rate (Phase B RTOS needed for 50 Hz)
- No TDMA collision avoidance for multi-tag (rely on backoff)
- Spatial error map requires re-capture when anchor layout changes
- DW1000 end-of-life (DW3000 migration path described)

---

## XIV. Future Work [ ]
- 3D with 4th anchor at different height (Phase 4.1)
- BNO085 IMU integration (Phase 4.2) → RPY output
- Tight coupling: raw ranges into EKF (Phase 4.5)
- MHE with Huber loss (Phase 3.3)
- RTOS for high-rate IMU prediction (Phase B)
- DW3000 port
- δ-solve in EKF (bias state, works with 3 anchors, Phase 3.2)
- Multi-tag TDMA superframe (Phase 6.1)

---

## XV. Conclusion [ ]
- We presented X (system name)
- Key results: <X cm RMSE, N anchors, Y Hz, multi-tag
- Open-source at [TODO: GitHub URL]

---

## Appendices [ ]
### A. DS-TWR Estimator Derivation
### B. LM Multilateration Derivation
### C. MDS Anchor Self-Survey Derivation
### D. System Parameters Table (all tunable parameters)

---

## References [ ]
(Populated from paper/bibliography/refs.bib — not yet created)
