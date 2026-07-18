# Paper TODO Tracker

*Living document. Updated 2026-06-30.*
*Format: [ ] not started | [~] in progress | [x] done | [?] needs decision*

---

## DECISIONS — Answered 2026-06-30

[x] Application: 6-wheeled Rocker-Bogie rover, 13 DOF, GPS-denied indoor environments
[x] Ground truth: Kinect v2 ceiling-mounted + ArUco markers (dynamic experiments primary)
[x] Host pipeline: Python ONLY — MATLAB does not appear in the paper
[x] IMU fusion is a primary contribution (BNO085 physically on Tag 1)
[x] Negative results to include: 6.8 Mbps reversion (Phase 1.3B) AND polynomial bias double-correction
[x] Python pipeline: confirm all math in state_estimation.tex is the Python FusionEKF
[x] Venue: not fixed; targeting IEEE RA-L or ICRA; no page limit constraint for now
[x] Hardware config: 5 anchors (4 coplanar + 1 elevated ~1m) + 2 tags at 50cm baseline
[x] Team: Atharva Chauhan, Adarsh M, Ashish Tata (3 authors)
[x] Cost: $54/board × 7 boards ≈ $378

[?] Decide final system name (e.g., "OpenRTLS" or "UWB-RTLS")
[?] Venue/year final decision
[?] Institution name and email addresses for author block
[?] Tag 0xF0 antenna delay (only 0xF1=16473 confirmed; Tag 0 value unknown)
[?] Anchor 5 (0x05) antenna delay (voice unclear: ~16485 or ~16408?)
[?] 2D EKF (current) vs. extend to 3D 9-state EKF — decide before writing Section 5
[?] Implement yaw complementary filter (UWB + BNO085 magnetometer) before submission?

---

## SECTIONS — Draft status

[x] Abstract — DRAFTED (has correct rover/5A/2T/Kinect/Singer context; needs RMSE numbers)
[x] Introduction — DRAFTED (has rover motivation, L1/L2/L3 library bugs, 5 contributions)
[x] Related Work — DRAFTED (5 subsections; citations need verification; research gap stated)
[x] Problem Formulation — DRAFTED (coordinate frames, range model, weighted NLS, observability)
[x] System Architecture — DRAFTED (hardware table, topology, dual-tag yaw, 6-DOF pipeline, wire format v3)
[x] Protocol (DS-TWR + Scheduler) — DRAFTED (4-message exchange, ToF formula, frame format, scheduler)
[x] NLOS Detection & Weighted Multilateration — DRAFTED (same-frame fix, w_i formula, MAD gate)
[x] Calibration — DRAFTED (Tier-1 table with 6 entries, Tier-2 IDW, self-survey MDS, failure mode)
[x] State Estimation — DRAFTED (FusionEKF Python only; Singer Q, NIS gate, ZUPT, 6-DOF output)
[x] Experimental Setup — DRAFTED (rover platform, 5-anchor table, Kinect GT, 5 scenarios, metrics)
[ ] Results — PLACEHOLDER ONLY (all numbers are \todo)
[ ] Ablation — embedded in results; all \todo
[~] Discussion — has 6.8 Mbps failure + calibration failure; needs hover on limitations
[x] Conclusion — DRAFTED (5 contributions listed, rover context, \$378, future work)
[x] Appendix A (DS-TWR derivation) — skeleton; needs full clock-offset cancellation derivation
[x] Appendix B (LM multilateration) — skeleton; Jacobian, W=diag(w_i), convergence

---

## EXPERIMENTS — Missing Data (hardware pending)

[ ] RMSE (2D, 3D if Z validated) — static, multiple grid positions
[ ] RMSE (2D) — dynamic rover trajectory with Kinect GT at 30 Hz
[ ] Heading RMSE ψ — dual-tag vs. Kinect heading; compare to BNO085 magnetometer heading
[ ] Ablation: no calibration → Tier-1 only → Tier-1 + Tier-2 (spatial map)
[ ] Ablation: no NLOS weighting → NLOS weighting → NLOS + bias correction
[ ] Ablation: CV-EKF → FusionEKF (IMU) quantitative comparison on rover data
[ ] Sweep rate measurement vs. N anchors (3, 4, 5) over 60 s each
[ ] Anchor self-survey: MDS result vs. tape-measure ground truth, repeat 5 times
[ ] NLOS scenario: known obstacle in front of one anchor, gap distribution, position error effect
[ ] Latency: radio pulse → Python output (ms)

---

## FIGURES — Not yet created

[ ] System overview / architecture block diagram
[ ] Photo: rover with two tags mounted and labelled (scale bar)
[ ] Photo: one anchor board showing OLED status
[ ] Room top-view + side-view: anchor topology (4 coplanar + 1 elevated), rover path
[ ] DS-TWR timing diagram (4-message exchange with timestamp labels)
[ ] Calibration pipeline diagram (Tier-1 binary search flowchart + Tier-2 grid)
[ ] NLOS gap diagram (FP power vs. total RX power; histogram LOS vs. NLOS)
[ ] Error CDF: our full system vs. uncalibrated baseline vs. Tier-1-only
[ ] Trajectory plot: ground truth vs. estimated (Kinect vs. EKF) on rover path
[ ] Sweep rate vs. N anchors line plot (demonstrates scalability)
[ ] Spatial error map heat-map (residual per anchor over grid)
[ ] EKF block diagram: predict (IMU), update (UWB), ZUPT branch

---

## TABLES — To complete

[x] Hardware specs (architecture.tex, Tab. hardware) — done
[x] Antenna delay calibration (calibration.tex, Tab. ant_delays) — done except A5, Tag 0xF0
[x] Anchor positions from self-survey (experimental_setup.tex, Tab. anchor_positions) — structure done, numbers TBD
[ ] Library comparison table (protocol.tex, \label{tab:library_comparison}) — referenced but not yet typeset as LaTeX table; exists as text in plan doc
[ ] Quantitative accuracy results (RMSE XY, Z, ψ; 50th/90th/95th percentile)
[ ] Ablation study table (7 rows: from no-calib to full system)
[ ] NLOS performance table (gap distributions, detection rate if labelled GT available)

---

## MATHEMATICS — Verify in appendices / sections

[~] DS-TWR estimator — skeleton present in appendix_dstwr.tex; full clock-offset cancellation derivation needed
[x] Jacobian J_i = (p-a_i)^T / ||p-a_i|| — in appendix_lm.tex
[x] LM update with W=diag(w_i) — in appendix_lm.tex
[x] EKF F matrix (6×6) — in state_estimation.tex
[x] Singer Q matrix (6×6) — in state_estimation.tex
[x] Adaptive R_eff — in state_estimation.tex
[x] NLOS weight w_i = 10^(−max(0,gap−3)/10) — in nlos.tex
[x] IDW spatial map formula — in calibration.tex
[x] MDS recovery B = -½JD²J — in calibration.tex
[x] Dual-tag yaw formula — in architecture.tex
[x] Yaw accuracy σ_ψ ≈ √2·σ_p / L — in architecture.tex
[ ] NIS gate threshold derivation (χ²(2, 0.95) = 5.991 — cite chi-squared table or source)
[ ] Observability argument: N≥4 anchors for 3D; why 1 elevated anchor fixes Z

---

## CITATIONS — Verification status

[~] Decawave DW1000 datasheet (dw1000_datasheet) — BibTeX entry exists, year/version TBD
[~] APS006 NLOS app note (aps006) — BibTeX entry exists, version TBD
[~] DS-TWR Neirynck 2016 (dstwr_neirynck) — entry exists, DOI 10.1109/WPNC.2016.7822844 to verify
[~] thotro library (thotro_dw1000) — entry exists
[~] Alarifi 2016 UWB survey (alarifi2016uwb) — entry exists with DOI; verify author list
[~] Torgerson 1952 MDS (torgerson1952mds) — entry exists
[~] Singer 1970 (singer1970) — entry exists
[~] ArUco Garrido-Jurado 2014 (aruco_garrido_2014) — entry exists with DOI; verify
[ ] All arxiv entries (gururaja2024nlos, gp_anchor_calib_2024, robust_ranging_imu_2309, aoi_fusionnet_2025, dual_anchor_uav_2025, zhao2024util, corbalan2021selfcal) — TODO fields; need real author/title verification
[ ] MDPI Sensors 2025 UWB vs BLE vs WiFi survey — not yet found
[ ] BNO085 datasheet / product page
[ ] Kinect v2 specification (kinect_v2 entry exists, verify URL)
[ ] IEEE 802.15.4a / 802.15.4z standard
[ ] EKF/Kalman reference (e.g., Thrun, Burgard, Fox 2005 "Probabilistic Robotics")
[ ] Competing indoor positioning systems (Pozyx, TREK1000/MDEK1001)
[ ] Rocker-Bogie rover reference (if we describe mechanical platform)
[ ] TIEMANN ATLAS paper (tiemann2017atlas entry exists; verify year, DOI)
