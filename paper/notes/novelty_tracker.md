# Novelty Tracker

*Each contribution listed with: claim, what makes it novel, evidence required, current status.*

---

## Contribution Candidates (to be refined after Q&A)

### C1 — Scalable addressed-TDMA ranging protocol
**Claim:** A custom addressed-only TWR protocol that removes the 4-anchor ceiling and
collision problem present in all existing open-source DW1000 libraries (thotro, Makerfabs,
jremington, pizzo00, dw1000-ng).

**Novel over prior work:**
- thotro overflows at 5 anchors (17B × 4 = 68B in a 90B buffer)
- All existing libraries use broadcast POLL causing RF collisions
- Our library: addressed POLL, software frame filtering, unlimited anchor count

**Evidence required:**
- [ ] Table comparing all existing libraries (anchor ceiling, collision, rate, multi-tag)
- [ ] Experiment: sweep rate vs. N anchors (3, 4, 6, 8) — demonstrate no degradation
- [ ] Comparison: measured accuracy with existing library vs. ours (same hardware)

**Status:** PARTIAL — comparison table partially written in plan doc; no hardware experiment yet.

---

### C2 — Firmware-level NLOS detection with same-frame FP/RX power ratio
**Claim:** We read first-path power and total RX power from the same received frame on the
tag side, producing a per-anchor NLOS score (rx − fp gap in dB) that is physically valid.
Existing libraries store FP/RX power but never use them, and prior implementations computed
the gap from two different link directions (physically meaningless).

**Novel over prior work:**
- thotro: reads `_FPPower` and `_RXPower` but never uses them (comment "TODO NLOS")
- Our fix: tag reads both from the same RANGE_REPORT frame → valid same-frame comparison

**Evidence required:**
- [ ] NLOS scenario experiment: anchor behind wall or obstacle; measure gap, show weighting
      reduces position error
- [ ] Verification: gap metric in LOS vs. NLOS (should cluster around 0-2 dB vs. >6 dB)
- [ ] Offline result: "weighting cut 0.4m NLOS anchor position error 21.8 → 7.5 cm" (from plan) — needs hardware confirmation

**Status:** CLAIMED (from offline sim); hardware pending.

---

### C3 — Automatic anchor self-survey via inter-anchor ranging + MDS
**Claim:** The tag orchestrates pairwise DS-TWR between anchors, computes their layout
via classical MDS (no tape measure, no camera), and writes anchors.json automatically.
~1-2 cm anchor position accuracy.

**Novel over prior work:**
- No existing DW1000 library implements anchor self-survey
- Decawave AltDS-TWR for anchor-to-anchor ranging exists at chip level but not integrated
  into an open-source system with MDS coordinate recovery

**Evidence required:**
- [ ] Compare self-survey result vs. tape-measure ground truth (e.g., 4 anchors in a room)
- [ ] Error: |MDS position − tape| per anchor (claim ~1-2 cm)
- [ ] Experiment: repeat survey 5 times, show repeatability

**Status:** NOT VALIDATED on hardware. MATLAB MDS code complete.

---

### C4 — Two-tier calibration: hardware antenna delay + spatial error map
**Claim:** A two-tier calibration pipeline: Tier-1 (empirical binary search of per-board
antenna delay stored in NVS) eliminates bulk constant bias; Tier-2 (spatial IDW error map
from grid placement) removes position-dependent multipath residuals without any software
bias that could double-correct.

**Novel over prior work:**
- Most systems use a single global range offset
- Our analysis shows residual after antenna-delay calibration is position-dependent and
  non-monotonic → a spatial map is the correct abstraction
- The double-correction bug in naive software-bias + hardware-push is non-obvious and
  documented

**Evidence required:**
- [ ] Before/after Tier-1: RMSE comparison (uncalibrated vs. calibrated)
- [ ] Before/after Tier-2: RMSE comparison at grid points and at held-out positions
- [ ] Heat-map of the spatial error field per anchor

**Status:** Algorithm designed and offline-verified (5.9 cm → 0.23 cm in simulation);
hardware calibration PENDING.

---

### C5 — Dual-tag geometric yaw for full 6-DOF rover pose
**Claim:** Two UWB tags at a known rigid separation on a rover chassis provide
geometric heading from the inter-tag baseline vector in the world frame,
combined with 5-anchor 3D positioning (Z observable via elevated anchor) and
BNO085 IMU roll/pitch, yielding full 6-DOF pose at \$378 total hardware cost.

**Novel over prior work:**
- No existing open-source UWB system demonstrates full 6-DOF pose from UWB+IMU
  on a physical ground robot
- Prior dual-anchor/tag work (PMC11859676) targets UAVs with anchors on-vehicle;
  our formulation is the transpose: two tags on rover, shared fixed anchor infrastructure
- The specific combination (dual-tag yaw + elevated-anchor Z + IMU roll/pitch + Singer EKF)
  has not been published for ground robot 6-DOF pose

**Evidence required:**
- [ ] Hardware yaw RMSE vs. Kinect ground truth (claimed ~4.8° for σ_p=3cm, L=50cm)
- [ ] Z-axis RMSE from 5-anchor 3D trilateration
- [ ] Roll/pitch accuracy from BNO085 static and dynamic experiments

**Status:** Theoretical; hardware validation pending.

---

### C6 — Low-cost, sub-5cm indoor positioning with multi-tag and N-anchor scaling
**Claim:** A complete, open-source, microcontroller-native RTLS achieving <5 cm 2D RMSE
with commercial off-the-shelf hardware ($54/board, $378 total), supporting unlimited anchors
and simultaneous multi-tag operation, with no cameras or proprietary infrastructure.

**Novel over prior work:**
- Existing open-source DW1000 systems are limited to 4 anchors, single-tag, or require
  commercial Decawave EVK hardware
- Makerfabs boards are readily available (order-to-ship days, not weeks)

**Evidence required:**
- [ ] Quantitative RMSE < 5 cm (hardware measurement, both static and dynamic rover)
- [ ] Multi-tag demonstration (2 tags tracked simultaneously on rover)
- [ ] Cost breakdown per node vs. commercial alternatives (Pozyx, Decawave MDEK1001)

**Status:** RMSE is claimed 3-10 cm (pre-Tier-1 calibration); Tier-1 calibrated values
pending. Cost confirmed: $54/board × 7 boards = $378.

---

## Contributions to DROP or defer

- **Phase 1.3B (6.8 Mbps mode):** REVERTED due to hardware incompatibility. Mention as
  negative result/failure mode but NOT as a contribution.
- **MHE (Phase 3.3):** Not implemented. Do not claim.
- **3D/RPY (Phase 4.x):** Not hardware-validated. Mention as future work unless we implement
  Phase 4.1-4.3 before submission.
- **On-device solve (Phase 6.3):** Not implemented. Future work.
- **δ-solve (Phase 2.5):** Deferred. Mention as future work.

---

## Novelty Assessment (honest)

**Strong:** C1 (protocol scalability) — directly verifiable against published libraries.
**Medium:** C2 (NLOS detection) — the same-frame fix is real but the accuracy improvement
needs hardware quantification to be a strong contribution.
**Medium:** C3 (self-survey) — MDS anchor recovery is known; novelty is the integrated
firmware trigger + MATLAB pipeline. Requires hardware validation of claimed 1-2 cm accuracy.
**Speculative:** C4 (spatial calibration) — the insight is real but the map itself is not
unusual in the localization literature; the novelty is the practical integration and the
double-correction bug analysis.
**Weak alone:** C5 (system integration) — RTLS systems exist; the contribution here is the
combination, open-source release, and validated performance. Needs very strong experimental
results to carry this as a standalone claim.

**Reviewer risk:** A Reviewer 2 will ask: "How does this compare to commercial Decawave TREK
units or DW3000 systems?" We need a direct comparison or a clear argument for why those are
not valid baselines (cost, openness, customizability).
