# DUNE / UWB_3 — Clarification Questionnaire (answered 2026-07-01)

Answered via popup; recorded here for reference.

## Q1. Codebase intent
**Answer:** Clean restart — build a **new MATLAB host pipeline** in UWB_3, porting the good parts
of the Python code (GP error field, NLOS weighting, rigid-body dual-tag EKF). Python retired.

## Q2. Hardware status
**Answer:** The two-tag token ring **works now** — the resets from the 1 July notes were fixed.
All boards assumed flashed with the current UWB_3 sketches.

## Q3. Definition of the 3 cm target
**Answer:** **2D RMSE ≤ 3 cm with the rover moving** (the hardest interpretation).

## Q4. Ground truth
**Answer:** **Trust the Kinect as-is** for calibration/verification.
*(Caveat noted by Claude: the stored world registration RMSE is 0.259 m — the Kinect world-frame
registration must at least be re-run once, folded into the first calibration session, otherwise
truth is offset by ~26 cm and 3 cm cannot be verified. Kinect per-point accuracy 35–100 mm caps
what a single GT sample can certify; averaging over trajectories mitigates this.)*

## Q5. Testbed environment
**Answer:** **Indoors, stable temperature.** Temperature drift deprioritised (observed drift was
likely board warm-up; optionally log DW1000 on-chip temp later).

## Q6. Allowed changes
**Answer:** **Protocol redesign is allowed but must be discussed in detail first.** No blanket
approval yet for reflashing, 6.8 Mb/s retry, or hardware changes — bring a concrete proposal.

## Q7. Consumer of the pose
**Answer:** **PC display/logging only.** Higher rate is nice-to-have, not a control requirement.

## Q8. Rig geometry
**Answer:** **0.5 m fixed baseline**, two tags on an acrylic plate with the AprilTag on it;
BNO085 is attached to one tag (TagWrover 0xF0).

## Q9. Anchor deployment
**Answer:** **Anchors are permanently installed** — a one-time spatial error map stays valid.

## Q10. Priorities & NLOS sources
**Answer:** Priority order: **Accuracy (3 cm) first**; protocol/rate and fusion follow.
NLOS sources: **fixed structures** and the **rover body itself** (heading-dependent self-blockage).
Yaw: magnetometer trustworthiness unverified — default to dual-tag yaw + gyro; test mag later.
