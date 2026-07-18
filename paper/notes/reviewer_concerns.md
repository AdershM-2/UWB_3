# Potential Reviewer Concerns

*Updated continuously. Each concern gets a response status: [?] open | [~] partial | [x] addressed.*

---

## METHODOLOGY

[?] **"What is your ground truth? Tape measure is not sufficient for a 3 cm accuracy claim."**
A motion capture system (OptiTrack/Vicon) or a high-accuracy camera-based reference would be
needed. We need to define our GT method carefully and state its accuracy.

[?] **"Why only 3 anchors? 3 anchors provide poor Z observability and marginal 2D geometry."**
We need to explain our anchor geometry, DOP, and report the 4-anchor configuration for the
3D case. 3 is the minimum for 2D — but reviewers will ask for 4 (adds redundancy + δ-solve).

[?] **"You report 3-10 cm accuracy — that's a very wide range. What is the median and 90th
percentile RMSE over a systematic evaluation?"**
We need a proper statistical evaluation (at least 10-20 static positions + a dynamic path).

[?] **"How does your system compare to commercial Decawave TREK/MDEK1001 or Pozyx?"**
These are the obvious baselines. We need either a direct comparison or a clear argument
(cost, openness, open-source, customizability, embedded on-device solve).

[?] **"The NLOS detection contribution seems incremental — NLOS with FP power is described
in Decawave APS006 and multiple prior papers. What is new?"**
The novelty is the same-frame measurement fix and the integrated software stack, not the
NLOS formula itself. We need to carefully position this.

[?] **"The anchor self-survey accuracy of 1-2 cm is claimed but not demonstrated."**
Need hardware experiment comparing self-survey vs. tape-measure ground truth.

[?] **"Your EKF is standard CV model. Why not a constant-acceleration model or tightly
coupled fusion?"**
Phase 3.1 (CA-EKF) and Phase 4.4 (tight coupling) address this. If not implemented by
submission, we need to explicitly acknowledge this as a limitation.

[?] **"ESP32 + DW1000 is an obsolete chip combination (DW1000 last production 2023, replaced
by DW3000). Why not use DW3000?"**
DW1000 is still widely deployed, well-documented, and has an active open-source ecosystem.
The techniques generalize to DW3000. Address this head-on in the intro.

[?] **"What is the update rate and latency? 10 Hz for a mobile robot is marginal."**
Phase B (RTOS, 50 Hz IMU prediction) addresses this. Acknowledge 10 Hz UWB is a current
limitation; IMU prediction between sweeps (Phase 4.4) gives continuous output at IMU rate.

[?] **"The spatial error map requires calibration time. How long does it take, and how
stable is it over time?"**
Need: time to capture 9-point grid, and a re-evaluation experiment showing error map
validity over days/weeks.

---

## RESULTS

[?] **"All experiments are offline / simulated. Where are the real-world hardware results?"**
Hardware experiments are the critical pending item. All code is complete; hardware validation
is the bottleneck.

[?] **"Figure X only shows 2 minutes of a trajectory. Show longer runs."**
Plan for at least 10-minute experiments for stability demonstration.

[?] **"The multi-tag experiment doesn't quantify inter-tag interference."**
We need to show 2-tag accuracy is comparable to 1-tag accuracy (no TDMA collisions
degrade performance significantly at 10 Hz).

---

## SCOPE / NOVELTY

[?] **"This is a systems paper, not an algorithm paper. What is the theoretical contribution?"**
The DS-TWR clock-offset estimator, the LM multilateration, and the FusionEKF with
Singer noise model are the algorithmic contributions. The systems integration is the
primary contribution, and that is legitimate for ICRA systems papers.

[?] **"Is this an ICRA contribution or just an Arduino library?"**
The contribution is the complete validated system design, including: the protocol that
breaks the 4-anchor ceiling, the calibration pipeline that avoids double-correction,
the NLOS weighting, and the demonstrated accuracy. Framing matters here.

[?] **"What is the intended application? The paper seems to be a generic positioning system."**
We must pick a specific application (robot manipulation, person tracking, warehouse) and
motivate it concretely. Generic RTLS papers are weak without a concrete use case.

---

## WRITING

[?] **"Section X has too many equations and not enough experimental validation."**
Balance equations with experimental validation. Every claimed accuracy number must be
backed by hardware data.

[?] **"Your contributions are listed but not prioritized."**
Order contributions from most to least novel/impactful.

[?] **"Related work section misses [X paper]."**
Maintain a comprehensive BibTeX file. Check for missing citations after every draft.
