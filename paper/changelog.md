# Manuscript Changelog

Track every substantive change to the manuscript here.
Format: `YYYY-MM-DD | section | what changed | triggered by`

---

## 2026-06-30 | ALL | Directory structure created; pre-writing phase initiated.

## 2026-06-30 | MOST SECTIONS | First full drafting pass after 50-question Q&A session.

### Sections drafted (not yet validated by experiments):

- `abstract.tex` — rover application, 5A/2T config, Kinect GT, Singer EKF, \$378 cost; RMSE TBD
- `introduction.tex` — rover motivation, library bug analysis (L1/L2/L3), 5 contributions
- `related_work.tex` — 5 subsections with found/placeholder citations; research gap stated
- `problem_formulation.tex` — coordinate frames, range model, weighted NLS, observability
- `architecture.tex` — hardware table, 5-anchor topology, dual-tag yaw, 6-DOF pipeline, wire format v3
- `protocol.tex` — DS-TWR 4-message exchange, ToF formula, frame/msg-type tables (actual LaTeX), scheduler backoff, library comparison table
- `nlos.tex` — same-frame NLOS fix, w_i formula, MAD gate, NLOS range bias
- `calibration.tex` — Tier-1 binary search, antenna delay table (6 entries), Tier-2 IDW, MDS survey, failure mode (double-correction bug)
- `state_estimation.tex` — FusionEKF (Python only), Singer Q, adaptive R_eff, NIS gate, ZUPT, roll/pitch/yaw assembly, 6-DOF output
- `experimental_setup.tex` — Rocker-Bogie rover, 5-anchor table, Kinect v2 + ArUco GT, 5 scenarios, metrics list
- `conclusion.tex` — 5 contributions, rover/\$378 context, future work
- `appendix_dstwr.tex` — skeleton (full clock-offset derivation TBD)
- `appendix_lm.tex` — Jacobian, weighted LM, covariance approximation

### Notes:
- `results.tex` and `discussion.tex` have stubs/partials; no experimental numbers yet
- `main.tex` updated: 3 authors (Atharva Chauhan, Adarsh M, Ashish Tata); institution TBD
- `refs.bib` populated with 14 entries including all found arXiv papers (need author/title verification)
- `novelty_tracker.md` updated: C5 = dual-tag 6-DOF, C6 = low-cost system integration
- `paper_todo.md` updated: all section statuses, experiment checklist, citation verification status

---

*This file should be updated whenever: (a) a section is drafted or substantially revised,
(b) experiments are added, (c) code changes affect claims in the paper, or (d) a contribution
is re-scoped.*
