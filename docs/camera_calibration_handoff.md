# Overhead-camera calibration — session handoff (2026-07-09)

Continuation notes for the Kinect ground-truth camera work. Read alongside the
CLAUDE.md progress entry "Overhead-camera in-situ calibration toolchain
(2026-07-09)". Everything below is in `matlab/vision/` unless noted; all pixel
math uses the pipeline's **fliplr'd frame convention**. MATLAB R2024b.

## Where things stand

- Camera: Kinect v2, roof-mounted (**cannot be tape-measured**), pointing
  straight down at the sandy testbed. imaq device `'kinect'`, ID 1,
  `BGR_1920x1080`.
- **Height h = 3.873 m measured by parallax** (`estimateCameraHeight.m`, 6
  chair hops with dz ≈ 0.45–0.49 m) and SAVED in
  `matlab/vision/calibration_data/camera_calibration.mat`, which
  `visionSystemConfig()` auto-loads (`config.camcal`; calibrated h supersedes
  the old 3.43 m constant). The hop session was noisy (per-hop implied-h std
  0.69 m — tag re-placement error; the clean hops clustered 3.48–3.72 m), so
  treat 3.873 as provisional ±0.2 m.
- `auditFloorScale.m` (1 m tape pairs) after the fix: **centre 0.970, mid
  0.961, edge 0.949** (measured/true; was 0.849 uniform before). Residual
  error budget: ~3% uniform = the f/h split (f ≈ 1020 implied vs nominal
  1050), ~2% extra at edges = barrel distortion (uncalibrated).
- **nadir_px (890.8, 529.5) and tilt 3.8° in camera_calibration.mat are
  UNRELIABLE** — they came from the noisy hop session.
- `world_registration.mat` is quarantined as
  `calibration_data/world_registration.mat.bad-20260707` (RMSE 1.68 m). The
  solver was fine; the cause was the anchors.json mismatch below. There is
  currently NO valid world registration.
- **`matlab/config/anchors.json` does not match the deployed anchors**: its
  6.3×3.1 m layout is aspirational (uncommitted edit; the committed version
  was a 3-anchor bench layout). The 2026-07-07 clicks imply the real anchors
  form a **~2.5×2.6 m rectangle with anchor 5 near the centre** (values
  h-corrected by 3.873/3.43). Nadir-estimated anchor-frame coordinates —
  TAPE-VERIFY before use, mirror chirality unverified:
  a1 (0, 0), a2 (2.47, 0), a3 (2.52, 2.59), a4 (0.03, 2.54), a5 (1.50, 1.42).

## The tools (all new/updated this session)

| File | Purpose |
|---|---|
| `auditFloorScale.m` | Click tape-verified mark pairs → centre/mid/edge and per-direction scale ratios. THE acceptance test. |
| `estimateCameraHeight.m` | f-free parallax: hops = same XY at floor and at known dz (AprilTag on box/chair, or click anchor base→antenna tip). Solves h + nadir; rejects slides < 30 px; warns on high spread; can write into camera_calibration.mat. |
| `calibrateOverheadCamera.m` | Full 3-stage in-situ calibration. A: plumb-line strings → k1/k2 (+ centre with ≥5 lines). B: AprilTag floor sweep (12+ spots) + tape scale bars → pixel↔floor homography (ALS solve). C: box hops → refined h/nadir/tilt + focal f_est. Stages skippable, results merge. |
| `undistortPts.m` / `distortPts.m` | Single-source radial distortion model (no-op until stage A). |
| `registerWorldFrameClicks.m` | World registration by clicking anchors. Now: undistorts clicks, nadir metric grid while clicking (0.5 m blue / 1.0 m yellow), solved-transform grid afterwards for verification, sanity gate (won't save RMSE > 50 mm or height off > 0.25 m without confirmation). |
| `visionSystemConfig.m` | Auto-loads camera_calibration.mat; calibrated f/h supersede nominal constants (`focalLengthNominal` / `heightTape` keep the originals). |
| `clickTagTruth.m` | Clicks undistorted before ray intersection. |

## Plan to improve accuracy (target ≤ 2 cm over the field)

1. **Clean parallax round** (`estimateCameraHeight`): 6–8 chair/box hops with
   plumb-bob spot transfer (chair first, hang a string+nut from the tag
   centre to mark the floor point → kills re-placement error), radius
   400–650 px, seat height measured IN PLACE (±2 mm; error amplifies ~9.5×).
   Plus the 5 anchor base→tip click hops (dz = antenna height, tape it; zero
   re-placement error). Accept the save. Target: h ±3–5 cm and a trustworthy
   nadir/tilt. Expected h ∈ [3.6, 4.0].
2. **Stage A** (strings): ~6 taut lines spanning the frame (x, y, diagonals,
   some near edges), ≥6 clicks each. Target: corrected max bow < 2 px.
3. **Stage B** (tag sweep): 12+ placements covering the frame + ≥1 scale bar
   of 2–3 m tape. Target: corner reprojection < 1 px, scale-bar error
   < 0.5%. Cross-checks the printed tag size (0.1778 m nominal; the 0.1700
   "effective size" fudge was derived under the wrong height — revisit it).
4. **Stage C** (box hops; needs B): refined h/nadir/tilt + **f_est** (then
   supersedes 1050 automatically). Cross-check against step 1's h and the
   implied f ≈ 1020.
5. **Re-run `auditFloorScale`**: acceptance = every zone 0.99–1.01.
6. **Fix anchors.json** to the real layout (tape survey, or Phase-1.5 UWB
   self-survey: `SURVEY` serial command + `matlab/runSurvey.m` MDS), then
   **re-run `registerWorldFrameClicks`** (older registrations are
   inconsistent with the new undistortion/height). Then `clickTagTruth` /
   the GT campaign per `docs/gt_calibration_campaign.md`.

## Cautions

- Changing any calibration invalidates prior registrations and anything
  derived from the old height (nadir-fallback poses, the AprilTag
  effective-size fudge, old depth readings). Re-register after recalibrating.
- Distortion params are defined about `dist_center` with radii normalized by
  `norm_f` AS SAVED in the .mat — don't mix with parameters normalized
  differently.
- Claude's per-machine memory does not travel between PCs; this file and
  CLAUDE.md do.
