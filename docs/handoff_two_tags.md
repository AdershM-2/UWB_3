# Handoff prompt — Step 7.3: second tag (yaw + rigid-body constraint)

_Paste the block below into a new chat to continue the DUNE UWB project. Written
2026-07-21 at the end of the estimator/MHE session._

---

Continue the DUNE UWB project (d:\UWB_3). We are starting **step 7.3: add the second
tag**. Read first: docs/rebuild_plan.md (step 7.3 and the PARKED banner). Project memory
has the full history.

**Where we are.** Single-tag pipeline is solid: live_tag over COM at 5–7 Hz, EKF display,
delay + power calibration done, parked wobble characterised (per-link RF physics) and
masked at the output (pin + EMA smoother). This session added an experimental **Moving
Horizon Estimator** (dune.MheEstimator, CasADi 3.7.0 + IPOPT at
C:\Users\itisa\Downloads\casadi-3.7.0) selectable via live_tag(estimator="mhe"): ~40%
smoother in motion than the EKF, guarded, opt-in. It has two motion models:
`model="cv"` (holonomic) and `model="unicycle"` (NON-HOLONOMIC car: gyro yaw-rate drives
a heading state, velocity forced along heading, no sideways slip). The tag is on a small
**front-steering RC car**, so unicycle is the physically-correct model. Wobble
investigation and broadcast-POLL are PARKED (documented, resumable). 7.4 Kinect accuracy
validation is LATER.

**Goal of this step.** Both tags reporting to the host at once → (1) **yaw from the rigid
baseline** (atan2 of tag1→tag2) cross-checked against the BNO085 IMU yaw — redundant with
the IMU but it validates the IMU frame (we found the IMU's absolute orientation frame is
rotated); (2) use the **fixed inter-tag distance as a solver constraint** to improve
position — the two tags are on a rigid plate a known distance apart.

**Hard constraint — transport.** The two tags CANNOT both be on USB. So host ingest must
come over **WiFi/UDP**, not serial. Facts (VERIFIED 2026-07-21 in the code):
- The firmware is ALREADY built with UDP: TagWrover.ino has `#define UWB_HOSTLINK_UDP`
  active, so it connects to WiFi and UDP-broadcasts every `RTLS,v4,...` line to
  `255.255.255.255:4100`, AND mirrors the same line to serial (HostLink.h sendSweep). So
  **no firmware change is needed to START sending UDP — it is already broadcasting.**
- What the HOST does today: live_tag reads SERIAL only, via dune.TagSerial (MATLAB
  serialport → COM). There is NO UDP receiver on the host side. That is why WiFi shows in
  the boot banner but everything runs over USB.
- So the two missing pieces are HOST-SIDE ONLY:
  1. A MATLAB UDP receiver (`udpport("datagram","IPV4")` bound to port 4100) that ingests
     both tags' lines; demux by `tag_id` (dune.parseRtlsLine / solveSweep / sweeps are
     already tag-agnostic — just route by tag_id). This replaces/supplements TagSerial.
  2. The network must actually DELIVER the broadcast. UDP was historically BLOCKED on the
     `iitk` WiFi (client-to-client / broadcast dropped) — the gating risk. Resolve via a
     phone hotspot / dedicated AP (verify a single packet arrives first). Consider setting
     HOST_IP to the host's real IP instead of 255.255.255.255 (unicast is often allowed
     when broadcast is blocked) — that IS a small firmware edit (propose it, then flash).
- Firmware rule stands: no reflash without showing the change first; keep libraries/UwbRtls
  synced to C:\Users\itisa\OneDrive\Documents\Arduino\libraries\UwbRtls. Tag 240 has a
  Phase-C read fix staged but unflashed (commit ac5743c — SAR temp/Vbat + CFO); fold it in
  if you reflash 240 for the HOST_IP change anyway. Command send TO the tags (delay push
  etc.) currently goes over serial; over UDP it would need a host→tag path (the firmware
  has a UDP command listener? verify — else keep one tag on serial for commands).

**The rigid-body idea (the real prize).** The two tag antennas are a fixed distance L
apart on the plate. Instead of solving each tag independently and just reading yaw off the
pair, solve the **rigid body pose** directly: state = [cx, cy, psi (yaw), speed], with
  tag1 = centre + (L/2)[cos psi, sin psi],  tag2 = centre − (L/2)[cos psi, sin psi].
Fit ALL ranges from BOTH tags to (cx, cy, psi) in one solve → position AND yaw, with the
baseline exact and outliers outvoted. This drops straight onto the MHE unicycle model
(state already has heading): extend MheEstimator to a rigid-body variant where the gyro
drives psi, the non-holonomic no-slip ties the centre velocity to psi, and both tags'
ranges feed the window. Re-measure L precisely first (physical ~0.50 m, an old fit gave
0.53 m; it is now a hard constraint, so measure antenna-centre to antenna-centre).

**Tasks, in order.**
1. Transport: get a UDP packet from ONE tag to the MATLAB host reliably (resolve the iitk
   block — hotspot/AP). Propose the firmware/build change for UDP, get approval, flash.
2. Dual-tag UDP ingest in the host: receive both streams, demux by tag_id, feed the
   existing sweep path. Extend live_tag to show both tags (two dots + the baseline).
3. Yaw: compute baseline yaw (tag1→tag2) and overlay BNO085 IMU yaw; report the offset /
   agreement (this quantifies the IMU frame rotation we suspect).
4. Rigid-body solver: add the fixed-distance constraint. Start with a soft joint solve
   (‖p1−p2‖=L penalty), then the full rigid-body MHE (state [cx cy psi speed], both tags'
   ranges, gyro→psi, unicycle no-slip). Compare vs independent solving.
5. Leave 7.4 (Kinect moving truth) for later — that is the overall-accuracy comparison of
   EKF vs MHE(cv/unicycle) vs rigid-body.

**Facts.** Tag 240 = 0xF0 = IMU (was COM12); tag 241 = 0xF1 = no IMU (was COM11); on a
rigid plate, AprilTag on top. Baseline ~0.50 m (re-measure). MATLAB R2024b at
"C:\Program Files\MATLAB\R2024b\bin\matlab.exe" (run via -batch; addpath D:\UWB_3\matlab
and matlab\scripts). CasADi at C:\Users\itisa\Downloads\casadi-3.7.0. RC car =
front-steering (non-holonomic). Firmware rule + library sync as above. Commit at each
milestone. Keep answers short; ask questions as popups.
