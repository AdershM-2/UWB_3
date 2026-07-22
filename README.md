# DUNE — Indoor Ultra-Wideband Positioning

Centimetre-scale indoor positioning for a small rover, using ultra-wideband (UWB)
radio ranging, with a ceiling camera providing independent ground truth.

Five fixed radio **anchors** sit around the room. One or two **tags** ride on the
rover and measure their distance to each anchor several times a second. A MATLAB
host turns those distances into a position — and, with two tags, a full pose
(position **and** heading). A ceiling Kinect watching an AprilTag on the rover gives
a trusted reference for checking the answers.

📄 **[Full technical report](docs/report/dune_report.pdf)** — how everything works, why
it was built this way, and a candid section on what was tried and failed.

---

## What works today

| | |
|---|---|
| Static accuracy (single reading, parked) | **94 mm** RMSE over 19 positions |
| Static accuracy (time-filtered) | **60–65 mm** |
| Update rate | 5–7 Hz single tag, ~3 Hz per tag with two |
| Heading (two tags) | fused from UWB geometry + gyroscope turn rate |
| Transport | USB serial **or** WiFi/UDP |

**Known limitation.** A parked tag drifts slowly by 5–7 cm over seconds to minutes.
This is understood, not mysterious: the antennas sit 24 cm above the floor, so the
direct signal and its floor reflection blend together and the apparent arrival time
wanders. Captured radio snapshots confirm the first path is ~20 dB below the strongest
reflection. It is a physical-mounting problem; the fix is to raise the antennas.
The live display suppresses this drift when the rover is known to be stopped, which is
cosmetic and clearly marked as such — the raw estimate is always what gets logged.

## Hardware

- **5 anchors** — DecaWave DW1000 + ESP32, mounted at 24 cm height
- **2 tags** — one with a BNO085 IMU (`0xF0` = 240), one without (`0xF1` = 241),
  52.7 cm apart on a rigid plate
- **Kinect v2** on the ceiling, calibrated (height 4.066 m, focal 1067.6 px,
  world registration RMSE 13.5 mm)
- Radio config: channel 5, 110 kb/s, PRF 64, long preamble

## Requirements

- **MATLAB R2024b** (no Instrument Control Toolbox needed — UDP uses Java sockets)
- **[CasADi 3.7.0](https://web.casadi.org/)** — only for the moving-horizon estimators.
  Download separately and pass its path, e.g. `casadiPath="C:\path\to\casadi-3.7.0"`.
- **Simulink 3D Animation** — only for `rover_teleop_uwb` (PS4 controller via `vrjoystick`)
- Arduino IDE + the vendored `libraries/UwbRtls` for firmware

## Quick start

```matlab
addpath('matlab', 'matlab/scripts')

% --- single tag over USB ---
live_tag("COM12")

% --- single tag over WiFi (tags already broadcast; check it arrives first) ---
udp_probe(15)
live_tag(transport="udp")

% --- experimental moving-horizon estimator (needs CasADi) ---
live_tag("COM12", estimator="mhe", model="unicycle")

% --- both tags, rigid rover pose over WiFi ---
live_rover(baseline=0.527)

% --- validate the rigid estimator with no hardware at all ---
test_mhe_rigid
```

Driving the rover and capturing UWB + camera truth together, then scoring it:

```matlab
rover_teleop_uwb      % PS4 drive; logs to matlab/results/rover_runs/
rover_run_report      % offline: estimator vs AprilTag truth, with diagnostics
```

## How it works, briefly

1. **Ranging.** Double-sided two-way ranging cancels the unknown reply delay and most
   clock-rate error between the two devices.
2. **Calibration.** Each board has an antenna delay constant (1 tick = 4.69 mm), tuned
   against camera-measured truth. A second correction removes a signal-strength bias —
   strong signals read long, at 7.7–9.5 mm per dB.
3. **Positioning.** Weighted least squares finds the point that disagrees least with all
   the distances, with outlier rejection and a guard that refuses to answer when the
   answering anchors are nearly collinear (the geometry is genuinely ambiguous).
4. **Filtering.** A Kalman filter adds time: zero-velocity updates when stopped,
   per-anchor bias memory so dropouts don't cause jumps, and robust updates that
   down-weight surprising readings rather than discarding them.
5. **Two tags.** The rover pose `[x, y, heading, speed]` is estimated directly, with both
   tag positions *derived* from it. The 52.7 cm separation is therefore exact by
   construction rather than a penalty, and "both stopped or both moving" is automatic.
   Being a front-steering car, sideways motion is not representable either.

## Repository layout

```
matlab/+dune/      core library (ranging parse, solver, EKF, MHE, transports)
matlab/scripts/    runnable tools (live apps, calibration, analysis)
matlab/vision/     Kinect calibration and AprilTag/world registration
matlab/config/     anchor layout, range corrections (JSON)
libraries/UwbRtls/ vendored firmware library (keep in sync with the Arduino copy)
sketches/          anchor and tag firmware
docs/              plan, findings, and the full report
```

Key classes: `dune.solveSweep` (ranges → position), `dune.FusionEkf`,
`dune.MheEstimator` (single tag), `dune.MheRigid` (two-tag rigid pose),
`dune.TagSerial` / `dune.TagUdp` (transports).

## Status

The system is built end to end and has run on hardware. Current work is testing and
tuning rather than new components — see [`docs/rebuild_plan.md`](docs/rebuild_plan.md)
for the live plan and [`docs/phase_a_findings.md`](docs/phase_a_findings.md) for the
drift investigation.

> Note: `paper/` contains an early manuscript that predates the MATLAB rewrite and does
> not describe the current system. Treat `docs/report/` as the accurate account.

## License

MIT — see [LICENSE](LICENSE).
