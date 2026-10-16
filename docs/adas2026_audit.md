# ADAS_2026 (2025 lane-keeping stack): audit

**Scope.** `ADAS_2026/` (RatheeswarS/ADAS_2026, 3 commits, last 2026-10-16) contains about 6 k lines of team
Python (`src/`, runners, bench tools), vendored YOLOP and `arduinocode.ino`. It was reviewed on 2026-10-16 by:
- three independent code reviews (perception, control/safety, runners/actuator/firmware);
- a synthetic-camera test that runs the team's own functions (`tools/adas2026/verify_signs.py`);
- a closed-loop MATLAB replica of the control chain (`scripts/verify_adas2025_control.m`).

The folders `Team-Asterix---Software-Submission/` and `Team-Asterix-AEB/` are empty.

**Pipeline as built.** ZED left eye → YOLOP lane-line segmentation (BDD100K weights, TensorRT/ONNX)
→ fixed hand-tuned bird's-eye trapezoid → blob selection + quadratic fit in pixels → CTE/heading
→ 4-state Kalman `[cte, cte_rate, heading, heading_rate]` → Stanley (K 0.8) → slew 60°/s → 1 s mean
→ Arduino `STA<deg>` (CONFIG mode).

**Verdict.** The perception and robustness ideas are good and were kept (see the end of this document). Four
things stop the 2025 chain from keeping a lane, even with a perfect lane measurement:
- the Stanley heading term has the wrong sign;
- the 1 s mean adds about 1 s of delay;
- the bird's-eye view is not metric;
- the lane width is wrong.

The hardware path also has three safety problems (C3–C5).

## Findings

Severity: **C** critical (wrong steering or unsafe hardware behaviour), **H** high, **M** medium, **L** low/docs.

| ID | Where | Finding | Evidence | Fix (Jetson phase) |
|---|---|---|---|---|
| C1 | `steering_v2.py:81` | Heading term has the wrong sign: a car pointing left of the lane is steered further left (unstable heading loop). | Synthetic test: car yawed 3° left → command −0.77° (right-positive convention), i.e. a left turn. Replica: the car leaves the road after 5.5 s (table below). | `np.radians(-heading_deg)` |
| C2 | `steering_v2.py:93` | Yaw-damping sign is flipped as well (latent: only with an IMU). Must be fixed together with C1. | Derivation in review; `lka_adas2025_steer` with `UseImu`. | `+ k_yaw_damp * yaw_rate_dps` |
| C3 | `sda_serial_backend.py:106`, `run_video_to_arduino.py:97`, `run_video_annotated*.py --port`, `bench_steering_to_arduino.py:70` | The `SDA <v>` protocol is not implemented in `arduinocode.ino`; the line falls into the `S<kmh>` branch (`:889-895`) and starts the throttle PID. | Firmware parser. | Retire SDA; parse `S<digits>` strictly and NAK the rest. |
| C4 | `arduinocode.ino:687-696` | No steering watchdog in CONFIG mode: on link loss the wheels hold the last angle. | Code path `if (sysState != STATE_RUN) return;` | Ramp the steering target to centre on timeout in every state. |
| C5 | `arduinocode.ino:692` | `setBrakePercent(100)` is re-issued every loop while tripped and resets the actuator stall timer, so `ACT_MOVE_TIMEOUT_MS` never fires (motor at full PWM). | Code path `:355-356, :406`. | Latch on the transition only. |
| H1 | `ipm.py` | Uncalibrated bird's-eye: heading is not metric, lateral offset leaks into heading, curvature is invisible at the read row. On the simulator camera (pitch 0, horizon at row 360) the trapezoid top (row 302) lies above the horizon. | Synthetic test: yaw 3° → 0.72° (×0.24); 0.5 m offset → 3.97° heading; R127.5 curve → 0.02°. | Calibrate the ZED (intrinsics + mount) and use a metric IPM, as in `lka_camera_model.m`. |
| H2 | `lane_detection_node_birdeye.py:108` (+ runners) | `lane_width_m = 3.0`; the track lanes are 3.5 m, so CTE is 14 % low and the jump gate is off. | Code. | `--lane-width 3.5` |
| H3 | `arduino_steering_link.py:220, :224` | The 1 s mean adds about 1 s of delay (≈ 8 m at 30 km/h): **with the sign fixed but the mean kept, the car still leaves the road** (replica). The lane-loss ramp is a 20° step per window, not 20°/s. | Replica table. | Per-frame command after C1; ramp with `rate * dt`. |
| H4 | `kalman_tracker.py:114`, `safety_monitor.py:72` | IMU sign errors (latent): the dropout prediction turns the wrong way; the yaw plausibility check trips for any steer above about 1.2°. | Review derivation. | Use one gyro sign convention (+ left) everywhere. |
| H5 | `bench_steering_to_arduino.py:72-76` | `STR/STL` are relative in the firmware (`:864-873`); the bench tool integrates commands towards the stop. | Firmware parser. | Send absolute `STA<deg>`. |
| H6 | `conditioner.py:54`, `safety_monitor.py:90`, `steering_v2.py:94` | NaN is never rejected and reaches the Kalman filter and the Arduino. | Review. | `math.isfinite` guards. |
| M1 | `lane_detection_node_birdeye.py:496` | A reconstructed (synthetic) side is committed as the continuity reference, so the real line is never re-acquired while the other side persists. | Review. | Commit direct fits only (done in `lka_vision_lane_step.m`). |
| M2 | `lane_detection_node_birdeye.py:472` | The per-side 1.8 s coast is unreachable (reconstruction runs first). | Review. | Coast before reconstructing, or update the docs. |
| M3 | `bird_eye_extraction.py:103` | The continuity search has no distance gate (last tier accepts any blob, even on the other side). | Review. | Windowed search (done in `lka_search_boundary.m`). |
| M4 | `kalman_tracker.py:54, :58` | Stale is a frame count (15) while the hold is time-based; `MAX_PREDICT_DT` silently truncates long gaps. | Review. | Time-based stale. |
| M5 | `lane_detection_node_birdeye.py:405`, `ego_state_estimator.py:54` | Jump gate and Stanley use the 5 m/s fallback speed, not the measured speed. | Review. | Use `$TEL` speed; default 8.33 m/s. |
| M6 | `lane_detection_node_birdeye.py:486` | Pair width and centre are evaluated at different rows when one fit ends higher. | Review. | One shared row. |
| M7 | `lane_detection_node_birdeye.py:450` | The width calibrator also learns from latched shoulders (both-historied pairs). | Review. | Gate `observe` with `_pair_width_ok`. |
| M8 | `conditioner.py:62` | Fixed 3 m/s CTE-rate limit rejects genuine corrections above ~21° heading at 30 km/h. | Review. | Scale with speed. |
| M9 | firmware `:160-161` vs `steering_v2.py:59` | ±24° firmware lock vs ±30° Python clamp. | Code. | Measure the road-wheel lock and use one value. |
| M10 | `steering_v2.py:108` | Slew dt uncapped: after a stall the limiter allows a large jump. | Review. | Cap dt at 0.1 s. |
| M11 | `run_video_annotated - v2.py:416, :906` | v2 drops the TensorRT default; stereo recovery overwrites the tracked state after the Kalman filter. | Review. | Restore `default_weights()`; feed recovery into the tracker. |
| L1 | `README.md`, `BRIDGE.md`, `BENCH.md`, `LKA_README.md` | Three different wire protocols are documented (SDA / STR / STA); only STA matches the firmware. `reference_generator.py:27` documents the CTE sign backwards. | Docs vs code. | One protocol section. |
| L2 | `tests/test_lane_hold_gate.py` | Imports torch through the node module; covers 3 helpers, not the coast/decay it claims. | Review. | Lazy import, add coast tests. |

### Synthetic camera test (team code, `tools/adas2026/verify_signs.py`)

Camera 1.2 m high, 10° pitch, 1280×720, f = 1109 px. The signs are valid for any camera; the scales
depend on this camera, not on the ZED mount. The command convention is + = right.

| Case | cte [m] | heading [deg] | Stanley [deg] | Correct steer |
|---|---|---|---|---|
| centred, straight | −0.003 | 0.02 | 0.00 | none |
| car 0.5 m left of centre | 0.117 | 3.97 | 4.55 | right (+) ✓ |
| car 0.5 m right of centre | −0.129 | −3.99 | −4.63 | left (−) ✓ |
| car yawed 3° left | −0.012 | −0.72 | **−0.77** | right (+) ✗ |
| car yawed 3° right | 0.003 | 0.72 | **0.73** | left (−) ✗ |
| left curve R127.5 | −0.006 | 0.02 | −0.01 | left (−) (no curvature seen) |
| right curve R127.5 | −0.002 | −0.04 | −0.05 | right (+) (no curvature seen) |

The offset cases only steer the right way because the uncalibrated warp turns offset into a fake
heading. A real heading error is steered the wrong way.

### Closed-loop replica (official track, Baja plant, ideal metric measurement 4 m ahead)

| Variant | Max lateral offset | In lane | First line touch | All 4 wheels out (DQ) |
|---|---|---|---|---|
| as coded (C1 + 1 s mean + 3.0 m + 5 m/s) | 5.12 m (left the road) | 61.6 % | 5.54 s | yes |
| as coded without the 1 s mean | 5.08 m (left the road) | 64.9 % | 4.81 s | yes |
| heading sign fixed, 1 s mean kept | 5.03 m (left the road) | 88.2 % | 6.28 s | yes |
| fixed (sign, per-frame, 3.5 m, measured speed) | **0.25 m** | **100 %** | never | no |

Figure: `results/adas2025/adas2025_replica_lateral_offset.png`. With C1, H2, H3 and M5 fixed, the 2025 Stanley law
keeps the lane on the official track (max 0.25 m). Pure pursuit in this repo reaches 0.10 m on the same scenario.

### Bug found in this repository while building the replica

`lka_compute_kpis.m` flagged "all 4 wheels out" with `all(margins < 0)`. Each margin is measured against the
tyre's own-side line, so a car that crossed a line completely was never flagged as DQ. It is fixed by
`src/evaluation/lka_wheels_out.m`, which checks both lines, with unit tests in `test_lka_core.m`.

## What was kept and where it lives now

| ADAS_2026 idea | MATLAB implementation |
|---|---|
| YOLOP lane head, letterbox, ImageNet normalisation on BGR | `lka_detector_onnx_seg.m` + `models/perception/yolop.json` |
| Bird's-eye warp | `lka_camera_model.m` (calibrated `monoCamera` / `birdsEyeView`, metric) |
| Continuity-aware selection, lateral jump gate | warm prior window in `lka_search_boundary.m` |
| Quality tiers, partial fits | boundary `Strength` from coverage and probability (`lka_fit_boundary.m`) |
| Width consistency gate, width calibrator | metric lane-width estimate and pair check in `lka_vision_lane_step.m` |
| Missing-side reconstruction, hold/coast, confidence decay | existing `lka_estimate_lane_center.m`, Kalman coast, supervisor DEGRADED/TAKEOVER |
| Kalman `[cte, rate, heading, rate]` | existing `lka_lane_tracker_step.m` (vehicle-motion model) |
| Stanley K 0.8 chain | `lka_adas2025_steer.m` (replica, verification only) |
| Annotated HUD video | `LkaVisionRecorder.m` |

## Jetson-phase fix queue (in this order)

1. C1 + C2 in one commit. Bench-check: `heading=+5, cte=0` must give a negative command; `cte=+0.5, heading=0` must give a positive one.
2. C3–C5 in the firmware (strict parser, steering watchdog, brake latch), then a wheels-up bench test.
3. H1: calibrate the ZED and use a metric IPM (port the `lka_camera_model.m` logic).
4. H2, H3, M5: lane width 3.5, per-frame command, measured speed.
5. H4, H6: gyro sign, NaN guards.
6. M1–M4, M6–M11, L1–L2.
