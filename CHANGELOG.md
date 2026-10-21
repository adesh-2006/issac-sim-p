# Changelog

All notable changes to this project will be documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [Unreleased]

### Added
- Initial project specification (`PROJECT_SPEC.md`)
- Repository structure: `vision/`, `control/`, `isaac/`, `tests/`, `docs/`
- Root `README.md` with quick-start and architecture overview
- `.gitignore` for Python, Isaac Sim, and IDE artifacts
- `CHANGELOG.md`

### Person 1 — Vision
- `vision/hand_tracking.py`: Full MediaPipe hand landmark pipeline
  - Webcam capture with configurable camera index
  - 21-landmark hand detection and visualization overlay
  - Palm-center computation (average of landmarks 0, 5, 9, 13, 17)
  - Normalized image coordinates (0.0–1.0, top-left origin)
  - Hand-size-relative pinch detection (thumb tip / index tip distance)
  - Hysteresis state machine for PINCH / OPEN / UNKNOWN
  - Exponential moving average (EMA) smoothing on hand center
  - Confidence proxy from MediaPipe presence score
  - UDP sender to 127.0.0.1:5005 (schema v1)
  - Graceful shutdown on webcam unavailable, ESC key, or KeyboardInterrupt
- `vision/requirements.txt`
- `vision/README.md`

### Person 3 — Control Bridge
- `control/teleoperation_bridge.py`: Full UDP bridge
  - Vision packet receiver on 127.0.0.1:5005
  - Packet validation (schema, types, ranges, NaN/Inf checks)
  - EMA smoothing on hand coordinates
  - Calibrate/recenter anchor without jump
  - Delta-based 2D mapping (hand_x → robot_x, hand_y → robot_z)
  - Dead-zone, per-update displacement cap, workspace bounds clamp
  - Normalized gripper command from gesture state
  - Robot command sender to 127.0.0.1:5006 (schema v1)
  - State machine: DISARMED / ACTIVE / NO_HAND / STALE_DATA / STOPPED
  - Keyboard controls: SPACE arm/disarm, C recalibrate, ESC stop+exit
- `control/config.yaml`
- `control/mapping.py`
- `control/protocol.py`
- `control/README.md`

### Person 2 — Isaac Sim
- `isaac/robot_controller.py`: Isaac Sim controller stub
  - UDP command receiver on 127.0.0.1:5006
  - Packet validation and workspace clamping
  - IK-based end-effector target execution (Franka Panda)
  - Normalized gripper mapping (0.0 closed → 1.0 open)
  - Hold / stop behavior
  - Offline demo mode with hard-coded targets
- `isaac/README.md`

### Tests
- `tests/fixtures/vision_valid.json`
- `tests/fixtures/vision_no_hand.json`
- `tests/fixtures/robot_command_valid.json`
- `tests/test_protocol.py`
- `tests/test_mapping.py`

### Docs
- `docs/setup.md`
- `docs/troubleshooting.md`
- `docs/demo_checklist.md`
