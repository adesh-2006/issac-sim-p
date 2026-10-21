# HandPilot — Project Specification

> **Version:** 1.0 MVP
> **Formal title:** Vision-Based Human Teleoperation of a Simulated Manipulator

---

## §1 — Objective

Build a real-time system that lets a person control a simulated Franka Panda robot arm in NVIDIA Isaac Sim using only webcam-captured hand gestures. The minimum viable product (MVP) demonstrates 2-DOF end-effector movement (horizontal X + vertical Z) and gripper open/close via a pinch gesture.

---

## §2 — System Architecture

```
Webcam → [Vision Process] --UDP:5005--> [Teleoperation Bridge] --UDP:5006--> [Isaac Sim Controller] → Robot Arm
```

The system is split into **three independent processes**, each maintained by a separate contributor:

| Process                | Owner    | Port      | Entry Point                        |
| ---------------------- | -------- | --------- | ---------------------------------- |
| Vision / Hand Tracking | Person 1 | sends :5005 | `vision/hand_tracking.py`        |
| Teleoperation Bridge   | Person 3 | :5005 → :5006 | `control/teleoperation_bridge.py` |
| Isaac Sim Controller   | Person 2 | receives :5006 | `isaac/robot_controller.py`     |

---

## §3 — Person 1: Vision Pipeline

### §3.1 — Hand Detection
- Use **MediaPipe Hands** (single-hand mode) for 21-landmark detection.
- Compute palm center as the average of landmarks 0, 5, 9, 13, 17.
- Normalise coordinates to `[0.0, 1.0]` image space (top-left origin).

### §3.2 — Pinch Detection
- Measure Euclidean distance between thumb tip (landmark 4) and index tip (landmark 8).
- Normalise by hand size (wrist-to-middle-finger-MCP distance) for distance-invariance.
- Use **hysteresis state machine** with PINCH and RELEASE thresholds:
  - `OPEN → PINCH` when ratio ≤ 0.3
  - `PINCH → OPEN` when ratio ≥ 0.5
  - `UNKNOWN` when hand is not detected.

### §3.3 — Smoothing
- Apply **Exponential Moving Average** (EMA, α = 0.4) on hand center coordinates.
- Confidence proxy from MediaPipe presence score.

### §3.4 — Output
- Send UDP JSON packets at webcam framerate to `127.0.0.1:5005`.

---

## §4 — Person 3: Teleoperation Bridge

### §4.1 — Packet Reception & Validation
- Listen on UDP `127.0.0.1:5005` for vision packets.
- Validate schema version, types, numeric ranges, and NaN/Inf.

### §4.2 — State Machine
The bridge operates in one of these states:

| State       | Description                                      |
| ----------- | ------------------------------------------------ |
| `DISARMED`  | Default. No commands sent. Press SPACE to arm.   |
| `ACTIVE`    | Tracking hand → sending robot commands.          |
| `NO_HAND`   | Tracking lost. Robot holds last safe target.     |
| `STALE_DATA`| No valid packet for > `stale_data_timeout_s`.    |
| `STOPPED`   | ESC pressed. Process exits gracefully.           |

### §4.3 — Calibration
- Press `C` to record current hand position and robot target as the anchor.
- Subsequent movements are relative to this anchor — prevents jump on recalibration.

---

## §5 — Coordinate Mapping

### §5.1 — Delta Mapping
1. Compute delta from calibration anchor in normalised image space.
2. Apply dead zone (default 0.02) with inward shift to avoid discontinuity.
3. Scale by per-axis gain (default 0.4 m per unit) to produce desired displacement in metres.
4. Invert Y axis: image-Y increases downward but robot-Z should increase upward.

### §5.2 — Safety Caps
- **Per-update displacement cap:** `max_delta_m = 0.02` metres per cycle.
- **Workspace bounds clamp:** Restrict target to the configured 3D bounding box.

### §5.3 — Gripper Mapping
- `PINCH` → gripper opening `0.0` (closed).
- `OPEN`  → gripper opening `1.0` (fully open).
- `UNKNOWN` → hold last value (no change).

---

## §6 — Person 2: Isaac Sim Controller

### §6.1 — Robot Setup
- Robot model: **Franka Panda** loaded from Isaac Sim asset library.
- Control mode: **Inverse Kinematics** (IK) on end-effector position target.
- Gripper: linear mapping of normalised `[0.0, 1.0]` to joint limits.

### §6.2 — Command Reception
- Listen on UDP `127.0.0.1:5006` for robot command packets.
- Validate independently (dual validation with bridge).

### §6.3 — Safety Behaviour
- On `enabled=false` or `stop=true`: hold current position, do not execute new targets.
- On stale data / no packets: hold last safe position (no extrapolation).
- Offline demo mode available with hard-coded targets for testing without vision.

---

## §7 — Communication Protocol

### §7.1 — Vision Packet (Port 5005)

```json
{
  "version":      1,
  "seq":          42,
  "timestamp_ms": 1698345678123,
  "tracking":     true,
  "confidence":   0.95,
  "gesture":      "OPEN",
  "hand": {
    "x": 0.52,
    "y": 0.48,
    "pinch_distance": 0.12
  }
}
```

| Field            | Type   | Range / Constraints                |
| ---------------- | ------ | ---------------------------------- |
| `version`        | int    | Must equal `1`                     |
| `seq`            | int    | Monotonically increasing           |
| `timestamp_ms`   | int    | Unix epoch milliseconds            |
| `tracking`       | bool   | `true` if hand detected            |
| `confidence`     | float  | `[0.0, 1.0]`, finite               |
| `gesture`        | string | `"PINCH"`, `"OPEN"`, or `"UNKNOWN"` |
| `hand`           | object | Present when `tracking=true`, else `null` |
| `hand.x`         | float  | `[0.0, 1.0]`, finite               |
| `hand.y`         | float  | `[0.0, 1.0]`, finite               |
| `hand.pinch_distance` | float | Normalised ratio, finite      |

### §7.2 — Robot Command Packet (Port 5006)

```json
{
  "version":         1,
  "seq":             42,
  "timestamp_ms":    1698345678200,
  "enabled":         true,
  "stop":            false,
  "mode":            "TARGET_XZ",
  "target_xyz_m":    [0.5, 0.0, 0.4],
  "gripper_opening": 1.0
}
```

| Field             | Type   | Range / Constraints                   |
| ----------------- | ------ | ------------------------------------- |
| `version`         | int    | Must equal `1`                        |
| `seq`             | int    | Monotonically increasing              |
| `timestamp_ms`    | int    | Unix epoch milliseconds               |
| `enabled`         | bool   | `false` → robot holds position        |
| `stop`            | bool   | `true` → emergency hold               |
| `mode`            | string | `"TARGET_XZ"` (MVP only mode)         |
| `target_xyz_m`    | list   | 3 finite floats `[x, y, z]` in metres |
| `gripper_opening` | float  | `[0.0, 1.0]`, `0.0` = closed          |

---

## §8 — Configuration

All tunable parameters are centralised in `control/config.yaml`. Key groups:

- **network** — hosts, ports, timeouts
- **smoothing** — EMA alpha
- **mapping** — axis gains, dead zone, max delta
- **workspace** — 3D bounds in metres
- **robot_home** — default anchor position
- **gesture** — minimum confidence threshold
- **keys** — keyboard bindings

---

## §9 — Safety Principles

1. **Motion starts disabled.** Nothing moves until the user deliberately arms.
2. **Bounded Cartesian targets.** Raw image coordinates are never sent directly to joints.
3. **Independent stop control.** `ESC` always stops — no gesture required.
4. **Hold on tracking loss.** Robot holds last safe target; does not extrapolate.
5. **Dual validation.** Bridge validates commands; Isaac Sim controller validates again.

> ⚠️ **This MVP is for simulation only.** Never connect to physical hardware without a separate safety review, hardware-rated emergency stop, and a different deployment procedure.

---

## §10 — Testing

- **`tests/test_protocol.py`** — validates parsing, validation, and round-trip of both packet schemas.
- **`tests/test_mapping.py`** — validates coordinate mapping, dead zone, displacement cap, workspace clamping, calibration, and gripper mapping.
- **Fixtures:** `tests/fixtures/` contains sample JSON payloads for both packet types.
