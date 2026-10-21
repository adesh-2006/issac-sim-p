# HandPilot — Vision-Based Robot Teleoperation in Isaac Sim

> **Demo name:** HandPilot | **Formal title:** Vision-Based Human Teleoperation of a Simulated Manipulator

Control a simulated Franka Panda robot arm in NVIDIA Isaac Sim using only your laptop webcam and hand gestures.

---

## System Overview

```
Webcam → [Vision Process] --UDP:5005--> [Teleoperation Bridge] --UDP:5006--> [Isaac Sim Controller] → Robot Arm
```

| Process | Owner | Port | Script |
|---|---|---|---|
| Vision / Hand Tracking | Person 1 | sends :5005 | `vision/hand_tracking.py` |
| Teleoperation Bridge | Person 3 | :5005 → :5006 | `control/teleoperation_bridge.py` |
| Isaac Sim Controller | Person 2 | receives :5006 | `isaac/robot_controller.py` |

---

## Quick Start

### 1. Prerequisites

- Python 3.10+
- NVIDIA Isaac Sim (see `isaac/README.md` for version requirements)
- Laptop or USB RGB webcam

### 2. Vision Environment (Person 1)

```powershell
cd vision
pip install -r requirements.txt
python hand_tracking.py
```

### 3. Teleoperation Bridge (Person 3)

```powershell
cd control
pip install pyyaml   # only extra dep if using config.yaml
python teleoperation_bridge.py
```

### 4. Isaac Sim Controller (Person 2)

Follow `isaac/README.md` — the controller runs inside Isaac Sim's Python environment.

---

## Control Keys (defaults)

| Key | Action |
|-----|--------|
| `SPACE` | Arm / Disarm teleoperation |
| `C` | Calibrate / Recenter hand anchor |
| `ESC` | Stop and exit |

---

## Gestures

| Gesture | Robot Action |
|---------|-------------|
| Move hand left/right | End-effector moves horizontally |
| Move hand up/down | End-effector moves vertically |
| Thumb–index pinch | Gripper **closes** |
| Open hand (release pinch) | Gripper **opens** |

---

## Repository Layout

```
isaac-hand-teleoperation/
├── README.md
├── PROJECT_SPEC.md
├── CHANGELOG.md
├── .gitignore
├── vision/          # Person 1 — Computer Vision
├── control/         # Person 3 — Teleoperation Bridge
├── isaac/           # Person 2 — Isaac Sim & Robotics
├── tests/           # Shared protocol & mapping tests
└── docs/            # Setup, troubleshooting, demo checklist
```

---

## Safety Principles

- **Motion starts disabled.** Nothing moves until you deliberately arm.
- **Bounded Cartesian targets.** Raw image coordinates are never sent directly to joints.
- **Independent stop control.** `ESC` always stops — no gesture required.
- **Hold on tracking loss.** Robot holds last safe target; does not extrapolate.
- **Dual validation.** Bridge validates commands; Isaac Sim controller validates again.

> ⚠️ **Never connect this MVP to physical hardware.** Hardware use requires a separate safety review, hardware-rated emergency stop, and a different deployment procedure.

---

## Links

- [Project Specification](PROJECT_SPEC.md)
- [Vision README](vision/README.md)
- [Control README](control/README.md)
- [Isaac Sim README](isaac/README.md)
- [Setup Guide](docs/setup.md)
- [Demo Checklist](docs/demo_checklist.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Changelog](CHANGELOG.md)
