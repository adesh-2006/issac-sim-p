# Setup Guide

## Prerequisites

| Requirement | Notes |
|---|---|
| Python 3.10 or 3.11 | Recommended; check `python --version` |
| NVIDIA Isaac Sim | Version confirmed by Person 2 |
| Laptop or USB webcam | RGB only — no depth sensor required |
| Git | For version control |

---

## 1. Clone the Repository

```powershell
git clone https://github.com/<OWNER>/isaac-hand-teleoperation.git
cd isaac-hand-teleoperation
```

---

## 2. Vision Environment (Person 1)

```powershell
# Create a dedicated environment (recommended)
python -m venv vision/.venv
vision\.venv\Scripts\activate

# Install dependencies
cd vision
pip install -r requirements.txt
```

Test the webcam:

```powershell
python hand_tracking.py --camera 0
```

If the wrong camera opens, try `--camera 1` or `--camera 2`.

---

## 3. Control Bridge Environment (Person 3)

The bridge uses only the Python standard library plus optional PyYAML:

```powershell
pip install pyyaml
```

Run from the `control/` directory:

```powershell
cd control
python teleoperation_bridge.py
```

---

## 4. Isaac Sim Controller (Person 2)

Follow `isaac/README.md` carefully. Key steps:

1. Launch Isaac Sim.
2. Open the saved scene (or create it following the README).
3. Open the Script Editor in Isaac Sim.
4. Run `robot_controller.py` inside Isaac Sim's Python environment.

For offline testing without Isaac Sim:

```powershell
cd isaac
python robot_controller.py --offline
```

---

## 5. Run the Tests

```powershell
# From the repository root
pip install pytest
python -m pytest tests/ -v
```

---

## 6. Launch Order for Full Demo

1. **Start Isaac Sim controller** → wait for "Controller ready" message.
2. **Start Teleoperation Bridge** → confirm "Listening on :5005" message.
3. **Start Vision process** → confirm preview window opens.
4. Keep teleoperation **DISARMED** while checking landmark tracking.
5. Press **C** to calibrate with hand in neutral position.
6. Press **SPACE** to arm.
7. Move hand slowly within the configured workspace.
8. Press **ESC** to stop.

---

## Environment Variables (optional)

Copy `.env.example` to `.env` and edit if needed:

```
VISION_PORT=5005
ROBOT_PORT=5006
CAMERA_INDEX=0
```

> ⚠️ Never commit a real `.env` file — it is in `.gitignore`.
