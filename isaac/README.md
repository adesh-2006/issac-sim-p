# Isaac Sim Controller — Person 2

## Overview

This module runs **inside** NVIDIA Isaac Sim's Python environment. It receives
robot command packets from the Teleoperation Bridge and moves the Franka Panda
arm using inverse kinematics.

---

## ⚠️ Required Setup Before Running

Person 2 must complete the following before the controller is functional:

- [ ] **Confirm Isaac Sim version** (recommended ≥ 4.x or 2023.x)
- [ ] **Load or create the Franka Panda scene** and save the USD file to `scene/`
- [ ] **Update constants** in `robot_controller.py`:
  - `ROBOT_PRIM_PATH` — prim path of the robot in the USD stage
  - `EE_LINK_NAME` — end-effector link name
  - `GRIPPER_JOINT_NAMES` — finger joint names
  - `WS_X_MIN/MAX`, `WS_Z_MIN/MAX` — verified reachable workspace bounds
- [ ] **Choose and implement IK/motion method** (Lula, RMPflow, ArticulationMotionPolicy — depends on installed Isaac Sim version)
- [ ] **Document the world frame** (coordinate axes, units = metres, origin)
- [ ] **Test offline demo mode** before connecting to live data

---

## Franka Panda — Reference

| Parameter | Expected value |
|---|---|
| URDF prim path | `/World/Franka` (typical) |
| End-effector link | `panda_hand` |
| Finger joints | `panda_finger_joint1`, `panda_finger_joint2` |
| Finger range | 0.0 m (closed) → 0.04 m (open) per joint |
| DOF count | 9 (7 arm + 2 fingers) |

> Verify these against your actual loaded asset — prim paths may vary by scene.

---

## Running the Controller

### Inside Isaac Sim (normal mode)

Open Isaac Sim → Script Editor → run:

```python
exec(open("d:/issasimp/isaac-hand-teleoperation/isaac/robot_controller.py").read())
run_controller()
```

Or attach to Isaac Sim's Python process and import:

```python
from isaac.robot_controller import run_controller
run_controller()
```

### Standalone offline demo (no Isaac Sim required)

Test the UDP receiver, packet validation, and control loop without Isaac Sim:

```powershell
python robot_controller.py --offline
```

This sends hard-coded targets through the control path and logs what would happen.

---

## Offline Testing with Synthetic Commands

Send fake robot command packets to verify the receiver:

```powershell
python -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cmd = {'version':1,'seq':1,'timestamp_ms':int(time.time()*1000),
       'enabled':True,'stop':False,'mode':'TARGET_XZ',
       'target_xyz_m':[0.5,0.0,0.4],'gripper_opening':1.0}
s.sendto(json.dumps(cmd).encode(), ('127.0.0.1', 5006))
print('Sent:', cmd)
"
```

---

## Safety Checklist

Before enabling live teleoperation:

- [ ] Robot loads and simulates reliably at rest.
- [ ] Arm reaches 3+ manually specified safe target positions.
- [ ] Gripper opens/closes with `gripper_opening = 0.0` and `1.0`.
- [ ] Out-of-workspace targets are clamped, not rejected with an error that halts.
- [ ] Malformed / missing packets do not cause uncontrolled motion.
- [ ] `stop = true` in any command immediately holds and disarms.
- [ ] Stale command detection is working (> 1 s without update → hold).
- [ ] Motion starts **disarmed** — `enabled = false` on startup.

---

## World Frame Documentation

*(Person 2: fill this in after confirming the scene)*

| Axis | Direction | Notes |
|------|-----------|-------|
| +X | | |
| +Y | | |
| +Z | | |
| Units | metres | |
| Origin | | |
| EE frame | | |
| Reachable X | 0.2 – 0.8 m | *update after testing* |
| Reachable Z | 0.1 – 0.7 m | *update after testing* |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ImportError: omni` | Script must run inside Isaac Sim's Python environment |
| Arm jumps on first command | Check calibration on bridge side; confirm workspace anchor |
| IK fails for many targets | Reduce workspace bounds; check EE link name |
| Gripper does not move | Verify `GRIPPER_JOINT_NAMES` match actual USD prim names |
| UDP commands not received | Check firewall; confirm bridge sends to port 5006 |
