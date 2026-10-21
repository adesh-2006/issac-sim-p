# Control Module — Person 3

## Overview

The teleoperation bridge sits between Person 1's vision process and Person 2's
Isaac Sim controller. It:

1. Receives vision packets on UDP `127.0.0.1:5005`
2. Validates, smooths, and maps hand position → robot XYZ target
3. Sends robot command packets to UDP `127.0.0.1:5006`

---

## Setup

### Requirements

- Python 3.10+
- PyYAML (for config.yaml loading)

```powershell
pip install pyyaml
```

No other external libraries are needed — the standard library covers sockets and JSON.

### Run

```powershell
cd control
python teleoperation_bridge.py
```

```powershell
# Optional: custom config file path
python teleoperation_bridge.py --config config.yaml
```

---

## Control Keys

| Key | Action |
|-----|--------|
| `SPACE` | Arm / Disarm teleoperation |
| `C` | Calibrate / Recenter hand anchor |
| `ESC` | Stop and exit immediately |

---

## State Machine

```
        ┌────────────────────────────┐
 Start  │          DISARMED          │ ← default startup state
        └──────────────┬─────────────┘
                   SPACE key
                       ▼
        ┌────────────────────────────┐
        │          ACTIVE            │ ← accepting hand movement commands
        └──────┬──────────────┬──────┘
         hand         lost     SPACE key / stale
         lost          ▼           ▼
          │      ┌──────────┐ ┌───────────┐
          │      │ NO_HAND  │ │STALE_DATA │
          │      └────┬─────┘ └─────┬─────┘
          │       hand back    fresh data
          └──────────►◄────────────┘
                       │
                   ESC key
                       ▼
        ┌────────────────────────────┐
        │          STOPPED           │ ← requires restart to re-arm
        └────────────────────────────┘
```

- **DISARMED**: startup default; no new movement targets are applied.
- **ACTIVE**: valid, fresh tracking drives the robot target.
- **NO_HAND**: tracking lost; robot holds last safe target.
- **STALE_DATA**: packets stopped arriving; robot holds last safe target.
- **STOPPED**: operator triggered stop; requires process restart to re-arm.

---

## Configuration

Edit `config.yaml` to tune all parameters. Key settings:

| Parameter | Default | Meaning |
|---|---|---|
| `mapping.gain_x` | 0.4 | m of robot X per unit of hand Δx |
| `mapping.gain_y` | 0.4 | m of robot Z per unit of hand Δy |
| `mapping.dead_zone` | 0.02 | Normalised image dead zone around anchor |
| `mapping.max_delta_m` | 0.02 | Max per-update displacement (m) |
| `network.stale_data_timeout_s` | 0.5 | Stale packet threshold |
| `smoothing.alpha` | 0.4 | EMA smoothing (lower = more smooth) |

Workspace bounds must be updated after Person 2 confirms the Isaac Sim scene's
world frame and reachable workspace.

---

## Calibration Procedure

1. Start the bridge and vision process.
2. Hold your hand in the **neutral** position you want to map to the robot home.
3. Press **C** — the bridge records:
   - Your current hand position as the anchor.
   - The current/last robot target as the robot anchor.
4. Press **SPACE** to arm.
5. Move your hand relative to the calibrated neutral position.

Recalibrating at any time (while running) re-anchors without causing a target
jump, because the robot anchor is also updated.

---

## Module Structure

| File | Description |
|------|-------------|
| `teleoperation_bridge.py` | Main bridge process |
| `mapping.py` | Pure-function mapping (dead zone, gains, workspace clamp) |
| `protocol.py` | Packet parsing, validation, and serialisation |
| `config.yaml` | All tunable parameters |
| `README.md` | This file |

---

## Testing Without Hardware

Run with synthetic vision packets:

```powershell
# Terminal 1 — start the bridge
python teleoperation_bridge.py

# Terminal 2 — send a synthetic valid hand packet
python -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pkt = {'version':1,'seq':1,'timestamp_ms':int(time.time()*1000),
       'tracking':True,'confidence':0.95,
       'hand':{'x':0.55,'y':0.40,'pinch_distance':0.04},'gesture':'OPEN'}
s.sendto(json.dumps(pkt).encode(), ('127.0.0.1', 5005))
print('Sent:', pkt)
"

# Terminal 3 — listen for robot commands
python -c "
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 5006))
while True:
    data, _ = s.recvfrom(4096)
    cmd = json.loads(data)
    print(f'seq={cmd[\"seq\"]} enabled={cmd[\"enabled\"]} target={cmd[\"target_xyz_m\"]}')
"
```
