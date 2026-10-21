# Vision Module — Person 1

## Overview

The vision module captures frames from a laptop webcam, detects one hand using
MediaPipe, and sends structured UDP/JSON packets to the Teleoperation Bridge
(Person 3) at 127.0.0.1:5005.

---

## Setup

### Requirements

- Python 3.10 or 3.11 (recommended)
- A working laptop webcam (built-in or USB RGB)
- Windows 10/11 or Linux (tested on both)

### Install dependencies

```powershell
cd vision
pip install -r requirements.txt
```

### Run

```powershell
python hand_tracking.py
```

Additional flags:

```
--camera 0        Webcam device index (default: 0; try 1 or 2 if wrong camera opens)
--host 127.0.0.1  UDP destination host
--port 5005       UDP destination port
--target-fps 30   Capture / send target FPS
--alpha 0.4       EMA smoothing factor (0 < α ≤ 1; lower = more smooth)
--no-show         Disable the preview window (headless / remote)
--debug           Enable DEBUG-level logging to console
```

Example with debug logging, camera index 1:

```powershell
python hand_tracking.py --camera 1 --debug
```

---

## How It Works

### Palm-center landmark formula

The 2D hand reference point is computed as the arithmetic mean of five palm-base
MediaPipe landmarks:

| Landmark | Index |
|---|---|
| Wrist | 0 |
| Index MCP | 5 |
| Middle MCP | 9 |
| Ring MCP | 13 |
| Pinky MCP | 17 |

These are the most stable landmarks during hand motion. The resulting point
tracks the geometric centre of the palm.

### Coordinate convention

- `x = 0.0` → left edge of frame; `x = 1.0` → right edge
- `y = 0.0` → top edge of frame; `y = 1.0` → bottom edge
- Origin is **top-left** (standard image convention)
- Frames are **horizontally flipped** (mirror view) for the operator

### Pinch detection

Pinch distance = Euclidean distance between thumb-tip (landmark 4) and
index-tip (landmark 8), divided by the bounding-box diagonal of the five
palm-base landmarks. This makes the threshold **hand-size-relative** and
independent of how far the hand is from the camera.

Default thresholds (in palm-diagonal fractions):

| Parameter | Default | Meaning |
|---|---|---|
| `pinch_close_threshold` | 0.12 | Below this → PINCH |
| `pinch_open_threshold` | 0.18 | Above this → OPEN |
| Band 0.12–0.18 | — | Hysteresis: keep current state |

### Smoothing

Coordinates are filtered with an Exponential Moving Average:

```
smooth = α × new_value + (1 − α) × previous_smooth
```

Default `α = 0.4` (tunable with `--alpha`). A lower alpha removes more jitter
but adds more lag.

### Confidence proxy

MediaPipe's legacy Hands solution does not expose a single confidence score
for each detected hand. We compute the mean of the per-landmark `presence`
scores (range 0–1) across all 21 landmarks. If `presence` is unavailable
(older MediaPipe builds), the proxy falls back to `1.0` whenever a hand is
detected.

---

## Protocol (§7.1 of PROJECT_SPEC.md)

Packets are sent as UTF-8 JSON over UDP to `127.0.0.1:5005`.

**Valid hand:**
```json
{
  "version": 1,
  "seq": 42,
  "timestamp_ms": 1791800000000,
  "tracking": true,
  "confidence": 0.93,
  "hand": { "x": 0.55, "y": 0.40, "pinch_distance": 0.04 },
  "gesture": "PINCH"
}
```

**No hand detected:**
```json
{
  "version": 1,
  "seq": 43,
  "timestamp_ms": 1791800000033,
  "tracking": false,
  "confidence": 0.0,
  "hand": null,
  "gesture": "UNKNOWN"
}
```

Possible `gesture` values: `"PINCH"`, `"OPEN"`, `"UNKNOWN"`

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Cannot open webcam at index 0` | Try `--camera 1` or `--camera 2`. Check that another app (e.g. Teams) is not holding the camera. |
| `MediaPipe is not installed` | Run `pip install mediapipe` in the correct Python environment. |
| Landmarks flicker / lose tracking | Improve lighting (avoid back-lighting). Keep hand within 30–80 cm of camera. |
| Pinch state flickers rapidly | Reduce `--alpha` for more smoothing, or widen the hysteresis band in `TrackerConfig`. |
| Preview window shows only black | OpenCV may not support the webcam codec. Try a USB webcam. |

---

## Test / Sample Log

To verify packets without a running bridge, run:

```powershell
# In one terminal: start the tracker
python hand_tracking.py

# In another terminal: listen on UDP 5005
python -c "
import socket, json
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('127.0.0.1', 5005))
while True:
    data, _ = s.recvfrom(4096)
    pkt = json.loads(data)
    print(f'seq={pkt[\"seq\"]} tracking={pkt[\"tracking\"]} gesture={pkt[\"gesture\"]}')
"
```

---

## Deliverables checklist (§4 of PROJECT_SPEC.md)

- [x] `hand_tracking.py` — full vision pipeline
- [x] `requirements.txt` — pinned dependency versions
- [x] `README.md` — this file
- [ ] Sample log or recorded test (record during integration Phase 2)
- [ ] Confirmation packets follow §7.1 schema ← verify during Phase 2 integration test
