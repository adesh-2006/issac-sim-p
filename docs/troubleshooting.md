# Troubleshooting Guide

---

## Vision Process (`hand_tracking.py`)

### Cannot open webcam

```
ERROR Cannot open webcam at index 0
```

**Fix:**
- Try `--camera 1` or `--camera 2`.
- Check that another application (Teams, Zoom, OBS) is not holding the camera.
- On Windows: check Camera Privacy settings in Windows Settings → Privacy → Camera.

---

### MediaPipe not installed

```
ERROR MediaPipe is not installed.
```

**Fix:**
```powershell
pip install mediapipe
```
Ensure you are in the correct Python environment (the one used by `vision/.venv`).

---

### Landmarks not detected / tracking lost frequently

**Possible causes:**
- Poor lighting (back-lighting, low light).
- Hand too far from camera (keep 30–80 cm).
- Occluded hand or unusual viewing angle.

**Fix:**
- Improve ambient lighting — diffuse front light works best.
- Try lowering `min_detection_confidence` in `TrackerConfig` (e.g. 0.5).
- Reduce `smoothing_alpha` to suppress jitter.

---

### Pinch state flickers

**Symptom:** Gripper opens and closes rapidly without deliberate gesture change.

**Fix:**
- Widen the hysteresis gap: increase `pinch_open_threshold` in `TrackerConfig`.
- Reduce `smoothing_alpha` (more smoothing).
- Improve lighting.

---

### Preview window is black / very dark

**Fix:**
- Webcam codec issue — try a USB webcam.
- On some laptops: check that the webcam driver is installed.

---

## Teleoperation Bridge (`teleoperation_bridge.py`)

### Vision packets not received

**Check:**
1. Vision process is running and sending to `127.0.0.1:5005`.
2. Bridge config (`config.yaml`) has the correct `vision_port`.
3. Windows Firewall is not blocking localhost UDP.

**Test:** Run the synthetic packet sender from `control/README.md`.

---

### Bridge state stuck in STALE_DATA immediately

**Cause:** Vision process is not sending packets, or is sending to a different port.

**Fix:** Confirm `vision/hand_tracking.py` `--port` matches `config.yaml` `vision_port`.

---

### Robot arm does not move after arming

**Check:**
1. Bridge is in **ACTIVE** state (not DISARMED/NO_HAND/STALE_DATA).
2. Calibration has been performed (press **C** before moving).
3. Isaac Sim controller is running and receiving on port 5006.
4. Hand is within the dead zone — move further from the calibration position.

---

### Arm jumps when calibrating

**Cause:** Calibration was triggered while the bridge had an old robot anchor.

**Fix:** The `calibrate()` function always anchors to the **current** robot target, so no jump should occur. If a jump is observed, verify that `current_target` is being passed correctly to `calibrate()` in the bridge.

---

## Isaac Sim Controller (`robot_controller.py`)

### `ImportError: No module named 'omni'`

**Cause:** Script is running outside Isaac Sim's Python environment.

**Fix:** Run using Isaac Sim's built-in Script Editor, or use `--offline` flag for standalone testing:
```powershell
python robot_controller.py --offline
```

---

### Robot arm not found / prim path error

**Fix:** Update `ROBOT_PRIM_PATH` in `robot_controller.py` to match the actual prim path of the Franka Panda in your USD scene. Check with:
```python
import omni.usd
stage = omni.usd.get_context().get_stage()
for prim in stage.Traverse():
    print(prim.GetPath())
```

---

### Arm moves erratically after receiving commands

**Check:**
1. Workspace bounds (`WS_*_MIN/MAX`) are set to verified reachable values.
2. Person 3's bridge gains (`gain_x`, `gain_y`) are conservative (start at 0.1).
3. Isaac Sim physics timestep is stable.

---

## Tests

### `pytest: command not found`

```powershell
pip install pytest
python -m pytest tests/ -v
```

### `ModuleNotFoundError: No module named 'protocol'`

The tests add `control/` to `sys.path` automatically. Run from the **repository root**:
```powershell
python -m pytest tests/ -v
```

---

## General Tips

| Issue | Quick check |
|---|---|
| UDP port conflict | `netstat -an | findstr 5005` (Windows) |
| Wrong Python env | `python --version` and `pip list` in the active venv |
| Git authorship wrong | `git config user.email` |
| Branch diverged | `git fetch origin && git log --oneline HEAD..origin/main` |
