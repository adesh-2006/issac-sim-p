# Demo Checklist

Use this checklist before every demonstration. Pass = ✅, Fail = ❌ (note issue).

---

## Pre-Demo Setup

- [ ] Isaac Sim scene opens and the Franka arm is stable at rest.
- [ ] Correct webcam is available (`--camera 0` or as configured).
- [ ] Repository is up to date (`git pull --ff-only origin main`).
- [ ] Config files match agreed workspace and ports.

---

## Launch Sequence

| Step | Command | Expected Output |
|---|---|---|
| 1 | Isaac Sim controller | "Controller ready. Waiting for commands…" |
| 2 | Teleoperation bridge | "Listening on 127.0.0.1:5005" |
| 3 | Vision process | Preview window opens with landmarks |

---

## Pre-Arm Verification

- [ ] Hand landmarks are visible and stable.
- [ ] Normalized coordinates are in [0.0, 1.0].
- [ ] PINCH / OPEN gesture switches correctly.
- [ ] Stop control (ESC) tested before arming — robot must hold/stop.
- [ ] Bridge shows **DISARMED** state on startup.

---

## Calibration

- [ ] Hand placed in comfortable neutral position.
- [ ] **C** pressed — bridge logs "Calibrated: hand anchor=… robot anchor=…"
- [ ] Robot arm **does not jump** on calibration.

---

## Motion Tests (arm enabled with SPACE)

- [ ] Move hand **left** → robot X decreases (or as configured).
- [ ] Move hand **right** → robot X increases.
- [ ] Move hand **up** → robot Z increases.
- [ ] Move hand **down** → robot Z decreases.
- [ ] Robot stays within configured workspace bounds.
- [ ] Motion feels smooth — no sudden jumps.

---

## Gripper Tests

- [ ] Pinch thumb to index finger → gripper **closes**.
- [ ] Release pinch → gripper **opens**.
- [ ] Gripper is stable in pinch-hold (no rapid open/close flicker).

---

## Failure Handling Tests

- [ ] Move hand out of frame → robot **holds last position** (NO_HAND state).
- [ ] Stop the vision process → bridge enters STALE_DATA after ~0.5 s.
- [ ] ESC pressed → robot stops/holds; bridge exits cleanly.
- [ ] Malformed packet injected → rejected; robot does not move.

---

## Acceptance Criteria (from §9.3)

- [ ] Webcam tracks one hand and displays landmarks.
- [ ] Hand coordinates are normalized and stable enough for use.
- [ ] Pinch reliably closes the gripper; releasing pinch opens it.
- [ ] Vision and robot command packets follow the agreed schemas.
- [ ] Arm can be moved from live hand position after deliberate arming and calibration.
- [ ] Movement is constrained to a defined workspace and uses configurable gains.
- [ ] No tracking / no stale data → no new motion target is generated.
- [ ] Stop/disarm is tested and does not depend on gesture recognition.
- [ ] Invalid data and unreachable targets fail safely.
- [ ] README contains setup instructions, known limitations, and this repeatable demo checklist.

---

## Post-Demo

- [ ] Disarm with SPACE.
- [ ] Exit vision and bridge processes.
- [ ] Exit Isaac Sim cleanly.
- [ ] Record any issues in the repository (GitHub Issues or CHANGELOG).
