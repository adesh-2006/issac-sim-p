"""
isaac/robot_controller.py
=========================
Person 2 — Isaac Sim and Robotics Engineer

This script runs INSIDE Isaac Sim's Python environment (omni.kit).
It provides:
  - UDP command receiver on 127.0.0.1:5006.
  - Packet validation and workspace clamping (second validation layer).
  - IK-based end-effector target execution for Franka Panda.
  - Normalised gripper control (0.0 closed → 1.0 open).
  - Safe hold / stop behavior.
  - Offline demo mode with hard-coded targets (no bridge/vision required).

IMPORTANT
---------
This file is a STUB / TEMPLATE. Person 2 must:
  1. Confirm the Isaac Sim version (recommended ≥ 4.x).
  2. Replace ROBOT_PRIM_PATH, EE_LINK, GRIPPER_JOINT_NAMES with actual values
     from the saved scene.
  3. Verify the IK / articulation controller API against the installed version.
  4. Confirm world-frame units (metres) and coordinate axes.
  5. Remove stub comments and fill in the TODO sections before the first demo.

Protocol reference: PROJECT_SPEC.md §7.2
"""

from __future__ import annotations

import json
import logging
import math
import socket
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# Isaac Sim / OmniKit imports
# ---------------------------------------------------------------------------
# These imports only work inside Isaac Sim's Python environment.
# For standalone testing (offline/demo mode), they are mocked below.
try:
    import omni
    import omni.kit.commands
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import add_reference_to_stage
    from omni.isaac.manipulators.grippers.parallel_gripper import ParallelGripper
    # IK / motion generation — adjust to your Isaac Sim version:
    # Isaac Sim 2023.x / 4.x may use different namespaces.
    # Option A: lula IK
    # from omni.isaac.motion_generation import LulaKinematicsSolver
    # Option B: RMPflow / ArticulationMotionPolicy (check your version)
    # from omni.isaac.motion_generation import RmpFlow, ArticulationMotionPolicy
    _ISAAC_AVAILABLE = True
except ImportError:
    _ISAAC_AVAILABLE = False

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("isaac_controller")

# ---------------------------------------------------------------------------
# Scene and robot configuration
# ---------------------------------------------------------------------------
# *** Person 2: update these constants after loading your scene ***

ROBOT_PRIM_PATH    = "/World/Franka"           # Prim path of the Franka Panda in the USD stage
EE_LINK_NAME       = "panda_hand"              # End-effector link name
GRIPPER_JOINT_NAMES = ["panda_finger_joint1",  # Left finger
                        "panda_finger_joint2"]  # Right finger
GRIPPER_OPEN_POS   = 0.04                      # metres (fully open per joint)
GRIPPER_CLOSE_POS  = 0.0                       # metres (fully closed per joint)

# Workspace bounds (metres, Isaac Sim world frame)
# *** Person 2: fill in after verifying reachable workspace in scene ***
WS_X_MIN, WS_X_MAX = 0.2, 0.8
WS_Y_MIN, WS_Y_MAX = 0.0, 0.0   # Y held constant in MVP
WS_Z_MIN, WS_Z_MAX = 0.1, 0.7

# Stale command timeout
STALE_CMD_TIMEOUT_S = 1.0

# Protocol version expected
PROTOCOL_VERSION = 1

# ---------------------------------------------------------------------------
# Packet validation (second layer, as required by §7.2)
# ---------------------------------------------------------------------------

class CommandError(ValueError):
    """Raised when a robot command packet fails validation."""


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def validate_and_clamp_command(pkt: Dict[str, Any]) -> Dict[str, Any]:
    """
    Second-layer validation of a robot command packet.
    Clamps target to workspace bounds.
    Raises CommandError on unrecoverable violations.
    """
    if not isinstance(pkt, dict):
        raise CommandError("Packet is not a dict")

    ver = pkt.get("version")
    if ver != PROTOCOL_VERSION:
        raise CommandError(f"Unexpected protocol version: {ver}")

    for field in ("seq", "timestamp_ms", "enabled", "stop", "mode",
                  "target_xyz_m", "gripper_opening"):
        if field not in pkt:
            raise CommandError(f"Missing required field: {field!r}")

    if pkt["mode"] not in {"TARGET_XZ"}:
        raise CommandError(f"Unknown mode: {pkt['mode']!r}")

    xyz = pkt["target_xyz_m"]
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise CommandError("target_xyz_m must be list of 3 numbers")
    for i, v in enumerate(xyz):
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            raise CommandError(f"target_xyz_m[{i}] non-finite: {v}")

    go = pkt.get("gripper_opening")
    if not isinstance(go, (int, float)) or not math.isfinite(go):
        raise CommandError(f"gripper_opening invalid: {go}")
    go = _clamp(float(go), 0.0, 1.0)

    # Clamp targets to workspace (even if bridge already clamped)
    x = _clamp(float(xyz[0]), WS_X_MIN, WS_X_MAX)
    y = _clamp(float(xyz[1]), WS_Y_MIN, WS_Y_MAX)
    z = _clamp(float(xyz[2]), WS_Z_MIN, WS_Z_MAX)

    pkt = dict(pkt)
    pkt["target_xyz_m"]   = [x, y, z]
    pkt["gripper_opening"] = go
    return pkt


# ---------------------------------------------------------------------------
# UDP receiver
# ---------------------------------------------------------------------------

class CommandReceiver:
    """Non-blocking UDP receiver for robot command packets."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5006):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None
        self._latest_cmd: Optional[Dict[str, Any]] = None
        self._last_recv_time: float = 0.0
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self.host, self.port))
        self._sock.settimeout(0.05)
        self._running = True
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()
        log.info("CommandReceiver listening on %s:%d", self.host, self.port)

    def stop(self):
        self._running = False
        if self._sock:
            self._sock.close()

    def get_latest(self) -> Optional[Dict[str, Any]]:
        """Return the most recent validated command, or None."""
        with self._lock:
            return self._latest_cmd

    def is_stale(self) -> bool:
        with self._lock:
            if self._last_recv_time == 0.0:
                return False
            return (time.monotonic() - self._last_recv_time) > STALE_CMD_TIMEOUT_S

    def _recv_loop(self):
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(4096)
                pkt = json.loads(raw.decode("utf-8"))
                pkt = validate_and_clamp_command(pkt)
                with self._lock:
                    self._latest_cmd  = pkt
                    self._last_recv_time = time.monotonic()
                log.debug("Cmd received: seq=%d enabled=%s target=%s",
                          pkt["seq"], pkt["enabled"], pkt["target_xyz_m"])
            except socket.timeout:
                pass
            except (json.JSONDecodeError, CommandError) as exc:
                log.warning("Invalid command rejected: %s", exc)
            except OSError:
                if self._running:
                    raise


# ---------------------------------------------------------------------------
# Gripper helper
# ---------------------------------------------------------------------------

def gripper_opening_to_joint_pos(opening: float) -> float:
    """
    Map normalised opening (0.0 closed → 1.0 open) to joint position in metres.
    Franka finger joints each move 0.0 – 0.04 m.
    """
    opening = _clamp(opening, 0.0, 1.0)
    return GRIPPER_CLOSE_POS + opening * (GRIPPER_OPEN_POS - GRIPPER_CLOSE_POS)


# ---------------------------------------------------------------------------
# Robot controller
# ---------------------------------------------------------------------------

class RobotController:
    """
    Wraps the Isaac Sim robot, IK solver, and gripper.

    *** STUB *** — Person 2 must implement the Isaac Sim-specific parts
    (IK solving, articulation stepping, gripper actuation) using the API
    available in the installed Isaac Sim version.
    """

    def __init__(self):
        self._world:  Optional[Any] = None
        self._robot:  Optional[Any] = None
        self._gripper: Optional[Any] = None
        self._current_target: Tuple[float, float, float] = (0.5, 0.0, 0.4)
        self._current_gripper: float = 1.0
        self._disarmed = True

    def setup(self):
        """
        TODO (Person 2):
          1. Create/open the Isaac Sim World.
          2. Add a reference to the saved USD scene (or load the Franka asset directly).
          3. Initialise the robot Article, IK solver, and gripper.
          4. Call world.reset() and confirm the arm is stable.
        """
        if not _ISAAC_AVAILABLE:
            log.warning("[STUB] Isaac Sim not available — running in offline mode.")
            return

        # --- Example (adapt to your Isaac Sim version) ---
        # self._world = World(stage_units_in_meters=1.0)
        # add_reference_to_stage(usd_path="<path>/scene.usd", prim_path="/World")
        # self._robot = self._world.scene.get_object("Franka")
        # self._world.reset()
        # *** IK initialisation goes here ***
        log.info("[STUB] setup() — implement Isaac Sim scene loading here.")

    def move_to_target(self, xyz: Tuple[float, float, float]):
        """
        TODO (Person 2):
          Send an IK request for the end-effector target xyz (metres,
          Isaac Sim world frame) and step the simulation.

          Fail safely if the target is unreachable — do NOT fall back to
          raw joint commands from webcam values.
        """
        self._current_target = xyz
        if not _ISAAC_AVAILABLE:
            log.info("[STUB] move_to_target(%.3f, %.3f, %.3f)", *xyz)
            return
        # *** IK solve and articulation stepping goes here ***
        log.info("[STUB] move_to_target not yet implemented — fill in after IK setup.")

    def set_gripper(self, opening: float):
        """
        TODO (Person 2):
          Apply gripper joint targets based on normalised opening (0.0–1.0).
        """
        pos = gripper_opening_to_joint_pos(opening)
        self._current_gripper = opening
        if not _ISAAC_AVAILABLE:
            log.info("[STUB] set_gripper(%.2f) → joint_pos=%.4f m", opening, pos)
            return
        # *** Apply joint targets to gripper articulation here ***
        log.info("[STUB] set_gripper not yet implemented.")

    def hold(self):
        """Hold the last safe target — do not accept new movement targets."""
        log.info("HOLD: last target=(%.3f, %.3f, %.3f)", *self._current_target)

    def stop(self):
        """Safe stop: hold position, disarm."""
        self._disarmed = True
        self.hold()
        log.info("Robot STOPPED and DISARMED.")


# ---------------------------------------------------------------------------
# Main controller loop
# ---------------------------------------------------------------------------

def run_controller(offline_demo: bool = False):
    """
    Main control loop.

    Args:
        offline_demo: If True, steps through hard-coded targets without
                      requiring the vision/bridge pipeline.
    """
    receiver = CommandReceiver(host="127.0.0.1", port=5006)
    robot    = RobotController()

    robot.setup()
    receiver.start()

    log.info("Isaac Sim controller ready. Waiting for commands…")
    log.info("Teleoperation starts DISARMED. Bridge must send enabled=True.")

    # --- Offline demo mode ---
    if offline_demo:
        log.info("=== OFFLINE DEMO MODE ===")
        demo_targets = [
            (0.5, 0.0, 0.4),
            (0.6, 0.0, 0.5),
            (0.4, 0.0, 0.3),
            (0.5, 0.0, 0.4),
        ]
        for tgt in demo_targets:
            log.info("Demo target: %s", tgt)
            robot.move_to_target(tgt)
            robot.set_gripper(1.0)
            time.sleep(2.0)
        log.info("Demo: closing gripper")
        robot.set_gripper(0.0)
        time.sleep(1.0)
        log.info("Demo: opening gripper")
        robot.set_gripper(1.0)
        time.sleep(1.0)
        log.info("Offline demo complete.")
        receiver.stop()
        return

    # --- Live loop ---
    try:
        while True:
            cmd = receiver.get_latest()

            if receiver.is_stale():
                log.warning("Stale commands detected — holding position.")
                robot.hold()
                time.sleep(0.1)
                continue

            if cmd is None:
                time.sleep(0.02)
                continue

            if cmd.get("stop"):
                robot.stop()
                break

            if not cmd.get("enabled", False):
                robot.hold()
                time.sleep(0.02)
                continue

            # Apply validated, clamped target
            xyz = tuple(cmd["target_xyz_m"])
            robot.move_to_target(xyz)
            robot.set_gripper(float(cmd["gripper_opening"]))

            time.sleep(0.02)  # ~50 Hz control loop

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt — stopping robot.")
        robot.stop()
    finally:
        receiver.stop()
        log.info("Controller shut down.")


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Isaac Sim Robot Controller")
    p.add_argument("--offline", action="store_true", default=False,
                   help="Run offline demo mode (no vision/bridge required)")
    p.add_argument("--debug", action="store_true", default=False)
    args = p.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    run_controller(offline_demo=args.offline)
