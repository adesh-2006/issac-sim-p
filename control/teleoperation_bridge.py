"""
control/teleoperation_bridge.py
===============================
Person 3 — Teleoperation, Mapping, and Integration Engineer

Responsibilities:
  - Receive vision packets from Person 1 on UDP 127.0.0.1:5005.
  - Validate fields, types, sequence numbers, timestamps, and tracking state.
  - Apply EMA smoothing to hand coordinates.
  - Implement calibration/recenter: anchor current hand → current robot target.
  - Map hand coordinate deltas to robot X/Z movement (delta accumulation).
  - Apply dead zone, scale gains, per-update cap, and workspace clamp.
  - Convert gesture to gripper opening command.
  - Send robot command packets to Person 2 on UDP 127.0.0.1:5006.
  - State machine: DISARMED → ACTIVE ↔ NO_HAND ↔ STALE_DATA → STOPPED.
  - Keyboard controls: SPACE arm/disarm, C calibrate, ESC stop+exit.

Usage:
    python teleoperation_bridge.py [--config config.yaml]

Protocol reference: PROJECT_SPEC.md §7.1 and §7.2
"""

from __future__ import annotations

import json
import logging
import os
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional, Tuple

# Try to load PyYAML for config (falls back to defaults if unavailable)
try:
    import yaml as _yaml
    _YAML_AVAILABLE = True
except ImportError:
    _YAML_AVAILABLE = False

# Local modules
sys.path.insert(0, os.path.dirname(__file__))
from mapping  import MappingConfig, CalibrationState, map_hand_to_robot, gesture_to_gripper, calibrate
from protocol import parse_vision_packet, build_robot_command, VisionPacketError

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("bridge")

# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class BridgeState(str, Enum):
    DISARMED    = "DISARMED"
    ACTIVE      = "ACTIVE"
    NO_HAND     = "NO_HAND"
    STALE_DATA  = "STALE_DATA"
    STOPPED     = "STOPPED"

# ---------------------------------------------------------------------------
# Config dataclass (populated from YAML or defaults)
# ---------------------------------------------------------------------------

@dataclass
class BridgeConfig:
    # Network
    vision_host: str  = "127.0.0.1"
    vision_port: int  = 5005
    robot_host:  str  = "127.0.0.1"
    robot_port:  int  = 5006
    socket_timeout_s:   float = 0.05
    stale_data_timeout_s: float = 0.5
    min_confidence: float = 0.5

    # EMA smoothing
    alpha: float = 0.4

    # Mapping / workspace
    mapping: MappingConfig = field(default_factory=MappingConfig)

    @classmethod
    def from_yaml(cls, path: str) -> "BridgeConfig":
        if not _YAML_AVAILABLE:
            log.warning("PyYAML not installed — using default configuration.")
            return cls()
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = _yaml.safe_load(f) or {}
        except FileNotFoundError:
            log.warning("Config file not found: %s — using defaults.", path)
            return cls()

        net  = data.get("network", {})
        smo  = data.get("smoothing", {})
        mp_  = data.get("mapping", {})
        ws   = data.get("workspace", {})
        home = data.get("robot_home", {})
        ges  = data.get("gesture", {})

        mc = MappingConfig(
            gain_x=mp_.get("gain_x", 0.4),
            gain_y=mp_.get("gain_y", 0.4),
            dead_zone=mp_.get("dead_zone", 0.02),
            max_delta_m=mp_.get("max_delta_m", 0.02),
            workspace_x_min=ws.get("x_min", 0.2),
            workspace_x_max=ws.get("x_max", 0.8),
            workspace_y_min=ws.get("y_min", 0.0),
            workspace_y_max=ws.get("y_max", 0.0),
            workspace_z_min=ws.get("z_min", 0.1),
            workspace_z_max=ws.get("z_max", 0.7),
            robot_home_x=home.get("x", 0.5),
            robot_home_y=home.get("y", 0.0),
            robot_home_z=home.get("z", 0.4),
        )

        return cls(
            vision_host=net.get("vision_host", "127.0.0.1"),
            vision_port=int(net.get("vision_port", 5005)),
            robot_host=net.get("robot_host", "127.0.0.1"),
            robot_port=int(net.get("robot_port", 5006)),
            socket_timeout_s=float(net.get("socket_timeout_s", 0.05)),
            stale_data_timeout_s=float(net.get("stale_data_timeout_s", 0.5)),
            alpha=float(smo.get("alpha", 0.4)),
            min_confidence=float(ges.get("min_confidence", 0.5)),
            mapping=mc,
        )


# ---------------------------------------------------------------------------
# EMA helper (mirrors vision module)
# ---------------------------------------------------------------------------

def _ema(prev: Optional[float], new_val: float, alpha: float) -> float:
    if prev is None:
        return new_val
    return alpha * new_val + (1.0 - alpha) * prev

# ---------------------------------------------------------------------------
# Bridge
# ---------------------------------------------------------------------------

class TeleoperationBridge:
    """
    The teleoperation bridge main class.

    Threading model:
      - Main thread: packet receive + processing loop.
      - Keyboard thread: non-blocking input handling (separate daemon thread).
    """

    def __init__(self, cfg: BridgeConfig):
        self.cfg    = cfg
        self.state  = BridgeState.DISARMED
        self._seq   = 0

        # Calibration
        mc = cfg.mapping
        self._calib = CalibrationState(
            hand_x0=0.5, hand_y0=0.5,
            robot_x0=mc.robot_home_x,
            robot_y0=mc.robot_home_y,
            robot_z0=mc.robot_home_z,
            calibrated=False,
        )
        self._current_target: Tuple[float, float, float] = (
            mc.robot_home_x, mc.robot_home_y, mc.robot_home_z
        )
        self._gripper_opening: float = 1.0   # default open

        # Smoothed hand coordinates
        self._smooth_x: Optional[float] = None
        self._smooth_y: Optional[float] = None

        # Last valid packet reception time (monotonic)
        self._last_packet_time: float = 0.0
        self._last_seq: int = -1

        # Sockets
        self._recv_sock: Optional[socket.socket] = None
        self._send_sock: Optional[socket.socket] = None

        # Keyboard thread flag
        self._running = True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self):
        """Open sockets and start keyboard thread."""
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.bind((self.cfg.vision_host, self.cfg.vision_port))
        self._recv_sock.settimeout(self.cfg.socket_timeout_s)
        log.info("Listening for vision packets on %s:%d",
                 self.cfg.vision_host, self.cfg.vision_port)

        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        log.info("Will send robot commands to %s:%d",
                 self.cfg.robot_host, self.cfg.robot_port)

        # Start keyboard listener in a daemon thread
        kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        kb_thread.start()

        self._print_status_banner()

    def stop(self):
        """Close sockets and stop."""
        self._running = False
        if self._recv_sock:
            self._recv_sock.close()
        if self._send_sock:
            self._send_sock.close()
        log.info("Bridge stopped.")

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self):
        """Receive → validate → map → send loop."""
        self.start()
        try:
            while self._running:
                self._loop_once()
        except KeyboardInterrupt:
            log.info("KeyboardInterrupt — exiting.")
        finally:
            self.stop()

    def _loop_once(self):
        """Process one iteration: receive packet (or timeout), decide, send."""
        # ---- Try to receive a vision packet ----
        pkt: Optional[Dict[str, Any]] = None
        try:
            raw, _ = self._recv_sock.recvfrom(4096)
            pkt = parse_vision_packet(raw)
        except socket.timeout:
            pass   # will check staleness below
        except VisionPacketError as exc:
            log.warning("Invalid vision packet rejected: %s", exc)
        except OSError as exc:
            if self._running:
                log.error("Recv socket error: %s", exc)

        now = time.monotonic()

        # ---- Check for stale data ----
        stale = (
            self._last_packet_time > 0
            and (now - self._last_packet_time) > self.cfg.stale_data_timeout_s
        )

        # ---- Process packet ----
        if pkt is not None:
            self._last_packet_time = now
            self._process_vision_packet(pkt)
        elif stale:
            if self.state == BridgeState.ACTIVE:
                self._set_state(BridgeState.STALE_DATA)

        # ---- Decide whether to send a robot command ----
        self._decide_and_send(stale)

    def _process_vision_packet(self, pkt: Dict[str, Any]):
        """Update smoothed coordinates and gesture from a validated vision packet."""
        # Sequence check (warn but don't reject out-of-order for now)
        seq = pkt["seq"]
        if seq <= self._last_seq:
            log.debug("Out-of-order packet: seq=%d last=%d", seq, self._last_seq)
        self._last_seq = seq

        tracking   = pkt["tracking"]
        confidence = float(pkt["confidence"])
        gesture    = pkt["gesture"]

        if not tracking or confidence < self.cfg.min_confidence:
            # No valid hand
            if self.state == BridgeState.ACTIVE:
                self._set_state(BridgeState.NO_HAND)
            return

        # Valid hand — update smoothed position
        hand = pkt["hand"]
        hx   = float(hand["x"])
        hy   = float(hand["y"])
        self._smooth_x = _ema(self._smooth_x, hx, self.cfg.alpha)
        self._smooth_y = _ema(self._smooth_y, hy, self.cfg.alpha)

        # Update gripper from gesture
        g = gesture_to_gripper(gesture)
        if g is not None:
            self._gripper_opening = g

        # Restore ACTIVE if we were in NO_HAND/STALE
        if self.state in (BridgeState.NO_HAND, BridgeState.STALE_DATA):
            self._set_state(BridgeState.ACTIVE)

    def _decide_and_send(self, stale: bool):
        """Build and send a robot command based on current bridge state."""
        enabled = self.state == BridgeState.ACTIVE
        stop    = self.state == BridgeState.STOPPED

        # Only compute a new target when ACTIVE and we have valid hand coords
        if enabled and self._smooth_x is not None and self._smooth_y is not None:
            new_target = map_hand_to_robot(
                hand_x=self._smooth_x,
                hand_y=self._smooth_y,
                calib=self._calib,
                cfg=self.cfg.mapping,
                last_target=self._current_target,
            )
            self._current_target = new_target

        # Build packet
        self._seq += 1
        try:
            cmd = build_robot_command(
                seq=self._seq,
                enabled=enabled,
                stop=stop,
                target_xyz_m=self._current_target,
                gripper_opening=self._gripper_opening,
            )
        except ValueError as exc:
            log.error("Failed to build robot command: %s", exc)
            return

        self._send_robot_command(cmd)

    def _send_robot_command(self, cmd: dict):
        """Serialise and send a robot command packet via UDP."""
        try:
            data = json.dumps(cmd, separators=(",", ":")).encode("utf-8")
            self._send_sock.sendto(data, (self.cfg.robot_host, self.cfg.robot_port))
            log.debug(
                "→ seq=%d enabled=%s target=(%.3f,%.3f,%.3f) gripper=%.2f",
                cmd["seq"], cmd["enabled"],
                *cmd["target_xyz_m"], cmd["gripper_opening"],
            )
        except OSError as exc:
            log.error("Send socket error: %s", exc)

    # ------------------------------------------------------------------
    # State machine transitions
    # ------------------------------------------------------------------

    def _set_state(self, new_state: BridgeState):
        if self.state != new_state:
            log.info("State: %s → %s", self.state.value, new_state.value)
            self.state = new_state

    def _action_arm_disarm(self):
        if self.state == BridgeState.DISARMED:
            if not self._calib.calibrated:
                log.warning(
                    "Arming without calibration — press C to calibrate first. "
                    "Defaulting to home position."
                )
            self._set_state(BridgeState.ACTIVE)
            log.info("Teleoperation ARMED. Move your hand to control the robot.")
        elif self.state in (BridgeState.ACTIVE, BridgeState.NO_HAND, BridgeState.STALE_DATA):
            self._set_state(BridgeState.DISARMED)
            log.info("Teleoperation DISARMED.")
        elif self.state == BridgeState.STOPPED:
            log.warning("Cannot arm from STOPPED state — press ESC to exit and restart.")

    def _action_calibrate(self):
        if self._smooth_x is None or self._smooth_y is None:
            log.warning("Cannot calibrate — no valid hand position available.")
            return
        self._calib = calibrate(
            hand_x=self._smooth_x,
            hand_y=self._smooth_y,
            current_target=self._current_target,
            cfg=self.cfg.mapping,
        )
        log.info(
            "Calibrated: hand anchor=(%.3f, %.3f) robot anchor=(%.3f, %.3f, %.3f)",
            self._calib.hand_x0, self._calib.hand_y0,
            self._calib.robot_x0, self._calib.robot_y0, self._calib.robot_z0,
        )

    def _action_stop(self):
        log.info("STOP action triggered.")
        self._set_state(BridgeState.STOPPED)
        self._running = False

    # ------------------------------------------------------------------
    # Keyboard input (daemon thread)
    # ------------------------------------------------------------------

    def _keyboard_loop(self):
        """
        Cross-platform single-character keyboard input.
        Uses msvcrt on Windows, termios on Unix.
        """
        try:
            import msvcrt
            log.info("Keyboard: SPACE=arm/disarm  C=calibrate  ESC=stop+exit")
            while self._running:
                if msvcrt.kbhit():
                    ch = msvcrt.getwch()
                    self._handle_key(ch)
                time.sleep(0.05)
        except ImportError:
            # Unix fallback
            try:
                import tty, termios
                fd = sys.stdin.fileno()
                old = termios.tcgetattr(fd)
                tty.setraw(fd)
                log.info("Keyboard: SPACE=arm/disarm  C=calibrate  ESC=stop+exit")
                try:
                    while self._running:
                        ch = sys.stdin.read(1)
                        if ch:
                            self._handle_key(ch)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception as exc:
                log.warning("Keyboard input unavailable: %s. Use Ctrl-C to exit.", exc)

    def _handle_key(self, ch: str):
        """Dispatch a key press to the appropriate action."""
        if ch in (" ", "\r"):       # SPACE or Enter → arm/disarm
            self._action_arm_disarm()
        elif ch.lower() == "c":     # C → calibrate
            self._action_calibrate()
        elif ch == "\x1b" or ch == "\x03":   # ESC or Ctrl-C → stop
            self._action_stop()

    # ------------------------------------------------------------------
    # Banner
    # ------------------------------------------------------------------

    def _print_status_banner(self):
        log.info("=" * 58)
        log.info("  HandPilot — Teleoperation Bridge")
        log.info("  State    : %s", self.state.value)
        log.info("  Recv     : %s:%d", self.cfg.vision_host, self.cfg.vision_port)
        log.info("  Send     : %s:%d", self.cfg.robot_host, self.cfg.robot_port)
        log.info("  Stale TO : %.1fs", self.cfg.stale_data_timeout_s)
        log.info("  EMA α    : %.2f", self.cfg.alpha)
        log.info("=" * 58)
        log.info("  SPACE → arm/disarm   C → calibrate   ESC → stop+exit")
        log.info("=" * 58)


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main():
    import argparse
    p = argparse.ArgumentParser(description="HandPilot Teleoperation Bridge")
    p.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "config.yaml"),
                   help="Path to config.yaml")
    p.add_argument("--debug", action="store_true", default=False)
    args = p.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    cfg = BridgeConfig.from_yaml(args.config)
    bridge = TeleoperationBridge(cfg)
    bridge.run()


if __name__ == "__main__":
    main()
