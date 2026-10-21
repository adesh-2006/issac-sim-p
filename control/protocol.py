"""
control/protocol.py
===================
Shared packet parsing, validation, and serialisation helpers.
Both teleoperation_bridge.py (sender) and any test code import this module.

Protocol reference: PROJECT_SPEC.md §7.1 and §7.2
"""

from __future__ import annotations

import json
import math
import time
from typing import Any, Dict, Optional, Tuple

PROTOCOL_VERSION = 1

VALID_GESTURES = {"PINCH", "OPEN", "UNKNOWN"}
VALID_MODES    = {"TARGET_XZ"}

# ---------------------------------------------------------------------------
# Vision packet (§7.1) — received from Person 1
# ---------------------------------------------------------------------------

class VisionPacketError(ValueError):
    """Raised when a received vision packet fails validation."""


def parse_vision_packet(raw: bytes) -> Dict[str, Any]:
    """
    Decode and validate a raw UDP vision packet from Person 1.

    Returns the validated packet dict.
    Raises VisionPacketError with a descriptive message on any failure.
    """
    try:
        pkt: Dict[str, Any] = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise VisionPacketError(f"JSON decode error: {exc}") from exc

    _require_field(pkt, "version", int)
    if pkt["version"] != PROTOCOL_VERSION:
        raise VisionPacketError(
            f"Unknown protocol version: {pkt['version']} (expected {PROTOCOL_VERSION})"
        )

    _require_field(pkt, "seq",          int)
    _require_field(pkt, "timestamp_ms", int)
    _require_field(pkt, "tracking",     bool)
    _require_field(pkt, "confidence",   (int, float))
    _require_field(pkt, "gesture",      str)

    conf = float(pkt["confidence"])
    if not (0.0 <= conf <= 1.0) or not math.isfinite(conf):
        raise VisionPacketError(f"confidence out of [0,1]: {conf}")

    if pkt["gesture"] not in VALID_GESTURES:
        raise VisionPacketError(f"Unknown gesture value: {pkt['gesture']!r}")

    if pkt["tracking"]:
        # hand must be a dict with x, y, pinch_distance
        hand = pkt.get("hand")
        if not isinstance(hand, dict):
            raise VisionPacketError("tracking=true but hand is not a dict")
        x  = _require_numeric(hand, "x",  "hand.x")
        y  = _require_numeric(hand, "y",  "hand.y")
        pd = hand.get("pinch_distance")
        if not (0.0 <= x <= 1.0):
            raise VisionPacketError(f"hand.x out of [0,1]: {x}")
        if not (0.0 <= y <= 1.0):
            raise VisionPacketError(f"hand.y out of [0,1]: {y}")
        if pd is not None and not math.isfinite(float(pd)):
            raise VisionPacketError(f"hand.pinch_distance is non-finite: {pd}")
    else:
        # When tracking is false, hand must be null and gesture should be UNKNOWN
        if pkt.get("hand") is not None:
            raise VisionPacketError("tracking=false but hand is not null")

    return pkt


# ---------------------------------------------------------------------------
# Robot command packet (§7.2) — sent to Person 2
# ---------------------------------------------------------------------------

def build_robot_command(
    seq: int,
    enabled: bool,
    stop: bool,
    target_xyz_m: Tuple[float, float, float],
    gripper_opening: float,
    mode: str = "TARGET_XZ",
) -> Dict[str, Any]:
    """
    Construct a validated robot command packet.
    Raises ValueError if inputs are invalid.
    """
    if mode not in VALID_MODES:
        raise ValueError(f"Unknown mode: {mode!r}")
    x, y, z = target_xyz_m
    for val, name in [(x, "x"), (y, "y"), (z, "z")]:
        if not math.isfinite(val):
            raise ValueError(f"target_xyz_m[{name}] is non-finite: {val}")
    if not (0.0 <= gripper_opening <= 1.0) or not math.isfinite(gripper_opening):
        raise ValueError(f"gripper_opening out of [0,1]: {gripper_opening}")

    return {
        "version":         PROTOCOL_VERSION,
        "seq":             seq,
        "timestamp_ms":    int(time.time() * 1000),
        "enabled":         enabled,
        "stop":            stop,
        "mode":            mode,
        "target_xyz_m":   [round(x, 6), round(y, 6), round(z, 6)],
        "gripper_opening": round(gripper_opening, 4),
    }


def parse_robot_command(raw: bytes) -> Dict[str, Any]:
    """
    Decode and validate a raw UDP robot command packet.
    Used by Person 2's controller and by tests.
    Raises VisionPacketError (reusing the exception class) on failure.
    """
    try:
        pkt = json.loads(raw.decode("utf-8"))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise VisionPacketError(f"JSON decode error: {exc}") from exc

    _require_field(pkt, "version",         int)
    if pkt["version"] != PROTOCOL_VERSION:
        raise VisionPacketError(f"Unknown protocol version: {pkt['version']}")

    _require_field(pkt, "seq",             int)
    _require_field(pkt, "timestamp_ms",    int)
    _require_field(pkt, "enabled",         bool)
    _require_field(pkt, "stop",            bool)
    _require_field(pkt, "mode",            str)
    if pkt["mode"] not in VALID_MODES:
        raise VisionPacketError(f"Unknown mode: {pkt['mode']!r}")

    xyz = pkt.get("target_xyz_m")
    if not isinstance(xyz, list) or len(xyz) != 3:
        raise VisionPacketError("target_xyz_m must be a list of 3 numbers")
    for i, v in enumerate(xyz):
        if not isinstance(v, (int, float)) or not math.isfinite(v):
            raise VisionPacketError(f"target_xyz_m[{i}] is non-finite or wrong type: {v}")

    go = pkt.get("gripper_opening")
    if not isinstance(go, (int, float)) or not math.isfinite(go) or not (0.0 <= go <= 1.0):
        raise VisionPacketError(f"gripper_opening invalid: {go}")

    return pkt


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _require_field(pkt: dict, key: str, expected_type):
    if key not in pkt:
        raise VisionPacketError(f"Missing required field: {key!r}")
    if not isinstance(pkt[key], expected_type):
        raise VisionPacketError(
            f"Field {key!r} must be {expected_type}, got {type(pkt[key]).__name__}"
        )


def _require_numeric(d: dict, key: str, label: str) -> float:
    val = d.get(key)
    if not isinstance(val, (int, float)):
        raise VisionPacketError(f"{label} must be numeric, got {type(val).__name__}")
    fval = float(val)
    if not math.isfinite(fval):
        raise VisionPacketError(f"{label} is non-finite: {val}")
    return fval
