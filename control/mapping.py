"""
control/mapping.py
==================
Pure-function coordinate mapping: hand image coordinates → robot target XYZ.

All functions here are dependency-free (no sockets, no config files) so they
can be unit-tested without any external resources.

Reference: PROJECT_SPEC.md §5.1 and §5.2
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class MappingConfig:
    """All mapping parameters — loaded from config.yaml by the bridge."""

    # Axis gains (metres per unit of normalised hand delta)
    gain_x: float = 0.4   # hand Δx → robot +X
    gain_y: float = 0.4   # hand Δy → robot +Z (sign inverted: hand-up → robot-up)

    # Dead zone around the calibrated anchor (normalised image units)
    dead_zone: float = 0.02

    # Per-update maximum target displacement (metres)
    max_delta_m: float = 0.02

    # Workspace bounds (metres, in Isaac Sim world frame)
    workspace_x_min: float = 0.2
    workspace_x_max: float = 0.8
    workspace_y_min: float = 0.0   # kept constant in MVP
    workspace_y_max: float = 0.0
    workspace_z_min: float = 0.1
    workspace_z_max: float = 0.7

    # Robot anchor in world frame (metres) — set after Person 2 confirms scene
    robot_home_x: float = 0.5
    robot_home_y: float = 0.0
    robot_home_z: float = 0.4


@dataclass
class CalibrationState:
    """Records the hand and robot anchor after a calibration action."""
    hand_x0:   float = 0.5
    hand_y0:   float = 0.5
    robot_x0:  float = 0.5
    robot_y0:  float = 0.0
    robot_z0:  float = 0.4
    calibrated: bool = False


def apply_dead_zone(delta: float, dead_zone: float) -> float:
    """
    Zero out small deltas within the dead zone.
    Values outside the zone are shifted inward so there is no discontinuity.
    """
    if abs(delta) < dead_zone:
        return 0.0
    sign = 1.0 if delta > 0 else -1.0
    return sign * (abs(delta) - dead_zone)


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]."""
    return max(lo, min(hi, value))


def map_hand_to_robot(
    hand_x: float,
    hand_y: float,
    calib: CalibrationState,
    cfg: MappingConfig,
    last_target: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """
    Map normalised hand image coordinates to an absolute robot target (x, y, z)
    in metres, using the calibrated anchor as the reference point.

    Steps:
      1. Compute delta from calibration anchor.
      2. Apply dead zone.
      3. Scale by gain to get desired delta in metres.
      4. Cap per-update displacement.
      5. Add to last target (delta-accumulation approach).
      6. Clamp to workspace bounds.

    Args:
        hand_x      : Current smoothed hand x (0–1 image space)
        hand_y      : Current smoothed hand y (0–1 image space, top-left origin)
        calib       : Current calibration anchor
        cfg         : Mapping parameters
        last_target : Last valid robot target (x, y, z) in metres

    Returns:
        New robot target (x, y, z) in metres.
    """
    # 1. Deltas from anchor
    hand_dx = hand_x - calib.hand_x0
    hand_dy = hand_y - calib.hand_y0

    # 2. Dead zone
    hand_dx = apply_dead_zone(hand_dx, cfg.dead_zone)
    hand_dy = apply_dead_zone(hand_dy, cfg.dead_zone)

    # 3. Scale to robot metres
    #    hand_dy sign is inverted: image-y increases downward,
    #    but robot-z should increase upward.
    desired_dx =  cfg.gain_x * hand_dx
    desired_dz = -cfg.gain_y * hand_dy   # negative because image y ↓ = robot z ↓

    # 4. Per-update displacement cap
    step = math.hypot(desired_dx, desired_dz)
    if step > cfg.max_delta_m and step > 1e-9:
        scale    = cfg.max_delta_m / step
        desired_dx *= scale
        desired_dz *= scale

    # 5. New absolute target (accumulate deltas on last known target)
    new_x = last_target[0] + desired_dx
    new_y = last_target[1]                # Y is kept constant in MVP
    new_z = last_target[2] + desired_dz

    # 6. Clamp to workspace
    new_x = clamp(new_x, cfg.workspace_x_min, cfg.workspace_x_max)
    new_y = clamp(new_y, cfg.workspace_y_min, cfg.workspace_y_max)
    new_z = clamp(new_z, cfg.workspace_z_min, cfg.workspace_z_max)

    return new_x, new_y, new_z


def gesture_to_gripper(gesture: str) -> float:
    """
    Convert a gesture string to a normalised gripper opening [0.0, 1.0].
      PINCH   → 0.0 (closed)
      OPEN    → 1.0 (fully open)
      UNKNOWN → None (no change; caller should hold last value)
    """
    if gesture == "PINCH":
        return 0.0
    if gesture == "OPEN":
        return 1.0
    return None   # UNKNOWN — do not change gripper


def calibrate(
    hand_x: float,
    hand_y: float,
    current_target: Tuple[float, float, float],
    cfg: MappingConfig,
) -> CalibrationState:
    """
    Record current hand position and current robot target as the new anchor.
    This ensures no jump when the user recenters their hand.

    Args:
        hand_x          : Current hand x (already smoothed)
        hand_y          : Current hand y (already smoothed)
        current_target  : Current robot end-effector target (x, y, z) in metres
        cfg             : Mapping config (used to initialise if no previous target)

    Returns:
        A new CalibrationState.
    """
    return CalibrationState(
        hand_x0=hand_x,
        hand_y0=hand_y,
        robot_x0=current_target[0],
        robot_y0=current_target[1],
        robot_z0=current_target[2],
        calibrated=True,
    )
