"""
tests/test_mapping.py
=====================
Coordinate mapping unit tests — Person 3

Tests for:
  - dead zone behaviour
  - hand delta → robot delta signs and magnitudes
  - per-update displacement cap
  - workspace bounds clamping
  - calibration anchor (no jump on recalibrate)
  - gesture-to-gripper conversion

Run:
    python -m pytest tests/test_mapping.py -v
"""

from __future__ import annotations

import math
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "control"))

from mapping import (
    MappingConfig,
    CalibrationState,
    map_hand_to_robot,
    gesture_to_gripper,
    calibrate,
    apply_dead_zone,
    clamp,
)


@pytest.fixture
def default_cfg() -> MappingConfig:
    return MappingConfig(
        gain_x=0.4,
        gain_y=0.4,
        dead_zone=0.02,
        max_delta_m=0.02,
        workspace_x_min=0.2, workspace_x_max=0.8,
        workspace_y_min=0.0, workspace_y_max=0.0,
        workspace_z_min=0.1, workspace_z_max=0.7,
        robot_home_x=0.5, robot_home_y=0.0, robot_home_z=0.4,
    )


@pytest.fixture
def centered_calib() -> CalibrationState:
    """Calibration with hand at image centre, robot at home."""
    return CalibrationState(
        hand_x0=0.5, hand_y0=0.5,
        robot_x0=0.5, robot_y0=0.0, robot_z0=0.4,
        calibrated=True,
    )


# ---------------------------------------------------------------------------
# Dead zone
# ---------------------------------------------------------------------------

class TestDeadZone:

    def test_within_dead_zone_returns_zero(self):
        assert apply_dead_zone(0.01, 0.02) == 0.0
        assert apply_dead_zone(-0.01, 0.02) == 0.0

    def test_exactly_at_boundary_returns_zero(self):
        assert apply_dead_zone(0.02, 0.02) == 0.0

    def test_outside_dead_zone_non_zero(self):
        result = apply_dead_zone(0.05, 0.02)
        assert result > 0.0

    def test_dead_zone_shifts_outside_value(self):
        # Outside value should be reduced by dead_zone amount
        result = apply_dead_zone(0.05, 0.02)
        assert abs(result - 0.03) < 1e-9

    def test_negative_outside(self):
        result = apply_dead_zone(-0.05, 0.02)
        assert result < 0.0
        assert abs(result - (-0.03)) < 1e-9


# ---------------------------------------------------------------------------
# Calibration anchor
# ---------------------------------------------------------------------------

class TestCalibration:

    def test_calibrate_records_hand_and_robot(self, default_cfg):
        target = (0.55, 0.0, 0.42)
        calib = calibrate(0.6, 0.4, target, default_cfg)
        assert calib.hand_x0  == pytest.approx(0.6)
        assert calib.hand_y0  == pytest.approx(0.4)
        assert calib.robot_x0 == pytest.approx(0.55)
        assert calib.robot_z0 == pytest.approx(0.42)
        assert calib.calibrated is True

    def test_recalibrate_no_jump(self, default_cfg, centered_calib):
        """
        After recalibrating at the same hand position, the next
        map_hand_to_robot call with the same hand position should produce
        a target equal to the calibrated robot anchor.
        """
        current_target = (0.6, 0.0, 0.5)
        hand_x, hand_y = 0.6, 0.3
        new_calib = calibrate(hand_x, hand_y, current_target, default_cfg)

        # Map with exactly the calibrated position → delta = 0 → no movement
        result = map_hand_to_robot(hand_x, hand_y, new_calib, default_cfg, current_target)
        # With zero delta, last_target should be returned (possibly clamped)
        assert result[0] == pytest.approx(current_target[0], abs=0.001)
        assert result[2] == pytest.approx(current_target[2], abs=0.001)


# ---------------------------------------------------------------------------
# Direction tests
# ---------------------------------------------------------------------------

class TestDirections:

    def test_hand_right_moves_robot_positive_x(self, default_cfg, centered_calib):
        """Moving hand right (Δx > 0) should increase robot X."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(0.6, 0.5, centered_calib, default_cfg, last)
        assert result[0] > last[0], "Moving hand right should increase robot X"

    def test_hand_left_moves_robot_negative_x(self, default_cfg, centered_calib):
        """Moving hand left (Δx < 0) should decrease robot X."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(0.4, 0.5, centered_calib, default_cfg, last)
        assert result[0] < last[0], "Moving hand left should decrease robot X"

    def test_hand_up_moves_robot_positive_z(self, default_cfg, centered_calib):
        """Moving hand up (Δy < 0 in image space) should increase robot Z."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(0.5, 0.4, centered_calib, default_cfg, last)
        # y decreased from 0.5 → 0.4 (hand moved up in image)
        assert result[2] > last[2], "Moving hand up should increase robot Z"

    def test_hand_down_moves_robot_negative_z(self, default_cfg, centered_calib):
        """Moving hand down (Δy > 0 in image space) should decrease robot Z."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(0.5, 0.6, centered_calib, default_cfg, last)
        assert result[2] < last[2], "Moving hand down should decrease robot Z"

    def test_y_held_constant(self, default_cfg, centered_calib):
        """Robot Y must remain constant in MVP."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(0.7, 0.3, centered_calib, default_cfg, last)
        assert result[1] == pytest.approx(0.0)

    def test_anchor_maps_to_zero_delta(self, default_cfg, centered_calib):
        """Hand exactly at anchor → target unchanged (within dead zone)."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(
            centered_calib.hand_x0,
            centered_calib.hand_y0,
            centered_calib, default_cfg, last
        )
        # Both deltas are 0 → no movement
        assert result[0] == pytest.approx(last[0], abs=1e-6)
        assert result[2] == pytest.approx(last[2], abs=1e-6)


# ---------------------------------------------------------------------------
# Per-update displacement cap
# ---------------------------------------------------------------------------

class TestDisplacementCap:

    def test_large_hand_delta_is_capped(self, default_cfg, centered_calib):
        """A very large hand displacement must never produce a step > max_delta_m."""
        last = (0.5, 0.0, 0.4)
        result = map_hand_to_robot(1.0, 0.0, centered_calib, default_cfg, last)
        dx = result[0] - last[0]
        dz = result[2] - last[2]
        step = math.hypot(dx, dz)
        assert step <= default_cfg.max_delta_m + 1e-9


# ---------------------------------------------------------------------------
# Workspace bounds
# ---------------------------------------------------------------------------

class TestWorkspaceBounds:

    def test_x_lower_bound_clamped(self, default_cfg, centered_calib):
        last = (default_cfg.workspace_x_min, 0.0, 0.4)
        result = map_hand_to_robot(0.0, 0.5, centered_calib, default_cfg, last)
        assert result[0] >= default_cfg.workspace_x_min

    def test_x_upper_bound_clamped(self, default_cfg, centered_calib):
        last = (default_cfg.workspace_x_max, 0.0, 0.4)
        # Move further right — should be clamped
        result = map_hand_to_robot(1.0, 0.5, centered_calib, default_cfg, last)
        assert result[0] <= default_cfg.workspace_x_max

    def test_z_lower_bound_clamped(self, default_cfg, centered_calib):
        last = (0.5, 0.0, default_cfg.workspace_z_min)
        result = map_hand_to_robot(0.5, 1.0, centered_calib, default_cfg, last)
        assert result[2] >= default_cfg.workspace_z_min

    def test_z_upper_bound_clamped(self, default_cfg, centered_calib):
        last = (0.5, 0.0, default_cfg.workspace_z_max)
        result = map_hand_to_robot(0.5, 0.0, centered_calib, default_cfg, last)
        assert result[2] <= default_cfg.workspace_z_max


# ---------------------------------------------------------------------------
# Gesture to gripper
# ---------------------------------------------------------------------------

class TestGestureToGripper:

    def test_pinch_returns_zero(self):
        assert gesture_to_gripper("PINCH") == pytest.approx(0.0)

    def test_open_returns_one(self):
        assert gesture_to_gripper("OPEN") == pytest.approx(1.0)

    def test_unknown_returns_none(self):
        assert gesture_to_gripper("UNKNOWN") is None

    def test_gripper_in_range(self):
        for g in ("PINCH", "OPEN"):
            val = gesture_to_gripper(g)
            if val is not None:
                assert 0.0 <= val <= 1.0


# ---------------------------------------------------------------------------
# Utility clamp
# ---------------------------------------------------------------------------

class TestClamp:

    def test_within_range(self):
        assert clamp(0.5, 0.0, 1.0) == 0.5

    def test_below_range(self):
        assert clamp(-1.0, 0.0, 1.0) == 0.0

    def test_above_range(self):
        assert clamp(2.0, 0.0, 1.0) == 1.0
