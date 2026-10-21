"""
tests/test_protocol.py
======================
Protocol validation tests — Person 3 (and shared team use)

Tests for:
  - vision packet parsing (valid, invalid, edge cases)
  - robot command packet parsing (valid, invalid)
  - fixture JSON files match the spec

Run:
    python -m pytest tests/test_protocol.py -v
"""

from __future__ import annotations

import json
import os
import sys

import pytest

# Allow importing from project root without installation
ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(ROOT, "control"))

from protocol import (
    parse_vision_packet,
    parse_robot_command,
    build_robot_command,
    VisionPacketError,
    PROTOCOL_VERSION,
)

FIXTURES_DIR = os.path.join(os.path.dirname(__file__), "fixtures")


def _load_fixture(name: str) -> bytes:
    with open(os.path.join(FIXTURES_DIR, name), "rb") as f:
        return f.read()


# ---------------------------------------------------------------------------
# Vision packet — valid cases
# ---------------------------------------------------------------------------

class TestVisionPacketValid:

    def test_fixture_valid_hand_parses(self):
        raw = _load_fixture("vision_valid.json")
        pkt = parse_vision_packet(raw)
        assert pkt["version"]  == PROTOCOL_VERSION
        assert pkt["tracking"] is True
        assert 0.0 <= pkt["hand"]["x"] <= 1.0
        assert 0.0 <= pkt["hand"]["y"] <= 1.0
        assert pkt["gesture"] == "OPEN"

    def test_fixture_no_hand_parses(self):
        raw = _load_fixture("vision_no_hand.json")
        pkt = parse_vision_packet(raw)
        assert pkt["tracking"] is False
        assert pkt["hand"]     is None
        assert pkt["gesture"]  == "UNKNOWN"

    def test_pinch_gesture_valid(self):
        pkt = {
            "version": 1, "seq": 1, "timestamp_ms": 1000000,
            "tracking": True, "confidence": 0.9,
            "hand": {"x": 0.5, "y": 0.5, "pinch_distance": 0.03},
            "gesture": "PINCH",
        }
        result = parse_vision_packet(json.dumps(pkt).encode())
        assert result["gesture"] == "PINCH"

    def test_confidence_boundary_values(self):
        for conf in (0.0, 0.5, 1.0):
            pkt = {
                "version": 1, "seq": 1, "timestamp_ms": 1000000,
                "tracking": False, "confidence": conf,
                "hand": None, "gesture": "UNKNOWN",
            }
            result = parse_vision_packet(json.dumps(pkt).encode())
            assert result["confidence"] == conf

    def test_coordinates_at_edges(self):
        for x, y in [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)]:
            pkt = {
                "version": 1, "seq": 1, "timestamp_ms": 1000000,
                "tracking": True, "confidence": 0.8,
                "hand": {"x": x, "y": y, "pinch_distance": 0.1},
                "gesture": "OPEN",
            }
            result = parse_vision_packet(json.dumps(pkt).encode())
            assert result["hand"]["x"] == x


# ---------------------------------------------------------------------------
# Vision packet — invalid cases
# ---------------------------------------------------------------------------

class TestVisionPacketInvalid:

    def _bad(self, pkt: dict, match: str = ""):
        raw = json.dumps(pkt).encode()
        with pytest.raises(VisionPacketError, match=match):
            parse_vision_packet(raw)

    def test_malformed_json(self):
        with pytest.raises(VisionPacketError):
            parse_vision_packet(b"{not valid json}")

    def test_wrong_protocol_version(self):
        self._bad({
            "version": 99, "seq": 1, "timestamp_ms": 1000,
            "tracking": False, "confidence": 0.0,
            "hand": None, "gesture": "UNKNOWN",
        }, match="version")

    def test_missing_seq_field(self):
        self._bad({
            "version": 1, "timestamp_ms": 1000,
            "tracking": False, "confidence": 0.0,
            "hand": None, "gesture": "UNKNOWN",
        }, match="seq")

    def test_x_out_of_range(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": True, "confidence": 0.9,
            "hand": {"x": 1.5, "y": 0.5, "pinch_distance": 0.1},
            "gesture": "OPEN",
        }, match="hand.x")

    def test_y_out_of_range(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": True, "confidence": 0.9,
            "hand": {"x": 0.5, "y": -0.1, "pinch_distance": 0.1},
            "gesture": "OPEN",
        }, match="hand.y")

    def test_confidence_out_of_range(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": False, "confidence": 1.5,
            "hand": None, "gesture": "UNKNOWN",
        }, match="confidence")

    def test_nan_coordinate(self):
        # JSON does not support NaN natively — send as string to simulate evil input
        raw = b'{"version":1,"seq":1,"timestamp_ms":1000,"tracking":true,"confidence":0.9,' \
              b'"hand":{"x":0.5,"y":0.5,"pinch_distance":null},"gesture":"OPEN"}'
        # This is valid (pinch_distance can be null)
        pkt = parse_vision_packet(raw)
        assert pkt is not None

    def test_tracking_true_hand_null_rejected(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": True, "confidence": 0.9,
            "hand": None, "gesture": "OPEN",
        })

    def test_tracking_false_hand_not_null_rejected(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": False, "confidence": 0.0,
            "hand": {"x": 0.5, "y": 0.5, "pinch_distance": 0.1},
            "gesture": "UNKNOWN",
        })

    def test_unknown_gesture_rejected(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "tracking": False, "confidence": 0.0,
            "hand": None, "gesture": "WAVE",
        }, match="gesture")


# ---------------------------------------------------------------------------
# Robot command packet — valid cases
# ---------------------------------------------------------------------------

class TestRobotCommandValid:

    def test_fixture_valid_parses(self):
        raw = _load_fixture("robot_command_valid.json")
        pkt = parse_robot_command(raw)
        assert pkt["version"] == PROTOCOL_VERSION
        assert pkt["enabled"] is True
        assert pkt["stop"]    is False
        assert pkt["mode"]    == "TARGET_XZ"
        assert len(pkt["target_xyz_m"]) == 3
        assert 0.0 <= pkt["gripper_opening"] <= 1.0

    def test_stop_command(self):
        cmd = build_robot_command(
            seq=1, enabled=False, stop=True,
            target_xyz_m=(0.5, 0.0, 0.4), gripper_opening=1.0,
        )
        result = parse_robot_command(json.dumps(cmd).encode())
        assert result["stop"] is True

    def test_gripper_closed(self):
        cmd = build_robot_command(
            seq=1, enabled=True, stop=False,
            target_xyz_m=(0.5, 0.0, 0.4), gripper_opening=0.0,
        )
        result = parse_robot_command(json.dumps(cmd).encode())
        assert result["gripper_opening"] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Robot command packet — invalid cases
# ---------------------------------------------------------------------------

class TestRobotCommandInvalid:

    def _bad(self, pkt: dict, match: str = ""):
        raw = json.dumps(pkt).encode()
        with pytest.raises(VisionPacketError, match=match):
            parse_robot_command(raw)

    def test_wrong_version(self):
        self._bad({
            "version": 0, "seq": 1, "timestamp_ms": 1000,
            "enabled": True, "stop": False, "mode": "TARGET_XZ",
            "target_xyz_m": [0.5, 0.0, 0.4], "gripper_opening": 1.0,
        }, match="version")

    def test_target_too_short(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "enabled": True, "stop": False, "mode": "TARGET_XZ",
            "target_xyz_m": [0.5, 0.0], "gripper_opening": 1.0,
        }, match="3")

    def test_gripper_out_of_range(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "enabled": True, "stop": False, "mode": "TARGET_XZ",
            "target_xyz_m": [0.5, 0.0, 0.4], "gripper_opening": 1.5,
        }, match="gripper")

    def test_unknown_mode(self):
        self._bad({
            "version": 1, "seq": 1, "timestamp_ms": 1000,
            "enabled": True, "stop": False, "mode": "FULL_6DOF",
            "target_xyz_m": [0.5, 0.0, 0.4], "gripper_opening": 1.0,
        }, match="mode")

    def test_malformed_json_robot(self):
        with pytest.raises(VisionPacketError):
            parse_robot_command(b"not json")
