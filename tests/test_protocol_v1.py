"""Tests for ProtocolV1Parser — Addverb TCP message parsing."""

import pytest
from fleet_integration.iogita_zone_node import ProtocolV1Parser


class TestProtocolV1Parser:
    """Test Protocol V1 message parsing with validation."""

    VALID_MSG = (
        "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|68.0|0.8|0.02|0|"
        "TASK_42|13|1.1|0"
    )

    def test_parse_valid_message(self):
        result = ProtocolV1Parser.parse(self.VALID_MSG)
        assert result is not None
        assert result["robot_id"] == "zippy10_3"
        assert result["pose_x"] == 15.2
        assert result["pose_y"] == 24.8
        assert abs(result["pose_theta"] - 1.55) < 0.001
        assert result["state"] == "MOVING"
        assert result["battery_soc"] == 68.0
        assert result["linear_vel"] == 0.8
        assert result["current_node"] == 13
        assert result["obstacle_range"] == 1.1
        assert result["obstacle_detected"] is False

    def test_parse_obstacle_detected(self):
        msg = self.VALID_MSG[:-1] + "1"  # Change last field to "1"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["obstacle_detected"] is True

    def test_parse_empty_message(self):
        assert ProtocolV1Parser.parse("") is None
        assert ProtocolV1Parser.parse("   ") is None

    def test_parse_none(self):
        assert ProtocolV1Parser.parse(None) is None

    def test_parse_too_few_fields(self):
        assert ProtocolV1Parser.parse("1.0|robot|1.0") is None

    def test_parse_invalid_float(self):
        msg = "1711545200.5|zippy10_3|INVALID|24.8|1.55|MOVING|68.0|0.8|0.02|0|TASK|13|1.1|0"
        assert ProtocolV1Parser.parse(msg) is None

    def test_battery_clamped_high(self):
        msg = "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|999.0|0.8|0.02|0|TASK|13|1.1|0"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["battery_soc"] == 100.0  # Clamped to max

    def test_battery_clamped_low(self):
        msg = "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|-50.0|0.8|0.02|0|TASK|13|1.1|0"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["battery_soc"] == 0.0  # Clamped to min

    def test_velocity_clamped(self):
        msg = "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|68.0|99.0|0.02|0|TASK|13|1.1|0"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["linear_vel"] == 5.0  # Clamped to max

    def test_obstacle_range_clamped(self):
        msg = "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|68.0|0.8|0.02|0|TASK|13|999.0|0"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["obstacle_range"] == 50.0  # Clamped

    def test_empty_robot_id_rejected(self):
        msg = "1711545200.5| |15.2|24.8|1.55|MOVING|68.0|0.8|0.02|0|TASK|13|1.1|0"
        assert ProtocolV1Parser.parse(msg) is None

    def test_zero_timestamp_rejected(self):
        msg = "0|zippy10_3|15.2|24.8|1.55|MOVING|68.0|0.8|0.02|0|TASK|13|1.1|0"
        assert ProtocolV1Parser.parse(msg) is None

    def test_extra_fields_ok(self):
        msg = self.VALID_MSG + "|extra1|extra2|extra3"
        result = ProtocolV1Parser.parse(msg)
        assert result is not None
        assert result["robot_id"] == "zippy10_3"
