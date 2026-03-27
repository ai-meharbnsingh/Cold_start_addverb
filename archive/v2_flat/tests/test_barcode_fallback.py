"""Tests for BarcodeFallbackHook — barcode failure state machine."""

import pytest
from fleet_integration.iogita_zone_node import ZoneIdentifier
from fleet_integration.barcode_fallback_hook import BarcodeFallbackHook
from fleet_integration.shared_config import default_test_config


class TestBarcodeFallbackHook:
    """Test barcode failure detection and recovery state machine."""

    @pytest.fixture
    def hook(self):
        config = default_test_config(5)
        zid = ZoneIdentifier(config)
        zid.build_network()
        return BarcodeFallbackHook(zid, fms_adapter=None)

    def test_initial_state_normal(self, hook):
        assert hook.state == BarcodeFallbackHook.NORMAL

    def test_success_stays_normal(self, hook):
        result = hook.on_barcode_read(True, barcode_id=5, row=0, col=0,
                                      robot_id="test")
        assert result["state"] == BarcodeFallbackHook.NORMAL

    def test_warning_threshold_triggers_degraded(self, hook):
        for i in range(3):
            result = hook.on_barcode_read(False, robot_id="test")
        assert result["state"] == BarcodeFallbackHook.DEGRADED

    def test_failure_threshold_triggers_fallback(self, hook):
        state = {
            "pose_x": 0, "pose_y": 0, "pose_theta": 0,
            "battery_soc": 70, "linear_vel": 0.5,
            "obstacle_range": 1.0, "obstacle_detected": False,
            "current_node": 1,
        }
        for i in range(5):
            result = hook.on_barcode_read(False, robot_state=state,
                                          robot_id="test")
        assert result["state"] == BarcodeFallbackHook.FALLBACK_ACTIVE
        assert result["action"] == "FALLBACK_ACTIVATED"
        assert hook.fallback_activations == 1

    def test_recovery_after_fallback(self, hook):
        state = {
            "pose_x": 0, "pose_y": 0, "pose_theta": 0,
            "battery_soc": 70, "linear_vel": 0.5,
            "obstacle_range": 1.0, "obstacle_detected": False,
            "current_node": 1,
        }
        # Trigger fallback
        for _ in range(5):
            hook.on_barcode_read(False, robot_state=state, robot_id="test")

        # First success -> RECOVERY
        result = hook.on_barcode_read(True, barcode_id=5, row=0, col=0,
                                      robot_id="test")
        assert result["state"] == BarcodeFallbackHook.RECOVERY

        # 3 consecutive successes -> NORMAL
        for _ in range(2):
            result = hook.on_barcode_read(True, barcode_id=6, row=0, col=0,
                                          robot_id="test")
        assert result["state"] == BarcodeFallbackHook.NORMAL
        assert result["action"] == "RETURNED_TO_NORMAL"

    def test_degraded_clears_on_success(self, hook):
        for _ in range(3):
            hook.on_barcode_read(False, robot_id="test")
        assert hook.state == BarcodeFallbackHook.DEGRADED

        result = hook.on_barcode_read(True, barcode_id=5, row=0, col=0,
                                      robot_id="test")
        assert result["state"] == BarcodeFallbackHook.NORMAL

    def test_status_report(self, hook):
        status = hook.get_status()
        assert "state" in status
        assert "consecutive_failures" in status
        assert "fallback_activations" in status
