"""Tests for ZoneIdentifier — core io-gita zone identification."""

import pytest
import numpy as np
from fleet_integration.iogita_zone_node import (
    ZoneIdentifier, extract_features_addverb, BarcodeGridTracker,
)
from fleet_integration.shared_config import default_test_config


class TestExtractFeatures:
    """Test feature extraction from Addverb sensor data."""

    def test_returns_16_features(self):
        features = extract_features_addverb(
            obstacle_range=1.0, obstacle_detected=False,
            heading_deg=90.0, dist_from_last_barcode=0.5,
            turns_since_barcode=1, velocity=0.8,
            last_barcode_row=5, last_barcode_col=10,
            battery_soc=70.0, time_since_barcode_s=2.0,
        )
        assert features.shape == (16,)

    def test_features_normalized_0_1(self):
        features = extract_features_addverb(
            obstacle_range=1.0, obstacle_detected=True,
            heading_deg=180.0, dist_from_last_barcode=1.0,
            turns_since_barcode=2, velocity=1.0,
            last_barcode_row=25, last_barcode_col=40,
            battery_soc=50.0, time_since_barcode_s=30.0,
        )
        assert np.all(features >= 0.0)
        assert np.all(features <= 1.0)

    def test_obstacle_detected_feature(self):
        detected = extract_features_addverb(
            obstacle_range=0.5, obstacle_detected=True,
            heading_deg=0, dist_from_last_barcode=0,
            turns_since_barcode=0, velocity=0,
            last_barcode_row=0, last_barcode_col=0,
            battery_soc=50, time_since_barcode_s=0,
        )
        not_detected = extract_features_addverb(
            obstacle_range=1.5, obstacle_detected=False,
            heading_deg=0, dist_from_last_barcode=0,
            turns_since_barcode=0, velocity=0,
            last_barcode_row=0, last_barcode_col=0,
            battery_soc=50, time_since_barcode_s=0,
        )
        assert detected[1] == 1.0  # obstacle_present
        assert not_detected[1] == 0.0


class TestBarcodeGridTracker:
    """Test barcode grid tracking."""

    def test_initial_state(self):
        tracker = BarcodeGridTracker()
        assert tracker.barcode_is_failing is False
        assert tracker.read_success_rate == 1.0

    def test_failure_threshold(self):
        tracker = BarcodeGridTracker()
        for _ in range(4):
            tracker.update_barcode_failure()
        assert tracker.barcode_is_failing is False
        tracker.update_barcode_failure()
        assert tracker.barcode_is_failing is True  # 5th failure

    def test_success_resets_failures(self):
        tracker = BarcodeGridTracker()
        for _ in range(4):
            tracker.update_barcode_failure()
        tracker.update_barcode_read(1, 1, 0.0, 0.0, 0.0)
        assert tracker.consecutive_failures == 0
        assert tracker.barcode_is_failing is False

    def test_success_rate(self):
        tracker = BarcodeGridTracker()
        tracker.update_barcode_read(1, 1, 0.0, 0.0, 0.0)  # success
        tracker.update_barcode_failure()  # fail
        assert tracker.read_success_rate == 0.5


class TestZoneIdentifier:
    """Test zone identification engine."""

    @pytest.fixture
    def zid(self):
        config = default_test_config(15)
        z = ZoneIdentifier(config)
        z.build_network()
        return z

    def test_build_network(self, zid):
        assert zid.net is not None
        assert zid.net.n_patterns > 0

    def test_adjacency_built(self, zid):
        assert len(zid.adjacency) > 0
        # HUB (2,2) should connect to SHELF_3 (2,1) and SHELF_4 (2,3)
        assert "SHELF_3" in zid.adjacency["HUB"]
        assert "SHELF_4" in zid.adjacency["HUB"]

    def test_barcode_to_zone_mapping(self, zid):
        assert len(zid.barcode_to_zone) > 0
        # First zone's first barcode should map correctly
        assert zid.barcode_to_zone[1] == "DOCK_A"

    def test_identify_from_barcode(self, zid):
        zone = zid.identify_from_barcode(1)
        assert zone == "DOCK_A"
        assert zid.last_zone == "DOCK_A"
        assert zid.last_zone_confidence == 1.0

    def test_identify_from_barcode_unknown(self, zid):
        zone = zid.identify_from_barcode(99999)
        assert zone is None

    def test_identify_from_sensors(self, zid):
        zid.barcode_tracker.last_barcode_row = 0
        zid.barcode_tracker.last_barcode_col = 0
        state = {
            "pose_x": 0.0, "pose_y": 0.0, "pose_theta": 1.57,
            "battery_soc": 70.0, "linear_vel": 0.0,
            "obstacle_range": 1.2, "obstacle_detected": False,
            "current_node": 1,
        }
        zone, method, conf, ode_ms = zid.identify_from_sensors(
            state, "zippy10")
        assert zone is not None
        assert isinstance(conf, float)
        assert 0.0 <= conf <= 1.0
        assert ode_ms >= 0.0

    def test_cold_start_hint(self, zid):
        zid.barcode_tracker.last_barcode_row = 2
        zid.barcode_tracker.last_barcode_col = 1
        zid.last_zone = "SHELF_3"
        state = {
            "pose_x": 10.0, "pose_y": 20.0, "pose_theta": 1.57,
            "battery_soc": 60.0, "linear_vel": 0.0,
            "obstacle_range": 1.0, "obstacle_detected": False,
            "current_node": 13,
        }
        hint = zid.get_cold_start_hint(state, "zippy10")
        assert "candidate_zones" in hint
        assert "primary_zone" in hint
        assert "confidence" in hint
        assert "nearest_barcodes" in hint
        assert len(hint["candidate_zones"]) > 0

    def test_save_and_load_state(self, zid, tmp_path):
        zid.config["cold_start"]["saved_state_file"] = str(
            tmp_path / "test_state.json")
        zid.last_zone = "HUB"
        zid.barcode_tracker.last_barcode_row = 2
        zid.barcode_tracker.last_barcode_col = 2
        zid.save_state()

        loaded = zid._load_last_state()
        assert loaded == "HUB"

    def test_load_corrupt_state_returns_none(self, zid, tmp_path):
        """Fix #3: Corrupt state file must not crash recovery."""
        state_file = tmp_path / "corrupt_state.json"
        state_file.write_text("{broken json!!! not valid")
        zid.config["cold_start"]["saved_state_file"] = str(state_file)
        result = zid._load_last_state()
        assert result is None

    def test_load_empty_state_returns_none(self, zid, tmp_path):
        state_file = tmp_path / "empty_state.json"
        state_file.write_text("")
        zid.config["cold_start"]["saved_state_file"] = str(state_file)
        result = zid._load_last_state()
        assert result is None

    def test_load_nondict_state_returns_none(self, zid, tmp_path):
        state_file = tmp_path / "array_state.json"
        state_file.write_text("[1, 2, 3]")
        zid.config["cold_start"]["saved_state_file"] = str(state_file)
        result = zid._load_last_state()
        assert result is None

    def test_load_missing_state_returns_none(self, zid, tmp_path):
        zid.config["cold_start"]["saved_state_file"] = str(
            tmp_path / "nonexistent.json")
        result = zid._load_last_state()
        assert result is None
