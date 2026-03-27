"""Tests for StructuredLogger — fleet-wide JSON logging."""

import json
import logging
import os
import pytest
from fleet_integration.structured_logger import StructuredLogger


class TestStructuredLogger:

    @pytest.fixture(autouse=True)
    def _clear_loggers(self):
        """Clear logging handlers between tests to prevent cross-pollution."""
        yield
        for name in ("iogita.zones", "iogita.events"):
            lg = logging.getLogger(name)
            lg.handlers.clear()

    @pytest.fixture
    def logger(self, tmp_path):
        config = {
            "logging": {
                "enabled": True,
                "log_dir": str(tmp_path),
                "zone_hints_log": "zone_hints.jsonl",
                "events_log": "events.jsonl",
                "max_file_size_mb": 1,
                "max_files": 2,
            }
        }
        return StructuredLogger(config), tmp_path

    def test_zone_hint_logged(self, logger):
        log, path = logger
        log.log_zone_hint("zippy10_3", "SHELF_3", 0.9, "GRAPH_FP_RANKED",
                          0.8, battery_soc=68.0, velocity=0.0)
        log_file = path / "zone_hints.jsonl"
        assert log_file.exists()
        line = log_file.read_text().strip()
        data = json.loads(line)
        assert data["event"] == "ZONE_HINT"
        assert data["robot_id"] == "zippy10_3"
        assert data["zone"] == "SHELF_3"
        assert data["confidence"] == 0.9
        assert data["ode_ms"] == 0.8

    def test_cold_start_logged(self, logger):
        log, path = logger
        log.log_cold_start("amr500_1", "DOCK_A", 0.85, "COLD_FP_ONLY",
                           1.2, 3.5, candidates=["DOCK_A", "AISLE_1"],
                           saved_zone="DOCK_A")
        data = json.loads((path / "events.jsonl").read_text().strip())
        assert data["event"] == "COLD_START"
        assert data["robot_id"] == "amr500_1"
        assert data["total_ms"] == 3.5
        assert "DOCK_A" in data["candidates"]

    def test_cold_start_confirmed_logged(self, logger):
        log, path = logger
        log.log_cold_start_confirmed("zippy10_3", 55, "SHELF_3", 1.3)
        data = json.loads((path / "events.jsonl").read_text().strip())
        assert data["event"] == "COLD_START_CONFIRMED"
        assert data["recovery_time_s"] == 1.3
        assert data["barcode_id"] == 55

    def test_barcode_fallback_logged(self, logger):
        log, path = logger
        log.log_barcode_fallback("zippy10_3", "FALLBACK_ACTIVE", 5,
                                 zone="SHELF_5", confidence=0.85)
        data = json.loads((path / "events.jsonl").read_text().strip())
        assert data["event"] == "BARCODE_FALLBACK"
        assert data["consecutive_failures"] == 5

    def test_map_change_logged(self, logger):
        log, path = logger
        log.log_map_change("SHELF_3", "layout_modified", 0.45, 1.0)
        data = json.loads((path / "events.jsonl").read_text().strip())
        assert data["event"] == "MAP_CHANGE"
        assert data["zone"] == "SHELF_3"

    def test_disabled_logger_no_crash(self):
        log = StructuredLogger({"logging": {"enabled": False}})
        log.log_zone_hint("test", "Z", 0.5, "M", 1.0)
        log.log_cold_start("test", "Z", 0.5, "M", 1.0, 2.0)
        log.log_barcode_fallback("test", "NORMAL", 0)

    def test_none_zone_logged(self, logger):
        log, path = logger
        log.log_zone_hint("zippy10_3", None, 0.0, "NO_NETWORK", 0.0)
        data = json.loads((path / "zone_hints.jsonl").read_text().strip())
        assert data["zone"] is None
