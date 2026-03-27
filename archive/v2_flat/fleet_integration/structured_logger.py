"""
Structured JSON Logger for Fleet-Wide Analysis
================================================

Every zone identification, cold start, fallback activation, and map change
is logged as a JSON line for post-mortem analysis across the fleet.

Log format (one JSON object per line):
{
    "ts": 1711545200.5,
    "event": "ZONE_HINT",
    "robot_id": "zippy10_3",
    "zone": "SHELF_3",
    "confidence": 0.90,
    "method": "GRAPH_FP_RANKED",
    "ode_ms": 0.8,
    "barcode_status": "OK",
    "battery_soc": 68.0,
    "velocity": 0.0
}

Analysis use cases:
- Misidentification trends: grep for confidence < 0.5
- Zone hotspots: count events per zone
- Recovery time distribution: filter COLD_START events
- Barcode failure zones: filter BARCODE_FALLBACK events
- Fleet health: aggregate across all robot_id values

Contact: ai.meharbansingh@gmail.com
"""

import json
import os
import time
from typing import Optional
from logging.handlers import RotatingFileHandler
import logging


class StructuredLogger:
    """JSON-lines logger with rotation for fleet-wide analysis."""

    def __init__(self, config: dict):
        """
        Args:
            config: Full warehouse config dict. Reads 'logging' section.
        """
        log_cfg = config.get("logging", {})
        self.enabled = log_cfg.get("enabled", True)

        if not self.enabled:
            self._zone_logger = None
            self._event_logger = None
            return

        log_dir = log_cfg.get("log_dir", "/var/lib/iogita/logs")
        zone_file = log_cfg.get("zone_hints_log", "zone_hints.jsonl")
        events_file = log_cfg.get("events_log", "events.jsonl")
        max_bytes = log_cfg.get("max_file_size_mb", 50) * 1024 * 1024
        backup_count = log_cfg.get("max_files", 10)

        # Create log directory (with fallback to /tmp if permission denied)
        try:
            os.makedirs(log_dir, exist_ok=True)
        except PermissionError:
            log_dir = "/tmp/iogita_logs"
            os.makedirs(log_dir, exist_ok=True)

        self._zone_logger = self._build_logger(
            "iogita.zones",
            os.path.join(log_dir, zone_file),
            max_bytes, backup_count,
        )
        self._event_logger = self._build_logger(
            "iogita.events",
            os.path.join(log_dir, events_file),
            max_bytes, backup_count,
        )

    @staticmethod
    def _build_logger(name: str, filepath: str,
                      max_bytes: int, backup_count: int) -> logging.Logger:
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        logger.propagate = False
        if not logger.handlers:
            handler = RotatingFileHandler(
                filepath,
                maxBytes=max_bytes,
                backupCount=backup_count,
            )
            handler.setFormatter(logging.Formatter("%(message)s"))
            logger.addHandler(handler)
        return logger

    def log_zone_hint(self, robot_id: str, zone: Optional[str],
                      confidence: float, method: str,
                      ode_ms: float, barcode_status: str = "OK",
                      battery_soc: float = 0.0,
                      velocity: float = 0.0,
                      extra: dict = None) -> None:
        """Log a zone identification event."""
        if not self.enabled or not self._zone_logger:
            return
        entry = {
            "ts": time.time(),
            "event": "ZONE_HINT",
            "robot_id": robot_id,
            "zone": zone,
            "confidence": round(confidence, 3),
            "method": method,
            "ode_ms": round(ode_ms, 2),
            "barcode_status": barcode_status,
            "battery_soc": round(battery_soc, 1),
            "velocity": round(velocity, 2),
        }
        if extra:
            entry.update(extra)
        self._zone_logger.info(json.dumps(entry))

    def log_cold_start(self, robot_id: str, zone: Optional[str],
                       confidence: float, method: str,
                       ode_ms: float, total_ms: float,
                       candidates: list = None,
                       saved_zone: str = None) -> None:
        """Log a cold start recovery event."""
        if not self.enabled or not self._event_logger:
            return
        entry = {
            "ts": time.time(),
            "event": "COLD_START",
            "robot_id": robot_id,
            "zone": zone,
            "confidence": round(confidence, 3),
            "method": method,
            "ode_ms": round(ode_ms, 2),
            "total_ms": round(total_ms, 2),
            "candidates": candidates or [],
            "saved_zone": saved_zone,
        }
        self._event_logger.info(json.dumps(entry))

    def log_cold_start_confirmed(self, robot_id: str,
                                 barcode_id: int,
                                 zone: str,
                                 recovery_time_s: float) -> None:
        """Log cold start recovery completion."""
        if not self.enabled or not self._event_logger:
            return
        entry = {
            "ts": time.time(),
            "event": "COLD_START_CONFIRMED",
            "robot_id": robot_id,
            "barcode_id": barcode_id,
            "zone": zone,
            "recovery_time_s": round(recovery_time_s, 2),
        }
        self._event_logger.info(json.dumps(entry))

    def log_barcode_fallback(self, robot_id: str,
                             state: str,
                             consecutive_failures: int,
                             zone: Optional[str] = None,
                             confidence: float = 0.0) -> None:
        """Log barcode failure / fallback state change."""
        if not self.enabled or not self._event_logger:
            return
        entry = {
            "ts": time.time(),
            "event": "BARCODE_FALLBACK",
            "robot_id": robot_id,
            "state": state,
            "consecutive_failures": consecutive_failures,
            "zone": zone,
            "confidence": round(confidence, 3),
        }
        self._event_logger.info(json.dumps(entry))

    def log_map_change(self, zone: str, change_type: str,
                       distance: float,
                       confidence: float) -> None:
        """Log map change detection."""
        if not self.enabled or not self._event_logger:
            return
        entry = {
            "ts": time.time(),
            "event": "MAP_CHANGE",
            "zone": zone,
            "change_type": change_type,
            "distance": round(distance, 3),
            "confidence": round(confidence, 3),
        }
        self._event_logger.info(json.dumps(entry))
