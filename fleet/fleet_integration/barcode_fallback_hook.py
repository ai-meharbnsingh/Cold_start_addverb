#!/usr/bin/env python
"""
Barcode Fallback Hook — Activates When Barcode Reader Fails
=============================================================

THIS IS THE FILE REFERENCED IN README AND INTEGRATION_GUIDE.

When barcode reader fails (5+ consecutive failures = irayple threshold),
this hook activates io-gita zone-level awareness so the robot can
navigate through dead barcode zones to the nearest working barcode.

Integration:
  - Monitors barcode read success/failure from TCP Protocol V1 telemetry
  - On failure threshold: activates io-gita sensor-based identification
  - Sends zone awareness to FMS so robot gets guided recovery route
  - Deactivates when barcode reads resume normally

Usage:
  Standalone:   python fleet_integration/barcode_fallback_hook.py
  As module:    from fleet_integration.barcode_fallback_hook import BarcodeFallbackHook

Contact: ai.meharbansingh@gmail.com
"""

import time
import json
import os
import sys
from typing import Optional

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fleet_integration.iogita_zone_node import ZoneIdentifier, BarcodeGridTracker
from fleet_integration.fms_adapter import FmsRestAdapter, ColdStartOrchestrator


class BarcodeFallbackHook:
    """
    Monitors barcode reader health and activates io-gita fallback
    when the reader degrades.

    State machine:
      NORMAL -> DEGRADED -> FALLBACK_ACTIVE -> RECOVERY -> NORMAL

    Transitions:
      NORMAL -> DEGRADED:       consecutive_failures >= warning_threshold (3)
      DEGRADED -> FALLBACK_ACTIVE: consecutive_failures >= failure_threshold (5)
      FALLBACK_ACTIVE -> RECOVERY: first successful barcode read
      RECOVERY -> NORMAL:        3 consecutive successful reads
    """

    # States
    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    FALLBACK_ACTIVE = "FALLBACK_ACTIVE"
    RECOVERY = "RECOVERY"

    def __init__(self, zone_identifier: ZoneIdentifier,
                 fms_adapter: Optional[FmsRestAdapter] = None,
                 failure_threshold: int = 5,
                 warning_threshold: int = 3,
                 recovery_reads: int = 3):
        """
        Args:
            zone_identifier: ZoneIdentifier instance for fallback identification
            fms_adapter: FMS REST adapter for sending alerts (None = log-only)
            failure_threshold: Consecutive failures before fallback activates
            warning_threshold: Consecutive failures before degraded warning
            recovery_reads: Consecutive successes to return to normal
        """
        self.zid = zone_identifier
        self.adapter = fms_adapter
        self.failure_threshold = failure_threshold
        self.warning_threshold = warning_threshold
        self.recovery_reads = recovery_reads

        self.state = self.NORMAL
        self.consecutive_failures = 0
        self.consecutive_successes = 0
        self.fallback_activations = 0
        self.total_failures = 0
        self.fallback_start_time = None
        self.log_entries = []

    def on_barcode_read(self, success: bool, barcode_id: int = 0,
                        row: int = 0, col: int = 0,
                        robot_state: dict = None,
                        robot_id: str = "unknown",
                        robot_type: str = "zippy10") -> dict:
        """
        Called on every barcode read attempt.

        Args:
            success: True if barcode was read successfully
            barcode_id: Barcode ID if success
            row, col: Grid position if success
            robot_state: Current robot telemetry (Protocol V1 parsed)
            robot_id: Robot identifier
            robot_type: Robot type key

        Returns:
            Status dict with current state and any actions taken
        """
        result = {"state": self.state, "action": None}

        if success:
            self.consecutive_failures = 0
            self.consecutive_successes += 1

            # Update zone identifier with valid read
            if barcode_id > 0:
                self.zid.identify_from_barcode(barcode_id)
                self.zid.barcode_tracker.update_barcode_read(
                    row, col,
                    robot_state.get("pose_x", 0) if robot_state else 0,
                    robot_state.get("pose_y", 0) if robot_state else 0,
                    robot_state.get("pose_theta", 0) if robot_state else 0,
                )

            # State transitions on success
            if self.state == self.FALLBACK_ACTIVE:
                self.state = self.RECOVERY
                result["action"] = "FALLBACK_DEACTIVATING"
                self._log(robot_id, "Barcode read resumed, entering recovery")

            elif self.state == self.RECOVERY:
                if self.consecutive_successes >= self.recovery_reads:
                    duration = time.time() - (self.fallback_start_time or 0)
                    self.state = self.NORMAL
                    result["action"] = "RETURNED_TO_NORMAL"
                    self._log(robot_id,
                              f"Barcode reader recovered after "
                              f"{duration:.1f}s fallback")

            elif self.state == self.DEGRADED:
                self.state = self.NORMAL
                result["action"] = "DEGRADATION_CLEARED"

        else:
            self.consecutive_failures += 1
            self.consecutive_successes = 0
            self.total_failures += 1
            self.zid.barcode_tracker.update_barcode_failure()

            # State transitions on failure
            if self.state == self.NORMAL:
                if self.consecutive_failures >= self.warning_threshold:
                    self.state = self.DEGRADED
                    result["action"] = "BARCODE_DEGRADED"
                    self._log(robot_id,
                              f"Barcode degraded: {self.consecutive_failures} "
                              f"consecutive failures")

            if self.state == self.DEGRADED:
                if self.consecutive_failures >= self.failure_threshold:
                    self.state = self.FALLBACK_ACTIVE
                    self.fallback_activations += 1
                    self.fallback_start_time = time.time()
                    result["action"] = "FALLBACK_ACTIVATED"
                    self._log(robot_id,
                              f"Fallback ACTIVATED: {self.consecutive_failures} "
                              f"failures, io-gita zone awareness enabled")

                    # Get zone guidance from io-gita
                    if robot_state:
                        hint = self.zid.get_barcode_failure_hint(
                            robot_state, robot_type)
                        result["zone_hint"] = hint

                        # Send to FMS
                        if self.adapter:
                            self.adapter.send_zone_update(
                                robot_id,
                                hint["current_zone"],
                                hint["confidence"],
                                "BARCODE_FALLBACK",
                            )

            # If fallback is active, keep providing zone updates
            if self.state == self.FALLBACK_ACTIVE and robot_state:
                zone, method, conf, ode_ms = self.zid.identify_from_sensors(
                    robot_state, robot_type)
                result["current_zone"] = zone
                result["confidence"] = conf
                result["method"] = method

                if self.adapter:
                    self.adapter.send_zone_update(
                        robot_id, zone, conf, f"FALLBACK_{method}")

        result["state"] = self.state
        result["consecutive_failures"] = self.consecutive_failures
        result["total_failures"] = self.total_failures
        result["fallback_activations"] = self.fallback_activations

        return result

    def get_status(self) -> dict:
        """Get current hook status."""
        return {
            "state": self.state,
            "consecutive_failures": self.consecutive_failures,
            "consecutive_successes": self.consecutive_successes,
            "total_failures": self.total_failures,
            "fallback_activations": self.fallback_activations,
            "fallback_active_since": self.fallback_start_time,
        }

    def _log(self, robot_id: str, message: str):
        entry = {
            "time": time.time(),
            "robot_id": robot_id,
            "state": self.state,
            "msg": message,
        }
        self.log_entries.append(entry)
        print(f"  [barcode-hook] {robot_id}: {message}")


# =====================================================================
# STANDALONE DEMO
# =====================================================================

def demo():
    """Demonstrate barcode fallback hook without fleet_core."""
    import numpy as np

    print("=" * 60)
    print("  Barcode Fallback Hook — Demo")
    print("  Simulates barcode degradation and io-gita recovery")
    print("=" * 60)

    # Build zone identifier
    zone_defs = [
        ("DOCK_A", 0, 0, "dock"), ("AISLE_1", 0, 1, "aisle"),
        ("SHELF_1", 1, 1, "shelf"), ("HUB", 1, 0, "hub"),
        ("SHELF_2", 1, 2, "shelf"),
    ]
    config = {
        "warehouse": {"grid_spacing_m": 0.8, "max_rows": 10, "max_cols": 10},
        "zones": [
            {"name": n, "row": r, "col": c, "type": t,
             "expected_heading": 90,
             "barcode_range": [i * 10 + 1, (i + 1) * 10],
             "graph_nodes": list(range(i * 3 + 1, (i + 1) * 3 + 1)),
             "center_x": c * 5, "center_y": r * 5,
             "has_charger": t == "dock"}
            for i, (n, r, c, t) in enumerate(zone_defs)
        ],
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                    "seed": 42, "generated_patterns": 15},
        "cold_start": {"saved_state_file": "/tmp/iogita_last_state.json",
                       "confidence_threshold": 0.6, "max_hint_zones": 3,
                       "teleport_confidence": 0.3,
                       "recovery_strategy": "nearest_barcode"},
        "map_change": {"enabled": True, "mismatch_threshold": 3,
                       "feature_tolerance": 0.3},
        "adjacency_overrides": [],
        "robot_types": {"zippy10": {"max_velocity": 1.4}},
    }

    zid = ZoneIdentifier(config)
    zid.build_network()
    hook = BarcodeFallbackHook(zid, fms_adapter=None)

    rng = np.random.default_rng(42)
    robot_state = {
        "pose_x": 5.0, "pose_y": 5.0, "pose_theta": 1.57,
        "battery_soc": 70.0, "linear_vel": 0.8,
        "obstacle_range": 1.2, "obstacle_detected": False,
        "current_node": 4,
    }

    # Phase 1: Normal reads
    print(f"\n-- Phase 1: Normal barcode reads --\n")
    for i in range(3):
        r = hook.on_barcode_read(True, barcode_id=15 + i, row=1, col=1,
                                 robot_state=robot_state, robot_id="zippy10_3")
        print(f"  Read {i+1}: OK -> state={r['state']}")

    # Phase 2: Failures begin
    print(f"\n-- Phase 2: Barcode failures begin --\n")
    for i in range(6):
        r = hook.on_barcode_read(False, robot_state=robot_state,
                                 robot_id="zippy10_3")
        print(f"  Fail {i+1}: state={r['state']} "
              f"failures={r['consecutive_failures']}"
              f"{' ACTION=' + r['action'] if r.get('action') else ''}")
        if r.get("zone_hint"):
            print(f"           zone={r['zone_hint']['current_zone']} "
                  f"conf={r['zone_hint']['confidence']:.2f}")

    # Phase 3: Barcode reads resume
    print(f"\n-- Phase 3: Barcode reads resume --\n")
    for i in range(4):
        r = hook.on_barcode_read(True, barcode_id=18 + i, row=1, col=2,
                                 robot_state=robot_state, robot_id="zippy10_3")
        print(f"  Read {i+1}: OK -> state={r['state']}"
              f"{' ACTION=' + r['action'] if r.get('action') else ''}")

    # Summary
    status = hook.get_status()
    print(f"\n-- Summary --")
    print(f"  Final state:          {status['state']}")
    print(f"  Total failures:       {status['total_failures']}")
    print(f"  Fallback activations: {status['fallback_activations']}")
    print(f"\n  Done.")


if __name__ == "__main__":
    demo()
