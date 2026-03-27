#!/usr/bin/env python
"""
io-gita Cold Start Demo — Simple (Addverb Fleet_Core Aligned)
===============================================================

Simple 5-zone demo showing cold start recovery using Addverb's
actual sensor suite: +-15deg obstacle sensor + barcode grid + odometry.

Run: python cold_start_aliasing/cold_start_demo.py

Contact: ai.meharbansingh@gmail.com
"""

import sys
import os
import time
import json
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sg_engine.network import Network
from fleet_integration.iogita_zone_node import (
    ZoneIdentifier, extract_features_addverb, ZONE_TYPE_CODES,
)


def main():
    print("=" * 60)
    print("  io-gita Cold Start — Simple Demo (5 Zones)")
    print("  Aligned to Addverb fleet_core")
    print("=" * 60)

    # 5-zone mini warehouse
    zone_defs = [
        ("DOCK_A",  0, 0, "dock"),
        ("AISLE_1", 0, 1, "aisle"),
        ("SHELF_1", 1, 1, "shelf"),
        ("HUB",     1, 0, "hub"),
        ("CHARGING", 1, 2, "charging"),
    ]

    config = {
        "warehouse": {"grid_spacing_m": 0.8, "max_rows": 10, "max_cols": 10},
        "zones": [
            {"name": n, "row": r, "col": c, "type": t,
             "expected_heading": 90,
             "barcode_range": [i * 10 + 1, (i + 1) * 10],
             "graph_nodes": list(range(i * 3 + 1, (i + 1) * 3 + 1)),
             "center_x": c * 5, "center_y": r * 5,
             "has_charger": t in ("dock", "charging")}
            for i, (n, r, c, t) in enumerate(zone_defs)
        ],
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                    "seed": 42, "generated_patterns": 15},
        "cold_start": {"saved_state_file": "/tmp/iogita_demo_state.json",
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
    print(f"\n  Zones: {len(config['zones'])}")
    print(f"  Patterns: {zid.net.n_patterns}")
    print(f"  Atoms: 16, D={zid.D}")

    # -- Scenario: Robot was in SHELF_1, restarts --
    print(f"\n{'─' * 60}")
    print(f"  SCENARIO: Zippy10 was in SHELF_1, crashes, restarts")
    print(f"{'─' * 60}\n")

    # Save state before "crash"
    zid.barcode_tracker.last_barcode_row = 1
    zid.barcode_tracker.last_barcode_col = 1
    zid.last_zone = "SHELF_1"
    zid.save_state()
    print(f"  [Before crash] Zone: SHELF_1, Barcode grid: (1,1)")

    # Simulate restart — first telemetry arrives
    print(f"  [Restart]      Robot reboots...")
    time.sleep(0.2)

    first_telemetry = {
        "pose_x": 5.3,
        "pose_y": 5.1,
        "pose_theta": 1.57,       # ~90 deg east
        "battery_soc": 55.0,
        "linear_vel": 0.0,        # Stopped
        "obstacle_range": 0.9,
        "obstacle_detected": False,
        "current_node": 5,
    }

    t0 = time.time()
    hint = zid.get_cold_start_hint(first_telemetry, "zippy10")
    total_ms = (time.time() - t0) * 1000

    print(f"\n  [io-gita]      Zone identified in {hint['ode_time_ms']:.1f}ms")
    print(f"  [io-gita]      Primary: {hint['primary_zone']} "
          f"(confidence: {hint['confidence']:.2f})")
    print(f"  [io-gita]      Candidates: {hint['candidate_zones']}")
    print(f"  [io-gita]      Method: {hint['method']}")
    print(f"  [io-gita]      Recovery: route to barcode "
          f"{hint['nearest_barcodes'][0]['barcode_id']} "
          f"in {hint['nearest_barcodes'][0]['zone']}")
    print(f"  [io-gita]      Total pipeline: {total_ms:.1f}ms")

    # Compare
    print(f"\n{'─' * 60}")
    print(f"  RESULT")
    print(f"{'─' * 60}")
    print(f"  Without io-gita: Robot drives blind 10-30 sec")
    print(f"  With io-gita:    Zone hint in {hint['ode_time_ms']:.1f}ms, "
          f"directed recovery")
    print(f"\n  Done.")

    # Save results
    results = {
        "demo": "simple_5_zone",
        "primary_zone": hint["primary_zone"],
        "confidence": hint["confidence"],
        "method": hint["method"],
        "ode_time_ms": hint["ode_time_ms"],
        "total_pipeline_ms": round(total_ms, 2),
        "candidates": hint["candidate_zones"],
    }
    return results


if __name__ == "__main__":
    results = main()
    print(f"\n  Results: {json.dumps(results, indent=2)}")
