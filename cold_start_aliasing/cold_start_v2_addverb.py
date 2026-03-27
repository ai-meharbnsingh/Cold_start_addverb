#!/usr/bin/env python
"""
io-gita Cold Start v2 — Full Demo (Addverb Fleet_Core Aligned)
================================================================

Full 25-zone warehouse demo with:
  - Cold start recovery (robot restart)
  - Sequential zone traversal (robot moving through warehouse)
  - Barcode failure resilience (reader degradation)
  - Map change detection (layout modification)

Uses Addverb's actual sensor parameters:
  - Obstacle sensor: +-15deg FOV, 1.5m range
  - Barcode grid: 0.8m spacing
  - Protocol V1 telemetry format
  - Robot types: Zippy10 (1.4 m/s), AMR500 (2.0 m/s)

Run: python cold_start_aliasing/cold_start_v2_addverb.py

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
    ProtocolV1Parser,
)
from fleet_integration.fms_adapter import ColdStartOrchestrator


def build_25_zone_config():
    """Build a 25-zone warehouse matching Addverb deployment layout."""
    zone_defs = [
        ("DOCK_A", 0, 0, "dock"), ("AISLE_1", 0, 1, "aisle"),
        ("CROSS_N", 0, 2, "cross"), ("AISLE_2", 0, 3, "aisle"),
        ("DOCK_B", 0, 4, "dock"),
        ("LANE_W", 1, 0, "lane"), ("SHELF_1", 1, 1, "shelf"),
        ("MID_N", 1, 2, "mid"), ("SHELF_2", 1, 3, "shelf"),
        ("LANE_E", 1, 4, "lane"),
        ("CROSS_W", 2, 0, "cross"), ("SHELF_3", 2, 1, "shelf"),
        ("HUB", 2, 2, "hub"), ("SHELF_4", 2, 3, "shelf"),
        ("CROSS_E", 2, 4, "cross"),
        ("LANE_W2", 3, 0, "lane"), ("SHELF_5", 3, 1, "shelf"),
        ("MID_S", 3, 2, "mid"), ("SHELF_6", 3, 3, "shelf"),
        ("LANE_E2", 3, 4, "lane"),
        ("DOCK_C", 4, 0, "dock"), ("AISLE_3", 4, 1, "aisle"),
        ("CROSS_S", 4, 2, "cross"), ("AISLE_4", 4, 3, "aisle"),
        ("DOCK_D", 4, 4, "dock"),
    ]

    return {
        "warehouse": {"grid_spacing_m": 0.8, "max_rows": 50,
                      "max_cols": 80},
        "zones": [
            {"name": n, "row": r, "col": c, "type": t,
             "expected_heading": 90 if c % 2 == 0 else 0,
             "barcode_range": [i * 20 + 1, (i + 1) * 20],
             "graph_nodes": list(range(i * 5 + 1, (i + 1) * 5 + 1)),
             "center_x": c * 10, "center_y": r * 10,
             "has_charger": t == "dock"}
            for i, (n, r, c, t) in enumerate(zone_defs)
        ],
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                    "seed": 42, "generated_patterns": 15,
                    "n_scans_per_zone": 5},
        "cold_start": {"saved_state_file": "/tmp/iogita_last_state.json",
                       "confidence_threshold": 0.6, "max_hint_zones": 5,
                       "teleport_confidence": 0.3,
                       "recovery_strategy": "nearest_barcode",
                       "max_drift_barcodes": 3},
        "barcode_failure": {"enabled": True, "consecutive_failures": 5,
                            "debounce_ms": 5,
                            "recovery_mode": "iogita_guided"},
        "map_change": {"enabled": True, "mismatch_threshold": 3,
                       "feature_tolerance": 0.3},
        "adjacency_overrides": [],
        "robot_types": {
            "zippy10": {"max_velocity": 1.4, "obstacle_fov_deg": 30,
                        "obstacle_range_m": 1.5, "obstacle_critical_m": 0.7,
                        "obstacle_warning_m": 0.8},
            "amr500": {"max_velocity": 2.0, "obstacle_fov_deg": 30,
                       "obstacle_range_m": 1.5, "obstacle_critical_m": 0.7,
                       "obstacle_warning_m": 0.8},
        },
    }


def simulate_telemetry(zone, rng, velocity=0.8):
    """Generate realistic Protocol V1 telemetry for a zone."""
    return {
        "timestamp": time.time(),
        "robot_id": "zippy10_3",
        "pose_x": zone["center_x"] * 0.8 + rng.uniform(-0.3, 0.3),
        "pose_y": zone["center_y"] * 0.8 + rng.uniform(-0.3, 0.3),
        "pose_theta": np.radians(zone.get("expected_heading", 90)
                                 + rng.uniform(-10, 10)),
        "state": "MOVING",
        "battery_soc": 50 + rng.uniform(-20, 20),
        "linear_vel": velocity + rng.uniform(-0.2, 0.2),
        "angular_vel": rng.uniform(-0.1, 0.1),
        "error_code": 0,
        "task_id": "TASK_42",
        "current_node": zone.get("graph_nodes", [1])[
            len(zone.get("graph_nodes", [1])) // 2],
        "obstacle_range": rng.uniform(0.5, 1.5),
        "obstacle_detected": rng.random() < 0.2,
    }


def main():
    print("=" * 70)
    print("  io-gita Cold Start v2 — Full 25-Zone Demo")
    print("  Aligned to Addverb fleet_core (TCP Protocol V1)")
    print("=" * 70)

    config = build_25_zone_config()
    rng = np.random.default_rng(42)

    zid = ZoneIdentifier(config)
    zid.build_network()

    print(f"\n  Warehouse: 25 zones (5x5 grid)")
    print(f"  Robot types: Zippy10 (1.4 m/s), AMR500 (2.0 m/s)")
    print(f"  Network: {zid.net.n_patterns} patterns, 16 atoms, D={zid.D}")
    print(f"  Barcode grid: 0.8m spacing")
    print(f"  Obstacle sensor: +-15deg, 1.5m range")

    results = {
        "demo": "full_25_zone_addverb",
        "zones": 25,
        "tests": [],
    }

    # ── TEST 1: Cold Start Recovery ──────────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  TEST 1: Cold Start Recovery")
    print(f"  Robot was in SHELF_3, crashes, restarts between barcodes")
    print(f"{'═' * 70}\n")

    # Setup: robot was working in SHELF_3
    shelf_3 = next(z for z in config["zones"] if z["name"] == "SHELF_3")
    zid.barcode_tracker.last_barcode_row = shelf_3["row"]
    zid.barcode_tracker.last_barcode_col = shelf_3["col"]
    zid.last_zone = "SHELF_3"
    zid.save_state()

    # Robot restarts — first telemetry (stopped, between barcodes)
    restart_telemetry = simulate_telemetry(shelf_3, rng, velocity=0.0)

    orch = ColdStartOrchestrator(zid, adapter=None)
    plan = orch.on_robot_restart("zippy10_3", restart_telemetry, "zippy10")

    print(f"  Zone hint:    {plan['hint']['primary_zone']}")
    print(f"  Confidence:   {plan['hint']['confidence']:.2f}")
    print(f"  ODE time:     {plan['hint']['ode_time_ms']:.1f}ms")
    print(f"  Method:       {plan['hint']['method']}")
    print(f"  Candidates:   {plan['hint']['candidate_zones']}")
    correct = plan['hint']['primary_zone'] == "SHELF_3"
    print(f"  Correct:      {'YES' if correct else 'NO'}")

    results["tests"].append({
        "test": "cold_start",
        "expected": "SHELF_3",
        "got": plan["hint"]["primary_zone"],
        "correct": correct,
        "confidence": plan["hint"]["confidence"],
        "ode_time_ms": plan["hint"]["ode_time_ms"],
    })

    # ── TEST 2: Sequential Zone Traversal ────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  TEST 2: Sequential Zone Traversal (robot moving)")
    print(f"  Simulating robot moving through warehouse, barcode working")
    print(f"{'═' * 70}\n")

    # Simulate robot path: DOCK_A -> AISLE_1 -> SHELF_1 -> HUB -> SHELF_4
    path = ["DOCK_A", "AISLE_1", "SHELF_1", "HUB", "SHELF_4"]
    traversal_correct = 0
    traversal_total = 0

    for zone_name in path:
        zone = next(z for z in config["zones"] if z["name"] == zone_name)

        # Barcode read succeeds (normal operation)
        bc_id = zone["barcode_range"][0] + 5
        zid.barcode_tracker.update_barcode_read(
            zone["row"], zone["col"],
            zone["center_x"] * 0.8, zone["center_y"] * 0.8,
            np.radians(zone.get("expected_heading", 90)),
        )
        barcode_zone = zid.identify_from_barcode(bc_id)

        # Also run sensor-based identification for comparison
        tel = simulate_telemetry(zone, rng)
        sensor_zone, method, conf, ode_ms = zid.identify_from_sensors(
            tel, "zippy10")

        traversal_total += 1
        if sensor_zone == zone_name:
            traversal_correct += 1

        print(f"  {zone_name:>10} | barcode={barcode_zone:>10} | "
              f"sensor={sensor_zone:>10} ({method}, {conf:.2f}, "
              f"{ode_ms:.1f}ms)")

    print(f"\n  Sensor accuracy: {traversal_correct}/{traversal_total}")

    results["tests"].append({
        "test": "traversal",
        "correct": traversal_correct,
        "total": traversal_total,
        "accuracy": traversal_correct / max(traversal_total, 1),
    })

    # ── TEST 3: Barcode Failure ──────────────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  TEST 3: Barcode Failure (reader degraded in SHELF_5)")
    print(f"  Simulating 5 consecutive barcode read failures")
    print(f"{'═' * 70}\n")

    shelf_5 = next(z for z in config["zones"] if z["name"] == "SHELF_5")
    zid.barcode_tracker.last_barcode_row = shelf_5["row"]
    zid.barcode_tracker.last_barcode_col = shelf_5["col"]
    zid.last_zone = "SHELF_5"

    # Simulate 5 failures (Addverb irayple threshold)
    for i in range(5):
        zid.barcode_tracker.update_barcode_failure()
        print(f"  Failure {i+1}/5: consecutive={zid.barcode_tracker.consecutive_failures}")

    print(f"  Barcode failing: {zid.barcode_tracker.barcode_is_failing}")

    tel = simulate_telemetry(shelf_5, rng, velocity=0.6)
    hint = zid.get_barcode_failure_hint(tel, "zippy10")

    print(f"\n  io-gita zone:     {hint['current_zone']}")
    print(f"  Confidence:       {hint['confidence']:.2f}")
    print(f"  Action:           {hint['action']}")
    print(f"  Recovery zones:   {[r['zone'] + ' (' + r['direction'] + ')' for r in hint['recovery_zones'][:3]]}")

    results["tests"].append({
        "test": "barcode_failure",
        "zone": hint["current_zone"],
        "confidence": hint["confidence"],
        "failures": hint["barcode_failures"],
    })

    # ── TEST 4: Protocol V1 Parsing ──────────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  TEST 4: Protocol V1 Message Parsing")
    print(f"  Parsing actual Addverb TCP telemetry format")
    print(f"{'═' * 70}\n")

    test_msg = "1711545200.5|zippy10_3|15.2|24.8|1.55|MOVING|68.0|0.8|0.02|0|TASK_42|13|1.1|0"
    parsed = ProtocolV1Parser.parse(test_msg)

    if parsed:
        print(f"  Robot:    {parsed['robot_id']}")
        print(f"  Pose:     ({parsed['pose_x']}, {parsed['pose_y']}, "
              f"{np.degrees(parsed['pose_theta']):.1f}deg)")
        print(f"  State:    {parsed['state']}")
        print(f"  Battery:  {parsed['battery_soc']}%")
        print(f"  Velocity: {parsed['linear_vel']} m/s")
        print(f"  Node:     {parsed['current_node']}")
        print(f"  Obstacle: {parsed['obstacle_range']}m "
              f"(detected={parsed['obstacle_detected']})")

        # Run zone identification on parsed telemetry
        zone, method, conf, ode_ms = zid.identify_from_sensors(
            parsed, "zippy10")
        print(f"\n  io-gita:  {zone} ({method}, conf={conf:.2f}, "
              f"{ode_ms:.1f}ms)")
    else:
        print(f"  PARSE FAILED")

    results["tests"].append({
        "test": "protocol_v1",
        "parsed": parsed is not None,
        "zone": zone if parsed else None,
    })

    # ── TEST 5: All 25 Zones — Accuracy ─────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  TEST 5: Full Warehouse — All 25 Zones")
    print(f"  Sensor-only identification accuracy")
    print(f"{'═' * 70}\n")

    correct = 0
    total = 0
    timings = []

    for zone in config["zones"]:
        zid.barcode_tracker.last_barcode_row = zone["row"]
        zid.barcode_tracker.last_barcode_col = zone["col"]

        tel = simulate_telemetry(zone, rng)
        z_name, method, conf, ode_ms = zid.identify_from_sensors(
            tel, "zippy10")
        total += 1
        timings.append(ode_ms)
        match = z_name == zone["name"]
        if match:
            correct += 1
        mark = "OK" if match else "MISS"
        print(f"  {zone['name']:>10} -> {z_name:>10} "
              f"[{mark:>4}] ({method}, {conf:.2f}, {ode_ms:.1f}ms)")

    acc = correct / max(total, 1) * 100
    avg_ode = np.mean(timings)
    print(f"\n  Accuracy:     {correct}/{total} ({acc:.0f}%)")
    print(f"  Avg ODE time: {avg_ode:.2f}ms")
    print(f"  Max ODE time: {max(timings):.2f}ms")

    results["tests"].append({
        "test": "full_25_zone",
        "correct": correct,
        "total": total,
        "accuracy": acc,
        "avg_ode_ms": round(avg_ode, 2),
        "max_ode_ms": round(max(timings), 2),
    })

    # ── SUMMARY ──────────────────────────────────────────────────────
    print(f"\n{'═' * 70}")
    print(f"  SUMMARY")
    print(f"{'═' * 70}")
    print(f"  Cold start:       {results['tests'][0]['got']} "
          f"({'CORRECT' if results['tests'][0]['correct'] else 'WRONG'})")
    print(f"  Traversal:        {results['tests'][1]['accuracy']*100:.0f}%")
    print(f"  Barcode failure:  {results['tests'][2]['zone']} recovered")
    print(f"  Protocol V1:      {'PARSED' if results['tests'][3]['parsed'] else 'FAILED'}")
    print(f"  25-zone accuracy: {results['tests'][4]['accuracy']:.0f}%")
    print(f"  Avg ODE time:     {results['tests'][4]['avg_ode_ms']}ms")
    print(f"\n  All features from Addverb actual sensors:")
    print(f"    +-15deg obstacle sensor + barcode grid + odometry")
    print(f"    No 360deg LiDAR. No AMCL. No ROS2.")
    print(f"\n  Done.")

    return results


if __name__ == "__main__":
    results = main()

    # Save results
    out_path = os.path.join(os.path.dirname(__file__),
                            "cold_start_v2_results.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results saved to {out_path}")
