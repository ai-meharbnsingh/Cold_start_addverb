"""
io-gita Debug Node — Live Diagnostics on Robot (ROS1 + ROS2)
==============================================================

Publishes debug info on /iogita/debug topic for real-time monitoring.
Logs all zone identifications, timing, confidence, and graph decisions.

Launch alongside the main iogita_zone_node:
    ROS1: rosrun iogita ros_debug_node.py
    ROS2: ros2 run iogita ros_debug_node

Subscribes to all /iogita/* topics and logs everything.

NOTE: This is a STANDALONE debug node. It does NOT depend on the main
      io-gita zone node. You can run it independently for testing.

Contact: ai.meharbansingh@gmail.com
"""

# ── This file is a TEMPLATE for ROS1/ROS2 integration ──
# ── It shows the exact node structure and topic names ──
# ── Uncomment the appropriate ROS version imports below ──

import json
import time
import sys
import os

# ─────────────────────────────────────────────────────────
# TEMPLATE: Uncomment these for real ROS2 deployment
# ─────────────────────────────────────────────────────────
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu


class IoGitaDebugNode:
    """
    Debug node that monitors all io-gita topics and logs diagnostics.

    TOPICS MONITORED:
        /iogita/zone              - Current zone identification
        /iogita/zone_hint         - Cold start zone candidates (JSON list)
        /iogita/zone_confidence   - Confidence 0.0-1.0
        /iogita/map_change        - Map change alerts (JSON)
        /scan                     - Raw LiDAR (for verification)
        /odom                     - Odometry (for graph checking)

    TOPICS PUBLISHED:
        /iogita/debug             - JSON debug output (all diagnostics)

    LOG FILE:
        iogita_debug.jsonl        - One JSON line per event
    """

    def __init__(self):
        self.log_file = open("iogita_debug.jsonl", "a")
        self.zone_history = []
        self.timing_history = []
        self.confidence_history = []
        self.event_count = 0
        self.start_time = time.time()

        # Zone adjacency (MUST MATCH your warehouse layout)
        # ── MODIFY THIS for your warehouse ──
        self.adjacency = {
            "DOCK_A": ["AISLE_1", "LANE_W"],
            "AISLE_1": ["DOCK_A", "CROSS_N", "SHELF_1"],
            "CROSS_N": ["AISLE_1", "AISLE_2", "MID_N"],
            "AISLE_2": ["CROSS_N", "DOCK_B", "SHELF_2"],
            "DOCK_B": ["AISLE_2", "LANE_E"],
            "LANE_W": ["DOCK_A", "CROSS_W", "SHELF_1"],
            "SHELF_1": ["LANE_W", "AISLE_1", "MID_N", "SHELF_3"],
            "MID_N": ["SHELF_1", "CROSS_N", "HUB", "SHELF_2"],
            "SHELF_2": ["MID_N", "AISLE_2", "LANE_E", "SHELF_4"],
            "LANE_E": ["DOCK_B", "SHELF_2", "CROSS_E"],
            "CROSS_W": ["LANE_W", "SHELF_3", "LANE_W2"],
            "SHELF_3": ["CROSS_W", "SHELF_1", "HUB", "SHELF_5"],
            "HUB": ["SHELF_3", "MID_N", "SHELF_4", "MID_S"],
            "SHELF_4": ["HUB", "SHELF_2", "CROSS_E", "SHELF_6"],
            "CROSS_E": ["SHELF_4", "LANE_E", "LANE_E2"],
            "LANE_W2": ["CROSS_W", "SHELF_5", "DOCK_C"],
            "SHELF_5": ["LANE_W2", "SHELF_3", "MID_S"],
            "MID_S": ["SHELF_5", "HUB", "SHELF_6", "CROSS_S"],
            "SHELF_6": ["MID_S", "SHELF_4", "LANE_E2"],
            "LANE_E2": ["CROSS_E", "SHELF_6", "DOCK_D"],
            "DOCK_C": ["LANE_W2", "AISLE_3"],
            "AISLE_3": ["DOCK_C", "CROSS_S"],
            "CROSS_S": ["AISLE_3", "MID_S", "AISLE_4"],
            "AISLE_4": ["CROSS_S", "DOCK_D"],
            "DOCK_D": ["AISLE_4", "LANE_E2"],
        }

    def log_event(self, event_type, data):
        """Log a debug event to file and topic."""
        self.event_count += 1
        event = {
            "seq": self.event_count,
            "timestamp": time.time(),
            "uptime_sec": round(time.time() - self.start_time, 2),
            "type": event_type,
            "data": data,
        }
        self.log_file.write(json.dumps(event) + "\n")
        self.log_file.flush()
        return event

    def on_zone_identified(self, zone_name, confidence, ode_time_ms,
                           candidates, method):
        """Called when io-gita identifies a zone."""

        # Check graph consistency
        graph_valid = True
        graph_warning = None
        if self.zone_history:
            last_zone = self.zone_history[-1]
            if last_zone in self.adjacency:
                reachable = set(self.adjacency[last_zone]) | {last_zone}
                if zone_name not in reachable:
                    graph_valid = False
                    graph_warning = (f"TELEPORT: {last_zone} -> {zone_name} "
                                     f"not in adjacency (reachable: {reachable})")

        self.zone_history.append(zone_name)
        self.timing_history.append(ode_time_ms)
        self.confidence_history.append(confidence)

        # Performance stats
        avg_time = sum(self.timing_history[-50:]) / len(self.timing_history[-50:])
        avg_conf = sum(self.confidence_history[-50:]) / len(self.confidence_history[-50:])

        event = self.log_event("zone_id", {
            "zone": zone_name,
            "confidence": confidence,
            "ode_time_ms": round(ode_time_ms, 2),
            "method": method,
            "candidates": candidates,
            "graph_valid": graph_valid,
            "graph_warning": graph_warning,
            "rolling_avg_time_ms": round(avg_time, 2),
            "rolling_avg_confidence": round(avg_conf, 3),
            "total_identifications": len(self.zone_history),
        })

        # Console output for debugging
        status = "OK" if graph_valid else "!! TELEPORT"
        print(f"[io-gita] Zone={zone_name} conf={confidence:.2f} "
              f"t={ode_time_ms:.1f}ms method={method} {status}")

        if graph_warning:
            print(f"[io-gita] WARNING: {graph_warning}")

        return event

    def on_cold_start(self, hint_zones, ode_time_ms, saved_zone):
        """Called during cold start recovery."""
        event = self.log_event("cold_start", {
            "saved_zone": saved_zone,
            "hint_zones": hint_zones,
            "n_candidates": len(hint_zones),
            "ode_time_ms": round(ode_time_ms, 2),
        })
        print(f"[io-gita] COLD START: saved={saved_zone} "
              f"hints={hint_zones} t={ode_time_ms:.1f}ms")
        return event

    def on_map_change(self, zone, change_type, old_sig, new_sig, confidence):
        """Called when map change detected."""
        event = self.log_event("map_change", {
            "zone": zone,
            "change_type": change_type,
            "old_signature": old_sig,
            "new_signature": new_sig,
            "confidence": confidence,
        })
        print(f"[io-gita] MAP CHANGE: {zone} — {change_type} (conf={confidence:.2f})")
        return event

    def get_status_report(self):
        """Generate a full status report."""
        report = {
            "uptime_sec": round(time.time() - self.start_time, 2),
            "total_identifications": len(self.zone_history),
            "unique_zones_visited": len(set(self.zone_history)),
            "current_zone": self.zone_history[-1] if self.zone_history else None,
        }

        if self.timing_history:
            import numpy as np
            times = np.array(self.timing_history)
            report["timing"] = {
                "mean_ms": round(float(np.mean(times)), 2),
                "p95_ms": round(float(np.percentile(times, 95)), 2),
                "max_ms": round(float(np.max(times)), 2),
                "min_ms": round(float(np.min(times)), 2),
            }

        if self.confidence_history:
            confs = np.array(self.confidence_history)
            report["confidence"] = {
                "mean": round(float(np.mean(confs)), 3),
                "min": round(float(np.min(confs)), 3),
                "below_50pct": int(np.sum(confs < 0.5)),
            }

        if len(self.zone_history) > 1:
            # Check for rapid zone flipping (instability)
            flips = sum(1 for i in range(1, len(self.zone_history))
                        if self.zone_history[i] != self.zone_history[i-1])
            report["stability"] = {
                "zone_changes": flips,
                "change_rate": round(flips / len(self.zone_history), 3),
                "is_stable": flips / len(self.zone_history) < 0.5,
            }

        return report

    def close(self):
        self.log_file.close()


# ── Standalone test mode ──────────────────────────────────

def run_standalone_test():
    """Test the debug node without ROS2."""
    print("=" * 60)
    print("  io-gita Debug Node — Standalone Test")
    print("=" * 60)

    debug = IoGitaDebugNode()

    # Simulate a robot journey
    journey = [
        ("DOCK_A", 0.98, 45.2, ["DOCK_A"], "UNIQUE"),
        ("AISLE_1", 0.85, 52.1, ["AISLE_1", "AISLE_2", "AISLE_3"], "GRAPH_RESOLVED"),
        ("SHELF_1", 0.72, 48.7, ["SHELF_1", "SHELF_3", "SHELF_5"], "GRAPH_RESOLVED"),
        ("SHELF_3", 0.68, 51.3, ["SHELF_1", "SHELF_3", "SHELF_5"], "GRAPH_RESOLVED"),
        ("HUB", 0.95, 43.1, ["HUB"], "UNIQUE"),
        ("SHELF_4", 0.71, 49.8, ["SHELF_2", "SHELF_4", "SHELF_6"], "GRAPH_RESOLVED"),
    ]

    print("\n── Simulated Journey ──\n")
    for zone, conf, ode_ms, cands, method in journey:
        debug.on_zone_identified(zone, conf, ode_ms, cands, method)

    # Simulate cold start
    print("\n── Simulated Cold Start ──\n")
    debug.on_cold_start(["SHELF_1", "SHELF_3", "LANE_W"], 87.5, "DOCK_A")

    # Simulate map change
    print("\n── Simulated Map Change ──\n")
    debug.on_map_change("SHELF_3", "layout_modified",
                        {"back": 1.5}, {"back": 4.0}, 0.95)

    # Status report
    print("\n── Status Report ──\n")
    report = debug.get_status_report()
    print(json.dumps(report, indent=2))

    debug.close()
    print("\n  Debug log saved: iogita_debug.jsonl")
    print("  Done.")


if __name__ == "__main__":
    run_standalone_test()
