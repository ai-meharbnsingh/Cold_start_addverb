#!/usr/bin/env python
"""
Fleet Debug Node — Live TCP Telemetry Monitor for io-gita
==========================================================

Replaces v1's ros_debug_node.py (which assumed ROS2 topics).
This reads Addverb TCP Protocol V1 telemetry directly.

Modes:
  1. TCP Listener — connect to FMS TCP port and read robot telemetry
  2. REST Poller — poll FMS REST API for robot status
  3. File Replay — read saved telemetry from file (for offline testing)

Usage:
  python debug/fleet_debug_node.py --mode rest --host 127.0.0.1
  python debug/fleet_debug_node.py --mode tcp --host 127.0.0.1 --port 65123
  python debug/fleet_debug_node.py --mode replay --file telemetry.log

Contact: ai.meharbansingh@gmail.com
"""

import sys
import os
import time
import json
import argparse
import socket
import urllib.request
import urllib.error

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from fleet_integration.iogita_zone_node import (
    ZoneIdentifier, ProtocolV1Parser,
)


class FleetDebugNode:
    """Live debugging tool for io-gita zone identification."""

    def __init__(self, zone_identifier: ZoneIdentifier):
        self.zid = zone_identifier
        self.message_count = 0
        self.zone_changes = 0
        self.last_displayed_zone = None
        self.log_file = None

    def start_logging(self, path: str = "/tmp/iogita_debug.jsonl"):
        self.log_file = open(path, "a")
        self._log("DEBUG_START", {"mode": "fleet_debug_node"})

    def stop_logging(self):
        if self.log_file:
            self._log("DEBUG_STOP", {
                "messages": self.message_count,
                "zone_changes": self.zone_changes,
            })
            self.log_file.close()
            self.log_file = None

    def process_telemetry(self, robot_state: dict,
                          robot_type: str = "zippy10"):
        """Process one telemetry message and display io-gita analysis."""
        self.message_count += 1

        # Run zone identification
        zone, method, conf, ode_ms = self.zid.identify_from_sensors(
            robot_state, robot_type
        )

        # Track zone changes
        if zone != self.last_displayed_zone:
            self.zone_changes += 1
            self.last_displayed_zone = zone
            change_marker = " << ZONE CHANGE"
        else:
            change_marker = ""

        # Display
        robot_id = robot_state.get("robot_id", "unknown")
        vel = robot_state.get("linear_vel", 0)
        bat = robot_state.get("battery_soc", 0)
        obs = robot_state.get("obstacle_range", 0)

        print(f"  [{self.message_count:>5}] {robot_id:>12} | "
              f"zone={zone:>10} ({conf:.2f}) | "
              f"vel={vel:.1f} bat={bat:.0f}% obs={obs:.1f}m | "
              f"{method}{change_marker}")

        # Log
        self._log("TELEMETRY", {
            "robot_id": robot_id,
            "zone": zone,
            "confidence": conf,
            "method": method,
            "ode_ms": round(ode_ms, 2),
            "velocity": vel,
            "battery": bat,
            "obstacle_range": obs,
        })

        return zone, conf

    def process_protocol_v1(self, raw_message: str,
                            robot_type: str = "zippy10"):
        """Parse and process a raw Protocol V1 TCP message."""
        parsed = ProtocolV1Parser.parse(raw_message)
        if parsed:
            return self.process_telemetry(parsed, robot_type)
        else:
            print(f"  [WARN] Failed to parse: {raw_message[:60]}...")
            return None, 0.0

    def _log(self, event: str, data: dict):
        if self.log_file:
            entry = {
                "time": time.time(),
                "event": event,
                **data,
            }
            self.log_file.write(json.dumps(entry) + "\n")
            self.log_file.flush()


# =====================================================================
# MODE: REST Poller
# =====================================================================

def run_rest_mode(debug_node: FleetDebugNode, host: str, port: int,
                  robot_id: str, interval: float):
    """Poll FMS REST API for robot status."""
    print(f"\n  Mode: REST Poller")
    print(f"  Target: http://{host}:{port}/api/robots/{robot_id}/status")
    print(f"  Interval: {interval}s")
    print(f"  Press Ctrl+C to stop\n")

    url = f"http://{host}:{port}/api/robots/{robot_id}/status"

    while True:
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                data = json.loads(resp.read().decode())
                debug_node.process_telemetry(data)
        except (urllib.error.URLError, TimeoutError) as e:
            print(f"  [ERR] REST failed: {e}")
        except KeyboardInterrupt:
            break

        time.sleep(interval)


# =====================================================================
# MODE: TCP Listener
# =====================================================================

def run_tcp_mode(debug_node: FleetDebugNode, host: str, port: int):
    """Connect to FMS TCP and read Protocol V1 messages."""
    print(f"\n  Mode: TCP Listener")
    print(f"  Target: {host}:{port}")
    print(f"  Press Ctrl+C to stop\n")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((host, port))
        print(f"  Connected to {host}:{port}")

        buffer = ""
        while True:
            try:
                data = sock.recv(4096).decode("utf-8")
                if not data:
                    print(f"  [WARN] Connection closed by FMS")
                    break

                buffer += data
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    if line.strip():
                        debug_node.process_protocol_v1(line.strip())

            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break

        sock.close()

    except (socket.error, ConnectionRefusedError) as e:
        print(f"  [ERR] TCP connection failed: {e}")
        print(f"  Is FMS running on {host}:{port}?")


# =====================================================================
# MODE: File Replay
# =====================================================================

def run_replay_mode(debug_node: FleetDebugNode, filepath: str):
    """Replay saved telemetry from file."""
    print(f"\n  Mode: File Replay")
    print(f"  File: {filepath}\n")

    if not os.path.exists(filepath):
        print(f"  [ERR] File not found: {filepath}")
        return

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            # Try Protocol V1 format first
            if "|" in line:
                debug_node.process_protocol_v1(line)
            else:
                # Try JSON format
                try:
                    data = json.loads(line)
                    debug_node.process_telemetry(data)
                except json.JSONDecodeError:
                    print(f"  [WARN] Unknown format: {line[:60]}...")

            time.sleep(0.067)  # Simulate 15 Hz


# =====================================================================
# STANDALONE DEMO (no FMS required)
# =====================================================================

def run_demo_mode(debug_node: FleetDebugNode):
    """Generate fake telemetry for demo."""
    import numpy as np

    print(f"\n  Mode: Demo (synthetic telemetry)")
    print(f"  Generating 15 Hz telemetry for 3 seconds\n")

    rng = np.random.default_rng(42)
    zones = debug_node.zid.config["zones"]

    # Simulate robot moving through zones
    for i in range(45):  # 3 sec @ 15 Hz
        zone = zones[i % len(zones)]
        debug_node.zid.barcode_tracker.last_barcode_row = zone["row"]
        debug_node.zid.barcode_tracker.last_barcode_col = zone["col"]

        state = {
            "robot_id": "zippy10_demo",
            "pose_x": zone["center_x"] * 0.8 + rng.uniform(-0.2, 0.2),
            "pose_y": zone["center_y"] * 0.8 + rng.uniform(-0.2, 0.2),
            "pose_theta": np.radians(zone.get("expected_heading", 90)),
            "battery_soc": 70 - i * 0.2,
            "linear_vel": 0.8 + rng.uniform(-0.3, 0.3),
            "obstacle_range": rng.uniform(0.5, 1.5),
            "obstacle_detected": rng.random() < 0.15,
            "current_node": zone.get("graph_nodes", [1])[0],
        }
        debug_node.process_telemetry(state)
        time.sleep(0.067)

    print(f"\n  Messages: {debug_node.message_count}")
    print(f"  Zone changes: {debug_node.zone_changes}")


# =====================================================================
# MAIN
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description="io-gita Fleet Debug Node — Live TCP/REST Monitor"
    )
    parser.add_argument("--mode", choices=["rest", "tcp", "replay", "demo"],
                        default="demo",
                        help="Operating mode (default: demo)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="FMS host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=0,
                        help="FMS port (default: auto from mode)")
    parser.add_argument("--robot", default="zippy10_3",
                        help="Robot ID for REST mode")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="REST poll interval in seconds")
    parser.add_argument("--file", default="",
                        help="Telemetry file for replay mode")
    parser.add_argument("--log", default="/tmp/iogita_debug.jsonl",
                        help="Debug log file path")
    args = parser.parse_args()

    print("=" * 60)
    print("  io-gita Fleet Debug Node v2")
    print("  Aligned to Addverb fleet_core (TCP Protocol V1)")
    print("=" * 60)

    # Build zone identifier with default test config
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
    ]
    config = {
        "warehouse": {"grid_spacing_m": 0.8, "max_rows": 50,
                      "max_cols": 80},
        "zones": [
            {"name": n, "row": r, "col": c, "type": t,
             "expected_heading": 90,
             "barcode_range": [i * 20 + 1, (i + 1) * 20],
             "graph_nodes": list(range(i * 5 + 1, (i + 1) * 5 + 1)),
             "center_x": c * 10, "center_y": r * 10,
             "has_charger": t == "dock"}
            for i, (n, r, c, t) in enumerate(zone_defs)
        ],
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                    "seed": 42, "generated_patterns": 15},
        "cold_start": {"saved_state_file": "/tmp/iogita_last_state.json",
                       "confidence_threshold": 0.6, "max_hint_zones": 5,
                       "teleport_confidence": 0.3,
                       "recovery_strategy": "nearest_barcode"},
        "map_change": {"enabled": True, "mismatch_threshold": 3,
                       "feature_tolerance": 0.3},
        "adjacency_overrides": [],
        "robot_types": {"zippy10": {"max_velocity": 1.4}},
    }

    zid = ZoneIdentifier(config)
    zid.build_network()

    debug_node = FleetDebugNode(zid)
    debug_node.start_logging(args.log)

    try:
        if args.mode == "demo":
            run_demo_mode(debug_node)
        elif args.mode == "rest":
            port = args.port or 7012
            run_rest_mode(debug_node, args.host, port,
                          args.robot, args.interval)
        elif args.mode == "tcp":
            port = args.port or 65123
            run_tcp_mode(debug_node, args.host, port)
        elif args.mode == "replay":
            if not args.file:
                print("  [ERR] --file required for replay mode")
                return
            run_replay_mode(debug_node, args.file)
    finally:
        debug_node.stop_logging()
        print(f"\n  Debug log: {args.log}")


if __name__ == "__main__":
    main()
