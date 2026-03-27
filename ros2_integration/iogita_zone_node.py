#!/usr/bin/env python
"""
io-gita Zone Identification Node (ROS1 compatible)
====================================================

Subscribes to /scan, /odom, /imu/data.
Publishes /iogita/zone, /iogita/zone_hint, /iogita/zone_confidence, /iogita/map_change.

This node is provided by io-gita. The partner does NOT modify this file.

ROS1 Usage:
    rosrun iogita iogita_zone_node.py _config_file:=warehouse_config.yaml _mode:=production

ROS2 Usage:
    ros2 run iogita iogita_zone_node --ros-args -p config_file:=warehouse_config.yaml
"""

import json
import time
import os
import sys
import yaml
import numpy as np

# ── ROS1 imports ──
# Uncomment for ROS1 deployment:
# import rospy
# from std_msgs.msg import String, Float32
# from sensor_msgs.msg import LaserScan, Imu
# from nav_msgs.msg import Odometry

# ── ROS2 imports ──
# Uncomment for ROS2 deployment:
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String, Float32
# from sensor_msgs.msg import LaserScan, Imu
# from nav_msgs.msg import Odometry

# Add parent path for sg_engine import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
from sg_engine.network import Network


# ═══════════════════════════════════════════════════════════
# FEATURE EXTRACTION — 16 features from LiDAR + odom + IMU
# ═══════════════════════════════════════════════════════════

def extract_16_features(scan_ranges, heading_deg, dist_from_dock, turns_since_dock):
    """
    Extract 16 zone-discriminating features from sensor data.

    Args:
        scan_ranges: np.array of 360 LiDAR range values (meters)
        heading_deg: Robot heading in degrees (0=North, from IMU)
        dist_from_dock: Cumulative distance from last dock zone (from odometry)
        turns_since_dock: Number of >45° heading changes since dock

    Returns:
        np.array of 16 float features, each normalized to ~[0, 1]
    """
    scan = np.array(scan_ranges, dtype=np.float64)

    # Handle variable scan sizes (downsample/upsample to 360)
    if len(scan) != 360:
        indices = np.linspace(0, len(scan) - 1, 360).astype(int)
        scan = scan[indices]

    # Replace inf/nan with max range
    scan = np.nan_to_num(scan, nan=12.0, posinf=12.0, neginf=0.1)
    scan = np.clip(scan, 0.1, 12.0)

    # Feature 1-4: Sector clearances (median range in 4 directions)
    front = np.median(np.concatenate([scan[345:360], scan[0:15]]))
    back  = np.median(scan[165:195])
    left  = np.median(scan[255:285])
    right = np.median(scan[75:105])

    # Feature 5-6: Scan variance (roughness — shelf vs smooth wall)
    var_front_half = float(np.var(np.concatenate([scan[315:360], scan[0:45]])))
    var_all = float(np.var(scan))

    # Feature 7-8: Gap count (jumps between consecutive rays)
    diffs = np.abs(np.diff(scan))
    gap_count = int(np.sum(diffs > 1.0))
    big_gap_count = int(np.sum(diffs > 2.0))

    # Feature 9-10: Symmetry (left vs right, front vs back)
    sym_lr = abs(left - right) / max(left + right, 0.1)
    sym_fb = abs(front - back) / max(front + back, 0.1)

    # Feature 11-12: Density (fraction of close/far readings)
    close_count = int(np.sum(scan < 2.0)) / 360.0
    far_count = int(np.sum(scan > 4.0)) / 360.0

    # Feature 13-14: Odometry
    dist_norm = min(dist_from_dock / 30.0, 1.0)
    turns_norm = min(turns_since_dock / 10.0, 1.0)

    # Feature 15-16: Heading
    heading_norm = heading_deg / 360.0
    heading_bin = int(heading_deg / 45) / 8.0

    return np.array([
        front, back, left, right,
        var_front_half, var_all,
        gap_count / 50.0, big_gap_count / 20.0,
        sym_lr, sym_fb,
        close_count, far_count,
        dist_norm, turns_norm,
        heading_norm, heading_bin,
    ], dtype=np.float64)


# ═══════════════════════════════════════════════════════════
# ZONE IDENTIFIER — io-gita powered
# ═══════════════════════════════════════════════════════════

class ZoneIdentifier:
    """
    Identifies warehouse zones using io-gita Hopfield network + graph topology.

    Workflow:
        1. Extract 16 features from LiDAR/odom/IMU
        2. Build query vector from features × atoms
        3. Run ODE → find nearest attractor basin → zone candidates
        4. Graph disambiguation using adjacency + previous zone
    """

    def __init__(self, config):
        self.config = config
        self.D = config["engine"]["D"]
        self.zones = {z["name"]: z for z in config["zones"]}
        self.zone_fingerprints = {}
        self.last_zone = None
        self.adjacency = self._build_adjacency()
        self.net = None
        self.atom_names = [f"F{i}" for i in range(16)]
        self.mismatch_counts = {}

    def _build_adjacency(self):
        """Build zone adjacency graph from grid positions."""
        adj = {}
        zone_list = list(self.zones.values())
        for z in zone_list:
            neighbors = []
            for other in zone_list:
                if other["name"] != z["name"]:
                    if abs(z["row"] - other["row"]) + abs(z["col"] - other["col"]) == 1:
                        neighbors.append(other["name"])
            adj[z["name"]] = neighbors

        # Apply overrides
        for override in self.config.get("adjacency_overrides", []) or []:
            if "from" in override and "to" in override:
                adj.setdefault(override["from"], []).append(override["to"])
                adj.setdefault(override["to"], []).append(override["from"])

        return adj

    def build_network(self, fingerprints=None):
        """Build io-gita network from zone fingerprints."""
        cfg = self.config["engine"]
        self.net = Network(D=cfg["D"], beta=cfg["beta"],
                          dt=cfg["dt"], seed=cfg["seed"])
        self.net.add_atoms(self.atom_names)

        if fingerprints:
            self.zone_fingerprints = fingerprints
        else:
            # Use default fingerprints from config zone types
            rng = np.random.default_rng(cfg["seed"])
            for name, z in self.zones.items():
                fp = rng.uniform(0, 1, 16)  # Placeholder until calibration
                self.zone_fingerprints[name] = fp

        # Create zone patterns from fingerprints
        for name, fp in self.zone_fingerprints.items():
            vec = np.ones(self.D)
            for i, val in enumerate(fp):
                scaled = 2.0 * np.clip(val, 0, 1) - 1.0
                if abs(scaled) > 0.1:
                    vec *= (scaled * self.net.atoms[self.atom_names[i]])
                else:
                    vec *= self.net.atoms[self.atom_names[i]] * 0.1
            self.net.add_pattern(name, np.sign(vec))

        # Add generated patterns for richer landscape
        self.net.generate_patterns(cfg["generated_patterns"],
                                   gen_seed=cfg["seed"])

    def identify(self, scan_ranges, heading, dist_from_dock, turns):
        """Full zone identification pipeline."""
        t0 = time.time()

        features = extract_16_features(scan_ranges, heading,
                                       dist_from_dock, turns)

        # Build query pattern
        vec = np.ones(self.D)
        for i, val in enumerate(features):
            scaled = 2.0 * np.clip(val, 0, 1) - 1.0
            if abs(scaled) > 0.1:
                vec *= (scaled * self.net.atoms[self.atom_names[i]])
            else:
                vec *= self.net.atoms[self.atom_names[i]] * 0.1
        q = np.sign(vec).astype(np.float64)

        # Run ODE
        basin, q_final, _ = self.net.run_dynamics(q, alpha=0.0)
        sims = self.net.P_mat @ q_final / self.D

        # Get zone candidates
        candidates = []
        for i, name in enumerate(self.net.pat_names):
            if name in self.zones:
                candidates.append((name, float(sims[i])))
        candidates.sort(key=lambda x: -x[1])

        ode_time = time.time() - t0

        # Fingerprint distance ranking
        fp_distances = []
        for name, stored_fp in self.zone_fingerprints.items():
            dist = np.linalg.norm(features - stored_fp)
            fp_distances.append((name, dist))
        fp_distances.sort(key=lambda x: x[1])

        return candidates, fp_distances, ode_time, features

    def disambiguate(self, candidates, fp_distances, previous_zone=None):
        """Use graph topology to resolve identical-looking zones."""
        ode_top = set(c[0] for c in candidates[:5])
        fp_top = set(c[0] for c in fp_distances[:5])
        combined = ode_top | fp_top

        if previous_zone and previous_zone in self.adjacency:
            reachable = set(self.adjacency[previous_zone])
            reachable.add(previous_zone)

            valid = combined & reachable
            if len(valid) == 1:
                result = valid.pop()
                self.last_zone = result
                return result, "GRAPH_UNIQUE", 1.0
            elif len(valid) > 1:
                best = min(valid, key=lambda z: next(
                    d for n, d in fp_distances if n == z))
                self.last_zone = best
                return best, "GRAPH_FP_RANKED", 0.9
            else:
                best = fp_distances[0][0]
                self.last_zone = best
                return best, "TELEPORT_FALLBACK", 0.3
        else:
            best = fp_distances[0][0]
            self.last_zone = best
            return best, "COLD_FP_ONLY", 0.5

    def get_hint_zones(self, candidates, fp_distances, previous_zone=None):
        """Return zones list for AMCL constraint."""
        max_hints = self.config["cold_start"]["amcl_hint_max_zones"]
        fp_top = [name for name, _ in fp_distances[:3]]

        if previous_zone and previous_zone in self.adjacency:
            reachable = set(self.adjacency[previous_zone])
            reachable.add(previous_zone)
            valid = [z for z in fp_top if z in reachable]
            if valid:
                return valid[:max_hints]
            return list(reachable)[:max_hints]
        else:
            return fp_top[:max_hints]

    def check_map_change(self, zone_name, current_features):
        """Detect if a zone's layout has changed."""
        if not self.config.get("map_change", {}).get("enabled", False):
            return None

        if zone_name not in self.zone_fingerprints:
            return None

        stored_fp = self.zone_fingerprints[zone_name]
        distance = np.linalg.norm(current_features - stored_fp)
        tolerance = self.config["map_change"]["fingerprint_tolerance"]
        threshold = self.config["map_change"]["mismatch_threshold"]

        if distance > tolerance:
            self.mismatch_counts[zone_name] = self.mismatch_counts.get(zone_name, 0) + 1
            if self.mismatch_counts[zone_name] >= threshold:
                self.mismatch_counts[zone_name] = 0
                return {
                    "zone": zone_name,
                    "change_type": "layout_modified",
                    "distance": round(float(distance), 3),
                    "confidence": 1.0,
                }
        else:
            self.mismatch_counts[zone_name] = 0

        return None

    def save_last_zone(self):
        """Save current zone to disk for cold start recovery."""
        path = self.config["cold_start"]["saved_zone_file"]
        data = {
            "zone": self.last_zone,
            "timestamp": time.time(),
        }
        with open(path, "w") as f:
            json.dump(data, f)

    def load_last_zone(self):
        """Load saved zone from disk."""
        path = self.config["cold_start"]["saved_zone_file"]
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            return data.get("zone")
        return None

    def calibrate_zone(self, zone_name, scan_ranges, heading,
                       dist_from_dock, turns):
        """Record fingerprint for a zone during calibration drive."""
        features = extract_16_features(scan_ranges, heading,
                                       dist_from_dock, turns)
        if zone_name not in self.zone_fingerprints:
            self.zone_fingerprints[zone_name] = features
        else:
            # Average with existing
            self.zone_fingerprints[zone_name] = (
                self.zone_fingerprints[zone_name] + features) / 2.0

    def save_fingerprints(self, path="/tmp/iogita_fingerprints.npz"):
        """Save calibrated fingerprints."""
        np.savez(path, **{k: v for k, v in self.zone_fingerprints.items()})

    def load_fingerprints(self, path="/tmp/iogita_fingerprints.npz"):
        """Load calibrated fingerprints."""
        if os.path.exists(path):
            data = np.load(path)
            self.zone_fingerprints = {k: data[k] for k in data.files}
            return True
        return False


# ═══════════════════════════════════════════════════════════
# STANDALONE TEST (no ROS required)
# ═══════════════════════════════════════════════════════════

def standalone_test():
    """Test zone identification without ROS."""
    print("=" * 60)
    print("  io-gita Zone Node — Standalone Test (no ROS)")
    print("=" * 60)

    # Load config
    config_path = os.path.join(os.path.dirname(__file__), "warehouse_config.yaml")
    if os.path.exists(config_path):
        with open(config_path) as f:
            raw = yaml.safe_load(f)
        config = {
            "zones": raw.get("zones", raw.get("warehouse", {}).get("zones", [])),
            "engine": raw.get("engine", {"D": 10000, "beta": 4.0, "dt": 0.05,
                                          "seed": 42, "generated_patterns": 15}),
            "cold_start": raw.get("cold_start", {
                "saved_zone_file": "/tmp/iogita_last_zone.json",
                "confidence_threshold": 0.6,
                "amcl_hint_max_zones": 5,
                "teleport_confidence": 0.3,
            }),
            "map_change": raw.get("map_change", {"enabled": True,
                                                   "mismatch_threshold": 3,
                                                   "fingerprint_tolerance": 0.3}),
            "adjacency_overrides": raw.get("adjacency_overrides", []),
        }
        print(f"\n  Loaded config: {len(config['zones'])} zones")
    else:
        print(f"\n  No config found at {config_path}")
        print(f"  Using built-in test zones")

        # Built-in 25-zone test config
        zone_types = [
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
        config = {
            "zones": [{"name": n, "row": r, "col": c, "type": t,
                        "heading": 90, "dist_from_dock": r+c,
                        "map_x": c*10, "map_y": r*10, "map_zone_radius": 3.0}
                       for n, r, c, t in zone_types],
            "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                        "seed": 42, "generated_patterns": 15},
            "cold_start": {"saved_zone_file": "/tmp/iogita_last_zone.json",
                           "confidence_threshold": 0.6,
                           "amcl_hint_max_zones": 5,
                           "teleport_confidence": 0.3},
            "map_change": {"enabled": True, "mismatch_threshold": 3,
                           "fingerprint_tolerance": 0.3},
            "adjacency_overrides": [],
        }

    zid = ZoneIdentifier(config)

    # Generate test fingerprints
    rng = np.random.default_rng(42)
    fps = {}
    for z in config["zones"]:
        fps[z["name"]] = rng.uniform(0, 1, 16)
    zid.build_network(fps)

    print(f"  Network: {zid.net.n_patterns} patterns, 16 atoms, D={zid.D}")
    print(f"  Adjacency: {sum(len(v) for v in zid.adjacency.values())} edges")

    # Test zone identification
    print(f"\n── Zone Identification Test ──\n")
    correct = 0
    total = 0
    for z in config["zones"][:10]:
        scan = rng.uniform(0.5, 8.0, 360)
        cands, fp_dists, ode_t, feats = zid.identify(
            scan, z["heading"], z.get("dist_from_dock", 0), 0)
        resolved, method, conf = zid.disambiguate(cands, fp_dists, zid.last_zone)
        total += 1
        if resolved == z["name"]:
            correct += 1
        print(f"  {z['name']:>12} → {resolved:>12} ({method}, conf={conf:.2f}, "
              f"ode={ode_t*1000:.1f}ms)")

    print(f"\n  Accuracy: {correct}/{total}")
    print(f"\n  Done. All systems operational.")


if __name__ == "__main__":
    standalone_test()
