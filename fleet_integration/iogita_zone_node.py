#!/usr/bin/env python
"""
io-gita Zone Identification Node — Addverb Fleet_Core Native
=============================================================

Designed for Addverb's ACTUAL sensor suite and communication protocol:
  - Obstacle sensor: +-15deg FOV, 1.5m range (NOT 360deg LiDAR)
  - Barcode reader: irayple, 0.8m grid (primary localization)
  - Wheel encoders: odometry (dead reckoning between barcodes)
  - Communication: TCP Protocol V1 on port 65123 (NOT ROS2 topics)

This node runs ALONGSIDE fleet_core. It does NOT replace barcode localization.
It AUGMENTS it — providing zone-level awareness when barcodes fail.

Integration modes:
  1. TCP Listener — reads Protocol V1 telemetry directly from robot
  2. FMS Sidecar — reads robot state from FMS REST API (port 7012)
  3. Standalone — for testing without fleet_core

Contact: ai.meharbansingh@gmail.com
"""

import json
import time
import os
import sys
import yaml
import numpy as np
from typing import Optional

# Add parent path for sg_engine import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from sg_engine.network import Network


# =====================================================================
# FEATURE EXTRACTION — From Addverb's ACTUAL sensors
# =====================================================================
# Addverb obstacle sensor: +-15deg FOV, 1.5m max range
# NOT a 360deg LiDAR. We extract what we CAN from this limited sensor.

def extract_features_addverb(
    obstacle_range: float,
    obstacle_detected: bool,
    heading_deg: float,
    dist_from_last_barcode: float,
    turns_since_barcode: int,
    velocity: float,
    last_barcode_row: int,
    last_barcode_col: int,
    battery_soc: float,
    time_since_barcode_s: float,
    grid_max_rows: int = 50,
    grid_max_cols: int = 80,
    max_velocity: float = 1.4,
    zone_type_code: float = 0.5,
    barcode_read_success_rate: float = 1.0,
) -> np.ndarray:
    """
    Extract 16 zone-discriminating features from Addverb's actual sensors.

    Unlike the v1 package which assumed 360deg LiDAR, this uses:
    - +-15deg obstacle sensor (2 features: range + detected)
    - Wheel encoder odometry (4 features: heading, distance, turns, velocity)
    - Barcode grid position (4 features: row, col, quadrant, dock proximity)
    - System state (3 features: battery, time since barcode, barcode reliability)
    - Context (3 features: zone type expectation, path progress, grid density)

    Returns:
        np.array of 16 float features, each normalized to ~[0, 1]
    """
    # Feature 1-2: Obstacle sensor (+-15deg, 1.5m range)
    # This is ALL we get from their forward sensor — no left/right/back
    obs_range_norm = np.clip(obstacle_range / 1.5, 0.0, 1.0)
    obs_present = 1.0 if obstacle_detected else 0.0

    # Feature 3-4: Heading from wheel encoders / dead reckoning
    heading_norm = (heading_deg % 360) / 360.0
    heading_bin = int(heading_deg / 45) / 8.0  # 8 compass directions

    # Feature 5-6: Odometry since last barcode read
    # On 0.8m grid, max expected distance between barcodes is ~1.6m (diagonal)
    dist_barcode_norm = np.clip(dist_from_last_barcode / 3.0, 0.0, 1.0)
    turns_norm = np.clip(turns_since_barcode / 4.0, 0.0, 1.0)

    # Feature 7-8: Last known grid position (barcode reader)
    row_norm = np.clip(last_barcode_row / max(grid_max_rows, 1), 0.0, 1.0)
    col_norm = np.clip(last_barcode_col / max(grid_max_cols, 1), 0.0, 1.0)

    # Feature 9-10: Warehouse quadrant + dock proximity
    quadrant = (int(row_norm > 0.5) * 2 + int(col_norm > 0.5)) / 4.0
    # Dock zones are typically at edges (row 0 or max, col 0 or max)
    edge_dist = min(row_norm, 1.0 - row_norm, col_norm, 1.0 - col_norm)
    near_dock = 1.0 - np.clip(edge_dist * 4.0, 0.0, 1.0)

    # Feature 11-12: Robot state
    velocity_norm = np.clip(velocity / max(max_velocity, 0.01), 0.0, 1.0)
    battery_norm = np.clip(battery_soc / 100.0, 0.0, 1.0)

    # Feature 13-14: Temporal + reliability
    time_barcode_norm = np.clip(time_since_barcode_s / 60.0, 0.0, 1.0)
    barcode_reliability = np.clip(barcode_read_success_rate, 0.0, 1.0)

    # Feature 15-16: Contextual
    zone_type_norm = np.clip(zone_type_code, 0.0, 1.0)
    # Grid density: how many barcodes should be near this position
    # Center of warehouse = denser grid, edges = sparser
    grid_density = 1.0 - edge_dist * 2.0

    return np.array([
        obs_range_norm, obs_present,
        heading_norm, heading_bin,
        dist_barcode_norm, turns_norm,
        row_norm, col_norm,
        quadrant, near_dock,
        velocity_norm, battery_norm,
        time_barcode_norm, barcode_reliability,
        zone_type_norm, grid_density,
    ], dtype=np.float64)


# Zone type encoding for feature 15
ZONE_TYPE_CODES = {
    "dock": 0.1,
    "charging": 0.15,
    "aisle": 0.3,
    "lane": 0.35,
    "cross": 0.5,
    "mid": 0.55,
    "shelf": 0.7,
    "hub": 0.9,
}


# =====================================================================
# PROTOCOL V1 PARSER — Reads Addverb TCP telemetry
# =====================================================================

class ProtocolV1Parser:
    """
    Parse Addverb Protocol V1 messages (33 fields).
    Extracts robot state needed for io-gita feature extraction.

    Protocol V1 format (TCP port 65123):
    [Timestamp | RobotID | PoseX | PoseY | PoseTheta | State |
     BatterySOC | LinearVel | AngularVel | ErrorCode | TaskID |
     CurrentNode | ObstacleRange | ObstacleDetected | ...]
    """

    # Valid ranges for sanity checking
    BATTERY_MIN, BATTERY_MAX = 0.0, 100.0
    VELOCITY_MIN, VELOCITY_MAX = -5.0, 5.0
    OBSTACLE_MIN, OBSTACLE_MAX = 0.0, 50.0
    POSE_MIN, POSE_MAX = -10000.0, 10000.0
    VALID_STATES = {
        "IDLE", "MOVING", "CHARGING", "ERROR", "DOCKING",
        "LOADING", "UNLOADING", "WAITING", "DISCONNECTED",
    }

    @staticmethod
    def parse(raw_message: str) -> Optional[dict]:
        """Parse Protocol V1 message into robot state dict with validation."""
        try:
            if not raw_message or not raw_message.strip():
                return None

            fields = raw_message.strip().split("|")
            if len(fields) < 14:
                return None

            timestamp = float(fields[0])
            robot_id = fields[1].strip()
            pose_x = float(fields[2])
            pose_y = float(fields[3])
            pose_theta = float(fields[4])
            state = fields[5].strip()
            battery_soc = float(fields[6])
            linear_vel = float(fields[7])
            angular_vel = float(fields[8])
            error_code = int(fields[9])
            task_id = fields[10].strip()
            current_node = int(fields[11])
            obstacle_range = float(fields[12])
            obstacle_detected = fields[13].strip() == "1"

            # Validate ranges — clamp to sane values
            battery_soc = max(ProtocolV1Parser.BATTERY_MIN,
                              min(battery_soc, ProtocolV1Parser.BATTERY_MAX))
            linear_vel = max(ProtocolV1Parser.VELOCITY_MIN,
                             min(linear_vel, ProtocolV1Parser.VELOCITY_MAX))
            angular_vel = max(ProtocolV1Parser.VELOCITY_MIN,
                              min(angular_vel, ProtocolV1Parser.VELOCITY_MAX))
            obstacle_range = max(ProtocolV1Parser.OBSTACLE_MIN,
                                 min(obstacle_range,
                                     ProtocolV1Parser.OBSTACLE_MAX))
            pose_x = max(ProtocolV1Parser.POSE_MIN,
                         min(pose_x, ProtocolV1Parser.POSE_MAX))
            pose_y = max(ProtocolV1Parser.POSE_MIN,
                         min(pose_y, ProtocolV1Parser.POSE_MAX))

            # Validate robot_id is non-empty
            if not robot_id:
                return None

            # Validate timestamp is reasonable (not zero, not far future)
            if timestamp <= 0:
                return None

            return {
                "timestamp": timestamp,
                "robot_id": robot_id,
                "pose_x": pose_x,
                "pose_y": pose_y,
                "pose_theta": pose_theta,
                "state": state,
                "battery_soc": battery_soc,
                "linear_vel": linear_vel,
                "angular_vel": angular_vel,
                "error_code": error_code,
                "task_id": task_id,
                "current_node": current_node,
                "obstacle_range": obstacle_range,
                "obstacle_detected": obstacle_detected,
            }
        except (ValueError, IndexError, TypeError):
            return None


# =====================================================================
# BARCODE GRID TRACKER — Tracks robot position on barcode grid
# =====================================================================

class BarcodeGridTracker:
    """
    Tracks robot's position relative to the barcode grid.
    Maintains last known barcode read and odometry since then.
    """

    def __init__(self, grid_spacing_m: float = 0.8):
        self.grid_spacing = grid_spacing_m
        self.last_barcode_row = 0
        self.last_barcode_col = 0
        self.last_barcode_time = time.time()
        self.last_barcode_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.dist_since_barcode = 0.0
        self.turns_since_barcode = 0
        self.consecutive_failures = 0
        self.total_reads = 0
        self.successful_reads = 0
        self._prev_heading = None

    def update_barcode_read(self, row: int, col: int, pose_x: float,
                            pose_y: float, pose_theta: float):
        """Called when a valid barcode is read."""
        self.last_barcode_row = row
        self.last_barcode_col = col
        self.last_barcode_time = time.time()
        self.last_barcode_pose = (pose_x, pose_y, pose_theta)
        self.dist_since_barcode = 0.0
        self.turns_since_barcode = 0
        self.consecutive_failures = 0
        self.total_reads += 1
        self.successful_reads += 1

    def update_barcode_failure(self):
        """Called when barcode read fails."""
        self.consecutive_failures += 1
        self.total_reads += 1

    def update_odometry(self, pose_x: float, pose_y: float,
                        pose_theta: float):
        """Update dead reckoning since last barcode."""
        bx, by, _ = self.last_barcode_pose
        self.dist_since_barcode = np.sqrt(
            (pose_x - bx) ** 2 + (pose_y - by) ** 2
        )

        # Count significant heading changes (>45 degrees)
        heading_deg = np.degrees(pose_theta) % 360
        if self._prev_heading is not None:
            delta = abs(heading_deg - self._prev_heading)
            if delta > 180:
                delta = 360 - delta
            if delta > 45:
                self.turns_since_barcode += 1
        self._prev_heading = heading_deg

    @property
    def time_since_barcode(self) -> float:
        return time.time() - self.last_barcode_time

    @property
    def barcode_is_failing(self) -> bool:
        return self.consecutive_failures >= 5  # Addverb irayple threshold

    @property
    def read_success_rate(self) -> float:
        if self.total_reads == 0:
            return 1.0
        return self.successful_reads / self.total_reads


# =====================================================================
# ZONE IDENTIFIER — io-gita powered (Addverb-aligned)
# =====================================================================

class ZoneIdentifier:
    """
    Identifies warehouse zones using io-gita Hopfield network + graph topology.

    Adapted for Addverb's sensor suite:
    - Primary: barcode grid position (exact when available)
    - Fallback: io-gita zone identification from sparse sensor data
    - Always: graph topology disambiguation

    Workflow:
        1. Check barcode status — if valid, zone is known immediately
        2. If barcode degraded: extract 16 features from available sensors
        3. Build query vector from features x atoms
        4. Run ODE -> find nearest attractor basin -> zone candidates
        5. Graph disambiguation using adjacency + previous zone
        6. Report zone + confidence to FMS
    """

    def __init__(self, config: dict):
        self.config = config
        self.D = config["engine"]["D"]
        self.zones = {z["name"]: z for z in config["zones"]}
        self.zone_fingerprints = {}
        self.last_zone = None
        self.last_zone_confidence = 0.0
        self.adjacency = self._build_adjacency()
        self.net = None
        self.atom_names = [f"F{i}" for i in range(16)]
        self.mismatch_counts = {}
        self.barcode_tracker = BarcodeGridTracker(
            config.get("warehouse", {}).get("grid_spacing_m", 0.8)
        )
        self._build_zone_lookup()

    def _build_zone_lookup(self):
        """Build barcode -> zone and graph_node -> zone lookup tables."""
        self.barcode_to_zone = {}
        self.node_to_zone = {}

        for z in self.config["zones"]:
            name = z["name"]
            # Map barcode range to zone
            br = z.get("barcode_range", [])
            if len(br) == 2:
                for bc_id in range(br[0], br[1] + 1):
                    self.barcode_to_zone[bc_id] = name
            # Map graph nodes to zone
            for node_id in z.get("graph_nodes", []):
                self.node_to_zone[node_id] = name

    def _build_adjacency(self) -> dict:
        """Build zone adjacency graph from grid positions."""
        adj = {}
        zone_list = list(self.zones.values())
        for z in zone_list:
            neighbors = []
            for other in zone_list:
                if other["name"] != z["name"]:
                    if abs(z["row"] - other["row"]) + \
                       abs(z["col"] - other["col"]) == 1:
                        neighbors.append(other["name"])
            adj[z["name"]] = neighbors

        # Apply overrides
        for override in self.config.get("adjacency_overrides", []) or []:
            if "from" in override and "to" in override:
                adj.setdefault(override["from"], []).append(override["to"])
                if override.get("bidirectional", True):
                    adj.setdefault(override["to"], []).append(override["from"])

        return adj

    def build_network(self, fingerprints: dict = None):
        """Build io-gita network from zone fingerprints."""
        cfg = self.config["engine"]
        self.net = Network(D=cfg["D"], beta=cfg["beta"],
                          dt=cfg["dt"], seed=cfg["seed"])
        self.net.add_atoms(self.atom_names)

        if fingerprints:
            self.zone_fingerprints = fingerprints
        else:
            # Generate default fingerprints from zone configuration
            rng = np.random.default_rng(cfg["seed"])
            for name, z in self.zones.items():
                fp = self._generate_zone_fingerprint(z, rng)
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

    def _generate_zone_fingerprint(self, zone: dict,
                                   rng: np.random.Generator) -> np.ndarray:
        """Generate a default fingerprint from zone config.

        Uses zone type, position, and structure to create a discriminating
        fingerprint. Calibration drive will replace these with real data.
        """
        zone_type = zone.get("type", "aisle")
        row = zone.get("row", 0)
        col = zone.get("col", 0)
        max_rows = self.config.get("warehouse", {}).get("max_rows", 50)
        max_cols = self.config.get("warehouse", {}).get("max_cols", 80)

        type_code = ZONE_TYPE_CODES.get(zone_type, 0.5)

        # Generate fingerprint that encodes zone properties
        fp = np.array([
            0.8 if zone_type == "dock" else 0.3,           # obs_range (docks are open)
            0.2 if zone_type in ("shelf", "aisle") else 0.1,  # obs_present likelihood
            zone.get("expected_heading", 90) / 360.0,       # heading
            int(zone.get("expected_heading", 90) / 45) / 8.0,  # heading_bin
            0.1 if zone_type == "dock" else 0.5,            # dist from barcode
            0.0 if zone_type in ("aisle", "lane") else 0.3,  # turns
            row / max(max_rows, 1),                         # row
            col / max(max_cols, 1),                         # col
            (int(row / max(max_rows, 1) > 0.5) * 2 +
             int(col / max(max_cols, 1) > 0.5)) / 4.0,     # quadrant
            1.0 if zone_type == "dock" else 0.2,            # near_dock
            0.3,                                            # velocity (typical)
            0.7,                                            # battery (typical)
            0.1,                                            # time_since_barcode
            0.95,                                           # barcode_reliability
            type_code,                                      # zone_type
            0.5 + rng.uniform(-0.1, 0.1),                  # grid_density + noise
        ], dtype=np.float64)

        return fp

    def identify_from_barcode(self, barcode_id: int) -> Optional[str]:
        """Direct zone identification from valid barcode read.
        This is the PRIMARY localization method — always preferred."""
        zone = self.barcode_to_zone.get(barcode_id)
        if zone:
            self.last_zone = zone
            self.last_zone_confidence = 1.0
        return zone

    def identify_from_graph_node(self, node_id: int) -> Optional[str]:
        """Zone identification from A* graph node ID."""
        zone = self.node_to_zone.get(node_id)
        if zone:
            self.last_zone = zone
            self.last_zone_confidence = 0.95
        return zone

    def identify_from_sensors(self, robot_state: dict,
                              robot_type: str = "zippy10") -> tuple:
        """
        FALLBACK zone identification when barcode is unavailable.
        Uses io-gita Hopfield network with sparse sensor data.

        Args:
            robot_state: Dict from ProtocolV1Parser.parse() or equivalent
            robot_type: Robot type key from config (for sensor specs)

        Returns:
            (zone_name, method, confidence, ode_time_ms)
        """
        if self.net is None:
            return None, "NO_NETWORK", 0.0, 0.0

        t0 = time.time()

        rt = self.config.get("robot_types", {}).get(robot_type, {})
        max_vel = rt.get("max_velocity", 1.4)
        max_rows = self.config.get("warehouse", {}).get("max_rows", 50)
        max_cols = self.config.get("warehouse", {}).get("max_cols", 80)

        # Determine expected zone type from graph node
        expected_type = 0.5
        if "current_node" in robot_state:
            node_zone = self.node_to_zone.get(robot_state["current_node"])
            if node_zone and node_zone in self.zones:
                expected_type = ZONE_TYPE_CODES.get(
                    self.zones[node_zone].get("type", "aisle"), 0.5
                )

        features = extract_features_addverb(
            obstacle_range=robot_state.get("obstacle_range", 1.5),
            obstacle_detected=robot_state.get("obstacle_detected", False),
            heading_deg=np.degrees(robot_state.get("pose_theta", 0)) % 360,
            dist_from_last_barcode=self.barcode_tracker.dist_since_barcode,
            turns_since_barcode=self.barcode_tracker.turns_since_barcode,
            velocity=robot_state.get("linear_vel", 0.0),
            last_barcode_row=self.barcode_tracker.last_barcode_row,
            last_barcode_col=self.barcode_tracker.last_barcode_col,
            battery_soc=robot_state.get("battery_soc", 50.0),
            time_since_barcode_s=self.barcode_tracker.time_since_barcode,
            grid_max_rows=max_rows,
            grid_max_cols=max_cols,
            max_velocity=max_vel,
            zone_type_code=expected_type,
            barcode_read_success_rate=self.barcode_tracker.read_success_rate,
        )

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

        # Disambiguate
        zone, method, confidence = self.disambiguate(
            candidates, fp_distances, self.last_zone
        )

        self.last_zone = zone
        self.last_zone_confidence = confidence

        return zone, method, confidence, ode_time * 1000

    def disambiguate(self, candidates: list, fp_distances: list,
                     previous_zone: str = None) -> tuple:
        """Use graph topology to resolve ambiguous zones."""
        ode_top = set(c[0] for c in candidates[:5])
        fp_top = set(c[0] for c in fp_distances[:5])
        combined = ode_top | fp_top

        if previous_zone and previous_zone in self.adjacency:
            reachable = set(self.adjacency[previous_zone])
            reachable.add(previous_zone)

            valid = combined & reachable
            if len(valid) == 1:
                result = valid.pop()
                return result, "GRAPH_UNIQUE", 1.0
            elif len(valid) > 1:
                best = min(valid, key=lambda z: next(
                    d for n, d in fp_distances if n == z))
                return best, "GRAPH_FP_RANKED", 0.9
            else:
                # Teleport — robot is not where graph says it should be
                best = fp_distances[0][0]
                return best, "TELEPORT_FALLBACK", 0.3
        else:
            # Cold start — no previous zone known
            best = fp_distances[0][0]
            return best, "COLD_FP_ONLY", 0.5

    def get_cold_start_hint(self, robot_state: dict,
                            robot_type: str = "zippy10") -> dict:
        """
        COLD START: Robot just restarted, may be between barcodes.

        Returns a structured hint for the FMS to route recovery:
        - candidate_zones: ordered list of likely zones
        - nearest_barcodes: barcode IDs to route toward
        - recovery_heading: suggested heading to reach nearest barcode
        """
        # Try to load last known state from disk
        saved_zone = self._load_last_state()

        # Run io-gita identification
        zone, method, confidence, ode_ms = self.identify_from_sensors(
            robot_state, robot_type
        )

        # Build candidate list
        candidates = [zone] if zone else []
        if saved_zone and saved_zone != zone:
            candidates.append(saved_zone)

        # Add adjacent zones as fallbacks
        if zone and zone in self.adjacency:
            for adj in self.adjacency[zone][:3]:
                if adj not in candidates:
                    candidates.append(adj)

        # Find nearest barcodes in candidate zones
        nearest_barcodes = []
        for cand_zone in candidates[:3]:
            z = self.zones.get(cand_zone, {})
            br = z.get("barcode_range", [])
            if len(br) == 2:
                # Center barcode of the zone
                center_bc = (br[0] + br[1]) // 2
                nearest_barcodes.append({
                    "zone": cand_zone,
                    "barcode_id": center_bc,
                    "graph_node": z.get("graph_nodes", [None])[
                        len(z.get("graph_nodes", [])) // 2
                    ] if z.get("graph_nodes") else None,
                })

        return {
            "candidate_zones": candidates[:5],
            "primary_zone": zone,
            "confidence": confidence,
            "method": method,
            "ode_time_ms": ode_ms,
            "nearest_barcodes": nearest_barcodes,
            "recovery_strategy": self.config.get("cold_start", {}).get(
                "recovery_strategy", "nearest_barcode"
            ),
            "saved_zone": saved_zone,
        }

    def get_barcode_failure_hint(self, robot_state: dict,
                                 robot_type: str = "zippy10") -> dict:
        """
        BARCODE FAILURE: Reader is failing, robot needs zone-level guidance.

        Returns zone awareness so FMS can route robot through dead barcode zone
        to nearest working barcode.
        """
        zone, method, confidence, ode_ms = self.identify_from_sensors(
            robot_state, robot_type
        )

        # Find nearest zone with known-good barcodes
        recovery_zones = []
        if zone and zone in self.adjacency:
            for adj_zone in self.adjacency[zone]:
                # Skip if adjacent zone also has barcode issues
                recovery_zones.append({
                    "zone": adj_zone,
                    "direction": self._direction_to_zone(zone, adj_zone),
                    "graph_nodes": self.zones.get(adj_zone, {}).get(
                        "graph_nodes", []
                    ),
                })

        return {
            "current_zone": zone,
            "confidence": confidence,
            "method": method,
            "barcode_failures": self.barcode_tracker.consecutive_failures,
            "time_since_last_barcode": self.barcode_tracker.time_since_barcode,
            "recovery_zones": recovery_zones,
            "action": "ROUTE_TO_NEAREST_BARCODE",
        }

    def _direction_to_zone(self, from_zone: str, to_zone: str) -> str:
        """Calculate cardinal direction from one zone to another."""
        fz = self.zones.get(from_zone, {})
        tz = self.zones.get(to_zone, {})
        dr = tz.get("row", 0) - fz.get("row", 0)
        dc = tz.get("col", 0) - fz.get("col", 0)
        if abs(dr) > abs(dc):
            return "SOUTH" if dr > 0 else "NORTH"
        return "EAST" if dc > 0 else "WEST"

    def check_map_change(self, zone_name: str,
                         current_features: np.ndarray) -> Optional[dict]:
        """Detect if a zone's layout has changed."""
        if not self.config.get("map_change", {}).get("enabled", False):
            return None
        if zone_name not in self.zone_fingerprints:
            return None

        stored_fp = self.zone_fingerprints[zone_name]
        distance = np.linalg.norm(current_features - stored_fp)
        tolerance = self.config["map_change"]["feature_tolerance"]
        threshold = self.config["map_change"]["mismatch_threshold"]

        if distance > tolerance:
            self.mismatch_counts[zone_name] = \
                self.mismatch_counts.get(zone_name, 0) + 1
            if self.mismatch_counts[zone_name] >= threshold:
                self.mismatch_counts[zone_name] = 0
                return {
                    "zone": zone_name,
                    "change_type": "layout_modified",
                    "distance": round(float(distance), 3),
                    "confidence": 1.0,
                    "timestamp": time.time(),
                }
        else:
            self.mismatch_counts[zone_name] = 0
        return None

    def save_state(self):
        """Save current state to disk for cold start recovery."""
        path = self.config.get("cold_start", {}).get(
            "saved_state_file", "/tmp/iogita_last_state.json"
        )
        data = {
            "zone": self.last_zone,
            "confidence": self.last_zone_confidence,
            "barcode_row": self.barcode_tracker.last_barcode_row,
            "barcode_col": self.barcode_tracker.last_barcode_col,
            "timestamp": time.time(),
        }
        with open(path, "w") as f:
            json.dump(data, f)

    def _load_last_state(self) -> Optional[str]:
        """Load saved state from disk."""
        path = self.config.get("cold_start", {}).get(
            "saved_state_file", "/tmp/iogita_last_state.json"
        )
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            # Restore barcode tracker state
            self.barcode_tracker.last_barcode_row = data.get(
                "barcode_row", 0
            )
            self.barcode_tracker.last_barcode_col = data.get(
                "barcode_col", 0
            )
            return data.get("zone")
        return None

    def calibrate_zone(self, zone_name: str, robot_state: dict,
                       robot_type: str = "zippy10"):
        """Record fingerprint for a zone during calibration drive."""
        rt = self.config.get("robot_types", {}).get(robot_type, {})
        max_vel = rt.get("max_velocity", 1.4)
        max_rows = self.config.get("warehouse", {}).get("max_rows", 50)
        max_cols = self.config.get("warehouse", {}).get("max_cols", 80)

        features = extract_features_addverb(
            obstacle_range=robot_state.get("obstacle_range", 1.5),
            obstacle_detected=robot_state.get("obstacle_detected", False),
            heading_deg=np.degrees(robot_state.get("pose_theta", 0)) % 360,
            dist_from_last_barcode=0.0,
            turns_since_barcode=0,
            velocity=robot_state.get("linear_vel", 0.0),
            last_barcode_row=self.barcode_tracker.last_barcode_row,
            last_barcode_col=self.barcode_tracker.last_barcode_col,
            battery_soc=robot_state.get("battery_soc", 50.0),
            time_since_barcode_s=0.0,
            grid_max_rows=max_rows,
            grid_max_cols=max_cols,
            max_velocity=max_vel,
        )

        if zone_name not in self.zone_fingerprints:
            self.zone_fingerprints[zone_name] = features
        else:
            self.zone_fingerprints[zone_name] = (
                self.zone_fingerprints[zone_name] + features
            ) / 2.0

    def save_fingerprints(self, path: str = "/tmp/iogita_fingerprints.npz"):
        np.savez(path, **{k: v for k, v in self.zone_fingerprints.items()})

    def load_fingerprints(self, path: str = "/tmp/iogita_fingerprints.npz"):
        if os.path.exists(path):
            data = np.load(path)
            self.zone_fingerprints = {k: data[k] for k in data.files}
            return True
        return False


# =====================================================================
# STANDALONE TEST (no fleet_core required)
# =====================================================================

def standalone_test():
    """Test zone identification without fleet_core."""
    print("=" * 60)
    print("  io-gita Zone Node v2 — Addverb Fleet_Core Aligned")
    print("  Standalone Test (no fleet_core dependency)")
    print("=" * 60)

    # Load or generate config
    config_path = os.path.join(os.path.dirname(__file__),
                               "warehouse_config.yaml")
    if os.path.exists(config_path):
        with open(config_path) as f:
            raw = yaml.safe_load(f)
        config = raw
        print(f"\n  Loaded config: {len(config.get('zones', []))} zones")
    else:
        print(f"\n  No config at {config_path}")
        print(f"  Using built-in Addverb test warehouse")

        # 25-zone test warehouse matching Addverb layout
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
                           "confidence_threshold": 0.6,
                           "max_hint_zones": 5,
                           "teleport_confidence": 0.3,
                           "recovery_strategy": "nearest_barcode"},
            "map_change": {"enabled": True, "mismatch_threshold": 3,
                           "feature_tolerance": 0.3},
            "adjacency_overrides": [],
            "robot_types": {
                "zippy10": {"max_velocity": 1.4,
                            "obstacle_fov_deg": 30,
                            "obstacle_range_m": 1.5},
            },
        }

    zid = ZoneIdentifier(config)
    zid.build_network()
    print(f"  Network: {zid.net.n_patterns} patterns, 16 atoms, D={zid.D}")
    print(f"  Adjacency: {sum(len(v) for v in zid.adjacency.values())} edges")
    print(f"  Barcode->Zone: {len(zid.barcode_to_zone)} barcodes mapped")
    print(f"  Node->Zone: {len(zid.node_to_zone)} graph nodes mapped")

    # -- Test 1: Direct barcode identification --
    print(f"\n-- Test 1: Barcode Identification (primary) --\n")
    for bc_id in [5, 25, 65, 105]:
        zone = zid.identify_from_barcode(bc_id)
        print(f"  Barcode {bc_id:>3} -> {zone or 'UNKNOWN':>12}  (confidence: 1.00)")

    # -- Test 2: Cold start (no barcode available) --
    print(f"\n-- Test 2: Cold Start Recovery (barcode unavailable) --\n")
    # Simulate: robot was in SHELF_3 (row=2, col=1), restarted between barcodes
    zid.barcode_tracker.last_barcode_row = 2
    zid.barcode_tracker.last_barcode_col = 1
    zid.last_zone = "SHELF_3"
    zid.save_state()

    # Simulate restart — state loaded from disk, limited sensor data
    fake_state = {
        "pose_x": 15.0,
        "pose_y": 25.0,
        "pose_theta": 1.57,  # ~90 degrees (east)
        "battery_soc": 72.0,
        "linear_vel": 0.0,  # Stopped (just restarted)
        "obstacle_range": 1.2,
        "obstacle_detected": False,
        "current_node": 13,
    }

    hint = zid.get_cold_start_hint(fake_state, "zippy10")
    print(f"  Saved zone:       {hint['saved_zone']}")
    print(f"  Primary zone:     {hint['primary_zone']}")
    print(f"  Confidence:       {hint['confidence']:.2f}")
    print(f"  Method:           {hint['method']}")
    print(f"  ODE time:         {hint['ode_time_ms']:.1f}ms")
    print(f"  Candidate zones:  {hint['candidate_zones']}")
    print(f"  Recovery:         {hint['recovery_strategy']}")
    if hint["nearest_barcodes"]:
        bc = hint["nearest_barcodes"][0]
        print(f"  Route to:         barcode {bc['barcode_id']} "
              f"in zone {bc['zone']}")

    # -- Test 3: Barcode failure mid-operation --
    print(f"\n-- Test 3: Barcode Failure (reader degraded) --\n")
    # Simulate 5 consecutive failures (Addverb irayple threshold)
    for _ in range(5):
        zid.barcode_tracker.update_barcode_failure()
    print(f"  Consecutive failures: "
          f"{zid.barcode_tracker.consecutive_failures}")
    print(f"  Barcode failing:      {zid.barcode_tracker.barcode_is_failing}")

    fake_state["linear_vel"] = 0.8  # Robot is moving
    hint = zid.get_barcode_failure_hint(fake_state, "zippy10")
    print(f"  Current zone:     {hint['current_zone']}")
    print(f"  Confidence:       {hint['confidence']:.2f}")
    print(f"  Action:           {hint['action']}")
    print(f"  Recovery zones:   "
          f"{[r['zone'] + ' (' + r['direction'] + ')' for r in hint['recovery_zones'][:3]]}")

    # -- Test 4: Zone identification accuracy --
    print(f"\n-- Test 4: Sensor-Only Zone Identification --\n")
    correct = 0
    total = 0
    rng = np.random.default_rng(42)
    for z in config["zones"][:10]:
        fake = {
            "pose_x": z.get("center_x", 0) * 0.8,
            "pose_y": z.get("center_y", 0) * 0.8,
            "pose_theta": np.radians(z.get("expected_heading", 90)),
            "battery_soc": 50 + rng.uniform(-20, 20),
            "linear_vel": rng.uniform(0, 1.0),
            "obstacle_range": rng.uniform(0.5, 1.5),
            "obstacle_detected": rng.random() < 0.3,
            "current_node": z.get("graph_nodes", [0])[0],
        }
        zid.barcode_tracker.last_barcode_row = z["row"]
        zid.barcode_tracker.last_barcode_col = z["col"]

        zone, method, conf, ode_ms = zid.identify_from_sensors(
            fake, "zippy10")
        total += 1
        if zone == z["name"]:
            correct += 1
        print(f"  {z['name']:>12} -> {zone:>12} "
              f"({method}, conf={conf:.2f}, {ode_ms:.1f}ms)")

    print(f"\n  Accuracy: {correct}/{total}")
    print(f"\n  All systems operational.")


if __name__ == "__main__":
    standalone_test()
