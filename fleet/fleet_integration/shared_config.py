"""
Shared Configuration — Single source of truth for test/demo configs.
Eliminates config duplication across 6+ files.
"""

import os
import yaml


def load_config(config_path: str = None) -> dict:
    """Load warehouse config from YAML file or return default test config."""
    if config_path and os.path.exists(config_path):
        with open(config_path) as f:
            return yaml.safe_load(f)
    return default_test_config()


def default_test_config(num_zones: int = 25) -> dict:
    """Generate default test warehouse config.

    Used by all demos, tests, and standalone scripts.
    Single source — change here, changes everywhere.
    """
    if num_zones == 5:
        return _build_5_zone()
    elif num_zones == 15:
        return _build_15_zone()
    return _build_25_zone()


def _build_zones(zone_defs: list) -> list:
    """Build zone config list from (name, row, col, type) tuples."""
    return [
        {"name": n, "row": r, "col": c, "type": t,
         "expected_heading": 90 if c % 2 == 0 else 0,
         "barcode_range": [i * 20 + 1, (i + 1) * 20],
         "graph_nodes": list(range(i * 5 + 1, (i + 1) * 5 + 1)),
         "center_x": c * 10, "center_y": r * 10,
         "has_charger": t in ("dock", "charging")}
        for i, (n, r, c, t) in enumerate(zone_defs)
    ]


def _base_config(zones: list) -> dict:
    """Build full config dict from zone list."""
    return {
        "warehouse": {"grid_spacing_m": 0.8, "max_rows": 50, "max_cols": 80},
        "zones": zones,
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05,
                    "seed": 42, "generated_patterns": 15,
                    "n_scans_per_zone": 5},
        "cold_start": {"saved_state_file": "/var/lib/iogita/last_state.json",
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


def _build_5_zone() -> dict:
    return _base_config(_build_zones([
        ("DOCK_A", 0, 0, "dock"), ("AISLE_1", 0, 1, "aisle"),
        ("SHELF_1", 1, 1, "shelf"), ("HUB", 1, 0, "hub"),
        ("CHARGING", 1, 2, "charging"),
    ]))


def _build_15_zone() -> dict:
    return _base_config(_build_zones([
        ("DOCK_A", 0, 0, "dock"), ("AISLE_1", 0, 1, "aisle"),
        ("CROSS_N", 0, 2, "cross"), ("AISLE_2", 0, 3, "aisle"),
        ("DOCK_B", 0, 4, "dock"),
        ("LANE_W", 1, 0, "lane"), ("SHELF_1", 1, 1, "shelf"),
        ("MID_N", 1, 2, "mid"), ("SHELF_2", 1, 3, "shelf"),
        ("LANE_E", 1, 4, "lane"),
        ("CROSS_W", 2, 0, "cross"), ("SHELF_3", 2, 1, "shelf"),
        ("HUB", 2, 2, "hub"), ("SHELF_4", 2, 3, "shelf"),
        ("CROSS_E", 2, 4, "cross"),
    ]))


def _build_25_zone() -> dict:
    return _base_config(_build_zones([
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
    ]))
