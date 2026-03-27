# io-gita Cold Start + Aliasing — Complete Integration Guide

**For:** Integration Partner
**From:** Meharban Singh, Adaptive Mind
**Date:** 2026-03-27
**Version:** 1.0
**License:** 90-day evaluation (expires 2026-06-27)
**ROS Version:** ROS1 (Noetic) — ROS2 alternative code included but commented out

---

## Quick Summary

io-gita solves two warehouse robot problems:
1. **Cold Start** — Robot restarts → 4-5 sec frozen → io-gita reduces to <1 sec
2. **Perceptual Aliasing** — Identical aisles confuse AMCL → io-gita uses graph topology to resolve

**Your work:** Add ONE ROS2 subscriber (~20 lines) + provide zone layout config.
**Our work:** Provide the io-gita engine + zone identification node (ready to run).

---

## Table of Contents

1. [What's In This Package](#1-whats-in-this-package)
2. [Prerequisites](#2-prerequisites)
3. [Installation](#3-installation)
4. [Configuration — WHAT YOU MODIFY](#4-configuration--what-you-modify)
5. [Files YOU Modify (Your Side)](#5-files-you-modify-your-side)
6. [Files WE Provide (Do NOT Modify)](#6-files-we-provide-do-not-modify)
7. [Step-by-Step Integration](#7-step-by-step-integration)
8. [Testing & Debugging](#8-testing--debugging)
9. [ROS2 Topic Reference](#9-ros2-topic-reference)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. What's In This Package

```
deliverable/
├── INTEGRATION_GUIDE.md          ← THIS FILE (read first)
├── sg_engine_locked/             ← io-gita engine (time-locked, do NOT modify)
│   └── __init__.py               ← License-wrapped engine
├── debug/                        ← Debug & diagnostic tools
│   ├── iogita_diagnostics.py     ← Run FIRST after install (validates everything)
│   └── ros2_debug_node.py        ← Live debug node for robot testing
├── ros2_integration/             ← ROS2 integration files
│   ├── iogita_zone_node.py       ← Main zone identification node (ours)
│   ├── warehouse_config.yaml     ← Warehouse layout config (YOU EDIT THIS)
│   └── amcl_hook_example.py      ← Example AMCL subscriber (YOU ADD THIS)
└── cold_start_aliasing/          ← Demo code + results (reference only)
    ├── cold_start_v2.py          ← Full demo with 25 zones
    ├── cold_start_demo.py        ← Simpler demo
    └── ENDPOINTS_COLD_START.md   ← All ROS2 topics documented

sg_engine/                        ← Core engine (included, do NOT modify)
├── __init__.py
├── network.py                    ← Hopfield network + ODE dynamics
├── atlas.py                      ← Transition graph mapper
├── planner.py                    ← Route planner
├── bridge.py                     ← Bridge pattern creator
└── ...
```

---

## 2. Prerequisites

**On the robot / development machine:**

```bash
# Python 3.8+
python3 --version   # Must be >= 3.8

# Required Python packages
pip install numpy scipy

# Optional (for diagnostics)
pip install scikit-learn pandas

# ROS2 (for deployment)
# Humble or Iron — either works
ros2 --version
```

**ROS1:**
```bash
# ROS Noetic required
rosversion -d   # Should print "noetic"
```

**Hardware:** No new hardware. Uses existing:
- 2D LiDAR (`/scan` topic)
- Wheel encoders (`/odom` topic)
- IMU (`/imu/data` topic, optional but recommended)

---

## 3. Installation

```bash
# Step 1: Install io-gita engine
cd /path/to/deliverable
pip install -e ../   # Install sg_engine from the repo root

# Step 2: Verify installation
python debug/iogita_diagnostics.py

# Expected output:
#   [OK] Import sg_engine: All core modules imported
#   [OK] NumPy performance: D=10000 dot product: <100us
#   [OK] Network build: Built network
#   [OK] ODE dynamics: ODE converged
#   [OK] Determinism: 3 runs → identical output
#   [OK] Cold start pipeline: Full pipeline: <200ms
#   SUMMARY: All systems operational

# Step 3: Check license
python -c "from sg_engine_locked import license_info; license_info()"
```

---

## 4. Configuration — WHAT YOU MODIFY

### There is ONE config file you must customize:

### `ros2_integration/warehouse_config.yaml`

This file defines YOUR warehouse layout. You provide:

```yaml
# ═══════════════════════════════════════════════════════════
# WAREHOUSE CONFIGURATION — EDIT THIS FOR YOUR WAREHOUSE
# ═══════════════════════════════════════════════════════════

warehouse:
  name: "YOUR_WAREHOUSE_NAME"         # ← Change this
  grid_spacing_m: 10.0                # Distance between zone centers (meters)

# ── ZONE DEFINITIONS ──────────────────────────────────────
# Each zone needs:
#   name:     Unique zone identifier (used in all io-gita outputs)
#   row, col: Grid position (for graph building)
#   type:     One of: dock, aisle, shelf, cross, hub, lane, mid, custom
#   heading:  Typical robot heading when entering (degrees, 0=North)
#   dist_from_dock: Approximate distance from nearest dock (meters)
#
# IMPORTANT: Include ALL navigable zones. Missing zones = gaps in graph.

zones:
  - name: "DOCK_A"
    row: 0
    col: 0
    type: "dock"
    heading: 90
    dist_from_dock: 0
    map_x: 5.0        # ← YOUR map coordinates (meters, from map origin)
    map_y: 5.0         # ← YOUR map coordinates

  - name: "AISLE_1"
    row: 0
    col: 1
    type: "aisle"
    heading: 90
    dist_from_dock: 5
    map_x: 15.0
    map_y: 5.0

  - name: "SHELF_1"
    row: 1
    col: 1
    type: "shelf"
    heading: 90
    dist_from_dock: 12
    map_x: 15.0
    map_y: 15.0

  # ... ADD ALL YOUR ZONES HERE ...

# ── ADJACENCY (optional — auto-detected from row/col if omitted) ──
# Override if your warehouse has non-grid connections (ramps, shortcuts)
adjacency_overrides:
  # - from: "DOCK_A"
  #   to: "SHELF_3"      # Direct connection that grid wouldn't detect
  # - remove: "AISLE_1"
  #   to: "CROSS_N"      # Block a connection that grid would detect

# ── ZONE TYPE SIGNATURES ──────────────────────────────────
# Default LiDAR signatures by zone type.
# Override per-zone if a specific zone looks different from its type.
# These are learned during the CALIBRATION DRIVE.
zone_type_defaults:
  dock:
    front_clear_m: 8.0
    back_clear_m: 0.5
    left_clear_m: 0.5
    right_clear_m: 4.0
    scan_variance: "low"
    gap_count: "few"

  aisle:
    front_clear_m: 6.0
    back_clear_m: 6.0
    left_clear_m: 1.2
    right_clear_m: 1.2
    scan_variance: "low"
    gap_count: "many"

  shelf:
    front_clear_m: 1.5
    back_clear_m: 1.5
    left_clear_m: 1.2
    right_clear_m: 1.2
    scan_variance: "high"
    gap_count: "many"

  cross:
    front_clear_m: 4.0
    back_clear_m: 4.0
    left_clear_m: 4.0
    right_clear_m: 4.0
    scan_variance: "low"
    gap_count: "few"

  hub:
    front_clear_m: 5.0
    back_clear_m: 5.0
    left_clear_m: 5.0
    right_clear_m: 5.0
    scan_variance: "low"
    gap_count: "few"

  lane:
    front_clear_m: 3.0
    back_clear_m: 3.0
    left_clear_m: 0.5
    right_clear_m: 2.0
    scan_variance: "medium"
    gap_count: "few"

# ── ENGINE PARAMETERS ────────────────────────────────────
# These are io-gita engine defaults. DO NOT CHANGE unless advised.
engine:
  D: 10000               # Dimension (leave at 10000)
  beta: 4.0              # Temperature (leave at 4.0)
  dt: 0.05               # ODE step size (leave at 0.05)
  seed: 42               # Random seed (leave at 42 for reproducibility)
  n_scans_per_zone: 5    # Scans to average during calibration drive
  generated_patterns: 15  # Additional patterns for richer landscape

# ── COLD START SETTINGS ──────────────────────────────────
cold_start:
  saved_zone_file: "/tmp/iogita_last_zone.json"  # ← Change path if needed
  confidence_threshold: 0.6      # Min confidence to accept zone ID
  amcl_hint_max_zones: 5         # Max zones to send as AMCL hint
  teleport_confidence: 0.3       # Below this → assume robot was moved

# ── MAP CHANGE DETECTION ─────────────────────────────────
map_change:
  enabled: true
  mismatch_threshold: 3          # Consecutive mismatches before alerting
  fingerprint_tolerance: 0.3     # Max feature distance to count as "same"

# ── DEBUG ─────────────────────────────────────────────────
debug:
  enabled: true
  log_file: "/tmp/iogita_debug.jsonl"
  publish_debug_topic: true      # Publish to /iogita/debug
  verbose: false                 # Extra console output
```

### What you MUST fill in:

| Field | What You Provide | Where to Get It |
|-------|-----------------|-----------------|
| `zones[].name` | Your zone names | Your warehouse map |
| `zones[].row, col` | Grid position | Count from top-left of warehouse |
| `zones[].type` | Zone type | dock/aisle/shelf/cross/hub/lane |
| `zones[].map_x, map_y` | Map coordinates (m) | From your Nav2 map (same frame as AMCL) |
| `zones[].heading` | Typical entry heading | Which direction robot usually faces |
| `cold_start.saved_zone_file` | Path on robot filesystem | Pick any writable path |

### What you should NOT change:

| Field | Why |
|-------|-----|
| `engine.D` | Must match the pre-trained patterns |
| `engine.beta` | Tuned for optimal ODE convergence |
| `engine.seed` | Changes patterns → breaks calibration |

---

## 5. Files YOU Modify (Your Side)

### File 1: `warehouse_config.yaml` (described above)

### File 2: AMCL Hook — Add ONE Node (~30 lines of actual code)

Run the provided `amcl_hook_example.py` alongside your AMCL node.
It subscribes to `/iogita/zone_hint` and publishes a constrained `/initialpose`.

```bash
# ROS1: just run it as a node
rosrun your_package amcl_hook_example.py
```

The key code inside (see `amcl_hook_example.py` for full version):

```python
import json
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

# ← CHANGE: Your zone → map coordinate mapping
ZONE_COORDINATES = {
    "DOCK_A":  {"x": 5.0,  "y": 5.0,  "radius": 5.0},
    "SHELF_1": {"x": 15.0, "y": 15.0, "radius": 3.0},
    # ... must match warehouse_config.yaml zones
}

def on_zone_hint(msg):
    zones = json.loads(msg.data)
    for zone_name in zones:
        if zone_name in ZONE_COORDINATES:
            region = ZONE_COORDINATES[zone_name]
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.pose.position.x = region["x"]
            pose.pose.pose.position.y = region["y"]
            pose.pose.pose.orientation.w = 1.0
            cov = region["radius"] ** 2
            pose.pose.covariance[0]  = cov     # x variance
            pose.pose.covariance[7]  = cov     # y variance
            pose.pose.covariance[35] = 1.0     # theta variance
            initial_pose_pub.publish(pose)
            break

rospy.init_node("iogita_amcl_hook")
initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
rospy.Subscriber("/iogita/zone_hint", String, on_zone_hint)
rospy.spin()
```

### File 3: Zone Coordinate Mapping

You need to provide a mapping from zone names to map coordinates. This is used by:
- The AMCL hook (above) to constrain particle search
- The debug node to verify zone transitions

**Where:** Either in `warehouse_config.yaml` (field `map_x`, `map_y`) or in your AMCL hook code.

---

## 6. Files WE Provide (Do NOT Modify)

| File | Purpose | Modify? |
|------|---------|---------|
| `sg_engine/network.py` | Core Hopfield network + ODE | **NO** |
| `sg_engine/atlas.py` | Transition graph | **NO** |
| `sg_engine/__init__.py` | Engine entry point | **NO** |
| `sg_engine_locked/__init__.py` | License wrapper | **NO** (expiry config only) |
| `ros2_integration/iogita_zone_node.py` | Zone identification ROS2 node | **NO** |
| `debug/iogita_diagnostics.py` | Diagnostic tool | **NO** |
| `debug/ros2_debug_node.py` | Debug node | **NO** |
| `cold_start_v2.py` | Demo (reference) | **NO** (read-only reference) |

---

## 7. Step-by-Step Integration

### Phase 1: Setup (Day 1)

```bash
# 1. Install on development machine
pip install numpy scipy
pip install -e /path/to/io-gita/

# 2. Run diagnostics
python deliverable/debug/iogita_diagnostics.py
# ALL tests must pass

# 3. Run the demo
python deliverable/cold_start_aliasing/cold_start_v2.py
# Verify it completes and prints results
```

### Phase 2: Configure (Day 1-2)

```bash
# 1. Edit warehouse_config.yaml with YOUR zone layout
#    - List all zones with names, grid positions, types
#    - Add map coordinates (map_x, map_y)
#    - Set cold_start.saved_zone_file path

# 2. Verify config
python -c "
import yaml
with open('deliverable/ros2_integration/warehouse_config.yaml') as f:
    cfg = yaml.safe_load(f)
zones = cfg['warehouse']['zones'] if 'warehouse' in cfg else cfg['zones']
print(f'Zones defined: {len(zones)}')
for z in zones:
    print(f'  {z[\"name\"]:>12} ({z[\"row\"]},{z[\"col\"]}) type={z[\"type\"]}')
"
```

### Phase 3: Calibration Drive (Day 2-3)

```bash
# Drive the robot through ALL zones once.
# Record LiDAR scans at each zone.
# The iogita_zone_node learns fingerprints during this drive.

# Launch the calibration:
rosrun iogita iogita_zone_node.py \
    _config_file:=deliverable/ros2_integration/warehouse_config.yaml \
    _mode:=calibration

# Drive robot through every zone. Node records fingerprints.
# When done, fingerprints are saved to: /tmp/iogita_fingerprints.npz
```

### Phase 4: Deploy (Day 3-4)

```bash
# 1. Launch io-gita zone node (runs alongside your navigation stack)
rosrun iogita iogita_zone_node.py \
    _config_file:=deliverable/ros2_integration/warehouse_config.yaml \
    _mode:=production

# 2. Launch the AMCL hook (the ~30 line file from Section 5)
rosrun your_package amcl_hook_example.py

# 3. Launch debug node (optional but recommended)
rosrun iogita ros2_debug_node.py

# 4. Monitor
rostopic echo /iogita/zone
rostopic echo /iogita/zone_confidence
rostopic echo /iogita/debug
```

### Phase 5: Verify (Day 4-5)

```bash
# Test cold start: kill and restart AMCL
rosnode kill /amcl
sleep 2
roslaunch your_package amcl.launch
# Watch: /iogita/zone_hint should publish within 100ms
# Watch: AMCL should converge within 1 second

# Test aliasing
# Navigate to a shelf zone, check /iogita/zone
rostopic echo /iogita/zone -n 1
# Should show correct shelf (not just any shelf)
```

---

## 8. Testing & Debugging

### Run diagnostics anytime:
```bash
python deliverable/debug/iogita_diagnostics.py
```

### Check debug log:
```bash
# Real-time tail
tail -f /tmp/iogita_debug.jsonl | python -m json.tool

# Check for errors
grep '"level":"FAIL"' /tmp/iogita_debug.jsonl
grep 'TELEPORT' /tmp/iogita_debug.jsonl
```

### Common debug commands:
```bash
# Check if io-gita node is running
rosnode list | grep iogita

# Check published topics
rostopic list | grep iogita

# Monitor zone identification rate
rostopic hz /iogita/zone

# Check confidence distribution
rostopic echo /iogita/zone_confidence

# Full debug output
rostopic echo /iogita/debug
```

### Performance check:
```bash
python deliverable/debug/iogita_diagnostics.py --benchmark
# Expected: <200ms per zone identification
```

---

## 9. ROS2 Topic Reference

### Topics WE PUBLISH (io-gita → your system)

| Topic | Type | Rate | Content |
|-------|------|------|---------|
| `/iogita/zone` | `std_msgs/String` | On zone change | Current zone name, e.g. `"SHELF_3"` |
| `/iogita/zone_hint` | `std_msgs/String` | On cold start | JSON list: `["SHELF_1","SHELF_3","LANE_W"]` |
| `/iogita/zone_confidence` | `std_msgs/Float32` | With every `/zone` | 0.0-1.0 confidence |
| `/iogita/map_change` | `std_msgs/String` | On change detected | JSON alert (see ENDPOINTS doc) |
| `/iogita/debug` | `std_msgs/String` | Always (if enabled) | JSON debug blob |

### Topics WE SUBSCRIBE TO (your system → io-gita)

| Topic | Type | What We Need | Already Exists? |
|-------|------|-------------|-----------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° LiDAR ranges | YES (your LiDAR driver) |
| `/odom` | `nav_msgs/Odometry` | Wheel position + heading | YES (your base driver) |
| `/imu/data` | `sensor_msgs/Imu` | Orientation | YES (if you have IMU) |

### Topic YOU ADD (one new subscription)

| Topic | Type | Direction | What |
|-------|------|-----------|------|
| `/iogita/zone_hint` → `/initialpose` | Subscribe to hint, publish pose | Your AMCL hook reads hint, publishes constrained pose | The 20-line hook from Section 5 |

---

## 10. Troubleshooting

### "Import error: sg_engine not found"
```bash
pip install -e /path/to/io-gita/
python -c "import sg_engine; print(sg_engine.__version__)"
```

### "License expired"
Contact meharban@adaptive-mind.com for renewal.

### "Zone identification wrong"
```bash
# Check fingerprints were calibrated
ls /tmp/iogita_fingerprints.npz

# Re-run calibration drive if fingerprints are old
ros2 run iogita iogita_zone_node --ros-args -p mode:=calibration
```

### "Cold start still slow (>2 sec)"
```bash
# Check AMCL is receiving the hint
rostopic echo /iogita/zone_hint
# If empty → io-gita node not running or /scan not publishing

# Check your AMCL hook is subscribed
rostopic info /iogita/zone_hint
# Should show 1+ subscriber
```

### "Teleport detected (wrong zone after movement)"
```bash
# Check graph adjacency
grep "TELEPORT" /tmp/iogita_debug.jsonl
# If frequent → your warehouse_config.yaml adjacency is wrong
# Add missing connections in adjacency_overrides
```

### "Low confidence (<0.5) on all zones"
```bash
# Fingerprints may not match current warehouse layout
# Re-run calibration drive
# OR check if warehouse layout changed since calibration
```

### "ODE too slow (>500ms)"
```bash
python deliverable/debug/iogita_diagnostics.py --benchmark
# If slow: check CPU load, other processes consuming compute
# io-gita needs ~50ms on modern hardware (ARM or x86)
```

---

## Summary: What You Do vs What We Do

| Task | Who | Effort |
|------|-----|--------|
| Install sg_engine | You | 5 min |
| Edit warehouse_config.yaml | You | 1-2 hours |
| Add AMCL hook (20 lines) | You | 30 min |
| Drive robot through zones (calibration) | You | 30 min |
| Run diagnostics | You | 5 min |
| Provide io-gita engine | Us | Done |
| Provide zone identification node | Us | Done |
| Provide debug tools | Us | Done |
| Production license + support | Us | On request |

**Total integration effort: 1-2 days**

---

**Contact:** meharban@adaptive-mind.com
**License:** 90-day evaluation. Production license available on request.
**Patent pending.** Unauthorized redistribution prohibited.

*io-gita v0.3.0-eval | Semantic Gravity Engine | (c) 2026 Adaptive Mind*
