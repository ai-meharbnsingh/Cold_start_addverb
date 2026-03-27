# io-gita Cold Start + Aliasing — Complete Integration Guide

**For:** Addverb Technologies
**From:** Meharban Singh
**Date:** 2026-03-27
**Version:** 1.1
**License:** 7-day evaluation (expires 2026-04-03)
**ROS:** Compatible with both ROS1 (Noetic) and ROS2 (Humble/Iron) — auto-detected

---

## Quick Summary

io-gita solves two warehouse robot problems:
1. **Cold Start** — Robot restarts -> 4-5 sec frozen -> io-gita reduces to <1 sec
2. **Perceptual Aliasing** — Identical aisles confuse AMCL -> io-gita uses graph topology to resolve

**Your work:** Edit ONE config file + update zone coordinates in the AMCL hook.
**Our work:** Provide the io-gita engine + zone identification node (ready to run).

---

## Table of Contents

1. [What's In This Package](#1-whats-in-this-package)
2. [Prerequisites](#2-prerequisites)
3. [Installation](#3-installation)
4. [Configuration — WHAT YOU MODIFY](#4-configuration--what-you-modify)
5. [Files YOU Modify (2 files)](#5-files-you-modify-2-files)
6. [Files WE Provide (Do NOT Modify)](#6-files-we-provide-do-not-modify)
7. [Step-by-Step Integration](#7-step-by-step-integration)
8. [Testing and Debugging](#8-testing-and-debugging)
9. [ROS Topic Reference](#9-ros-topic-reference)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. What's In This Package

```
cold_start_addverb/
├── README.md                      <- Read first
├── INTEGRATION_GUIDE.md           <- THIS FILE (detailed steps)
├── pyproject.toml                 <- pip install -e .
│
├── sg_engine/                     <- io-gita engine (COMPILED BINARY — do NOT modify)
│   ├── __init__.py
│   ├── network.cpython-311-*.so   <- Compiled engine (no source code)
│   └── atlas.py                   <- Transition graph (used by diagnostics)
│
├── sg_engine_locked/              <- License wrapper (7-day evaluation)
│   └── __init__.py
│
├── ros_integration/               <- ROS integration (ROS1 + ROS2 compatible)
│   ├── warehouse_config.yaml      <- YOU EDIT THIS — your zone layout
│   ├── amcl_hook_example.py       <- YOU EDIT THIS — zone-to-map coordinates
│   └── iogita_zone_node.py        <- Zone identification node (ours — do NOT modify)
│
├── debug/                         <- Debug and diagnostic tools
│   ├── iogita_diagnostics.py      <- Run FIRST after install
│   └── ros2_debug_node.py         <- Live debug node for robot testing
│
└── cold_start_aliasing/           <- Demo code + results (reference only)
    ├── cold_start_v2.py           <- Full demo with 25 zones
    ├── cold_start_demo.py         <- Simpler demo
    ├── cold_start_results.json
    ├── cold_start_v2_results.json
    └── ENDPOINTS_COLD_START.md    <- All ROS topics documented
```

---

## 2. Prerequisites

**On the robot / development machine:**

```bash
# Python 3.11 required (compiled binary is Python 3.11)
python3 --version   # Must be 3.11.x

# Required Python packages
pip install numpy scipy pyyaml

# ROS1 (Noetic) OR ROS2 (Humble/Iron) — either works
# Auto-detected at runtime. No code changes needed.
rosversion -d       # ROS1: prints "noetic"
ros2 --version      # ROS2: prints version
```

**Hardware:** No new hardware. Uses existing:
- 2D LiDAR (`/scan` topic)
- Wheel encoders (`/odom` topic)
- IMU (`/imu/data` topic — optional but recommended)

---

## 3. Installation

```bash
# Step 1: Clone / extract this package
cd cold_start_addverb

# Step 2: Install io-gita engine
pip install -e .

# Step 3: Verify installation
python debug/iogita_diagnostics.py

# Expected output:
#   [OK] Import sg_engine: All core modules imported
#   [OK] NumPy performance: D=10000 dot product: <100us
#   [OK] Network build: Built network
#   [OK] ODE dynamics: ODE converged
#   [OK] Determinism: 3 runs -> identical output
#   [OK] Cold start pipeline: Full pipeline: <200ms
#   SUMMARY: All systems operational

# Step 4: Check license
python -c "from sg_engine_locked import license_info; license_info()"
```

---

## 4. Configuration — WHAT YOU MODIFY

### There are TWO files you must customize:

### File 1: `ros_integration/warehouse_config.yaml`

This file defines YOUR warehouse layout. You provide:

```yaml
warehouse:
  name: "YOUR_WAREHOUSE_NAME"         # <- Change this

zones:
  - name: "DOCK_A"                    # <- Your zone name
    row: 0                            # <- Grid row (0-indexed)
    col: 0                            # <- Grid column (0-indexed)
    type: "dock"                      # <- dock | aisle | shelf | cross | hub | lane | mid
    heading: 90                       # <- Typical robot heading entering zone (degrees)
    dist_from_dock: 0                 # <- Approx distance from nearest dock (meters)
    map_x: 5.0                        # <- X in YOUR map frame (meters)
    map_y: 5.0                        # <- Y in YOUR map frame (meters)
    map_zone_radius: 5.0              # <- Search radius for AMCL constraint (meters)

  - name: "AISLE_1"
    row: 0
    col: 1
    type: "aisle"
    # ... add ALL your zones
```

### What you MUST fill in:

| Field | What You Provide | Where to Get It |
|-------|-----------------|-----------------|
| `zones[].name` | Your zone names | Your warehouse map |
| `zones[].row, col` | Grid position | Count from top-left of warehouse |
| `zones[].type` | Zone type | dock / aisle / shelf / cross / hub / lane |
| `zones[].map_x, map_y` | Map coordinates (m) | From your AMCL map (same frame) |
| `zones[].heading` | Typical entry heading | Which direction robot usually faces |

### What you should NOT change:

| Field | Why |
|-------|-----|
| `engine.D` | Must be 10000 (matches compiled binary) |
| `engine.beta` | Tuned for optimal ODE convergence |
| `engine.seed` | Changes patterns -> breaks calibration |

### File 2: `ros_integration/amcl_hook_example.py`

Update the `ZONE_COORDINATES` dictionary with your actual map coordinates:

```python
ZONE_COORDINATES = {
    "DOCK_A":   {"x":  5.0, "y":  5.0, "radius": 5.0},   # <- YOUR coordinates
    "AISLE_1":  {"x": 15.0, "y":  5.0, "radius": 3.0},   # <- YOUR coordinates
    # ... must match warehouse_config.yaml zone names
}
```

These coordinates must be in the same map frame as your AMCL `/map` topic.

---

## 5. Files YOU Modify (2 files)

| # | File | What You Do | Effort |
|---|------|-------------|--------|
| 1 | `ros_integration/warehouse_config.yaml` | Fill in zone names, positions, types, map coordinates | 1-2 hours |
| 2 | `ros_integration/amcl_hook_example.py` | Update `ZONE_COORDINATES` dict with your map coords | 30 min |

That's it. Two files. Everything else runs as-is.

---

## 6. Files WE Provide (Do NOT Modify)

| File | Purpose | Why Not Modify |
|------|---------|----------------|
| `sg_engine/network.*.so` | Compiled Hopfield network + ODE engine | Binary — no source |
| `sg_engine/atlas.py` | Transition graph mapper | Used internally by diagnostics |
| `sg_engine/__init__.py` | Engine entry point | Import gate |
| `sg_engine_locked/__init__.py` | 7-day license wrapper | Enforces evaluation window |
| `ros_integration/iogita_zone_node.py` | Zone identification node | Core logic — do not touch |
| `debug/iogita_diagnostics.py` | Diagnostic tool | Run to verify installation |
| `debug/ros2_debug_node.py` | Live debug node | Optional monitoring |

---

## 7. Step-by-Step Integration

### Phase 1: Setup (Day 1)

```bash
# 1. Install
pip install numpy scipy pyyaml
pip install -e .

# 2. Run diagnostics — ALL tests must pass
python debug/iogita_diagnostics.py

# 3. Run the demo — verify it completes
python cold_start_aliasing/cold_start_v2.py
```

### Phase 2: Configure (Day 1-2)

```bash
# 1. Edit warehouse_config.yaml with YOUR zone layout
#    List ALL navigable zones with names, grid positions, types, map coordinates

# 2. Update ZONE_COORDINATES in amcl_hook_example.py
#    Must match the zone names in warehouse_config.yaml

# 3. Verify config
python -c "
import yaml
with open('ros_integration/warehouse_config.yaml') as f:
    cfg = yaml.safe_load(f)
zones = cfg.get('zones', [])
print(f'Zones defined: {len(zones)}')
for z in zones:
    print(f'  {z[\"name\"]:>12} ({z[\"row\"]},{z[\"col\"]}) type={z[\"type\"]}')
"
```

### Phase 3: Calibration Drive (Day 2-3)

```bash
# Drive the robot through ALL zones once.
# The iogita_zone_node records LiDAR fingerprints during this drive.

# ROS1:
rosrun iogita iogita_zone_node.py \
    _config_file:=ros_integration/warehouse_config.yaml \
    _mode:=calibration

# ROS2:
ros2 run iogita iogita_zone_node \
    --ros-args -p config_file:=ros_integration/warehouse_config.yaml -p mode:=calibration

# Drive robot through every zone. Fingerprints saved to /tmp/iogita_fingerprints.npz
```

### Phase 4: Deploy (Day 3-4)

```bash
# 1. Launch io-gita zone node (runs alongside your navigation stack)

# ROS1:
rosrun iogita iogita_zone_node.py \
    _config_file:=ros_integration/warehouse_config.yaml \
    _mode:=production

# ROS2:
ros2 run iogita iogita_zone_node \
    --ros-args -p config_file:=ros_integration/warehouse_config.yaml -p mode:=production

# 2. Launch the AMCL hook (auto-detects ROS1 or ROS2)
# ROS1: rosrun your_package amcl_hook_example.py
# ROS2: ros2 run your_package amcl_hook_example

# 3. Monitor
# ROS1: rostopic echo /iogita/zone
# ROS2: ros2 topic echo /iogita/zone
```

### Phase 5: Verify (Day 4-5)

```bash
# Test cold start: kill and restart AMCL

# ROS1:
rosnode kill /amcl
sleep 2
roslaunch your_package amcl.launch

# ROS2:
ros2 lifecycle set /amcl shutdown
sleep 2
ros2 launch your_package amcl.launch.py

# Watch: /iogita/zone_hint should publish within 100ms
# Watch: AMCL should converge within 1 second (down from 4-5 sec)
```

---

## 8. Testing and Debugging

### Run diagnostics anytime:
```bash
python debug/iogita_diagnostics.py
python debug/iogita_diagnostics.py --benchmark    # Performance test
python debug/iogita_diagnostics.py --check-license # License status
```

### Check debug log:
```bash
# Real-time tail
tail -f /tmp/iogita_debug.jsonl | python -m json.tool

# Check for errors
grep '"level":"FAIL"' /tmp/iogita_debug.jsonl
```

### Standalone test (no ROS needed):
```bash
python ros_integration/iogita_zone_node.py
# Runs zone identification test without any ROS dependency
```

---

## 9. ROS Topic Reference

### Topics io-gita PUBLISHES (io-gita -> your system)

| Topic | Type | When | Content |
|-------|------|------|---------|
| `/iogita/zone` | `std_msgs/String` | On zone change | Current zone name, e.g. `"SHELF_3"` |
| `/iogita/zone_hint` | `std_msgs/String` | On cold start | JSON list: `["SHELF_1","SHELF_3","LANE_W"]` |
| `/iogita/zone_confidence` | `std_msgs/Float32` | With every `/zone` | 0.0-1.0 confidence |
| `/iogita/map_change` | `std_msgs/String` | On change detected | JSON alert |
| `/iogita/debug` | `std_msgs/String` | Always (if enabled) | JSON debug blob |

### Topics io-gita SUBSCRIBES TO (your system -> io-gita)

| Topic | Type | What We Need | Already Exists? |
|-------|------|-------------|-----------------|
| `/scan` | `sensor_msgs/LaserScan` | 360-degree LiDAR ranges | YES (your LiDAR driver) |
| `/odom` | `nav_msgs/Odometry` | Wheel position + heading | YES (your base driver) |
| `/imu/data` | `sensor_msgs/Imu` | Orientation | YES (if you have IMU) |

### Topic YOU ADD (one new subscription in your AMCL launch)

```
/iogita/zone_hint  ->  amcl_hook_example.py  ->  /initialpose
```

The hook reads the zone hint, looks up map coordinates, and publishes a constrained `/initialpose` to AMCL.

---

## 10. Troubleshooting

### "Import error: sg_engine not found"
```bash
pip install -e /path/to/cold_start_addverb/
python -c "from sg_engine.network import Network; print('OK')"
```

### "Python version mismatch"
The compiled `.so` requires Python 3.11. Check:
```bash
python3 --version   # Must be 3.11.x
```

### "License expired"
Contact ai.meharbansingh@gmail.com for renewal or production license.

### "Zone identification wrong"
```bash
# Check fingerprints were calibrated
ls /tmp/iogita_fingerprints.npz

# Re-run calibration drive if needed
```

### "Cold start still slow (>2 sec)"
```bash
# ROS1: rostopic echo /iogita/zone_hint
# ROS2: ros2 topic echo /iogita/zone_hint

# If empty -> io-gita node not running or /scan not publishing
# Check your AMCL hook is subscribed to /iogita/zone_hint
```

### "Teleport detected (wrong zone after movement)"
```bash
grep "TELEPORT" /tmp/iogita_debug.jsonl
# If frequent -> your warehouse_config.yaml adjacency is wrong
# Add missing connections in adjacency_overrides
```

---

## Summary: What You Do vs What We Do

| Task | Who | Effort |
|------|-----|--------|
| Install sg_engine | You | 5 min |
| Run diagnostics | You | 5 min |
| Edit warehouse_config.yaml | You | 1-2 hours |
| Update ZONE_COORDINATES in AMCL hook | You | 30 min |
| Calibration drive (robot through all zones) | You | 30 min |
| Verify cold start works | You | 30 min |
| Provide io-gita engine (compiled) | Us | Done |
| Provide zone identification node | Us | Done |
| Provide debug tools | Us | Done |
| Production license + support | Us | On request |

**Total integration effort: 1-2 days**

---

**Contact:** ai.meharbansingh@gmail.com
**License:** 7-day evaluation (expires 2026-04-03). Production license available on request.
**Patent pending.** Unauthorized redistribution prohibited.

*(c) 2026 Adaptive Mind / Meharban Singh*
