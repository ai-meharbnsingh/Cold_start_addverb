# Cold Start + Perceptual Aliasing — Endpoints

**Date:** 2026-03-25
**Problem:** Robot restart = 4-5 sec frozen. Identical aisles = wrong location.

---

## What We SUBSCRIBE To (already exist on robot)

| # | Topic | Type | What | Used For |
|---|-------|------|------|----------|
| 1 | `/scan` | `sensor_msgs/LaserScan` | 360° LiDAR ranges | Extract 16 zone features |
| 2 | `/odom` | `nav_msgs/Odometry` | Wheel position + velocity | Distance from last zone + heading |
| 3 | `/imu/data` | `sensor_msgs/Imu` | Orientation (heading) | Disambiguate identical zones (N/S/E/W) |

## What We PUBLISH (new)

| # | Topic | Type | What | Who Reads | When |
|---|-------|------|------|-----------|------|
| 1 | `/iogita/zone` | `std_msgs/String` | Current zone name (e.g. "SHELF_3") | Fleet dashboard, Nav2 | Always (every zone change) |
| 2 | `/iogita/zone_hint` | `std_msgs/String` | JSON list of candidate zones | AMCL | On cold start only |
| 3 | `/iogita/zone_confidence` | `std_msgs/Float32` | 0.0-1.0 how certain | AMCL, dashboard | Always |
| 4 | `/iogita/map_change` | `std_msgs/String` | JSON alert: which zone changed | Fleet management | When change detected |

---

## Cold Start Flow (Endpoints in Order)

```
ROBOT RESTARTS
     │
     ▼
[1] Read last_known_zone from disk (saved before shutdown)
     │
     ▼
[2] /scan ──► io-gita extracts 16 features from ONE LiDAR scan
     │
     ▼
[3] io-gita ODE classifies → 3-5 candidate zones (<100ms)
     │
     ▼
[4] Graph filter: which candidates are reachable from last_known_zone?
     │
     ▼
[5] PUBLISH /iogita/zone_hint ──► ["SHELF_1", "SHELF_3", "LANE_W"]
     │
     ▼
[6] AMCL receives hint → constrains particles to those 3 zones only
     │
     ▼
[7] AMCL converges in <1 second (instead of 4-5 seconds)
     │
     ▼
[8] PUBLISH /iogita/zone ──► "SHELF_1"
    PUBLISH /iogita/zone_confidence ──► 0.95
```

## Aliasing Flow (Endpoints in Order)

```
ROBOT ENTERS ZONE THAT LOOKS LIKE 6 OTHER ZONES
     │
     ▼
[1] /scan ──► 16 features → matches SHELF_1, SHELF_2, SHELF_3, SHELF_4, SHELF_5, SHELF_6
     │
     ▼
[2] /odom ──► heading = EAST, distance from DOCK_A = 12 meters
     │
     ▼
[3] /imu/data ──► compass heading confirms EAST
     │
     ▼
[4] Graph check: last zone was LANE_W
     LANE_W connects to → SHELF_1, SHELF_3 only
     Heading EAST → SHELF_1 (SHELF_3 is SOUTH)
     │
     ▼
[5] PUBLISH /iogita/zone ──► "SHELF_1" (CERTAIN)
    PUBLISH /iogita/zone_confidence ──► 0.98
```

## Map Change Detection Flow

```
ROBOT VISITS SHELF_3 DURING NORMAL OPERATION
     │
     ▼
[1] /scan ──► extract 16 features for current zone
     │
     ▼
[2] Compare against stored SHELF_3 fingerprint
     │
     ▼
[3] Mismatch detected (back wall removed — FAR_BACK instead of NEAR_BACK)
     │
     ▼
[4] Count: 3 consecutive mismatches → CHANGE CONFIRMED
     │
     ▼
[5] PUBLISH /iogita/map_change ──► {
       "zone": "SHELF_3",
       "change_type": "layout_modified",
       "old_signature": ["NEAR_FRONT", "NEAR_BACK", ...],
       "new_signature": ["NEAR_FRONT", "FAR_BACK", ...],
       "confidence": 1.0,
       "timestamp": "2026-03-25T21:30:00"
     }
     │
     ▼
[6] Update local atlas → push to fleet
     No re-drive needed. Detected during normal operation.
```

---

## What Addverb Adds (ONE hook — ~20 lines)

```python
# In their AMCL launch wrapper:
import json
from std_msgs.msg import String

def on_zone_hint(msg):
    """Accept io-gita zone hint to speed up cold start."""
    zones = json.loads(msg.data)
    # Convert zone names to map regions (Addverb has this mapping)
    for zone_name in zones:
        region = zone_map[zone_name]  # their existing zone→coordinates lookup
        # Constrain AMCL initial pose to these regions
        amcl.set_initial_pose(
            x=region['center_x'],
            y=region['center_y'],
            covariance=region['size']  # search only within this zone
        )

# Subscribe
node.create_subscription(String, '/iogita/zone_hint', on_zone_hint, 10)
```

---

## What Addverb Gives Us ONCE (for setup)

| # | What | Why |
|---|------|-----|
| 1 | Zone names + boundaries | We need to know "SHELF_3 is x=10-15, y=20-25 on the map" |
| 2 | Zone adjacency | Which zones connect (we can auto-detect but manual is faster) |
| 3 | One drive-through recording | 30 min robot drive → we record zone fingerprints |

---

## 16 Features Extracted From Existing Sensors

All from `/scan` + `/odom` + `/imu/data`. No new hardware.

| # | Feature | Source | How Computed | Distinguishes |
|---|---------|--------|-------------|---------------|
| 1 | front_clear | /scan | Median range 0°±15° | Open vs blocked ahead |
| 2 | back_clear | /scan | Median range 180°±15° | Dead end vs through-aisle |
| 3 | left_clear | /scan | Median range 270°±15° | Wall vs open side |
| 4 | right_clear | /scan | Median range 90°±15° | Wall vs open side |
| 5 | scan_variance | /scan | Variance of all ranges | Shelves (high) vs empty (low) |
| 6 | gap_count | /scan | Gaps >1m in 360° scan | Dock (few) vs aisle (many) |
| 7 | symmetry_lr | /scan | |left_clear - right_clear| | Symmetric aisle vs corner |
| 8 | symmetry_fb | /scan | |front_clear - back_clear| | Through-aisle vs dead end |
| 9 | front_density | /scan | Points per meter in front 60° | Cluttered vs clear |
| 10 | overall_density | /scan | Total points / total range | Dense shelves vs open |
| 11 | max_range | /scan | Maximum range in any direction | Open space indicator |
| 12 | min_range | /scan | Minimum range in any direction | Closest obstacle |
| 13 | heading | /imu | Compass heading (N/S/E/W) | East aisle vs west aisle |
| 14 | heading_bin | /imu | Quantized to 8 directions | Finer direction resolution |
| 15 | dist_from_dock | /odom | Cumulative distance since last dock zone | Near dock vs deep warehouse |
| 16 | turns_since_dock | /odom | Count of >45° heading changes | Straight aisle vs after turns |
