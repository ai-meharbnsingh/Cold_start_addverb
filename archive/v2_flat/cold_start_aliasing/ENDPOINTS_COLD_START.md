# Cold Start Recovery — Endpoints & Flow (Addverb Fleet_Core Aligned)

**Date:** 2026-03-27
**Problem:** Robot restarts -> needs to re-localize on the barcode grid map ->
blind until it reads a barcode -> wastes 10-30 sec searching.
**Solution:** io-gita gives instant zone identification (<1ms) from first sensor
reading after restart -> FMS routes robot directly to nearest barcode -> confirmed
position in <2 sec.

---

## Addverb's Actual Communication Stack

| Channel | Protocol | Port | What |
|---------|----------|------|------|
| Robot <-> FMS | TCP Protocol V1 | 65123 | 15 Hz telemetry + commands |
| WCS <-> FMS | RabbitMQ (AMQP) | 5672 | Task queue |
| Dashboard <-> FMS | REST (HTTP) | 7012 | 200+ API endpoints |
| DB | MongoDB | 27017 | Persistence |

**io-gita integrates via REST (port 7012) or TCP (port 65123).**

---

## What io-gita READS (already exists on robot)

| # | Data | Source | How io-gita Gets It |
|---|------|--------|---------------------|
| 1 | Obstacle range (0-1.5m) | +-15deg sensor | TCP Protocol V1 field 12 |
| 2 | Obstacle detected (bool) | +-15deg sensor | TCP Protocol V1 field 13 |
| 3 | Robot pose (x, y, theta) | Dead reckoning | TCP Protocol V1 fields 2-4 |
| 4 | Velocity (linear, angular) | Wheel encoders | TCP Protocol V1 fields 7-8 |
| 5 | Battery SOC | BMS | TCP Protocol V1 field 6 |
| 6 | Current graph node | A* planner | TCP Protocol V1 field 11 |
| 7 | Last barcode position | Barcode reader | Tracked by io-gita (saved to disk) |

## What io-gita SENDS (new)

| # | Data | Destination | Method | When |
|---|------|-------------|--------|------|
| 1 | Zone hint (cold start) | FMS | REST POST or TCP | On robot restart |
| 2 | Zone update | FMS | REST POST | On every zone change |
| 3 | Map change alert | FMS | REST POST | When layout mismatch detected |

---

## Cold Start Flow (Exact Sequence)

```
ROBOT RESTARTS (crash, power cycle, FMS disconnect timeout)
     |
     v
[1] io-gita loads last known state from disk:
    - Last zone: "SHELF_3"
    - Last barcode: row=2, col=1
    - Last timestamp: 14:32:07
     |
     v
[2] First TCP telemetry arrives from robot (Protocol V1, 15 Hz):
    "1711545200.5|zippy10_3|15.2|24.8|1.55|IDLE|68.0|0.0|0.0|0|NONE|13|1.1|0"
     |                       |    |    |     |    |    |         |   |   |
     timestamp     robot_id--+  x-+  y-+ theta state batt vel  node obs_r obs_det
     |
     v
[3] io-gita extracts 16 features from this SINGLE telemetry message:
    - obstacle_range: 1.1 / 1.5 = 0.73
    - obstacle_detected: 0.0
    - heading: 88.8deg / 360 = 0.25
    - heading_bin: 2 / 8 = 0.25
    - dist_from_barcode: 0.4m / 3.0 = 0.13
    - turns: 0 / 4 = 0.0
    - grid_row: 2 / 50 = 0.04
    - grid_col: 1 / 80 = 0.01
    - quadrant: 0.0
    - near_dock: 0.92
    - velocity: 0.0 / 1.4 = 0.0 (just restarted)
    - battery: 68.0 / 100 = 0.68
    - time_since_barcode: 5.0 / 60 = 0.08
    - barcode_reliability: 0.95
    - zone_type: 0.7 (shelf)
    - grid_density: 0.92
     |
     v
[4] Hopfield ODE classifies zone from features (<1ms):
    - Build query vector: features x 16 atoms in D=10000 space
    - Run ODE dynamics -> converge to attractor basin
    - Top candidates: SHELF_3 (0.87), SHELF_1 (0.72), MID_N (0.65)
     |
     v
[5] Graph disambiguation:
    - Last zone: SHELF_3
    - Reachable from SHELF_3: [CROSS_W, MID_N, SHELF_5, LANE_W]
    - Of candidates, MID_N is reachable
    - But SHELF_3 itself is also reachable (robot might not have moved)
    - Result: SHELF_3 (GRAPH_FP_RANKED, confidence=0.90)
     |
     v
[6] FMS adapter sends hint:
    POST http://127.0.0.1:7012/api/robots/zippy10_3/zone_hint
    {
      "robot_id": "zippy10_3",
      "event": "COLD_START_HINT",
      "primary_zone": "SHELF_3",
      "confidence": 0.90,
      "candidate_zones": ["SHELF_3", "MID_N", "CROSS_W"],
      "nearest_barcodes": [
        {"zone": "SHELF_3", "barcode_id": 55, "graph_node": 13},
        {"zone": "MID_N",   "barcode_id": 45, "graph_node": 8}
      ],
      "recovery_strategy": "nearest_barcode",
      "ode_time_ms": 0.8
    }
     |
     v
[7] FMS receives hint -> routes robot:
    - Robot is probably in SHELF_3 near graph node 13
    - Nearest barcode: ID 55 (0.4m away based on odometry)
    - FMS sends MOVE command via TCP to node where barcode 55 is
    - Robot moves 0.4m forward
     |
     v
[8] Robot reads barcode 55 -> position CONFIRMED:
    - Grid position: row=2, col=1 (exact)
    - Zone: SHELF_3 (confirmed)
    - Localization complete
     |
     v
[9] Resume normal operation.
    Total recovery time: ~1.5 sec (vs 10-30 sec blind search)
```

---

## Without io-gita (Current Addverb Behavior)

```
ROBOT RESTARTS
     |
     v
[1] Robot has no idea where it is
     |
     v
[2] Strategy: drive slowly in last known heading
     |
     v
[3] Keep reading barcode reader
     |
     v
[4] Hope to pass over a barcode
     |
     v
[5] 10-30 sec of blind movement in a warehouse
    with 9 other robots moving at 1.4 m/s
     |
     v
[6] Risk: collision, traffic jam, wrong aisle
     |
     v
[7] Eventually reads a barcode -> position known
     |
     v
[8] Resume. But 10-30 sec of downtime already happened.
    If in a narrow aisle: blocked other robots too.
```

---

## Timing Comparison

| Phase | Without io-gita | With io-gita |
|-------|----------------|--------------|
| Restart -> first telemetry | 0.5 sec | 0.5 sec |
| Zone identification | N/A (blind) | <1 ms |
| FMS hint delivery | N/A | <50 ms |
| Movement to barcode | 10-30 sec (random) | 0.5-1.5 sec (directed) |
| Barcode read confirmed | After random walk | After directed movement |
| **Total** | **10-30 sec** | **<2 sec** |

---

## What Addverb Adds (Integration Effort)

### Option A: REST Integration (30 min)

Add one REST endpoint handler in fleet_core:

```cpp
// In fleet_core src/fleet/ui/RESTInterface.cpp
// Accept io-gita zone hint and route robot to nearest barcode

void handleZoneHint(const json& body) {
    std::string robotId = body["robot_id"];
    std::string zone = body["primary_zone"];
    double confidence = body["confidence"];
    auto barcodes = body["nearest_barcodes"];

    if (confidence > 0.6 && !barcodes.empty()) {
        int targetNode = barcodes[0]["graph_node"];
        // Route robot to this node (standard A* path)
        fleetManager->routeRobotToNode(robotId, targetNode);
        LOG_INFO("io-gita cold start: routing {} to node {} in zone {}",
                 robotId, targetNode, zone);
    }
}
```

### Option B: TCP Integration (1 hour)

Parse io-gita hint message in Protocol V1 parser:

```cpp
// In fleet_core src/robot/parser/
// Add IOGITA_HINT message type handling

if (messageType == "IOGITA_HINT") {
    std::string zone = fields[2];
    double confidence = std::stod(fields[3]);
    // Same routing logic as Option A
}
```

### Option C: Standalone (0 effort)

io-gita runs completely standalone. Reads robot telemetry from FMS REST API,
writes zone hints to a file or log. No fleet_core modification needed.
Addverb engineers read the zone hints manually during testing.

---

## 16 Features Extracted From Addverb's Actual Sensors

| # | Feature | Source | How Computed | Discriminates |
|---|---------|--------|-------------|---------------|
| 1 | obstacle_range | +-15deg sensor | range / 1.5m | Open vs blocked ahead |
| 2 | obstacle_present | +-15deg sensor | 1 if detected | Active obstacle |
| 3 | heading | Wheel encoders | degrees / 360 | Direction facing |
| 4 | heading_bin | Wheel encoders | Quantized to 8 | Cardinal direction |
| 5 | dist_from_barcode | Odometry | meters / 3.0 | How far from last known |
| 6 | turns_since_barcode | Odometry | count / 4 | Straight vs after turns |
| 7 | grid_row | Last barcode | row / max_rows | North vs south of warehouse |
| 8 | grid_col | Last barcode | col / max_cols | East vs west of warehouse |
| 9 | quadrant | Derived | 0-3 / 4 | Warehouse quadrant |
| 10 | near_dock | Derived | Edge proximity | Near dock zone |
| 11 | velocity | Protocol V1 | vel / max_vel | Moving vs stopped |
| 12 | battery | Protocol V1 | soc / 100 | State of charge |
| 13 | time_since_barcode | Clock | seconds / 60 | How stale is position |
| 14 | barcode_reliability | Tracker | success_rate | Reader health |
| 15 | zone_type_expected | Graph node | Encoded type | Expected zone type |
| 16 | grid_density | Derived | Edge distance | Dense vs sparse area |

**Key difference from v1:** These features come from their ACTUAL sensors
(+-15deg obstacle + odometry + barcode grid), not from a 360deg LiDAR that
they don't have.
