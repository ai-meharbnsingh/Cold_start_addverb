# io-gita Cold Start v2 — Integration Guide (Addverb Fleet_Core)

**For:** Addverb Technologies
**From:** Meharban Singh
**Date:** 2026-03-27
**Version:** 2.0 — Aligned to fleet_core production architecture
**License:** 7-day evaluation (expires 2026-04-03)

---

## The Problem

Robot restarts (crash, power cycle, FMS disconnect). It needs to re-localize
on the barcode grid. Until it reads a barcode, it's blind — it doesn't know
where it is on the map. Current behavior: drive slowly hoping to hit a barcode.
Takes 10-30 seconds. During that time, the robot blocks an aisle and can't
take tasks.

## The Solution

io-gita provides instant zone identification (<1ms) from the robot's first
telemetry message after restart. The FMS gets a zone hint immediately and
routes the robot directly to the nearest barcode. Total recovery: <2 seconds.

**No new hardware. No new sensors. Uses what you already have.**

---

## What's In This Package

```
io-gita-addverb-v2/
├── README.md                           <- Read first
├── INTEGRATION_GUIDE.md                <- THIS FILE
├── pyproject.toml                      <- pip install -e .
│
├── sg_engine/                          <- io-gita engine (COMPILED — do NOT modify)
│   ├── __init__.py
│   ├── network.cpython-311-*.so        <- Hopfield ODE engine binary
│   └── atlas.py                        <- Transition graph mapper
│
├── sg_engine_locked/                   <- License wrapper (7-day eval)
│   └── __init__.py
│
├── fleet_integration/                  <- Fleet_core integration (TCP + REST)
│   ├── warehouse_config.yaml           <- YOU EDIT: your barcode grid layout
│   ├── iogita_zone_node.py             <- Zone identification engine (ours)
│   ├── fms_adapter.py                  <- YOU EDIT: FMS connection details
│   └── barcode_fallback_hook.py        <- Barcode failure handler (ours)
│
├── debug/                              <- Debug tools
│   ├── iogita_diagnostics.py           <- Run FIRST after install
│   └── fleet_debug_node.py             <- Live debug (reads TCP telemetry)
│
└── cold_start_aliasing/                <- Demo + docs
    ├── cold_start_v2_addverb.py        <- Full demo matching fleet_core
    ├── ENDPOINTS_COLD_START.md         <- All endpoints documented
    └── cold_start_results.json         <- Reference results
```

---

## Prerequisites

```bash
# Python 3.11 required (compiled binary is 3.11)
python3 --version   # Must be 3.11.x

# Required packages
pip install numpy scipy pyyaml

# Fleet_core FMS running (for live integration)
# Or: standalone mode works without fleet_core
```

**Hardware required:** None new. Uses robot's existing:
- Obstacle sensor (+-15deg, 1.5m range) — via TCP Protocol V1
- Wheel encoders (odometry) — via TCP Protocol V1
- Barcode reader (irayple) — tracked by io-gita
- Battery monitor — via TCP Protocol V1

---

## Installation

```bash
# Step 1: Extract package
cd io-gita-addverb-v2

# Step 2: Install
pip install -e .

# Step 3: Verify
python debug/iogita_diagnostics.py

# Expected output:
#   [OK] Import sg_engine: All core modules imported
#   [OK] NumPy performance: D=10000 dot product: <100us
#   [OK] Network build: Built network
#   [OK] ODE dynamics: ODE converged
#   [OK] Determinism: 3 runs -> identical output
#   [OK] Cold start pipeline: Full pipeline: <200ms
#   [OK] Protocol V1 parser: Parsed test message
#   [OK] Barcode tracker: State tracking works
#   SUMMARY: All systems operational

# Step 4: Check license
python -c "from sg_engine_locked import license_info; license_info()"
```

---

## Configuration (2 Files)

### File 1: `fleet_integration/warehouse_config.yaml`

Map your warehouse barcode grid to io-gita zones.

**What you fill in:**

| Field | What You Provide | Where to Get It |
|-------|-----------------|-----------------|
| `warehouse.max_rows/cols` | Grid dimensions | Your map config |
| `zones[].name` | Zone names | Your warehouse zones |
| `zones[].row, col` | Zone grid position | Your zone layout |
| `zones[].type` | dock / aisle / shelf / cross / hub / lane / charging | Zone function |
| `zones[].barcode_range` | Barcode IDs in this zone | Your barcode assignment table |
| `zones[].graph_nodes` | A* node IDs in this zone | Your nav graph |
| `zones[].center_x, center_y` | Zone center in grid coords | Your map |
| `fms.host, fms.tcp_port` | FMS address | Your network config |

**What you do NOT change:**

| Field | Why |
|-------|-----|
| `engine.D` | Must be 10000 (matches compiled binary) |
| `engine.beta` | Tuned for optimal ODE convergence |
| `engine.seed` | Changes patterns -> breaks calibration |
| `cold_start.recovery_strategy` | Keep as "nearest_barcode" |

### File 2: `fleet_integration/fms_adapter.py`

Update connection details at the top of the file:

```python
FMS_REST_HOST = "127.0.0.1"    # <- CHANGE if FMS is remote
FMS_REST_PORT = 7012            # Addverb default
FMS_TCP_HOST = "127.0.0.1"     # <- CHANGE if FMS is remote
FMS_TCP_PORT = 65123            # Addverb default
```

---

## Step-by-Step Integration

### Phase 1: Setup (30 min)

```bash
pip install -e .
python debug/iogita_diagnostics.py       # ALL must pass
python fleet_integration/iogita_zone_node.py   # Standalone test
python fleet_integration/fms_adapter.py        # Adapter demo
```

### Phase 2: Configure (1-2 hours)

1. Edit `fleet_integration/warehouse_config.yaml`:
   - Add all your zones with barcode ranges and graph nodes
   - Set FMS connection details

2. Update `fleet_integration/fms_adapter.py`:
   - Set FMS host/port if different from defaults

3. Verify config:
```bash
python -c "
import yaml
with open('fleet_integration/warehouse_config.yaml') as f:
    cfg = yaml.safe_load(f)
zones = cfg.get('zones', [])
print(f'Zones: {len(zones)}')
for z in zones:
    br = z.get('barcode_range', [0,0])
    gn = z.get('graph_nodes', [])
    print(f'  {z[\"name\"]:>12} ({z[\"row\"]},{z[\"col\"]}) '
          f'type={z[\"type\"]:>8}  barcodes={br[0]}-{br[1]}  '
          f'nodes={len(gn)}')
"
```

### Phase 3: Calibration Drive (30 min)

Drive one robot through all zones. io-gita records sensor fingerprints.

```bash
# Start io-gita in calibration mode
python -m fleet_integration.iogita_zone_node \
    --config fleet_integration/warehouse_config.yaml \
    --mode calibration \
    --fms-host 127.0.0.1

# Drive robot through every zone (normal operation or manual)
# io-gita reads TCP telemetry and records fingerprints per zone

# When done, fingerprints saved to /tmp/iogita_fingerprints.npz
```

### Phase 4: Test Cold Start (30 min)

```bash
# Start io-gita in production mode
python -m fleet_integration.iogita_zone_node \
    --config fleet_integration/warehouse_config.yaml \
    --mode production \
    --fms-host 127.0.0.1

# In another terminal: simulate robot restart
# Option A: Kill robot process and restart it
# Option B: Disconnect TCP and reconnect

# Watch io-gita output:
#   [io-gita] Cold start detected: zippy10_3
#   [io-gita] Zone hint: SHELF_3 (conf=0.90, ode=0.8ms)
#   [io-gita] Sent to FMS: route to barcode 55 in SHELF_3
#   [io-gita] Barcode confirmed: recovery complete (1.3s)
```

### Phase 5: Measure (1 hour)

Run 10 restart tests. Measure:
- Time from restart to zone hint (should be <10ms)
- Time from restart to barcode confirmed (should be <2 sec)
- Zone identification accuracy (should be >85%)

Compare with baseline (without io-gita):
- Time from restart to barcode found (typically 10-30 sec)

---

## Integration Options

### Option A: REST Sidecar (Simplest — No fleet_core Changes)

io-gita runs as a separate process. Reads robot state from FMS REST API.
Sends zone hints back via REST POST.

```
FMS (port 7012) <--REST--> io-gita <--REST--> FMS
```

**Addverb effort:** 0 code changes. io-gita reads existing REST endpoints.
Zone hints are logged. Engineers review them during testing.

### Option B: REST Active (30 min fleet_core change)

Same as Option A, but FMS actually acts on the zone hint:

```cpp
// Add to fleet_core RESTInterface:
// When io-gita POSTs a zone hint, route the robot
if (endpoint == "/api/robots/{id}/zone_hint") {
    auto zone = body["primary_zone"];
    auto node = body["nearest_barcodes"][0]["graph_node"];
    fleetManager->routeToNode(robotId, node);
}
```

### Option C: TCP Inline (1 hour fleet_core change)

io-gita sends hints directly in Protocol V1 format. Lowest latency.
Requires adding IOGITA_HINT message type to the parser.

---

## Troubleshooting

### "Zone identification wrong after restart"
```bash
# Check last saved state
cat /tmp/iogita_last_state.json

# Check fingerprints were calibrated
python -c "import numpy as np; d=np.load('/tmp/iogita_fingerprints.npz'); print(list(d.keys()))"

# Re-run calibration drive if fingerprints are stale
```

### "FMS not receiving hints"
```bash
# Test REST connectivity
curl http://127.0.0.1:7012/api/robots

# Check FMS adapter config
grep -n "FMS_REST" fleet_integration/fms_adapter.py
```

### "ODE too slow (>10ms)"
```bash
# Run benchmark
python debug/iogita_diagnostics.py --benchmark

# D=10000 should complete in <2ms on modern hardware
# If slow: check Python 3.11, check NumPy is using BLAS
```

### "License expired"
Contact ai.meharbansingh@gmail.com for renewal or production license.

---

## Summary

| Task | Who | Effort |
|------|-----|--------|
| Install io-gita | You | 5 min |
| Run diagnostics | You | 5 min |
| Edit warehouse_config.yaml | You | 1-2 hours |
| Update FMS host/port | You | 5 min |
| Calibration drive | You | 30 min |
| Test cold start recovery | You | 30 min |
| (Optional) Add REST handler in fleet_core | You | 30 min |
| Provide io-gita engine + adapter | Us | Done |
| Production license + support | Us | On request |

**Total integration effort: 1 day**

---

**Contact:** ai.meharbansingh@gmail.com
**License:** 7-day evaluation (expires 2026-04-03). Production license available.
**Patent pending.** Unauthorized redistribution prohibited.

*(c) 2026 Adaptive Mind / Meharban Singh*
