# io-gita Cold Start — System Flowchart

## How Everything Connects

```
┌─────────────────────────────────────────────────────────────────────┐
│                    ADDVERB WAREHOUSE FLOOR                         │
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐           │
│  │ Zippy10  │  │ Zippy10  │  │ AMR500   │  │ Zippy10  │  ...x10   │
│  │  _1      │  │  _2      │  │  _1      │  │  _3      │           │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────┬─────┘           │
│       │              │              │              │                 │
│  ┌────┴──────────────┴──────────────┴──────────────┴────┐          │
│  │           TCP Protocol V1 (port 65123)               │          │
│  │   15 Hz telemetry: pose, battery, obstacle, node     │          │
│  └────────────────────────┬─────────────────────────────┘          │
│                           │                                         │
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
│  ░  BARCODE GRID (0.8m spacing) on warehouse floor               ░  │
│  ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  │
└─────────────────────────────────────────────────────────────────────┘
                            │
                            │ TCP
                            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                  ADDVERB FMS (MoveCT / fleet_core)                  │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐             │
│  │  TCP Server   │  │  REST API    │  │  COPP        │             │
│  │  port 65123   │  │  port 7012   │  │  Controller  │             │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘             │
│         │                  │                                        │
│  ┌──────┴──────┐  ┌───────┴───────┐  ┌──────────────┐             │
│  │ Task Manager│  │  200+ REST    │  │  A* + ILP    │             │
│  │ FIFO Alloc  │  │  endpoints    │  │  Node Reserv │             │
│  └─────────────┘  └───────┬───────┘  └──────────────┘             │
│                           │                                        │
│         ┌─────────────────┘                                        │
│         │  REST (port 7012)                                        │
│         ▼                                                          │
│  ┌──────────────────────────────────────────┐                      │
│  │  NEW: /api/robots/{id}/zone_hint         │ ◄── io-gita writes  │
│  │  NEW: /api/events (IOGITA_*)             │     zone hints here  │
│  └──────────────────────────────────────────┘                      │
└─────────────────────────────────────────────────────────────────────┘
                            ▲
                            │ REST POST
                            │
┌─────────────────────────────────────────────────────────────────────┐
│                     io-gita SYSTEM (THIS PACKAGE)                   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   FMS Adapter Layer                          │   │
│  │  ┌─────────────────┐  ┌─────────────────┐                  │   │
│  │  │ FmsRestAdapter   │  │ FmsTcpAdapter    │                  │   │
│  │  │ (port 7012)      │  │ (port 65123)     │                  │   │
│  │  └────────┬─────────┘  └────────┬─────────┘                  │   │
│  │           │                      │                            │   │
│  │  ┌────────┴──────────────────────┴─────────┐                  │   │
│  │  │      ColdStartOrchestrator              │                  │   │
│  │  │  • Detects restart                      │                  │   │
│  │  │  • Validates zone != None               │                  │   │
│  │  │  • Sends hint to FMS                    │                  │   │
│  │  │  • Tracks recovery time                 │                  │   │
│  │  └─────────────────┬───────────────────────┘                  │   │
│  └────────────────────┼────────────────────────────────────────┘   │
│                       │                                             │
│  ┌────────────────────┼────────────────────────────────────────┐   │
│  │                    │   Zone Identification Engine            │   │
│  │                    ▼                                         │   │
│  │  ┌───────────────────────────────────────────────────┐      │   │
│  │  │            ZoneIdentifier                         │      │   │
│  │  │                                                   │      │   │
│  │  │  1. Barcode OK? ──yes──► identify_from_barcode()  │      │   │
│  │  │     │                     (confidence: 1.0)       │      │   │
│  │  │     no                                            │      │   │
│  │  │     │                                             │      │   │
│  │  │  2. Graph node? ──yes──► identify_from_graph()    │      │   │
│  │  │     │                     (confidence: 0.95)      │      │   │
│  │  │     no                                            │      │   │
│  │  │     │                                             │      │   │
│  │  │  3. Sensors ──────────► identify_from_sensors()   │      │   │
│  │  │     +-15deg obstacle      (confidence: 0.3-0.9)   │      │   │
│  │  │     + odometry            uses Hopfield ODE <1ms  │      │   │
│  │  │     + battery                                     │      │   │
│  │  │     + grid position       then disambiguate()     │      │   │
│  │  │                           using graph topology    │      │   │
│  │  └──────────────┬────────────────────────────────────┘      │   │
│  │                 │                                            │   │
│  │  ┌──────────────┴────────────────────────────────────┐      │   │
│  │  │        sg_engine (Compiled Binary)                │      │   │
│  │  │                                                   │      │   │
│  │  │  Network(D=10000, beta=4.0)                       │      │   │
│  │  │    │                                              │      │   │
│  │  │    ├── add_atoms(16 features)                     │      │   │
│  │  │    ├── add_pattern(zone_fingerprints)             │      │   │
│  │  │    ├── run_dynamics(query) ──► attractor basin    │      │   │
│  │  │    │   └── ODE convergence: <1ms                  │      │   │
│  │  │    └── similarities ──► ranked zone candidates    │      │   │
│  │  │                                                   │      │   │
│  │  │  Atlas (transition graph)                         │      │   │
│  │  │    └── zone adjacency for disambiguation          │      │   │
│  │  └───────────────────────────────────────────────────┘      │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                   Support Systems                           │   │
│  │                                                             │   │
│  │  ┌──────────────────┐  ┌──────────────────┐                │   │
│  │  │ BarcodeGrid      │  │ BarcodeFallback  │                │   │
│  │  │ Tracker          │  │ Hook             │                │   │
│  │  │ • last position  │  │ • NORMAL         │                │   │
│  │  │ • failure count  │  │ • DEGRADED       │                │   │
│  │  │ • success rate   │  │ • FALLBACK       │                │   │
│  │  │ • dead reckoning │  │ • RECOVERY       │                │   │
│  │  └──────────────────┘  └──────────────────┘                │   │
│  │                                                             │   │
│  │  ┌──────────────────┐  ┌──────────────────┐                │   │
│  │  │ StructuredLogger │  │ State Persistence│                │   │
│  │  │ • zone_hints.jsonl│  │ /var/lib/iogita/ │                │   │
│  │  │ • events.jsonl   │  │ • last_state.json│                │   │
│  │  │ • 50MB rotation  │  │ • fingerprints   │                │   │
│  │  └──────────────────┘  └──────────────────┘                │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Cold Start Recovery Flow

```
ROBOT CRASHES / POWER CYCLES / FMS DISCONNECT TIMEOUT
 │
 ▼
[0.0s] Robot reboots, sensors come online
 │
 ▼
[0.5s] First TCP telemetry arrives at FMS (Protocol V1, 15 Hz)
 │      "1711545200.5|zippy10_3|15.2|24.8|1.55|IDLE|68.0|0.0|0.0|0|..."
 │
 ▼
[0.5s] io-gita reads telemetry ──────────────────────────────────────┐
 │                                                                    │
 │  ┌─────────────────────────────────────────────────────────────┐  │
 │  │  Step 1: Load saved state from /var/lib/iogita/             │  │
 │  │          Last zone: "SHELF_3", Last barcode: (2,1)          │  │
 │  │                                                             │  │
 │  │  Step 2: Extract 16 features from telemetry                 │  │
 │  │          obstacle: 1.1m  heading: 89deg  battery: 68%       │  │
 │  │          grid pos: (2,1)  velocity: 0.0  node: 13           │  │
 │  │                                                             │  │
 │  │  Step 3: Hopfield ODE classification (<1ms)                 │  │
 │  │          Query → attractor basin → SHELF_3 (sim=0.87)       │  │
 │  │                                                             │  │
 │  │  Step 4: Graph disambiguation                               │  │
 │  │          Last zone SHELF_3, reachable: [CROSS_W, MID_N,     │  │
 │  │          SHELF_5, LANE_W]. SHELF_3 itself reachable.        │  │
 │  │          Result: SHELF_3 (confidence: 0.90)                 │  │
 │  └─────────────────────────────────────────────────────────────┘  │
 │                                                                    │
 ▼◄───────────────────────────────────────────────────────────────────┘
[0.501s] io-gita sends zone hint to FMS
 │
 │  POST http://fms:7012/api/robots/zippy10_3/zone_hint
 │  {
 │    "primary_zone": "SHELF_3",
 │    "confidence": 0.90,
 │    "nearest_barcodes": [{"barcode_id": 55, "graph_node": 13}]
 │  }
 │
 ▼
[0.55s] FMS routes robot to barcode 55 (graph node 13)
 │       FMS sends MOVE command via TCP to robot
 │       Robot moves ~0.4m to nearest barcode
 │
 ▼
[1.5s] Robot reads barcode 55 ──► Position CONFIRMED
 │     Grid: (2,1), Zone: SHELF_3
 │
 ▼
[1.5s] io-gita logs confirmation, saves state, resumes normal
 │
 ▼
 DONE. Total: ~1.5 sec
```

---

## Without io-gita (Current Addverb Behavior)

```
ROBOT CRASHES / POWER CYCLES
 │
 ▼
[0.0s] Robot reboots
 │
 ▼
[0.5s] Robot has NO IDEA where it is
 │      No saved zone. No spatial awareness.
 │
 ▼
[0.5s] Robot drives slowly in last known heading
 │      Hoping to pass over a barcode on the floor
 │
 │      ┌──────────────────────────────────────────────┐
 │      │  WAREHOUSE AISLE (top view)                  │
 │      │                                              │
 │      │  ░░░░░ ← barcodes on floor (0.8m apart)     │
 │      │                                              │
 │      │       🤖 ← robot driving blind               │
 │      │       │                                      │
 │      │       │  wrong direction?                    │
 │      │       │  blocked by another robot?           │
 │      │       │  in the middle between two barcodes? │
 │      │       ▼                                      │
 │      │       ???                                    │
 │      └──────────────────────────────────────────────┘
 │
 ▼
[10-30s] Eventually reads a barcode ──► Position known
 │
 ▼
 DONE. Total: 10-30 sec of BLIND movement
        Risk: collision, traffic jam, wrong aisle
```

---

## Barcode Failure State Machine

```
                    barcode reads OK
              ┌─────────────────────────┐
              │                         │
              ▼                         │
         ┌─────────┐            ┌──────┴────┐
         │ NORMAL  │───3 fail──►│ DEGRADED  │
         └─────────┘            └─────┬─────┘
              ▲                       │
              │                    5 fail
         3 success                    │
              │                       ▼
         ┌────┴─────┐          ┌──────────────┐
         │ RECOVERY │◄─1 ok───│  FALLBACK    │
         └──────────┘          │  ACTIVE      │
                               │              │
                               │ io-gita zone │
                               │ awareness ON │
                               └──────────────┘
```

---

## 16 Feature Vector (From Addverb's Actual Sensors)

```
                    ┌──────────────────────────────────┐
                    │     TCP Protocol V1 Telemetry     │
                    └────────┬─────────────────────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
     ┌────────────┐  ┌────────────┐  ┌────────────┐
     │  Obstacle   │  │  Odometry  │  │  System    │
     │  Sensor     │  │  + Grid    │  │  State     │
     │  +-15deg    │  │            │  │            │
     ├────────────┤  ├────────────┤  ├────────────┤
     │ F1: range   │  │ F3: heading│  │ F11: veloc │
     │ F2: present │  │ F4: h_bin  │  │ F12: batt  │
     │             │  │ F5: dist   │  │ F13: time  │
     │             │  │ F6: turns  │  │ F14: reliab│
     │             │  │ F7: row    │  │ F15: z_type│
     │             │  │ F8: col    │  │ F16: g_dens│
     │             │  │ F9: quad   │  │            │
     │             │  │ F10: dock  │  │            │
     └──────┬──────┘  └─────┬──────┘  └─────┬──────┘
            │               │               │
            └───────────────┼───────────────┘
                            │
                            ▼
              ┌──────────────────────────┐
              │  16-dim feature vector   │
              │  [0.73, 0.0, 0.25, ...]  │
              └────────────┬─────────────┘
                           │
                           ▼
              ┌──────────────────────────┐
              │  Hopfield ODE Engine     │
              │  D=10000, beta=4.0       │
              │  query × 16 atoms        │
              │  ──► attractor basin     │
              │  ──► zone candidates     │
              │  TIME: <1ms              │
              └────────────┬─────────────┘
                           │
                           ▼
              ┌──────────────────────────┐
              │  Graph Disambiguation    │
              │  adjacency + last zone   │
              │  ──► final zone          │
              │  ──► confidence 0-1      │
              └──────────────────────────┘
```

---

## File Dependencies

```
warehouse_config.yaml
        │
        │ loaded by
        ▼
shared_config.py ◄──── tests/*
        │
        │ provides config to
        ▼
iogita_zone_node.py ─────────────────────────────────┐
  │  ZoneIdentifier                                    │
  │  BarcodeGridTracker                                │
  │  ProtocolV1Parser                                  │
  │  extract_features_addverb()                        │
  │                                                    │
  │  imports                                           │
  └──► sg_engine/                                      │
       │  network.so (Hopfield ODE)                    │
       │  atlas.py (transition graph)                  │
       │  __init__.py (import isolation guard)          │
                                                       │
barcode_fallback_hook.py ──────────────────────────────┤
  │  BarcodeFallbackHook                               │
  │  uses ZoneIdentifier                               │
  │                                                    │
fms_adapter.py ────────────────────────────────────────┤
  │  FmsRestAdapter (port 7012)                        │
  │  FmsTcpAdapter (port 65123)                        │
  │  ColdStartOrchestrator                             │
  │  uses ZoneIdentifier                               │
  │                                                    │
fms_adapter_async.py                                   │
  │  AsyncFmsRestAdapter                               │
  │  AsyncFmsTcpAdapter                                │
  │                                                    │
structured_logger.py ──────────────────────────────────┘
  │  StructuredLogger
  │  zone_hints.jsonl + events.jsonl
  │
  ▼
/var/lib/iogita/
  ├── last_state.json      (persists across reboots)
  ├── fingerprints.npz     (calibration data)
  └── logs/
      ├── zone_hints.jsonl (fleet analysis)
      └── events.jsonl     (cold starts, fallbacks)
```

---

## Timing Budget (15 Hz = 67ms per cycle)

```
FMS Main Loop Budget:
├── Receive telemetry     2-3ms
├── Update agent states   3-5ms
├── Allocate tasks        2-5ms
├── A* pathfinding        2-5ms
├── ILP node reservation  5-15ms  ◄── most expensive
├── Send commands         1-2ms
├── DB flush              async
├── Event dispatch        1-3ms
├── ──────────────────────────────
├── Addverb total:        15-38ms (22-57% of budget)
│
├── io-gita zone hint     <1ms    ◄── fits easily
├── io-gita logging       <0.1ms
├── ──────────────────────────────
└── Total with io-gita:   16-39ms (24-58% of budget)
                          ✓ well within 67ms
```

---

## Quick Reference: Ports & Protocols

```
┌───────────┬──────────┬───────────────────────────────────┐
│ Port      │ Protocol │ What                              │
├───────────┼──────────┼───────────────────────────────────┤
│ 65123     │ TCP      │ Robot ↔ FMS telemetry + commands  │
│ 7012      │ REST     │ Dashboard + io-gita zone hints    │
│ 5672      │ AMQP     │ WCS ↔ FMS task queue (RabbitMQ)   │
│ 27017     │ MongoDB  │ FMS persistence                   │
│ 50051     │ gRPC     │ Python integration (optional)     │
│ 65023     │ OPC UA   │ Conveyor/shuttle control          │
└───────────┴──────────┴───────────────────────────────────┘
```
