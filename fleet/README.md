# io-gita Fleet Integration — Full Addverb Fleet_Core Package

Complete integration package for Addverb's production fleet.
Aligned to fleet_core: TCP Protocol V1, barcode grid, +-15deg obstacle sensor.

**Prerequisite:** You've already run `lab/` and seen the cold start timing.
This package scales that to your full warehouse with 10+ robots.

## Quick Start

```bash
pip install numpy scipy pyyaml
pip install -e .
python debug/iogita_diagnostics.py     # Verify install
python -m pytest tests/ -v             # Run 46 tests
```

## What's Included

```
fleet/
├── README.md                          <- You're reading this
├── INTEGRATION_GUIDE.md               <- Step-by-step setup (1 day)
├── FLOWCHART.md                       <- System architecture diagrams
├── CHANGELOG.md                       <- Version history
├── LICENSE                            <- 7-day evaluation terms
├── pyproject.toml                     <- pip install -e .
├── requirements.txt                   <- Pinned dependencies
├── Dockerfile                         <- Containerized deployment
├── pytest.ini                         <- Test config
│
├── fleet_integration/                 <- Core integration code
│   ├── iogita_zone_node.py            <- Zone identification engine
│   ├── fms_adapter.py                 <- FMS REST/TCP bridge
│   ├── fms_adapter_async.py           <- Async FMS adapter
│   ├── barcode_fallback_hook.py       <- Barcode failure state machine
│   ├── structured_logger.py           <- Fleet-wide JSON logging
│   ├── shared_config.py               <- Shared test configs
│   └── warehouse_config.yaml          <- YOUR warehouse layout (edit this)
│
├── cold_start_aliasing/               <- Demos + reference
│   ├── cold_start_demo.py             <- Simple 5-zone demo
│   ├── cold_start_v2_addverb.py       <- Full 25-zone demo
│   ├── ENDPOINTS_COLD_START.md        <- Protocol + endpoint docs
│   └── *.json                         <- Reference results
│
├── debug/                             <- Monitoring tools
│   ├── iogita_diagnostics.py          <- Installation verifier
│   └── fleet_debug_node.py            <- Live TCP/REST monitor
│
├── tests/                             <- 46 unit tests
│   ├── test_protocol_v1.py            <- TCP message parsing
│   ├── test_zone_identifier.py        <- Zone identification
│   ├── test_barcode_fallback.py       <- Failure state machine
│   └── test_structured_logger.py      <- Fleet logging
│
├── sg_engine/                         <- Compiled engine (do NOT modify)
└── sg_engine_locked/                  <- 7-day license
```

## Integration Steps

1. **Edit** `fleet_integration/warehouse_config.yaml` — your zones, barcodes, graph nodes
2. **Run** calibration drive — robot through all zones
3. **Connect** to FMS — REST (port 7012) or TCP (port 65123)
4. **Test** cold start — kill robot process, watch recovery

Full details in `INTEGRATION_GUIDE.md`.

## Connects To

```
Robot (TCP 65123) → FMS (MoveCT) ← io-gita (REST 7012)
                         ↕
                    RabbitMQ (5672)
                         ↕
                    WCS / WES
```

## License

7-day evaluation. Expires 2026-04-03.
Contact: ai.meharbansingh@gmail.com
(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
