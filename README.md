# io-gita Cold Start v2 — Addverb Fleet Integration Package

Aligned to Addverb's **actual fleet_core architecture**: TCP Protocol V1, barcode grid
localization, +-15deg obstacle sensor, BTCPP behavior trees. No ROS2 assumed.

Solves two warehouse robot problems using Semantic Gravity (Hopfield attractor networks):

1. **Cold Start Recovery** — Robot restarts between barcodes -> lost -> io-gita directs
   recovery to nearest barcode in <2 sec (vs random search taking 10-30 sec)
2. **Barcode Failure Resilience** — Damaged/missing floor barcodes -> robot stops dead ->
   io-gita provides zone-level awareness to navigate through dead zones

## Quick Start

```bash
pip install numpy scipy pyyaml
pip install -e .
python debug/iogita_diagnostics.py
python cold_start_aliasing/cold_start_v2_addverb.py
```

Read `INTEGRATION_GUIDE.md` for complete step-by-step instructions.

## Compatibility

Works with **Addverb fleet_core** (MoveCT FMS) directly:
- TCP Protocol V1 (port 65123) — reads robot telemetry natively
- Barcode grid localization (0.8m grid) — augments, does not replace
- +-15deg obstacle sensor — uses what you have, no new hardware
- REST API (port 7012) — exposes io-gita status to dashboard

Also works standalone (no fleet_core dependency) for testing.

## What You Modify

| File | What You Do |
|------|-------------|
| `fleet_integration/warehouse_config.yaml` | Fill in YOUR barcode grid layout, zone names, graph nodes |
| `fleet_integration/fms_adapter.py` | Update FMS IP/port if different from defaults |

## What You Do NOT Modify

| File | Why |
|------|-----|
| `sg_engine/` | Compiled engine (binary .so) |
| `sg_engine_locked/` | License wrapper — 7-day evaluation |
| `fleet_integration/iogita_zone_node.py` | Zone identification engine — provided by io-gita |
| `fleet_integration/barcode_fallback_hook.py` | Cold start recovery logic — provided by io-gita |

## License

7-day evaluation license. Expires 2026-04-03.
Contact: ai.meharbansingh@gmail.com

(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
