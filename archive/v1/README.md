# io-gita Cold Start — Addverb Evaluation Package

Solves two warehouse robot problems using Semantic Gravity (Hopfield attractor networks):

1. **Cold Start** — Robot restarts -> 4-5 sec frozen -> io-gita reduces to <1 sec
2. **Perceptual Aliasing** — Identical aisles confuse AMCL -> graph topology resolves

## Quick Start

```bash
pip install numpy scipy
pip install -e .
python debug/iogita_diagnostics.py
python cold_start_aliasing/cold_start_v2.py
```

Read `INTEGRATION_GUIDE.md` for complete step-by-step instructions.

## ROS Compatibility

Works with **both ROS1 (Noetic)** and **ROS2 (Humble/Iron)**. Auto-detects at runtime.

## What You Modify

| File | What You Do |
|------|-------------|
| `ros_integration/warehouse_config.yaml` | Fill in YOUR zone names, grid positions, map coordinates |
| `ros_integration/amcl_hook_example.py` | Update `ZONE_COORDINATES` dict with your map coords |

## What You Do NOT Modify

| File | Why |
|------|-----|
| `sg_engine/` | Compiled engine (binary .so) — black box |
| `sg_engine_locked/` | License wrapper — 7-day evaluation |
| `ros_integration/iogita_zone_node.py` | Zone identification node — provided by io-gita |

## License

7-day evaluation license. Expires 2026-04-03.
Contact: ai.meharbansingh@gmail.com

(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
