#!/usr/bin/env python
"""
io-gita Lab Demo — ONE Robot, ONE Cold Start, ONE Number
==========================================================

Run:  python lab_demo.py

(c) 2026 Adaptive Mind / Meharban Singh
"""

import time
import sys
import os
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from sg_engine.network import Network

D = 10000
ATOMS = [f"F{i}" for i in range(16)]


def load_config():
    config_path = os.path.join(os.path.dirname(__file__), "lab_config.yaml")
    if os.path.exists(config_path):
        with open(config_path) as f:
            return yaml.safe_load(f)
    # Fallback built-in
    return {
        "zones": [
            {"name": "DOCK",    "row": 0, "col": 0, "type": "dock",     "heading": 90},
            {"name": "AISLE",   "row": 0, "col": 1, "type": "aisle",    "heading": 90},
            {"name": "SHELF_1", "row": 1, "col": 1, "type": "shelf",    "heading": 0},
            {"name": "SHELF_2", "row": 1, "col": 0, "type": "shelf",    "heading": 180},
            {"name": "CHARGER", "row": 1, "col": 2, "type": "charging", "heading": 270},
        ],
        "engine": {"D": 10000, "beta": 4.0, "dt": 0.05, "seed": 42,
                    "generated_patterns": 15},
    }


def extract_features(obstacle_range, heading_deg, grid_row, grid_col,
                     velocity, battery):
    return np.array([
        np.clip(obstacle_range / 1.5, 0, 1),
        1.0 if obstacle_range < 0.7 else 0.0,
        (heading_deg % 360) / 360.0,
        int(heading_deg / 45) / 8.0,
        0.1, 0.0,
        np.clip(grid_row / 10, 0, 1),
        np.clip(grid_col / 10, 0, 1),
        0.0,
        1.0 if grid_row == 0 else 0.2,
        np.clip(velocity / 1.4, 0, 1),
        np.clip(battery / 100, 0, 1),
        0.05, 0.95, 0.7, 0.5,
    ], dtype=np.float64)


def build_network(zones, engine_cfg):
    net = Network(D=engine_cfg["D"], beta=engine_cfg["beta"],
                  dt=engine_cfg["dt"], seed=engine_cfg["seed"])
    net.add_atoms(ATOMS)
    for z in zones:
        fp = extract_features(
            0.8 if z["type"] == "shelf" else 1.3,
            z["heading"], z["row"], z["col"], 0.0, 70.0,
        )
        vec = np.ones(D)
        for i, val in enumerate(fp):
            s = 2.0 * np.clip(val, 0, 1) - 1.0
            vec *= (s * net.atoms[ATOMS[i]]) if abs(s) > 0.1 else (net.atoms[ATOMS[i]] * 0.1)
        net.add_pattern(z["name"], np.sign(vec))
    net.generate_patterns(engine_cfg["generated_patterns"],
                          gen_seed=engine_cfg["seed"])
    return net


def identify(net, features, zone_names):
    vec = np.ones(D)
    for i, val in enumerate(features):
        s = 2.0 * np.clip(val, 0, 1) - 1.0
        vec *= (s * net.atoms[ATOMS[i]]) if abs(s) > 0.1 else (net.atoms[ATOMS[i]] * 0.1)
    q = np.sign(vec).astype(np.float64)
    _, q_final, _ = net.run_dynamics(q, alpha=0.0)
    sims = net.P_mat @ q_final / D
    results = [(name, float(sims[i])) for i, name in enumerate(net.pat_names)
               if name in zone_names]
    results.sort(key=lambda x: -x[1])
    return results


def main():
    cfg = load_config()
    zones = cfg["zones"]
    engine_cfg = cfg.get("engine", {"D": 10000, "beta": 4.0, "dt": 0.05,
                                     "seed": 42, "generated_patterns": 15})
    zone_names = {z["name"] for z in zones}

    print()
    print("  ╔══════════════════════════════════════════════════╗")
    print("  ║   io-gita Lab Demo — One Robot Cold Start       ║")
    print("  ╚══════════════════════════════════════════════════╝")
    print()

    net = build_network(zones, engine_cfg)
    print(f"  Warehouse:  {len(zones)} zones "
          f"({', '.join(z['name'] for z in zones)})")
    print(f"  Robot:      Zippy10 (+-15deg sensor, barcode grid)")
    print(f"  Engine:     D={D}, {net.n_patterns} patterns")
    print()

    # Normal operation
    last_zone = "SHELF_1"
    print(f"  Robot working in {last_zone}. Barcode reads OK.")
    print()

    # CRASH
    print("  ██████████████████████████████████████████████████")
    print("  ██         ROBOT CRASHES. RESTARTS.             ██")
    print("  ██████████████████████████████████████████████████")
    print()

    time.sleep(0.3)
    print("  First sensor reading after reboot:")
    print("    obstacle: 0.9m | heading: 5deg | battery: 68%")
    print()

    # Identify
    features = extract_features(0.9, 5.0, 1, 1, 0.0, 68.0)
    t0 = time.perf_counter()
    results = identify(net, features, zone_names)
    ode_ms = (time.perf_counter() - t0) * 1000

    print("  ┌──────────────────────────────────────────────┐")
    print(f"  │  Zone identified in {ode_ms:.2f} ms                │")
    print("  └──────────────────────────────────────────────┘")
    print()
    for name, sim in results[:3]:
        bar = "█" * int(max(sim, 0) * 20)
        tag = " ◄── MATCH" if name == last_zone else ""
        print(f"    {name:>10}  {sim:+.3f}  {bar}{tag}")

    print()
    print("  ══════════════════════════════════════════════════")
    print(f"  WITHOUT io-gita:  10-30 sec blind barcode search")
    print(f"  WITH io-gita:     {ode_ms:.2f} ms zone identification")
    print("  ══════════════════════════════════════════════════")

    # Stats
    print()
    print("  100-run timing statistics:")
    rng = np.random.default_rng(99)
    timings = []
    for _ in range(100):
        f = extract_features(0.5 + rng.random(), rng.random() * 360,
                             rng.integers(0, 2), rng.integers(0, 3),
                             rng.random() * 1.4, 30 + rng.random() * 70)
        t0 = time.perf_counter()
        identify(net, f, zone_names)
        timings.append((time.perf_counter() - t0) * 1000)
    timings = np.array(timings)
    print(f"    Mean:   {timings.mean():.2f} ms")
    print(f"    Median: {np.median(timings):.2f} ms")
    print(f"    P99:    {np.percentile(timings, 99):.2f} ms")
    print(f"    Max:    {timings.max():.2f} ms")
    print()


if __name__ == "__main__":
    main()
