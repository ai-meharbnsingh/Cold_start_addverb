# io-gita Lab Demo — One Robot Cold Start

One robot. One crash. One number.

## What This Proves

Robot restarts → io-gita identifies zone in **<1ms** from first sensor reading.
Without io-gita: **10-30 sec** blind search for a barcode.

## Run It

```bash
pip install numpy scipy pyyaml
python lab_demo.py
```

That's it. No FMS, no TCP server, no fleet, no RabbitMQ.

## What Happens

1. Simulates a Zippy10 in a 5-zone warehouse
2. Robot crashes in SHELF_1
3. Robot restarts — first sensor data arrives
4. io-gita Hopfield ODE identifies zone in <1ms
5. Prints the timing + 100-run statistics

## Files

```
lab/
├── README.md               <- You're reading this
├── lab_demo.py             <- Run this
├── lab_config.yaml         <- 5-zone warehouse (edit for your lab)
├── sg_engine/              <- io-gita engine (compiled)
│   ├── __init__.py
│   ├── network.*.so        <- Hopfield ODE (Python 3.11 only)
│   └── atlas.py
└── sg_engine_locked/       <- 7-day evaluation license
    └── __init__.py
```

## Your Lab Setup

Edit `lab_config.yaml` with your actual lab zones:

```yaml
zones:
  - name: "START"
    row: 0
    col: 0
    type: "dock"
    heading: 90
```

## Requirements

- Python 3.11 (the compiled binary requires exactly 3.11)
- numpy, scipy, pyyaml

## Next Step

When ready for full fleet integration → ask for the `fleet/` package.

## License

7-day evaluation. Expires 2026-04-03.
Contact: ai.meharbansingh@gmail.com
(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
