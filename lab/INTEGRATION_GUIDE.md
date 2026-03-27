# Lab Integration Guide — One Robot Setup

## What You Need

- One Zippy10 or AMR500 robot
- Python 3.11 on the test machine
- 5 minutes

## Steps

### Step 1: Install (2 min)

```bash
cd lab
pip install numpy scipy pyyaml
```

### Step 2: Edit your lab layout (2 min)

Open `lab_config.yaml`. Change zone names to match your lab:

```yaml
zones:
  - name: "YOUR_DOCK"       # Where robot starts
    row: 0
    col: 0
    type: "dock"
    heading: 90

  - name: "YOUR_AISLE"      # Main corridor
    row: 0
    col: 1
    type: "aisle"
    heading: 90

  # Add your actual lab zones...
```

### Step 3: Run (10 sec)

```bash
python lab_demo.py
```

### Step 4: Read the number

```
  WITHOUT io-gita:  10-30 sec blind barcode search
  WITH io-gita:     0.85 ms zone identification
```

## What It Proves

The Hopfield ODE engine identifies the robot's zone from a single sensor
reading in <1ms. On a real robot, this means:

1. Robot restarts after crash
2. First telemetry arrives (obstacle sensor + odometry)
3. io-gita says "you're in SHELF_1" in <1ms
4. FMS routes robot to nearest barcode (0.4m away)
5. Barcode confirmed in <2 sec total

Without io-gita: robot drives blind 10-30 sec looking for a barcode.

## Next Step

When satisfied with the timing → ask for the `fleet/` package for full
fleet_core integration (TCP Protocol V1, REST API, multi-robot, logging).

## Contact

ai.meharbansingh@gmail.com
(c) 2026 Adaptive Mind / Meharban Singh
