# io-gita — Addverb Cold Start Recovery

Two packages. Give them in order.

## 1. `lab/` — Start here

One robot. One crash. One timing number.

```bash
cd lab
pip install numpy scipy pyyaml
python lab_demo.py
```

7 files. No infrastructure needed.

## 2. `fleet/` — When ready to scale

Full fleet_core integration. TCP Protocol V1, REST API, 46 tests.

```bash
cd fleet
pip install -e .
python -m pytest tests/ -v
```

37 files. Complete standalone package.

---

(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
Contact: ai.meharbansingh@gmail.com
