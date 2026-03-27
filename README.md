# io-gita Cold Start — Addverb Evaluation Package

Solves two warehouse robot problems using Semantic Gravity (Hopfield attractor networks):

1. **Cold Start** — Robot restarts → 4-5 sec frozen → io-gita reduces to <1 sec
2. **Perceptual Aliasing** — Identical aisles confuse AMCL → graph topology resolves

## Quick Start

```bash
pip install numpy scipy
pip install -e .
python debug/iogita_diagnostics.py
python cold_start_aliasing/cold_start_v2.py
```

Read `INTEGRATION_GUIDE.md` for complete step-by-step instructions.

## License

7-day evaluation license. Expires 2026-04-03.
Contact: meharban@adaptive-mind.com

(c) 2026 Adaptive Mind / Meharban Singh. Patent pending.
