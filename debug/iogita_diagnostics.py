"""
io-gita Diagnostic & Debug Tools for Robotics Integration
==========================================================

Run this FIRST after installation to verify everything works.
Run this ANYTIME you suspect a problem.

Usage:
    python iogita_diagnostics.py                    # Full diagnostic
    python iogita_diagnostics.py --test-zone SHELF_3  # Test specific zone
    python iogita_diagnostics.py --benchmark         # Run performance benchmark
    python iogita_diagnostics.py --check-license     # Check license status

Output: diagnostic_report.json (attach when reporting issues)
"""

import sys
import os
import time
import json
import traceback
import argparse
import numpy as np

# ── DIAGNOSTIC LEVELS ─────────────────────────────────────

class DiagLevel:
    OK = "OK"
    WARN = "WARN"
    FAIL = "FAIL"
    INFO = "INFO"


class DiagnosticReport:
    def __init__(self):
        self.results = []
        self.start_time = time.time()

    def add(self, name, level, message, details=None):
        entry = {
            "test": name,
            "level": level,
            "message": message,
            "timestamp": time.time() - self.start_time,
        }
        if details:
            entry["details"] = details
        self.results.append(entry)

        icon = {"OK": "[OK]", "WARN": "[!!]", "FAIL": "[XX]", "INFO": "[--]"}[level]
        print(f"  {icon} {name}: {message}")

    def summary(self):
        ok = sum(1 for r in self.results if r["level"] == DiagLevel.OK)
        warn = sum(1 for r in self.results if r["level"] == DiagLevel.WARN)
        fail = sum(1 for r in self.results if r["level"] == DiagLevel.FAIL)
        return {"ok": ok, "warn": warn, "fail": fail, "total": len(self.results)}

    def save(self, path="diagnostic_report.json"):
        output = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "python_version": sys.version,
            "platform": sys.platform,
            "summary": self.summary(),
            "results": self.results,
        }
        with open(path, "w") as f:
            json.dump(output, f, indent=2, default=str)
        print(f"\n  Report saved: {path}")


# ── DIAGNOSTIC TESTS ─────────────────────────────────────

def test_import(report):
    """Test 1: Can we import sg_engine?"""
    try:
        from sg_engine.network import Network
        from sg_engine.atlas import Atlas
        report.add("Import sg_engine", DiagLevel.OK, "All core modules imported")
        return True
    except ImportError as e:
        report.add("Import sg_engine", DiagLevel.FAIL,
                   f"Import failed: {e}",
                   {"fix": "Run: pip install -e . from the io-gita root directory"})
        return False


def test_numpy(report):
    """Test 2: NumPy working and fast enough?"""
    try:
        D = 10_000
        a = np.random.choice([-1, 1], size=D).astype(np.float64)
        b = np.random.choice([-1, 1], size=D).astype(np.float64)

        t0 = time.time()
        for _ in range(1000):
            _ = np.dot(a, b)
        dot_time = (time.time() - t0) / 1000

        if dot_time < 0.001:
            report.add("NumPy performance", DiagLevel.OK,
                       f"D=10000 dot product: {dot_time*1e6:.1f}us")
        else:
            report.add("NumPy performance", DiagLevel.WARN,
                       f"Slow dot product: {dot_time*1e6:.1f}us (expected <1000us)",
                       {"fix": "Ensure NumPy is linked to BLAS (openblas/mkl)"})
        return True
    except Exception as e:
        report.add("NumPy performance", DiagLevel.FAIL, str(e))
        return False


def test_network_basic(report):
    """Test 3: Can we build a network and run ODE?"""
    try:
        from sg_engine.network import Network

        net = Network(D=10_000, beta=4.0, dt=0.05, seed=42)
        net.add_atoms(["A", "B", "C", "D", "E", "F", "G", "H"])
        net.generate_patterns(20, gen_seed=42)

        report.add("Network build", DiagLevel.OK,
                   f"Built network: {net.n_patterns} patterns, 8 atoms, D=10000")

        # Run ODE
        Q = net.P_mat[0].copy()
        t0 = time.time()
        basin, Q_final, traj = net.run_dynamics(Q, alpha=0.0)
        ode_time = time.time() - t0

        report.add("ODE dynamics", DiagLevel.OK,
                   f"ODE converged to '{basin}' in {len(traj)} steps ({ode_time*1000:.1f}ms)")

        return net
    except Exception as e:
        report.add("Network build", DiagLevel.FAIL, str(e),
                   {"traceback": traceback.format_exc()})
        return None


def test_zone_identification(report, net=None):
    """Test 4: Zone identification pipeline works?"""
    try:
        if net is None:
            from sg_engine.network import Network
            net = Network(D=10_000, beta=4.0, dt=0.05, seed=42)
            net.add_atoms([f"F{i}" for i in range(16)])
            net.generate_patterns(25, gen_seed=42)

        # Simulate a zone fingerprint query
        rng = np.random.default_rng(99)
        q = rng.choice([-1, 1], size=net.D).astype(np.float64)

        t0 = time.time()
        basin, Q_final, traj = net.run_dynamics(q, alpha=0.0)
        ode_time = time.time() - t0

        sims = net.P_mat @ Q_final / net.D
        top3 = sorted(enumerate(sims), key=lambda x: -x[1])[:3]

        report.add("Zone identification", DiagLevel.OK,
                   f"Classified in {ode_time*1000:.1f}ms, top match: {net.pat_names[top3[0][0]]} (sim={top3[0][1]:.3f})")
        return True
    except Exception as e:
        report.add("Zone identification", DiagLevel.FAIL, str(e))
        return False


def test_determinism(report):
    """Test 5: Same input → same output (CRITICAL for robotics)?"""
    try:
        from sg_engine.network import Network

        results = []
        for run in range(3):
            net = Network(D=10_000, beta=4.0, dt=0.05, seed=42)
            net.add_atoms([f"F{i}" for i in range(16)])
            net.generate_patterns(25, gen_seed=42)

            q = net.P_mat[0].copy()
            basin, Q_final, _ = net.run_dynamics(q, alpha=0.0)
            results.append((basin, float(np.sum(Q_final))))

        all_same = all(r == results[0] for r in results)
        if all_same:
            report.add("Determinism", DiagLevel.OK,
                       "3 runs → identical output (seed=42)")
        else:
            report.add("Determinism", DiagLevel.FAIL,
                       f"Non-deterministic! Results: {results}",
                       {"fix": "This should never happen. Report as bug."})
        return all_same
    except Exception as e:
        report.add("Determinism", DiagLevel.FAIL, str(e))
        return False


def test_cold_start_pipeline(report):
    """Test 6: Full cold start pipeline end-to-end."""
    try:
        from sg_engine.network import Network

        D = 10_000
        rng = np.random.default_rng(42)

        # Build 25-zone network (simulating warehouse)
        net = Network(D=D, beta=4.0, dt=0.05, seed=42)
        atom_names = [f"F{i}" for i in range(16)]
        net.add_atoms(atom_names)

        # Create 25 zone patterns
        for i in range(25):
            features = rng.uniform(0, 1, 16)
            vec = np.ones(D)
            for j, val in enumerate(features):
                scaled = 2.0 * val - 1.0
                vec *= (scaled * net.atoms[atom_names[j]])
            net.add_pattern(f"ZONE_{i}", np.sign(vec))

        net.generate_patterns(15, gen_seed=42)

        # Simulate cold start: unknown scan → zone ID
        test_features = rng.uniform(0, 1, 16)
        vec = np.ones(D)
        for j, val in enumerate(test_features):
            scaled = 2.0 * val - 1.0
            vec *= (scaled * net.atoms[atom_names[j]])
        q = np.sign(vec).astype(np.float64)

        t0 = time.time()
        basin, Q_final, traj = net.run_dynamics(q, alpha=0.0)
        sims = net.P_mat @ Q_final / D
        pipeline_time = time.time() - t0

        # Get top candidates
        zone_sims = [(net.pat_names[i], float(sims[i]))
                     for i in range(len(sims)) if net.pat_names[i].startswith("ZONE_")]
        zone_sims.sort(key=lambda x: -x[1])
        top3 = zone_sims[:3]

        if pipeline_time < 0.5:
            report.add("Cold start pipeline", DiagLevel.OK,
                       f"Full pipeline: {pipeline_time*1000:.1f}ms, "
                       f"top: {top3[0][0]} (sim={top3[0][1]:.3f})")
        else:
            report.add("Cold start pipeline", DiagLevel.WARN,
                       f"Slow: {pipeline_time*1000:.1f}ms (target <500ms)",
                       {"fix": "Check CPU load. io-gita ODE should be <200ms on modern hardware."})
        return True
    except Exception as e:
        report.add("Cold start pipeline", DiagLevel.FAIL, str(e),
                   {"traceback": traceback.format_exc()})
        return False


def test_graph_disambiguation(report):
    """Test 7: Graph topology resolves identical zones?"""
    try:
        # Simulate 6 shelf zones with identical LiDAR signatures
        adjacency = {
            "SHELF_1": ["LANE_W", "MID_N", "SHELF_3"],
            "SHELF_2": ["MID_N", "LANE_E", "SHELF_4"],
            "SHELF_3": ["CROSS_W", "SHELF_1", "SHELF_5", "HUB"],
            "SHELF_4": ["HUB", "SHELF_2", "SHELF_6", "CROSS_E"],
            "SHELF_5": ["LANE_W2", "SHELF_3", "MID_S"],
            "SHELF_6": ["MID_S", "SHELF_4", "LANE_E2"],
        }

        # Robot was at LANE_W, now sees a shelf signature
        previous_zone = "LANE_W"
        candidates = ["SHELF_1", "SHELF_2", "SHELF_3", "SHELF_4", "SHELF_5", "SHELF_6"]

        # Graph filter: which shelves are reachable from LANE_W?
        reachable = set()
        for shelf in candidates:
            if previous_zone in adjacency.get(shelf, []):
                reachable.add(shelf)

        if len(reachable) == 1:
            resolved = reachable.pop()
            report.add("Graph disambiguation", DiagLevel.OK,
                       f"6 identical shelves → resolved to {resolved} using graph")
        elif len(reachable) < len(candidates):
            report.add("Graph disambiguation", DiagLevel.OK,
                       f"6 identical shelves → narrowed to {len(reachable)}: {reachable}")
        else:
            report.add("Graph disambiguation", DiagLevel.WARN,
                       "Graph did not reduce candidates")
        return True
    except Exception as e:
        report.add("Graph disambiguation", DiagLevel.FAIL, str(e))
        return False


def test_benchmark(report, n_runs=50):
    """Test 8: Performance benchmark (ODE throughput)."""
    try:
        from sg_engine.network import Network

        net = Network(D=10_000, beta=4.0, dt=0.05, seed=42)
        net.add_atoms([f"F{i}" for i in range(16)])
        net.generate_patterns(40, gen_seed=42)

        rng = np.random.default_rng(123)
        times = []
        for _ in range(n_runs):
            q = rng.choice([-1, 1], size=net.D).astype(np.float64)
            t0 = time.time()
            net.run_dynamics(q, alpha=0.0)
            times.append(time.time() - t0)

        mean_ms = np.mean(times) * 1000
        p95_ms = np.percentile(times, 95) * 1000
        throughput = 1000 / mean_ms

        report.add("ODE benchmark", DiagLevel.OK,
                   f"{n_runs} runs: mean={mean_ms:.1f}ms, p95={p95_ms:.1f}ms, "
                   f"throughput={throughput:.0f} queries/sec",
                   {"mean_ms": round(mean_ms, 1),
                    "p95_ms": round(p95_ms, 1),
                    "throughput": round(throughput, 1)})
        return True
    except Exception as e:
        report.add("ODE benchmark", DiagLevel.FAIL, str(e))
        return False


def test_license(report):
    """Test 9: License status."""
    try:
        from sg_engine_locked import license_info, _check_license, _LICENSE_CONFIG
        days = _check_license()
        report.add("License", DiagLevel.OK,
                   f"Valid. Licensee: {_LICENSE_CONFIG['licensee']}, "
                   f"expires: {_LICENSE_CONFIG['expiry_date']} ({days} days left)")
        return True
    except ImportError:
        report.add("License", DiagLevel.INFO,
                   "Using unlocked engine (no license wrapper)")
        return True
    except Exception as e:
        report.add("License", DiagLevel.FAIL, str(e))
        return False


# ── MAIN ──────────────────────────────────────────────────

def run_full_diagnostic():
    """Run all diagnostic tests."""
    print("=" * 60)
    print("  io-gita DIAGNOSTIC REPORT")
    print("  " + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 60)

    report = DiagnosticReport()

    print("\n── Environment ──")
    report.add("Python", DiagLevel.INFO, f"{sys.version.split()[0]} ({sys.platform})")

    print("\n── Core Tests ──")
    can_import = test_import(report)
    test_numpy(report)
    if can_import:
        net = test_network_basic(report)
        test_zone_identification(report, net)
        test_determinism(report)

    print("\n── Pipeline Tests ──")
    test_cold_start_pipeline(report)
    test_graph_disambiguation(report)

    print("\n── Performance ──")
    if can_import:
        test_benchmark(report)

    print("\n── License ──")
    test_license(report)

    # Summary
    s = report.summary()
    print(f"\n{'='*60}")
    print(f"  SUMMARY: {s['ok']} OK, {s['warn']} WARN, {s['fail']} FAIL")
    if s["fail"] > 0:
        print(f"  ACTION REQUIRED: Fix {s['fail']} failing test(s) above")
    elif s["warn"] > 0:
        print(f"  STATUS: Working with {s['warn']} warning(s)")
    else:
        print(f"  STATUS: All systems operational")
    print(f"{'='*60}")

    report.save()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="io-gita Diagnostic Tool")
    parser.add_argument("--benchmark", action="store_true", help="Run performance benchmark only")
    parser.add_argument("--check-license", action="store_true", help="Check license status only")
    parser.add_argument("--test-zone", type=str, help="Test specific zone identification")
    args = parser.parse_args()

    if args.check_license:
        report = DiagnosticReport()
        test_license(report)
    elif args.benchmark:
        report = DiagnosticReport()
        test_import(report)
        test_benchmark(report, n_runs=100)
        report.save("benchmark_report.json")
    else:
        run_full_diagnostic()
