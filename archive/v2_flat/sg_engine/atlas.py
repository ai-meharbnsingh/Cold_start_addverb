"""
Atlas: Directed transition graph mapped from a Network.

Maps all N*(N-1) edges at multiple alpha levels.
Provides reachability queries, degree stats, and SCC analysis.

P2.2: n_workers parameter accepted for API compatibility (not yet functional;
      Network objects contain lambdas/Generators that cannot be pickled).
P2.3: Incremental atlas updates via add_and_remap().
P2.4: Pattern hashing for cache validation via pattern_hash(), save/load with hash.
"""

import hashlib
import json
import warnings
import numpy as np
from collections import defaultdict, Counter, deque


# ── Top-level worker functions for multiprocessing (must be picklable) ──

def _test_edge_multi_alpha(args):
    """Worker: test one (src, dst) pair across multiple alpha levels.

    Args is a tuple: (network, src, dst, alphas, n_trials)
    Returns: (src, dst, {alpha: hit_rate})
    """
    network, src, dst, alphas, n_trials = args
    hits_by_alpha = {}
    for alpha in alphas:
        rate = network.test_edge(src, dst, alpha=alpha, n_trials=n_trials)
        hits_by_alpha[alpha] = rate
    return (src, dst, hits_by_alpha)


def _test_edge_single_alpha(args):
    """Worker: test one (src, dst) pair at a single alpha level.

    Args is a tuple: (network, src, dst, alpha, n_trials)
    Returns: (src, dst, alpha, hit_rate)
    """
    network, src, dst, alpha, n_trials = args
    rate = network.test_edge(src, dst, alpha=alpha, n_trials=n_trials)
    return (src, dst, alpha, rate)


class Atlas:
    def __init__(self, network=None):
        self.network = network
        self.edges = {}       # (src, dst) -> {alpha: hit_rate}
        self.graph = {}       # alpha -> {src: set(dst)}
        self.in_degree = Counter()
        self.out_degree = Counter()
        self.scc = set()
        self._alphas = []

    # ── P2.4: Pattern hashing ──────────────────────────────────────────

    def pattern_hash(self):
        """Compute SHA256 hash of the network's pattern matrix.

        Returns:
            str: hex digest of the hash, or None if no network/patterns.
        """
        if self.network is None or self.network.P_mat is None:
            return None
        # Deterministic: convert to bytes via tobytes() on the float64 array
        data = self.network.P_mat.astype(np.float64).tobytes()
        return hashlib.sha256(data).hexdigest()

    # ── P2.2: Parallel mapping ─────────────────────────────────────────

    def map(self, alphas=(0.10, 0.30, 0.50), n_trials=3, threshold=2,
            verbose=True, n_workers=1):
        """Map the full transition graph at multiple alpha levels.

        Args:
            alphas: tuple of alpha forcing levels to test.
            n_trials: number of trials per edge per alpha.
            threshold: minimum hits to count edge as existing.
            verbose: print progress.
            n_workers: NOT YET SUPPORTED. Accepted for API compatibility but
                always runs sequentially. Network objects contain lambdas and
                numpy Generators that cannot be pickled for multiprocessing.
        """
        if self.network is None:
            raise ValueError("No network set. Pass network to constructor.")

        self._alphas = list(alphas)
        net = self.network
        names = net.pat_names
        N = len(names)
        total = N * (N - 1)

        if verbose:
            print(f"  Mapping {total} edges at alphas={alphas}...")

        for alpha in alphas:
            self.graph[alpha] = defaultdict(set)

        if n_workers > 1:
            warnings.warn(
                "Multiprocessing is not yet supported: Network objects contain "
                "lambdas and numpy Generators that cannot be pickled. "
                "Falling back to sequential mapping. Use n_workers=1 to "
                "suppress this warning.",
                RuntimeWarning,
                stacklevel=2,
            )
        self._map_sequential(names, alphas, n_trials, threshold,
                             verbose, total)

        self._compute_stats()
        if verbose:
            self._print_summary()
        return self

    def _map_sequential(self, names, alphas, n_trials, threshold,
                        verbose, total):
        """Sequential full mapping (original algorithm)."""
        net = self.network
        done = 0
        for i, src in enumerate(names):
            for j, dst in enumerate(names):
                if i == j:
                    continue
                hits_by_alpha = {}
                for alpha in alphas:
                    rate = net.test_edge(src, dst, alpha=alpha,
                                         n_trials=n_trials)
                    hits = int(rate * n_trials)
                    hits_by_alpha[alpha] = rate
                    if hits >= threshold:
                        self.graph[alpha][src].add(dst)
                self.edges[(src, dst)] = hits_by_alpha
                done += 1
                if verbose and done % 500 == 0:
                    print(f"    {done}/{total} edges mapped")

    def _map_parallel(self, names, alphas, n_trials, threshold,
                      verbose, n_workers, total):
        """Parallel full mapping using multiprocessing.Pool."""
        try:
            import multiprocessing
        except ImportError:
            if verbose:
                print("  multiprocessing unavailable, falling back to sequential")
            self._map_sequential(names, alphas, n_trials, threshold,
                                 verbose, total)
            return

        net = self.network
        # Build work items
        work = []
        for i, src in enumerate(names):
            for j, dst in enumerate(names):
                if i == j:
                    continue
                work.append((net, src, dst, alphas, n_trials))

        try:
            with multiprocessing.Pool(processes=n_workers) as pool:
                results = pool.map(_test_edge_multi_alpha, work)
        except Exception:
            # Fallback to sequential on any multiprocessing failure
            if verbose:
                print("  multiprocessing failed, falling back to sequential")
            self._map_sequential(names, alphas, n_trials, threshold,
                                 verbose, total)
            return

        # Collect results
        for src, dst, hits_by_alpha in results:
            self.edges[(src, dst)] = hits_by_alpha
            for alpha, rate in hits_by_alpha.items():
                hits = int(rate * n_trials)
                if hits >= threshold:
                    self.graph[alpha][src].add(dst)

    def map_sampled(self, n_samples=600, alpha=0.50, n_trials=5, threshold=3,
                    seed=2828, verbose=True, n_workers=1):
        """Map a random sample of edges (faster for large N).

        Args:
            n_samples: number of edges to sample and test.
            alpha: forcing alpha level.
            n_trials: trials per edge.
            threshold: minimum hits to count edge.
            seed: RNG seed for reproducible sampling.
            verbose: print progress.
            n_workers: NOT YET SUPPORTED. Accepted for API compatibility but
                always runs sequentially. Network objects contain lambdas and
                numpy Generators that cannot be pickled for multiprocessing.
        """
        net = self.network
        names = net.pat_names
        N = len(names)
        self._alphas = [alpha]
        self.graph[alpha] = defaultdict(set)

        rng = np.random.default_rng(seed)
        sampled = set()
        while len(sampled) < n_samples:
            si, di = rng.integers(0, N), rng.integers(0, N)
            if si != di and (si, di) not in sampled:
                sampled.add((si, di))

        if verbose:
            print(f"  Mapping {n_samples} sampled edges at alpha={alpha}...")

        if n_workers > 1:
            warnings.warn(
                "Multiprocessing is not yet supported: Network objects contain "
                "lambdas and numpy Generators that cannot be pickled. "
                "Falling back to sequential mapping. Use n_workers=1 to "
                "suppress this warning.",
                RuntimeWarning,
                stacklevel=2,
            )
        self._map_sampled_sequential(names, sampled, alpha, n_trials,
                                      threshold, verbose, n_samples)

        self._compute_stats()
        if verbose:
            self._print_summary()
        return self

    def _map_sampled_sequential(self, names, sampled, alpha, n_trials,
                                 threshold, verbose, n_samples):
        """Sequential sampled mapping (original algorithm)."""
        net = self.network
        for count, (si, di) in enumerate(sampled):
            src, dst = names[si], names[di]
            rate = net.test_edge(src, dst, alpha=alpha, n_trials=n_trials)
            self.edges[(src, dst)] = {alpha: rate}
            if int(rate * n_trials) >= threshold:
                self.graph[alpha][src].add(dst)
            if verbose and (count + 1) % 150 == 0:
                print(f"    {count+1}/{n_samples} done")

    def _map_sampled_parallel(self, names, sampled, alpha, n_trials,
                               threshold, verbose, n_workers, n_samples):
        """Parallel sampled mapping using multiprocessing.Pool."""
        try:
            import multiprocessing
        except ImportError:
            if verbose:
                print("  multiprocessing unavailable, falling back to sequential")
            self._map_sampled_sequential(names, sampled, alpha, n_trials,
                                          threshold, verbose, n_samples)
            return

        net = self.network
        work = []
        for si, di in sampled:
            src, dst = names[si], names[di]
            work.append((net, src, dst, alpha, n_trials))

        try:
            with multiprocessing.Pool(processes=n_workers) as pool:
                results = pool.map(_test_edge_single_alpha, work)
        except Exception:
            if verbose:
                print("  multiprocessing failed, falling back to sequential")
            self._map_sampled_sequential(names, sampled, alpha, n_trials,
                                          threshold, verbose, n_samples)
            return

        for src, dst, _alpha, rate in results:
            self.edges[(src, dst)] = {alpha: rate}
            if int(rate * n_trials) >= threshold:
                self.graph[alpha][src].add(dst)

    # ── P2.3: Incremental atlas updates ────────────────────────────────

    def add_and_remap(self, name, vector, validate_pct=0.10,
                      drift_threshold=0.05, n_trials=3, threshold=2,
                      alpha=None, verbose=True):
        """Add a pattern to the network and map edges involving it.

        Also re-tests a sample of old->old edges to detect landscape drift.

        Args:
            name: name for the new pattern.
            vector: pattern vector (numpy array of dimension D).
            validate_pct: fraction of old->old edges to re-test (0.0 to 1.0).
            drift_threshold: if more than this fraction of sampled old edges
                             changed, return a drift warning.
            n_trials: trials per edge test.
            threshold: minimum hits to count edge as existing.
            alpha: alpha level for edge testing (default: best alpha).
            verbose: print progress.

        Returns:
            dict with keys:
              - new_edges_tested: int
              - new_edges_found: int
              - old_edges_sampled: int
              - old_edges_changed: int
              - drift_pct: float
              - drift_warning: bool (True if drift_pct > drift_threshold)
              - message: str
        """
        if self.network is None:
            raise ValueError("No network set. Pass network to constructor.")

        net = self.network
        a = alpha or (max(self._alphas) if self._alphas else 0.50)

        # Ensure alpha is in our alphas list
        if a not in self._alphas:
            self._alphas.append(a)
        if a not in self.graph:
            self.graph[a] = defaultdict(set)

        # Add pattern to network
        net.add_pattern(name, vector)
        net._compile()

        old_names = [n for n in net.pat_names if n != name]

        # Map new->old and old->new edges
        new_edges_tested = 0
        new_edges_found = 0

        for other in old_names:
            # new -> other
            rate = net.test_edge(name, other, alpha=a, n_trials=n_trials)
            self.edges[(name, other)] = {a: rate}
            if int(rate * n_trials) >= threshold:
                self.graph[a][name].add(other)
                new_edges_found += 1
            new_edges_tested += 1

            # other -> new
            rate = net.test_edge(other, name, alpha=a, n_trials=n_trials)
            self.edges[(other, name)] = {a: rate}
            if int(rate * n_trials) >= threshold:
                self.graph[a][other].add(name)
                new_edges_found += 1
            new_edges_tested += 1

        # Validate a sample of old->old edges
        old_edges = [(s, d) for (s, d) in self.edges
                     if s != name and d != name and s in old_names
                     and d in old_names]

        n_sample = max(1, int(len(old_edges) * validate_pct))
        n_sample = min(n_sample, len(old_edges))

        rng = np.random.default_rng(42)
        if old_edges:
            sample_indices = rng.choice(len(old_edges), size=n_sample,
                                        replace=False)
            sampled_edges = [old_edges[i] for i in sample_indices]
        else:
            sampled_edges = []

        old_edges_changed = 0
        for src, dst in sampled_edges:
            old_rate = self.edges.get((src, dst), {}).get(a, 0.0)
            old_hit = int(old_rate * n_trials) >= threshold

            new_rate = net.test_edge(src, dst, alpha=a, n_trials=n_trials)
            new_hit = int(new_rate * n_trials) >= threshold

            # Update edge data
            if (src, dst) not in self.edges:
                self.edges[(src, dst)] = {}
            self.edges[(src, dst)][a] = new_rate

            if new_hit:
                self.graph[a][src].add(dst)
            elif dst in self.graph[a].get(src, set()):
                self.graph[a][src].discard(dst)

            if old_hit != new_hit:
                old_edges_changed += 1

        drift_pct = (old_edges_changed / len(sampled_edges)
                     if sampled_edges else 0.0)
        drift_warning = drift_pct > drift_threshold

        self._compute_stats()

        if drift_warning:
            msg = (f"DRIFT WARNING: {drift_pct:.1%} of sampled old edges "
                   f"changed (threshold: {drift_threshold:.1%}). "
                   f"Consider full remap.")
        else:
            msg = (f"Incremental update OK: {new_edges_found} new edges "
                   f"found, drift {drift_pct:.1%}")

        if verbose:
            print(f"  {msg}")

        return {
            "new_edges_tested": new_edges_tested,
            "new_edges_found": new_edges_found,
            "old_edges_sampled": len(sampled_edges),
            "old_edges_changed": old_edges_changed,
            "drift_pct": drift_pct,
            "drift_warning": drift_warning,
            "message": msg,
        }

    # ── Core mapping internals ─────────────────────────────────────────

    def _compute_stats(self):
        """Compute degree distributions and SCC."""
        self.in_degree = Counter()
        self.out_degree = Counter()

        best_alpha = max(self._alphas) if self._alphas else 0.50
        g = self.graph.get(best_alpha, {})

        for src, dsts in g.items():
            self.out_degree[src] += len(dsts)
            for d in dsts:
                self.in_degree[d] += 1

        # Find largest SCC via Tarjan-like approach
        all_nodes = set()
        for src, dsts in g.items():
            all_nodes.add(src)
            all_nodes.update(dsts)

        # Find ALL SCCs, keep the largest
        if all_nodes:
            # Kosaraju: find largest SCC
            rev_g = defaultdict(set)
            for src, dsts in g.items():
                for d in dsts:
                    rev_g[d].add(src)

            best_scc = set()
            for start_node in all_nodes:
                fwd = self._bfs(start_node, g)
                bwd = self._bfs(start_node, rev_g)
                scc_candidate = fwd & bwd
                if len(scc_candidate) > len(best_scc):
                    best_scc = scc_candidate
            self.scc = best_scc
        else:
            self.scc = set()

    def _bfs(self, start, graph):
        visited = set()
        queue = deque([start])
        visited.add(start)
        while queue:
            node = queue.popleft()
            for neighbor in graph.get(node, set()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        return visited

    def _print_summary(self):
        for alpha in self._alphas:
            g = self.graph.get(alpha, {})
            n_edges = sum(len(v) for v in g.values())
            print(f"    alpha={alpha}: {n_edges} edges")
        print(f"    SCC: {len(self.scc)} nodes")
        if self.in_degree:
            top_sinks = self.in_degree.most_common(3)
            print(f"    Top sinks: {top_sinks}")

    # ── Query methods ──────────────────────────────────────────────────

    def edge_exists(self, src, dst, alpha=None):
        """Check if edge exists at given alpha (or best alpha)."""
        a = alpha or max(self._alphas)
        return dst in self.graph.get(a, {}).get(src, set())

    def reachable_from(self, src, alpha=None):
        """BFS: all nodes reachable from src at given alpha."""
        a = alpha or max(self._alphas)
        return self._bfs(src, self.graph.get(a, {})) - {src}

    def shortest_path(self, src, dst, alpha=None):
        """BFS shortest path from src to dst."""
        a = alpha or max(self._alphas)
        g = self.graph.get(a, {})
        visited = set()
        queue = deque([(src, [src])])
        visited.add(src)
        while queue:
            node, path = queue.popleft()
            if node == dst:
                return path
            for neighbor in g.get(node, set()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

    def min_alpha_path(self, src, dst):
        """Find the lowest alpha at which src can reach dst."""
        for alpha in sorted(self._alphas):
            path = self.shortest_path(src, dst, alpha)
            if path:
                return alpha, path
        return None, None

    def hub_sinks(self, top_n=5):
        """Return top-N nodes by in-degree."""
        return self.in_degree.most_common(top_n)

    def hub_sources(self, top_n=5):
        """Return top-N nodes by out-degree."""
        return self.out_degree.most_common(top_n)

    # ── P2.4: Save/load with pattern hash ──────────────────────────────

    def save(self, path, cache_hash=True):
        """Save atlas to JSON.

        Args:
            path: file path to write.
            cache_hash: if True (default), include SHA256 hash of pattern
                        matrix for cache validation on load.
        """
        data = {
            "alphas": self._alphas,
            "edges": {f"{s}\t{d}": v for (s, d), v in self.edges.items()},
            "scc": list(self.scc),
            "in_degree": dict(self.in_degree),
            "out_degree": dict(self.out_degree),
        }
        if cache_hash:
            h = self.pattern_hash()
            if h is not None:
                data["pattern_hash"] = h
        with open(path, "w") as f:
            json.dump(data, f, indent=2, default=float)

    def load(self, path, verify_hash=True):
        """Load atlas from JSON.

        Args:
            path: file path to read.
            verify_hash: if True (default), verify pattern hash matches.
                         Raises ValueError on mismatch.

        Returns:
            self
        """
        with open(path) as f:
            data = json.load(f)

        # P2.4: Verify pattern hash if present and requested
        stored_hash = data.get("pattern_hash")
        if verify_hash and stored_hash is not None:
            current_hash = self.pattern_hash()
            if current_hash is not None and current_hash != stored_hash:
                raise ValueError(
                    f"Pattern hash mismatch: stored={stored_hash[:16]}... "
                    f"current={current_hash[:16]}... "
                    f"Patterns have changed; atlas cache is stale."
                )

        self._alphas = data["alphas"]
        self.scc = set(data.get("scc", []))
        self.in_degree = Counter(data.get("in_degree", {}))
        self.out_degree = Counter(data.get("out_degree", {}))
        self.edges = {}
        self.graph = {a: defaultdict(set) for a in self._alphas}
        for key, hits in data["edges"].items():
            # Support both tab (new) and pipe (legacy) delimiters
            if "\t" in key:
                src, dst = key.split("\t", 1)
            else:
                src, dst = key.split("|", 1)
            self.edges[(src, dst)] = {float(k): v for k, v in hits.items()}
            for a_str, rate in hits.items():
                a = float(a_str)
                if rate >= 0.5 and a in self.graph:
                    self.graph[a][src].add(dst)
        self._compute_stats()
        return self

    # ── Summary / repr ─────────────────────────────────────────────────

    def summary(self):
        """Return a summary dict."""
        best_alpha = max(self._alphas) if self._alphas else 0.50
        g = self.graph.get(best_alpha, {})
        n_edges = sum(len(v) for v in g.values())
        return {
            "alphas": self._alphas,
            "n_edges": n_edges,
            "scc_size": len(self.scc),
            "top_sinks": self.in_degree.most_common(3),
            "top_sources": self.out_degree.most_common(3),
        }

    def __repr__(self):
        best = max(self._alphas) if self._alphas else "?"
        n_edges = (sum(len(v) for v in self.graph.get(best, {}).values())
                   if self._alphas else 0)
        return (f"Atlas(edges={n_edges}, scc={len(self.scc)}, "
                f"alphas={self._alphas})")
