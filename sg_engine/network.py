"""
Network: Core Hopfield attractor network with compositional patterns.

Handles atom generation, pattern construction, weight matrix,
ODE dynamics, and basin identification.
"""

import hashlib
import numpy as np
from collections import Counter


def _stable_hash(s: str) -> int:
    """Deterministic hash that is stable across processes and machines."""
    return int(hashlib.sha256(s.encode()).hexdigest()[:8], 16)


NONLINEARITIES = {
    "tanh": lambda x, beta: np.tanh(beta * x),
    "hard_tanh": lambda x, beta: np.clip(beta * x, -1, 1),
    "sign": lambda x, beta: np.sign(x),
    "relu_symmetric": lambda x, beta: np.clip(x * beta, -1, 1) * (np.abs(x * beta) > 0.1),
    "staircase": lambda x, beta: np.sign(x) * np.minimum(np.floor(np.abs(beta * x) * 3 + 0.5) / 3, 1.0),
}


class Network:
    def __init__(self, D=10_000, beta=4.0, dt=0.05, seed=42, nonlinearity="tanh"):
        self.D = D
        self.beta = beta
        self.dt = dt
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.inv_sqrt_D = 1.0 / np.sqrt(D)

        # F5: pluggable nonlinearity
        if callable(nonlinearity):
            self._activation = nonlinearity
            self._nl_name = "custom"
        elif nonlinearity in NONLINEARITIES:
            self._activation = NONLINEARITIES[nonlinearity]
            self._nl_name = nonlinearity
        else:
            raise ValueError(f"Unknown nonlinearity: {nonlinearity}. "
                             f"Options: {list(NONLINEARITIES.keys())} or a callable(x, beta)")

        self.atoms = {}
        self.patterns = {}
        self.pat_names = []
        self.P_mat = None
        self.P_mat_T = None
        self.pat_idx = {}
        self.forces = {}
        self.basin_energy = {}

    def add_atoms(self, names, rng=None):
        """Add named atoms as random bipolar vectors."""
        r = rng or self.rng
        for name in names:
            if name not in self.atoms:
                self.atoms[name] = r.choice([-1, 1], size=self.D).astype(np.float64)
        return self

    def add_atoms_from_vectors(self, atom_dict):
        """Add atoms from pre-built vectors."""
        self.atoms.update(atom_dict)
        return self

    def bind(self, a, b):
        """Element-wise multiplication (relational binding)."""
        return a * b

    def cosim(self, a, b):
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-10 or nb < 1e-10:
            return 0.0
        return float(np.dot(a, b) / (na * nb))

    def add_pattern(self, name, vector):
        """Add a named pattern vector."""
        self.patterns[name] = np.sign(vector).astype(np.float64)

    def generate_patterns(self, n, gen_seed=None):
        """Generate n structured patterns from atoms."""
        import warnings
        n_atoms = len(self.atoms)
        if n_atoms > 0:
            ratio = n / n_atoms
            if ratio > 5.0:
                warnings.warn(
                    f"Pattern/atom ratio {ratio:.1f} (n={n}, atoms={n_atoms}) "
                    f"exceeds 5.0. ODE may collapse to single attractor at "
                    f"ratio > 8. Consider clustering patterns into families.",
                    stacklevel=2,
                )
        gen_rng = np.random.default_rng(gen_seed or n * 7)
        atom_names = list(self.atoms.keys())

        # First 6: hand-constructed from atom combos
        specs = []
        for i in range(min(6, n)):
            gen_rng.shuffle(atom_names)
            n_pos = gen_rng.integers(2, 5)
            n_neg = gen_rng.integers(1, 3)
            pos = atom_names[:n_pos]
            neg = atom_names[n_pos:n_pos + n_neg]
            vec = 3.0 * sum(self.atoms[a] for a in pos) - 3.0 * sum(self.atoms[a] for a in neg)
            self.patterns[f"p:{i}"] = np.sign(vec)

        # Rest: structured random
        attempts = 0
        while len(self.patterns) < n and attempts < n * 50:
            attempts += 1
            n_pos = gen_rng.integers(2, 5)
            n_neg = gen_rng.integers(0, 3)
            names = list(atom_names)
            gen_rng.shuffle(names)
            pos = names[:n_pos]
            neg = names[n_pos:n_pos + n_neg]
            w_pos = gen_rng.uniform(1.5, 4.0, size=n_pos)
            w_neg = gen_rng.uniform(1.0, 3.5, size=max(n_neg, 1))[:n_neg]
            vec = sum(w * self.atoms[a] for w, a in zip(w_pos, pos))
            if neg:
                vec -= sum(w * self.atoms[a] for w, a in zip(w_neg, neg))
            pat = np.sign(vec)
            if all(abs(self.cosim(pat, ep)) < 0.75 for ep in self.patterns.values()):
                self.patterns[f"s:{len(self.patterns)}"] = pat

        # Fill remaining with noise-structured
        while len(self.patterns) < n:
            n_a = gen_rng.integers(3, 7)
            names = list(atom_names)
            gen_rng.shuffle(names)
            vec = sum(gen_rng.normal(0, 2) * self.atoms[a] for a in names[:n_a])
            vec += gen_rng.normal(0, 0.1, size=self.D)
            self.patterns[f"s:{len(self.patterns)}"] = np.sign(vec)

        self._compile()
        return self

    def load_patterns(self, pattern_dict):
        """Load patterns from a dictionary {name: vector}."""
        for name, vec in pattern_dict.items():
            self.patterns[name] = np.sign(np.array(vec, dtype=np.float64))
        self._compile()
        return self

    def _compile(self):
        """Build matrices from patterns."""
        self.pat_names = list(self.patterns.keys())
        self.P_mat = np.array([self.patterns[n] for n in self.pat_names])
        self.P_mat_T = self.P_mat.T
        self.pat_idx = {n: i for i, n in enumerate(self.pat_names)}

        # Forces
        self.forces = {}
        for pn in self.pat_names:
            f = self.P_mat[self.pat_idx[pn]].copy().astype(np.float64)
            norm = np.linalg.norm(f)
            if norm > 1e-10:
                self.forces[pn] = f / norm * np.sqrt(self.D)
            else:
                self.forces[pn] = np.zeros(self.D)

        # Basin energies
        self.basin_energy = {}
        for pn in self.pat_names:
            Qp = self.P_mat[self.pat_idx[pn]].astype(np.float64)
            sims = self.P_mat @ Qp / self.D
            self.basin_energy[pn] = -float(np.sum(sims ** 2))

    MAX_ODE_STEPS = 10_000  # Hard safety limit

    def run_dynamics(self, Q_init, alpha=0.0, force_name=None, force_vec=None,
                     max_steps=800, noise=0.0, dt=None, integrator='euler'):
        """Run ODE dynamics from initial state. Returns (final_basin, final_state, trajectory).

        Args:
            force_name: name of pattern to use as force direction
            force_vec: raw force vector (overrides force_name)
            dt: step size (overrides self.dt if provided)
            integrator: 'euler' (fast, default) or 'rk2' (Heun's method, accurate)
        """
        Q = Q_init.astype(np.float64).copy()
        if force_vec is not None:
            force = force_vec
        else:
            force = self.forces.get(force_name) if force_name else None
        step_dt = dt or self.dt
        max_steps = min(max_steps, self.MAX_ODE_STEPS)
        use_rk2 = integrator == 'rk2'

        trajectory = []
        for step in range(max_steps):
            # Compute dQ at current state
            sims = self.P_mat @ Q / self.D
            h = self.P_mat_T @ sims
            dQ = -Q + self._activation(h, self.beta)
            if force is not None and alpha > 0:
                dQ += alpha * force
            if noise > 0:
                dQ += noise * self.rng.normal(0, 1, self.D) * self.inv_sqrt_D

            if use_rk2:
                # Heun's method: predictor-corrector
                Q_pred = Q + step_dt * dQ
                sims2 = self.P_mat @ Q_pred / self.D
                h2 = self.P_mat_T @ sims2
                dQ2 = -Q_pred + self._activation(h2, self.beta)
                if force is not None and alpha > 0:
                    dQ2 += alpha * force
                Q += step_dt * 0.5 * (dQ + dQ2)
            else:
                Q += step_dt * dQ

            trajectory.append(Q.copy())
            if np.linalg.norm(dQ) * self.inv_sqrt_D < 1e-7:
                break

        final_sims = self.P_mat @ Q / self.D
        final_idx = int(np.argmax(final_sims))
        return self.pat_names[final_idx], Q, trajectory

    def identify(self, Q):
        """Identify which basin a state belongs to."""
        sims = self.P_mat @ Q / self.D
        idx = int(np.argmax(sims))
        return self.pat_names[idx], float(sims[idx])

    def test_edge(self, src, dst, alpha=0.50, n_trials=5, noise=0.02):
        """Test if edge src→dst exists. Returns hit rate."""
        hits = 0
        for trial in range(n_trials):
            trial_rng = np.random.default_rng(
                self.seed + _stable_hash(f"{src}{dst}") % 10000 + trial)
            Q = self.P_mat[self.pat_idx[src]].copy()
            mask = trial_rng.random(self.D) < noise
            Q[mask] *= -1
            final, _, _ = self.run_dynamics(Q, alpha=alpha, force_name=dst)
            if final == dst:
                hits += 1
        return hits / n_trials

    @property
    def n_patterns(self):
        return len(self.pat_names)

    def __repr__(self):
        return (f"Network(D={self.D}, beta={self.beta}, "
                f"atoms={len(self.atoms)}, patterns={self.n_patterns})")
