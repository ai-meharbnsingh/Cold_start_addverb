"""
sg_engine_locked: Time-locked evaluation build of io-gita Semantic Gravity Engine.
==================================================================================

This is a LICENSED EVALUATION BUILD.
- Expires after the configured evaluation period.
- All core functionality is available during the evaluation window.
- After expiry, the engine raises LicenseExpiredError on any operation.
- Contact: meharban@adaptive-mind.com for production license.

(c) 2026 Adaptive Mind / Meharban Singh. All rights reserved.
Patent pending. Unauthorized redistribution prohibited.
"""

import datetime
import hashlib
import os
import json
import sys

# ─── License Configuration ────────────────────────────────────────────
# Change these values to control the evaluation window.

_LICENSE_CONFIG = {
    "licensee": "ADDVERB",                       # Company name
    "issued_date": "2026-03-27",                # Date issued
    "expiry_date": "2026-04-03",                # 7-day evaluation window
    "max_patterns": 100,                        # Max patterns in evaluation
    "max_dimensions": 10000,                    # Max D in evaluation
    "allow_atlas": True,                        # Allow atlas mapping
    "allow_bridge": True,                       # Allow bridge creation
    "watermark": True,                          # Watermark all outputs
}

# ─── License Enforcement ──────────────────────────────────────────────

class LicenseExpiredError(Exception):
    """Raised when the evaluation license has expired."""
    pass

class LicenseLimitError(Exception):
    """Raised when evaluation limits are exceeded."""
    pass


def _check_license():
    """Validate license. Raises LicenseExpiredError if expired."""
    expiry = datetime.date.fromisoformat(_LICENSE_CONFIG["expiry_date"])
    today = datetime.date.today()
    if today > expiry:
        raise LicenseExpiredError(
            f"\n{'='*60}\n"
            f"  io-gita EVALUATION LICENSE EXPIRED\n"
            f"  Expired on: {expiry}\n"
            f"  Licensee:   {_LICENSE_CONFIG['licensee']}\n"
            f"\n"
            f"  Contact meharban@adaptive-mind.com for:\n"
            f"  - Production license\n"
            f"  - Extended evaluation\n"
            f"  - Custom integration support\n"
            f"{'='*60}"
        )
    days_left = (expiry - today).days
    if days_left <= 14:
        print(f"[io-gita] WARNING: Evaluation license expires in {days_left} days "
              f"({expiry}). Contact meharban@adaptive-mind.com to renew.")
    return days_left


def _check_limits(n_patterns=0, D=0):
    """Check evaluation limits."""
    if n_patterns > _LICENSE_CONFIG["max_patterns"]:
        raise LicenseLimitError(
            f"Evaluation limit: max {_LICENSE_CONFIG['max_patterns']} patterns "
            f"(requested {n_patterns}). Contact meharban@adaptive-mind.com "
            f"for production license."
        )
    if D > _LICENSE_CONFIG["max_dimensions"]:
        raise LicenseLimitError(
            f"Evaluation limit: max D={_LICENSE_CONFIG['max_dimensions']} "
            f"(requested D={D}). Contact meharban@adaptive-mind.com "
            f"for production license."
        )


def license_info():
    """Print current license status."""
    days_left = _check_license()
    print(f"\n  io-gita Evaluation License")
    print(f"  ─────────────────────────")
    print(f"  Licensee:    {_LICENSE_CONFIG['licensee']}")
    print(f"  Issued:      {_LICENSE_CONFIG['issued_date']}")
    print(f"  Expires:     {_LICENSE_CONFIG['expiry_date']}")
    print(f"  Days left:   {days_left}")
    print(f"  Max patterns:{_LICENSE_CONFIG['max_patterns']}")
    print(f"  Max D:       {_LICENSE_CONFIG['max_dimensions']}")
    print(f"  Atlas:       {'enabled' if _LICENSE_CONFIG['allow_atlas'] else 'disabled'}")
    print(f"  Bridge:      {'enabled' if _LICENSE_CONFIG['allow_bridge'] else 'disabled'}")
    print()


# ─── Import gate ──────────────────────────────────────────────────────
# Validate license on first import.

_days_remaining = _check_license()

# Re-export only what's needed for cold start
from sg_engine.network import Network as _Network
from sg_engine.atlas import Atlas as _Atlas


# ─── Wrapped Network with license checks ─────────────────────────────

class Network(_Network):
    """Time-locked Network wrapper. All functionality available during evaluation."""

    def __init__(self, D=10_000, beta=4.0, dt=0.05, seed=42, nonlinearity="tanh"):
        _check_license()
        _check_limits(D=D)
        super().__init__(D=D, beta=beta, dt=dt, seed=seed, nonlinearity=nonlinearity)
        self._eval_watermark = _LICENSE_CONFIG["watermark"]

    def add_pattern(self, name, vector):
        _check_license()
        _check_limits(n_patterns=self.n_patterns + 1)
        return super().add_pattern(name, vector)

    def generate_patterns(self, n, gen_seed=None):
        _check_license()
        _check_limits(n_patterns=self.n_patterns + n)
        return super().generate_patterns(n, gen_seed=gen_seed)

    def run_dynamics(self, Q_init, **kwargs):
        _check_license()
        basin, Q, traj = super().run_dynamics(Q_init, **kwargs)
        return basin, Q, traj


class Atlas(_Atlas):
    """Time-locked Atlas wrapper."""

    def __init__(self, network=None):
        _check_license()
        if not _LICENSE_CONFIG["allow_atlas"]:
            raise LicenseLimitError("Atlas mapping disabled in this evaluation license.")
        super().__init__(network=network)

    def map(self, **kwargs):
        _check_license()
        return super().map(**kwargs)


__version__ = "0.3.0-eval"
__all__ = ["Network", "Atlas", "license_info",
           "LicenseExpiredError", "LicenseLimitError"]
