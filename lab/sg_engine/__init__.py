"""
SG-Engine: Topological Computation on Attractor Networks
========================================================

A deterministic computation engine built on compositional Hopfield networks.

IMPORTANT: This evaluation package contains a compiled binary (.so) that
REQUIRES Python 3.11.x exactly. The binary will NOT load on other versions.

If you see this warning at import time:
  "sg_engine: compiled binary requires Python 3.11, you have X.Y"
then install Python 3.11 or use a 3.11 virtualenv.

Usage:
    from sg_engine import Network, Atlas

    net = Network(D=10000, beta=4.0)
    net.add_atoms(["A", "B", "C", "D", "E", "F", "G", "H"])
    net.generate_patterns(60)
    atlas = Atlas(net)
    atlas.map(alphas=[0.10, 0.30, 0.50])
"""

import sys
import os
import warnings
import importlib

__version__ = "0.3.0"

# =====================================================================
# PYTHON VERSION CHECK — Binary is cpython-311 ONLY
# =====================================================================

_REQUIRED_MAJOR = 3
_REQUIRED_MINOR = 11
_PYTHON_OK = (sys.version_info.major == _REQUIRED_MAJOR and
              sys.version_info.minor == _REQUIRED_MINOR)

if not _PYTHON_OK:
    warnings.warn(
        f"sg_engine: compiled binary requires Python "
        f"{_REQUIRED_MAJOR}.{_REQUIRED_MINOR}, "
        f"you have {sys.version_info.major}.{sys.version_info.minor}. "
        f"The Hopfield ODE engine will NOT load. "
        f"Use Python 3.11 or a 3.11 virtualenv.",
        RuntimeWarning,
        stacklevel=2,
    )

# =====================================================================
# ISOLATED IMPORT — Only load from THIS package directory
# =====================================================================
# Prevent cross-project leakage: on Python != 3.11, the .so won't load
# and Python may find sg_engine.network from another project on sys.path.
# We guard against this by checking the loaded module's __file__ path.

_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))


def _safe_import(module_name: str):
    """Import a module and verify it comes from THIS package, not elsewhere.

    Returns the module if it's from this package directory, None otherwise.
    Prevents cross-project leakage when .so can't load on wrong Python version.
    """
    full_name = f"sg_engine.{module_name}"
    try:
        mod = importlib.import_module(full_name)
    except ImportError:
        return None

    # Verify the module comes from THIS directory, not another project
    mod_file = getattr(mod, "__file__", None)
    if mod_file is None:
        # Built-in or namespace package — accept it
        return mod

    mod_dir = os.path.dirname(os.path.abspath(mod_file))
    if not mod_dir.startswith(_PACKAGE_DIR):
        # CROSS-PROJECT LEAKAGE DETECTED
        warnings.warn(
            f"sg_engine.{module_name} resolved to {mod_file} which is "
            f"OUTSIDE this package ({_PACKAGE_DIR}). This is cross-project "
            f"leakage — likely because the .so binary requires Python 3.11 "
            f"and you're running {sys.version_info.major}."
            f"{sys.version_info.minor}. "
            f"Install Python 3.11 to use the compiled engine.",
            RuntimeWarning,
            stacklevel=3,
        )
        return None

    return mod


# =====================================================================
# CORE IMPORTS — Network + Atlas
# =====================================================================

Network = None
NONLINEARITIES = None
Atlas = None

_network_mod = _safe_import("network")
if _network_mod is not None:
    Network = getattr(_network_mod, "Network", None)
    NONLINEARITIES = getattr(_network_mod, "NONLINEARITIES", None)

_atlas_mod = _safe_import("atlas")
if _atlas_mod is not None:
    Atlas = getattr(_atlas_mod, "Atlas", None)

# Verify core loaded
if Network is None:
    warnings.warn(
        "sg_engine.Network could not be loaded. "
        "The Hopfield ODE engine is unavailable. "
        "Ensure Python 3.11 is used and the package is installed correctly.",
        RuntimeWarning,
        stacklevel=2,
    )

# =====================================================================
# OPTIONAL IMPORTS — Extended modules (not in evaluation package)
# =====================================================================

Planner = None
TieredInference = None
BridgeBuilder = None
ScheduleRunner = None
from_data = None
from_text = None
Verifier = None
build_gita_network = None
gita_8 = None
gita_20 = None
ATOM_MEANINGS = None
PATTERN_MEANINGS = None

_OPTIONAL_MODULES = [
    ("planner", ["Planner", "TieredInference"]),
    ("bridge", ["BridgeBuilder"]),
    ("schedule", ["ScheduleRunner"]),
    ("from_data", ["from_data"]),
    ("from_text", ["from_text"]),
    ("verifier", ["Verifier"]),
    ("presets", ["build_gita_network", "gita_8", "gita_20",
                 "ATOM_MEANINGS", "PATTERN_MEANINGS"]),
]

for _mod_name, _names in _OPTIONAL_MODULES:
    _mod = _safe_import(_mod_name)
    if _mod is not None:
        for _name in _names:
            _val = getattr(_mod, _name, None)
            if _val is not None:
                globals()[_name] = _val


def __getattr__(name):
    """Lazy-import data_pipeline functions (requires pandas/scipy/sklearn)."""
    _data_pipeline_names = {
        "prepare_for_engine", "auto_calibrate", "full_rca",
        "auto_select_target", "auto_select_k",
    }
    if name in _data_pipeline_names:
        _mod = _safe_import("data_pipeline")
        if _mod is not None:
            return getattr(_mod, name)
        raise AttributeError(
            f"sg_engine.{name} requires pandas/scipy/sklearn and Python 3.11"
        )
    raise AttributeError(f"module 'sg_engine' has no attribute {name!r}")


__all__ = ["Network", "NONLINEARITIES", "Atlas", "Planner",
           "TieredInference", "ScheduleRunner", "BridgeBuilder",
           "from_data", "from_text", "Verifier",
           "build_gita_network", "gita_8", "gita_20",
           "ATOM_MEANINGS", "PATTERN_MEANINGS"]
