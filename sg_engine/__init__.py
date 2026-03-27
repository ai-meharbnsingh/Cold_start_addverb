"""
SG-Engine: Topological Computation on Attractor Networks
========================================================

A deterministic computation engine built on compositional Hopfield networks.
Maps energy landscapes, builds directed transition graphs, and routes
multi-hop plans through topology — not gradient descent.

Usage:
    from sg_engine import Network, Atlas, Planner

    net = Network(D=10000, beta=4.0)
    net.add_atoms(["A", "B", "C", "D", "E", "F", "G", "H"])
    net.generate_patterns(60)
    atlas = Atlas(net)
    atlas.map(alphas=[0.10, 0.30, 0.50])
    plan = Planner(atlas).route("s:34", "s:12")
"""

# Core — always available (compiled binary + atlas.py)
from sg_engine.network import Network, NONLINEARITIES
from sg_engine.atlas import Atlas

# Extended modules — available in full sg_engine distribution.
# This evaluation package includes only Network + Atlas.
# Guard imports so cold start package works standalone.
_OPTIONAL_IMPORTS = {}

def _try_import(module_path, names):
    """Import optional modules without failing."""
    try:
        mod = __import__(module_path, fromlist=names)
        for name in names:
            _OPTIONAL_IMPORTS[name] = getattr(mod, name, None)
    except ImportError:
        pass

_try_import("sg_engine.planner", ["Planner", "TieredInference"])
_try_import("sg_engine.bridge", ["BridgeBuilder"])
_try_import("sg_engine.schedule", ["ScheduleRunner"])
_try_import("sg_engine.from_data", ["from_data"])
_try_import("sg_engine.from_text", ["from_text"])
_try_import("sg_engine.verifier", ["Verifier"])
_try_import("sg_engine.presets", [
    "build_gita_network", "gita_8", "gita_20",
    "ATOM_MEANINGS", "PATTERN_MEANINGS",
])

# Expose optional imports at module level
Planner = _OPTIONAL_IMPORTS.get("Planner")
TieredInference = _OPTIONAL_IMPORTS.get("TieredInference")
BridgeBuilder = _OPTIONAL_IMPORTS.get("BridgeBuilder")
ScheduleRunner = _OPTIONAL_IMPORTS.get("ScheduleRunner")
from_data = _OPTIONAL_IMPORTS.get("from_data")
from_text = _OPTIONAL_IMPORTS.get("from_text")
Verifier = _OPTIONAL_IMPORTS.get("Verifier")
build_gita_network = _OPTIONAL_IMPORTS.get("build_gita_network")
gita_8 = _OPTIONAL_IMPORTS.get("gita_8")
gita_20 = _OPTIONAL_IMPORTS.get("gita_20")
ATOM_MEANINGS = _OPTIONAL_IMPORTS.get("ATOM_MEANINGS")
PATTERN_MEANINGS = _OPTIONAL_IMPORTS.get("PATTERN_MEANINGS")


def __getattr__(name):
    """Lazy-import data_pipeline functions (requires pandas/scipy/sklearn)."""
    _data_pipeline_names = {
        "prepare_for_engine", "auto_calibrate", "full_rca",
        "auto_select_target", "auto_select_k",
    }
    if name in _data_pipeline_names:
        try:
            from sg_engine import data_pipeline
            return getattr(data_pipeline, name)
        except ImportError:
            raise AttributeError(
                f"sg_engine.{name} requires pandas/scipy/sklearn"
            )
    raise AttributeError(f"module 'sg_engine' has no attribute {name!r}")


__version__ = "0.3.0"
__all__ = ["Network", "NONLINEARITIES", "Atlas", "Planner",
           "TieredInference", "ScheduleRunner", "BridgeBuilder",
           "from_data", "from_text", "Verifier",
           "build_gita_network", "gita_8", "gita_20",
           "ATOM_MEANINGS", "PATTERN_MEANINGS"]
