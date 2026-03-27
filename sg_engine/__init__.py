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

from sg_engine.network import Network, NONLINEARITIES
from sg_engine.atlas import Atlas
from sg_engine.planner import Planner, TieredInference
from sg_engine.bridge import BridgeBuilder
from sg_engine.schedule import ScheduleRunner
from sg_engine.from_data import from_data
from sg_engine.from_text import from_text
from sg_engine.verifier import Verifier
from sg_engine.presets import (
    build_gita_network, gita_8, gita_20,
    ATOM_MEANINGS, PATTERN_MEANINGS,
)


def __getattr__(name):
    """Lazy-import data_pipeline functions (requires pandas/scipy/sklearn)."""
    _data_pipeline_names = {
        "prepare_for_engine", "auto_calibrate", "full_rca",
        "auto_select_target", "auto_select_k",
    }
    if name in _data_pipeline_names:
        from sg_engine import data_pipeline
        return getattr(data_pipeline, name)
    raise AttributeError(f"module 'sg_engine' has no attribute {name!r}")


__version__ = "0.3.0"
__all__ = ["Network", "NONLINEARITIES", "Atlas", "Planner",
           "TieredInference", "ScheduleRunner", "BridgeBuilder",
           "from_data", "from_text", "prepare_for_engine", "Verifier",
           "build_gita_network", "gita_8", "gita_20",
           "ATOM_MEANINGS", "PATTERN_MEANINGS"]
