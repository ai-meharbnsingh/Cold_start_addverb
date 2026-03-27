# Changelog

## [2.0.1] - 2026-03-27

### Fixed
- Add `fleet_integration/barcode_fallback_hook.py` (referenced in docs but was missing)
- Guard `sg_engine/__init__.py` imports — optional modules no longer crash
- Add `__init__.py` to all package directories
- Add input validation to `ProtocolV1Parser` (range clamping, null checks)
- Add `.gitignore`, `requirements.txt`, `LICENSE`, `CHANGELOG.md`
- Add unit tests for ProtocolV1Parser, ZoneIdentifier, BarcodeFallbackHook

## [2.0.0] - 2026-03-27

### Changed (Breaking)
- Complete rewrite aligned to Addverb fleet_core production architecture
- Replace `ros_integration/` with `fleet_integration/` (TCP-native)
- Replace AMCL hook with FMS REST/TCP adapter
- Replace 360deg LiDAR features with +-15deg obstacle sensor + barcode grid + odometry
- Config now uses barcode_range, graph_nodes, robot_types (Zippy10/AMR500)

### Added
- `ProtocolV1Parser` — Addverb TCP Protocol V1 (33-field) message parser
- `BarcodeGridTracker` — Tracks barcode reader health and dead reckoning
- `ColdStartOrchestrator` — Full cold start recovery sequence
- `FmsRestAdapter` / `FmsTcpAdapter` — FMS communication adapters
- `BarcodeFallbackHook` — Barcode failure state machine
- `fleet_debug_node.py` — TCP/REST monitor (replaces ros_debug_node.py)

### Archived
- v1 files moved to `archive/v1/` (ROS2/AMCL-based, not deleted)

## [1.0.0] - 2026-03-27

### Added
- Initial release targeting ROS2 Humble + AMCL
- 360deg LiDAR feature extraction (16 features)
- AMCL /initialpose hook for cold start acceleration
- 25-zone warehouse demo
