#!/usr/bin/env python
"""
Async FMS Adapter — Non-blocking FMS communication for io-gita
================================================================

Async version of fms_adapter.py for high-throughput scenarios where
blocking I/O on REST/TCP calls would stall the 15 Hz telemetry loop.

Uses asyncio + aiohttp (if available) or falls back to
concurrent.futures ThreadPoolExecutor.

Usage:
    import asyncio
    from fleet_integration.fms_adapter_async import AsyncFmsRestAdapter

    adapter = AsyncFmsRestAdapter()
    await adapter.send_cold_start_hint("zippy10_3", hint)

Contact: ai.meharbansingh@gmail.com
"""

import asyncio
import json
import time
import socket
from typing import Optional
from concurrent.futures import ThreadPoolExecutor

# Try aiohttp first, fall back to thread-pool wrapped urllib
try:
    import aiohttp
    _HAS_AIOHTTP = True
except ImportError:
    _HAS_AIOHTTP = False
    import urllib.request
    import urllib.error


class AsyncFmsRestAdapter:
    """
    Async REST adapter for FMS communication.

    Uses aiohttp if available, otherwise wraps synchronous urllib
    in a ThreadPoolExecutor for non-blocking behavior.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 7012,
                 timeout: float = 2.0, max_workers: int = 4):
        self.base_url = f"http://{host}:{port}"
        self.timeout = timeout
        self._session: Optional[aiohttp.ClientSession] = None
        self._executor = ThreadPoolExecutor(max_workers=max_workers)

    async def _get_session(self) -> "aiohttp.ClientSession":
        if _HAS_AIOHTTP and (self._session is None or self._session.closed):
            timeout = aiohttp.ClientTimeout(total=self.timeout)
            self._session = aiohttp.ClientSession(timeout=timeout)
        return self._session

    async def send_cold_start_hint(self, robot_id: str,
                                   hint: dict) -> bool:
        """Send cold start zone hint to FMS (non-blocking)."""
        payload = {
            "robot_id": robot_id,
            "event": "COLD_START_HINT",
            "timestamp": time.time(),
            "primary_zone": hint["primary_zone"],
            "confidence": hint["confidence"],
            "method": hint["method"],
            "candidate_zones": hint["candidate_zones"],
            "nearest_barcodes": hint.get("nearest_barcodes", []),
            "recovery_strategy": hint.get("recovery_strategy",
                                          "nearest_barcode"),
            "ode_time_ms": hint.get("ode_time_ms", 0),
        }
        url = f"{self.base_url}/api/robots/{robot_id}/zone_hint"
        return await self._post(url, payload)

    async def send_zone_update(self, robot_id: str, zone: str,
                               confidence: float,
                               method: str) -> bool:
        """Send continuous zone awareness update (non-blocking)."""
        payload = {
            "robot_id": robot_id,
            "event": "ZONE_UPDATE",
            "timestamp": time.time(),
            "zone": zone,
            "confidence": confidence,
            "method": method,
        }
        url = f"{self.base_url}/api/robots/{robot_id}/zone"
        return await self._post(url, payload)

    async def send_map_change_alert(self, zone: str,
                                    change_data: dict) -> bool:
        """Alert FMS of layout change (non-blocking)."""
        payload = {
            "event": "MAP_CHANGE_DETECTED",
            "timestamp": time.time(),
            "zone": zone,
            "change_type": change_data.get("change_type"),
            "confidence": change_data.get("confidence"),
        }
        url = f"{self.base_url}/api/events"
        return await self._post(url, payload)

    async def get_robot_state(self, robot_id: str) -> Optional[dict]:
        """Read robot state from FMS REST API (non-blocking)."""
        url = f"{self.base_url}/api/robots/{robot_id}/status"

        if _HAS_AIOHTTP:
            try:
                session = await self._get_session()
                async with session.get(url) as resp:
                    if resp.status == 200:
                        return await resp.json()
            except (aiohttp.ClientError, asyncio.TimeoutError):
                return None
        else:
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(
                self._executor, self._sync_get, url)

        return None

    async def _post(self, url: str, payload: dict) -> bool:
        """POST JSON to FMS (async)."""
        if _HAS_AIOHTTP:
            try:
                session = await self._get_session()
                async with session.post(url, json=payload) as resp:
                    return resp.status in (200, 201, 202)
            except (aiohttp.ClientError, asyncio.TimeoutError):
                return False
        else:
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(
                self._executor, self._sync_post, url, payload)

    def _sync_post(self, url: str, payload: dict) -> bool:
        """Synchronous POST fallback."""
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                url, data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.status in (200, 201, 202)
        except (urllib.error.URLError, TimeoutError):
            return False

    def _sync_get(self, url: str) -> Optional[dict]:
        """Synchronous GET fallback."""
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return json.loads(resp.read().decode())
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError):
            return None

    async def close(self) -> None:
        """Clean up resources."""
        if _HAS_AIOHTTP and self._session and not self._session.closed:
            await self._session.close()
        self._executor.shutdown(wait=False)


class AsyncFmsTcpAdapter:
    """
    Async TCP adapter using asyncio streams.
    Sends io-gita zone hints directly via TCP Protocol V1 format.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 65123,
                 timeout: float = 2.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None

    async def connect(self) -> bool:
        """Establish async TCP connection to FMS."""
        try:
            self._reader, self._writer = await asyncio.wait_for(
                asyncio.open_connection(self.host, self.port),
                timeout=self.timeout,
            )
            return True
        except (OSError, asyncio.TimeoutError):
            return False

    async def send_zone_hint_v1(self, robot_id: str, zone: str,
                                confidence: float,
                                candidate_zones: list) -> bool:
        """Send zone hint as Protocol V1 extension message (async)."""
        if not self._writer:
            if not await self.connect():
                return False

        msg = (
            f"IOGITA_HINT|{robot_id}|{zone}|{confidence:.3f}|"
            f"{json.dumps(candidate_zones)}\n"
        )
        try:
            self._writer.write(msg.encode("utf-8"))
            await self._writer.drain()
            return True
        except (OSError, ConnectionResetError):
            self._writer = None
            return False

    async def close(self) -> None:
        """Close TCP connection."""
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except OSError:
                pass
            self._writer = None


# =====================================================================
# DEMO
# =====================================================================

async def demo() -> None:
    """Demo async adapter (log-only — no real FMS)."""
    print("=" * 60)
    print("  Async FMS Adapter — Demo")
    print(f"  aiohttp available: {_HAS_AIOHTTP}")
    print("=" * 60)

    adapter = AsyncFmsRestAdapter()

    hint = {
        "primary_zone": "SHELF_3",
        "confidence": 0.90,
        "method": "GRAPH_FP_RANKED",
        "candidate_zones": ["SHELF_3", "MID_N", "CROSS_W"],
        "nearest_barcodes": [{"zone": "SHELF_3", "barcode_id": 55}],
        "ode_time_ms": 0.8,
    }

    print("\n  Sending cold start hint (async)...")
    result = await adapter.send_cold_start_hint("zippy10_3", hint)
    print(f"  Result: {'ACK' if result else 'FAILED (expected — no FMS running)'}")

    print("\n  Sending zone update (async)...")
    result = await adapter.send_zone_update("zippy10_3", "SHELF_3", 0.9,
                                            "GRAPH_FP_RANKED")
    print(f"  Result: {'ACK' if result else 'FAILED (expected — no FMS running)'}")

    await adapter.close()
    print("\n  Done.")


if __name__ == "__main__":
    asyncio.run(demo())
