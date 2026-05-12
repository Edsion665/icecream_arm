"""UDP broadcast to camera (``pi2camera`` v2, port 9982 by default)."""

from __future__ import annotations

import asyncio
import json
import logging
import socket
import time
from typing import TYPE_CHECKING

from .payload import build_camera_state_payload

if TYPE_CHECKING:
    from ...config import PiCameraUdpConfig
    from ...state_store import StateStore

LOGGER = logging.getLogger(__name__)


async def run_pi2camera_udp_broadcast(store: "StateStore", cfg: "PiCameraUdpConfig") -> None:
    """Send UTF-8 JSON ``camera_state`` packets at ``cfg.hz`` (best-effort)."""
    dt = 1.0 / float(cfg.hz)
    seq = 0
    sock: socket.socket | None = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if cfg.broadcast:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        addr = (cfg.host, int(cfg.port))
        LOGGER.info("pi2camera UDP broadcast -> %s:%d at %.1f Hz", cfg.host, cfg.port, cfg.hz)
        while True:
            body = build_camera_state_payload(store)
            if body is not None:
                seq += 1
                msg = {**body, "seq": seq, "ts": time.time()}
                data = json.dumps(msg, separators=(",", ":")).encode("utf-8")
                try:
                    sock.sendto(data, addr)
                except OSError as exc:
                    LOGGER.warning("pi2camera UDP send failed: %s", exc)
            await asyncio.sleep(dt)
    finally:
        if sock is not None:
            sock.close()

