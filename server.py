"""WebSocket state broadcaster and servo command sink."""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Set

import websockets
from websockets.server import WebSocketServerProtocol

from .config import ServerConfig
from .domain.command_handler import StateCommandHandler
from .infra.state.presenter import build_state_payload
from .state_store import StateStore

LOGGER = logging.getLogger(__name__)


class StateServer:
    """Broadcast state snapshots and accept servo commands (store only)."""

    def __init__(self, cfg: ServerConfig, state_store: StateStore) -> None:
        self._cfg = cfg
        self._store = state_store
        self._command_handler = StateCommandHandler(state_store)

    async def run(self) -> None:
        clients: Set[WebSocketServerProtocol] = set()

        async def handler(ws: WebSocketServerProtocol) -> None:
            clients.add(ws)
            LOGGER.info("ws client connected: %s", ws.remote_address)
            try:
                async for raw in ws:
                    self._handle_message(raw)
            finally:
                clients.discard(ws)
                LOGGER.info("ws client disconnected: %s", ws.remote_address)

        async with websockets.serve(handler, self._cfg.host, self._cfg.port):
            LOGGER.info("ws server listening at ws://%s:%d", self._cfg.host, self._cfg.port)
            while True:
                payload = json.dumps({"type": "state", "data": build_state_payload(self._store)})
                if clients:
                    disconnected: Set[WebSocketServerProtocol] = set()
                    for client in list(clients):
                        try:
                            await client.send(payload)
                        except Exception:  # noqa: BLE001
                            disconnected.add(client)
                    for client in disconnected:
                        clients.discard(client)
                await asyncio.sleep(self._cfg.state_push_interval_sec)

    def _handle_message(self, raw: str) -> None:
        self._command_handler.handle_raw(raw)

