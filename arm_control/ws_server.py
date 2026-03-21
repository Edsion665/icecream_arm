"""WebSocket 服务端：对主机提供状态上报与命令接收接口。"""

from __future__ import annotations

import asyncio
import json
import logging
from asyncio import Queue
from typing import Any, Dict, Set

import websockets
from websockets.server import WebSocketServerProtocol

from .config import CONFIG
from .state_store import StateStore

LOGGER = logging.getLogger(__name__)


async def _handle_client(
    ws: WebSocketServerProtocol,
    state_store: StateStore,
    command_queue: "Queue[Dict[str, Any]]",
) -> None:
    async for msg in ws:
        try:
            data = json.loads(msg)
        except json.JSONDecodeError:
            LOGGER.warning("收到非法 JSON：%s", msg)
            continue

        if data.get("type") == "command":
            await command_queue.put(data)
        else:
            LOGGER.debug("收到非 command 消息：%s", data)


async def _broadcast_state_task(
    clients: "Set[WebSocketServerProtocol]", state_store: StateStore
) -> None:
    interval = CONFIG.network.state_push_interval_sec
    while True:
        if clients:
            payload = {
                "type": "state",
                "data": state_store.to_payload(),
            }
            msg = json.dumps(payload)

            # websockets 新版中不再提供 .open 属性，这里直接发送，
            # 出现连接关闭等异常时忽略即可。
            send_tasks = []
            for c in list(clients):
                try:
                    send_tasks.append(c.send(msg))
                except Exception:
                    # 这里不抛出，让上层连接管理去清理
                    continue

            if send_tasks:
                await asyncio.gather(*send_tasks, return_exceptions=True)
        await asyncio.sleep(interval)


async def start_ws_server(
    state_store: StateStore,
    command_queue: "Queue[Dict[str, Any]]",
) -> None:
    """启动 WebSocket 服务端。"""

    clients: Set[WebSocketServerProtocol] = set()

    async def handler(ws: WebSocketServerProtocol) -> None:
        clients.add(ws)
        LOGGER.info("WebSocket 客户端连接：%s", ws.remote_address)
        try:
            await _handle_client(ws, state_store, command_queue)
        finally:
            clients.discard(ws)
            LOGGER.info("WebSocket 客户端断开：%s", ws.remote_address)

    host = CONFIG.network.host
    port = CONFIG.network.port
    LOGGER.info("启动 WebSocket 服务：ws://%s:%d", host, port)

    async with websockets.serve(handler, host, port):
        await _broadcast_state_task(clients, state_store)

