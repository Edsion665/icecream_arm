#!/usr/bin/env python3
"""
PiListener 测试程序：订阅 pi2camera WebSocket 广播并打印内容。

示例：
  python3 -m arm_control_bridge.pi_listener_test --host 192.168.43.108
  python3 -m arm_control_bridge.pi_listener_test --uri ws://192.168.43.108:8765 --raw
"""

from __future__ import annotations

import argparse
import asyncio
import json
from datetime import datetime
from typing import Any

import websockets


class PiListener:
    """连接 Pi 广播 WS，打印状态消息。"""

    def __init__(self, uri: str, *, raw: bool = False, retry_sec: float = 1.0):
        self._uri = uri
        self._raw = raw
        self._retry_sec = max(float(retry_sec), 0.1)

    @staticmethod
    def _now() -> str:
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _print_summary(self, msg: dict[str, Any]) -> None:
        mtype = msg.get("type")
        data = msg.get("data") or {}
        udp = data.get("udp") or {}
        fb = data.get("feedback") or {}
        runtime = data.get("runtime") or {}
        seq = udp.get("seq")
        age_ms = udp.get("age_ms")
        p_rel = udp.get("p_rel_deg")
        omega = udp.get("omega_rad_s")
        safety = runtime.get("safety_reason")
        src = runtime.get("control_source")
        hmat = fb.get("link5_hmat")
        print(
            f"[{self._now()}] type={mtype} seq={seq} age_ms={age_ms} "
            f"src={src} safety={safety} "
            f"p_rel_deg={p_rel} omega_rad_s={omega} "
            f"link5_hmat_null={hmat is None}",
            flush=True,
        )

    async def run_forever(self) -> None:
        print(f"[{self._now()}] PiListener connecting: {self._uri}", flush=True)
        while True:
            try:
                async with websockets.connect(self._uri, ping_interval=20, ping_timeout=20) as ws:
                    print(f"[{self._now()}] PiListener connected", flush=True)
                    async for raw in ws:
                        if self._raw:
                            print(f"[{self._now()}] RAW {raw}", flush=True)
                            continue
                        try:
                            msg = json.loads(raw)
                        except json.JSONDecodeError:
                            print(f"[{self._now()}] invalid_json: {raw}", flush=True)
                            continue
                        self._print_summary(msg)
            except asyncio.CancelledError:
                raise
            except Exception as ex:  # noqa: BLE001
                print(f"[{self._now()}] disconnected: {ex}; retry in {self._retry_sec:.1f}s", flush=True)
                await asyncio.sleep(self._retry_sec)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="PiListener 测试：订阅并打印 pi2camera 广播")
    p.add_argument("--uri", default=None, help="完整 WS 地址，例如 ws://192.168.43.108:8765")
    p.add_argument("--host", default="127.0.0.1", help="Pi 主机地址（当未传 --uri 时使用）")
    p.add_argument("--port", type=int, default=8765, help="Pi WS 端口（默认 8765）")
    p.add_argument("--raw", action="store_true", help="打印原始 JSON 文本")
    p.add_argument("--retry-sec", type=float, default=1.0, help="断线重连间隔（秒）")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    uri = args.uri or f"ws://{args.host}:{args.port}"
    listener = PiListener(uri, raw=args.raw, retry_sec=args.retry_sec)
    try:
        asyncio.run(listener.run_forever())
    except KeyboardInterrupt:
        print(f"[{listener._now()}] PiListener stopped", flush=True)


if __name__ == "__main__":
    main()

