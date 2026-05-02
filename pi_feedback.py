"""树莓派 WebSocket 回传订阅客户端（pi2camera v1 协议）。

订阅 ws://rpi_ip:8765，解析 type=state 消息，提取
feedback.fb_arm_rad（优先）或 feedback.mit_arm_rad，
供 is_reached() 使用实机关节角做到位判定。

无 Pi 连接时静默降级：get_fb_arm_rad() 返回 None，
控制循环自动回退到 state.q_cmd。
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import numpy as np


class PiFeedbackClient:
    """后台线程订阅树莓派 WebSocket 状态广播。"""

    _RECONNECT_INTERVAL_S = 3.0

    def __init__(self, rpi_ip: str, port: int = 8765) -> None:
        self._uri = f"ws://{rpi_ip}:{port}"
        self._lock = threading.Lock()
        self._fb_arm_rad: Optional[np.ndarray] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_fb_arm_rad(self) -> Optional[np.ndarray]:
        """返回最新实机关节角（弧度，4轴），无回传时返回 None。"""
        with self._lock:
            return self._fb_arm_rad.copy() if self._fb_arm_rad is not None else None

    def close(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------------

    def _run(self) -> None:
        try:
            import websocket  # websocket-client
        except ImportError:
            print("[PiFeedbackClient] websocket-client 未安装，Pi 回传不可用")
            return

        while not self._stop.is_set():
            try:
                ws = websocket.create_connection(self._uri, timeout=5)
                print(f"[PiFeedbackClient] 已连接 {self._uri}")
                try:
                    while not self._stop.is_set():
                        raw = ws.recv()
                        self._handle(raw)
                except Exception:
                    pass
                finally:
                    try:
                        ws.close()
                    except Exception:
                        pass
                print(f"[PiFeedbackClient] 断开，{self._RECONNECT_INTERVAL_S}s 后重连")
            except Exception:
                pass
            self._stop.wait(self._RECONNECT_INTERVAL_S)

    def _handle(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return
        if msg.get("type") != "state":
            return
        fb = (msg.get("data") or {}).get("feedback") or {}
        arm_rad = fb.get("fb_arm_rad") or fb.get("mit_arm_rad")
        if arm_rad is None:
            return
        try:
            arr = np.array(arm_rad, dtype=float).ravel()[:4]
            if len(arr) < 4:
                return
            with self._lock:
                self._fb_arm_rad = arr
        except (TypeError, ValueError):
            pass
