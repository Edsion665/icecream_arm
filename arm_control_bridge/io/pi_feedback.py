"""树莓派 WebSocket 回传订阅客户端（pi2camera v1 协议）。

订阅树莓派 WebSocket 状态（端口见 ``CONFIG.rpi_ws_port``），解析 ``type=state``，提取
``feedback.fb_arm_rad``（优先）或 ``feedback.mit_arm_rad``，
供 ``is_reached()`` 使用实机关节角。

无 Pi 连接时 ``get_fb_arm_rad()`` 返回 ``None``，控制循环回退到 ``state.q_cmd``。
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import numpy as np

from ..config import CONFIG


class PiFeedbackClient:
    """后台线程订阅树莓派 WebSocket 状态广播。"""

    def __init__(self, rpi_ip: str, port: int = CONFIG.rpi_ws_port) -> None:
        self._uri = f"ws://{rpi_ip}:{port}"
        self._lock = threading.Lock()
        self._fb_arm_rad: Optional[np.ndarray] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_fb_arm_rad(self) -> Optional[np.ndarray]:
        """返回最新实机关节角（弧度，4 轴），无回传时返回 ``None``。"""
        with self._lock:
            return self._fb_arm_rad.copy() if self._fb_arm_rad is not None else None

    def close(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        try:
            import websocket
        except ImportError:
            print("[PiFeedbackClient] websocket-client 未安装，Pi 回传不可用")
            return

        while not self._stop.is_set():
            ws = None
            try:
                ws = websocket.create_connection(self._uri, timeout=5)
                print(f"[PiFeedbackClient] 已连接 {self._uri}")
                while not self._stop.is_set():
                    raw = ws.recv()
                    self._handle(raw)
            except websocket.WebSocketConnectionClosedException as exc:
                print(f"[PiFeedbackClient] 连接关闭: {exc}")
            except (OSError, ConnectionError) as exc:
                print(f"[PiFeedbackClient] 连接/收发错误 ({type(exc).__name__}): {exc}")
            except Exception as exc:
                print(f"[PiFeedbackClient] 未预期错误 ({type(exc).__name__}): {exc}")
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except OSError as exc:
                        print(f"[PiFeedbackClient] ws.close: {type(exc).__name__}: {exc}")
            if not self._stop.is_set():
                print(
                    f"[PiFeedbackClient] 断开，{CONFIG.pi_feedback_reconnect_interval_s:g}s 后重连"
                )
            self._stop.wait(CONFIG.pi_feedback_reconnect_interval_s)

    def _handle(self, raw: str | bytes) -> None:
        if isinstance(raw, bytes):
            try:
                raw = raw.decode("utf-8")
            except UnicodeDecodeError:
                return
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
            return
