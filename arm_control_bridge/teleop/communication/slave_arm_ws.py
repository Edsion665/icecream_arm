"""从臂 WebSocket 状态订阅客户端。

与主臂 ``PiFeedbackClient`` 对齐：后台线程连接从臂树莓派 WS 8765，
解析 ``type=state`` 帧，提取 ``feedback.fb_arm_rad``（优先）或
``feedback.mit_arm_rad``，以弧度返回四轴关节角。

同时暴露 ``calibration_rad`` 供零位对齐使用。
"""

from __future__ import annotations

import json
import math
import threading
from typing import Optional

import numpy as np


class SlaveArmWsClient:
    """后台线程订阅从臂树莓派 WebSocket 状态广播。"""

    _RECONNECT_S = 3.0

    def __init__(self, rpi_ip: str, port: int = 8765) -> None:
        self._uri = f"ws://{rpi_ip}:{port}"
        self._lock = threading.Lock()
        self._fb_arm_rad: Optional[np.ndarray] = None
        self._calibration_rad: Optional[np.ndarray] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="SlaveArmWs")
        self._thread.start()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_fb_arm_rad(self) -> Optional[np.ndarray]:
        """最新实机关节角（弧度，4 轴），无回传时返回 None。"""
        with self._lock:
            return self._fb_arm_rad.copy() if self._fb_arm_rad is not None else None

    def get_rel_arm_rad(self) -> Optional[np.ndarray]:
        """相对标定零位的关节角（弧度，4 轴），无回传或无标定时返回 None。

        计算：fb_arm_rad - calibration_rad，与 test_sim.py fb_to_q() 语义一致。
        """
        with self._lock:
            if self._fb_arm_rad is None or self._calibration_rad is None:
                return None
            return self._fb_arm_rad - self._calibration_rad

    def get_calibration_rad(self) -> Optional[np.ndarray]:
        """从臂标定零位（弧度，4 轴），未收到时返回 None。"""
        with self._lock:
            return self._calibration_rad.copy() if self._calibration_rad is not None else None

    def close(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run(self) -> None:
        try:
            import websocket
        except ImportError:
            print("[SlaveArmWsClient] websocket-client 未安装，从臂回传不可用")
            return

        while not self._stop.is_set():
            ws = None
            try:
                ws = websocket.create_connection(self._uri, timeout=5)
                print(f"[SlaveArmWsClient] 已连接 {self._uri}")
                while not self._stop.is_set():
                    raw = ws.recv()
                    self._handle(raw)
            except websocket.WebSocketConnectionClosedException as exc:
                print(f"[SlaveArmWsClient] 连接关闭: {exc}")
            except (OSError, ConnectionError) as exc:
                print(f"[SlaveArmWsClient] 连接/收发错误 ({type(exc).__name__}): {exc}")
            except Exception as exc:
                print(f"[SlaveArmWsClient] 未预期错误 ({type(exc).__name__}): {exc}")
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except OSError:
                        pass
            if not self._stop.is_set():
                print(f"[SlaveArmWsClient] 断开，{self._RECONNECT_S:g}s 后重连")
            self._stop.wait(self._RECONNECT_S)

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
        data = msg.get("data") or {}

        # 关节角：fb_arm_rad 优先，回退 mit_arm_rad（与主臂 PiFeedbackClient 一致）
        fb = data.get("feedback") or {}
        arm_rad = fb.get("fb_arm_rad") or fb.get("mit_arm_rad")
        if arm_rad is not None:
            try:
                arr = np.array(arm_rad, dtype=float).ravel()[:4]
                if len(arr) == 4:
                    with self._lock:
                        self._fb_arm_rad = arr
            except (TypeError, ValueError):
                pass

        # 标定零位（弧度）
        cal = data.get("calibration_rad")
        if cal is not None:
            try:
                arr = np.array(cal, dtype=float).ravel()[:4]
                if len(arr) == 4:
                    with self._lock:
                        self._calibration_rad = arr
            except (TypeError, ValueError):
                pass
