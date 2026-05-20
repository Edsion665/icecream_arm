"""Pi → PC 关节反馈（pi2camera v2 UDP，见 ``head/doc/pi2camera.md``）。

监听 Pi 下行 ``camera_state`` JSON（默认 UDP ``9982``），用 ``motor_rad[4]`` 与
bridge 下发的 ``joint_rel_deg_4``（经 ``bridge2pi`` 电机映射反算为相对角）比较是否到位。

不使用 WebSocket ``fb_arm_rad``。
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import numpy as np

from ..config import CONFIG

# bridge2pi §4.1：motor_i = calib[i] + sign_i * rad(p_rel_deg[i])
_MOTOR_SIGN = np.array([-1.0, 1.0, -1.0, -1.0], dtype=float)


def motor_rad_to_rel_deg(motor_rad: np.ndarray, q_calib_rad: np.ndarray) -> np.ndarray:
    """电机绝对角(rad) → 相对标定角(deg)，与 ``p_rel_deg[0:4]`` 同语义。"""
    m = np.asarray(motor_rad, dtype=float).ravel()[:4]
    c = np.asarray(q_calib_rad, dtype=float).ravel()[:4]
    return np.rad2deg(_MOTOR_SIGN * (m - c))


@dataclass(frozen=True)
class PiFeedbackSnapshot:
    """最近一次 pi2camera v2 ``camera_state`` UDP 包。"""

    udp_listening: bool
    motor_rad: Optional[np.ndarray]
    packet_age_ms: Optional[float]
    pi_seq: Optional[int]
    updated_monotonic: float


class PiFeedbackClient:
    """后台线程：UDP 接收 Pi ``camera_state``，解析 ``motor_rad``。"""

    def __init__(
        self,
        q_calib_rad: np.ndarray,
        *,
        listen_host: str | None = None,
        listen_port: int | None = None,
    ) -> None:
        self._q_calib_rad = np.asarray(q_calib_rad, dtype=float).ravel()[:4].copy()
        self._listen_host = listen_host or CONFIG.camera_udp_listen_host
        self._listen_port = int(listen_port or CONFIG.camera_udp_listen_port)
        self._lock = threading.Lock()
        self._listening = False
        self._motor_rad: Optional[np.ndarray] = None
        self._packet_age_ms: Optional[float] = None
        self._pi_seq: Optional[int] = None
        self._updated_monotonic: float = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="PiCameraUdpFeedback")
        self._thread.start()

    def get_snapshot(self) -> PiFeedbackSnapshot:
        with self._lock:
            age = None
            if self._updated_monotonic > 0:
                age = (time.monotonic() - self._updated_monotonic) * 1000.0
            return PiFeedbackSnapshot(
                udp_listening=self._listening,
                motor_rad=self._motor_rad.copy() if self._motor_rad is not None else None,
                packet_age_ms=age,
                pi_seq=self._pi_seq,
                updated_monotonic=self._updated_monotonic,
            )

    def get_fb_arm_rad(self) -> Optional[np.ndarray]:
        """供 FK 使用：返回最新 ``motor_rad``（rad，4 轴）。"""
        with self._lock:
            return self._motor_rad.copy() if self._motor_rad is not None else None

    def compute_arm_reached(
        self,
        bridge_target_rel_deg: np.ndarray,
        *,
        tol_deg: float = CONFIG.reached_joints_tol_deg,
        require_feedback: bool = CONFIG.require_pi_feedback_for_reached,
        max_packet_age_ms: Optional[float] = CONFIG.max_camera_packet_age_ms_for_reached,
    ) -> tuple[bool, float, dict[str, Any]]:
        """反馈 ``motor_rad`` 反算的相对角 vs bridge 设定 ``joint_rel_deg_4``。"""
        snap = self.get_snapshot()
        meta: dict[str, Any] = {
            "feedback_available": False,
            "reach_source": "pi2camera_udp",
            "udp_listening": snap.udp_listening,
        }
        target_rel = np.asarray(bridge_target_rel_deg, dtype=float).ravel()[:4]
        meta["target_source"] = "bridge.joint_rel_deg_4"
        meta["target_rel_deg"] = [round(float(x), 3) for x in target_rel]

        if not snap.udp_listening:
            meta["reach_reason"] = "udp_not_listening"
            return False, float("inf"), meta

        if require_feedback and snap.motor_rad is None:
            meta["reach_reason"] = "no_motor_rad"
            return False, float("inf"), meta

        if snap.motor_rad is None:
            meta["reach_reason"] = "no_motor_rad"
            return False, float("inf"), meta

        if max_packet_age_ms is not None and snap.packet_age_ms is not None:
            if float(snap.packet_age_ms) > float(max_packet_age_ms):
                meta["reach_reason"] = "packet_stale"
                meta["packet_age_ms"] = round(float(snap.packet_age_ms), 1)
                return False, float("inf"), meta

        motor = snap.motor_rad[:4]
        actual_rel = motor_rad_to_rel_deg(motor, self._q_calib_rad)
        err_deg = np.abs(actual_rel - target_rel)
        error = float(np.linalg.norm(err_deg))
        reached = bool(np.all(err_deg < tol_deg))

        meta.update(
            {
                "feedback_available": True,
                "motor_rad": [round(float(x), 4) for x in motor],
                "actual_rel_deg": [round(float(x), 3) for x in actual_rel],
                "per_axis_err_deg": [round(float(x), 3) for x in err_deg],
                "reached_tol_deg": float(tol_deg),
                "packet_age_ms": round(float(snap.packet_age_ms), 1) if snap.packet_age_ms is not None else None,
                "pi_seq": snap.pi_seq,
                "reach_reason": "ok" if reached else "over_tol",
            }
        )
        return reached, error, meta

    def close(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self._listen_host, self._listen_port))
            sock.settimeout(0.5)
            with self._lock:
                self._listening = True
            print(
                f"[PiFeedbackClient] pi2camera v2 UDP 监听 {self._listen_host}:{self._listen_port} "
                f"（camera_state.motor_rad，不用 WebSocket）"
            )
            while not self._stop.is_set():
                try:
                    data, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self._stop.is_set():
                        print(f"[PiFeedbackClient] UDP recv 错误: {type(exc).__name__}: {exc}")
                    continue
                self._handle_packet(data)
        except OSError as exc:
            print(
                f"[PiFeedbackClient] 无法绑定 UDP {self._listen_host}:{self._listen_port}: "
                f"{type(exc).__name__}: {exc}"
            )
        finally:
            with self._lock:
                self._listening = False
            try:
                sock.close()
            except OSError:
                pass

    def _handle_packet(self, data: bytes) -> None:
        try:
            raw = data.decode("utf-8")
            msg = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError):
            return
        if msg.get("type") != "camera_state":
            return
        arm = msg.get("motor_rad")
        if arm is None:
            return
        try:
            arr = np.array(arm, dtype=float).ravel()[:4]
            if len(arr) < 4:
                return
        except (TypeError, ValueError):
            return
        seq = msg.get("seq")
        with self._lock:
            self._motor_rad = arr
            self._updated_monotonic = time.monotonic()
            try:
                self._pi_seq = int(seq) if seq is not None else None
            except (TypeError, ValueError):
                self._pi_seq = None
