"""后台线程接收 PC 仿真 UDP 关节流，供 tau_ff 环路与 MIT 下行同频使用。"""

from __future__ import annotations

import logging
import math
import os
import socket
import struct
import threading
import time
from typing import Optional, Sequence, Tuple

from .rpi_udp_packet import PACKET_SIZE, unpack_packet

logger = logging.getLogger(__name__)

# 关节 1..4 → 电机 M1..M4：1、3、4 位置与速度与 PC 约定取反；2 不变；关节 5 仅上行包占位，不参与四电机指令。
_SIGN_J1_J4: Tuple[float, float, float, float] = (-1.0, 1.0, -1.0, -1.0)


def rpi_udp_stream_enabled() -> bool:
    v = os.environ.get("ARM_CONTROL_RPI_UDP", "").strip().lower()
    return v in ("1", "true", "yes", "on")


def _udp_port() -> int:
    try:
        return int(os.environ.get("ARM_CONTROL_RPI_UDP_PORT", "9870"))
    except ValueError:
        return 9870


def _stale_sec() -> float:
    try:
        return max(0.05, float(os.environ.get("ARM_CONTROL_RPI_UDP_STALE_SEC", "0.35")))
    except ValueError:
        return 0.35


def pc_deg_omega_to_motor_p_v_rad(
    p_rel_deg_5: Sequence[float],
    omega_rad_s_5: Sequence[float],
    calibration_rad_4: Sequence[float],
) -> tuple[list[float], list[float]]:
    """PC 相对角(°) + ω(rad/s) → 四电机 MIT 用绝对 p(rad)、v(rad/s)。关节 5 忽略。"""
    if len(calibration_rad_4) != 4:
        raise ValueError("calibration_rad_4 须为 4 个浮点数")
    p_cmd: list[float] = []
    v_cmd: list[float] = []
    for i in range(4):
        s = _SIGN_J1_J4[i]
        p_cmd.append(
            float(calibration_rad_4[i]) + s * math.radians(float(p_rel_deg_5[i]))
        )
        v_cmd.append(s * float(omega_rad_s_5[i]))
    return p_cmd, v_cmd


class RpiUdpJointSource:
    """非阻塞：接收线程写最新包；tau_ff 在同周期读取。"""

    def __init__(self, port: int, stale_sec: float) -> None:
        self._port = port
        self._stale_sec = stale_sec
        self._lock = threading.Lock()
        self._seq: Optional[int] = None
        self._p_deg: list[float] = [0.0] * 5
        self._omega: list[float] = [0.0] * 5
        self._recv_mono: float = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._drops = 0

    @classmethod
    def from_env(cls) -> Optional["RpiUdpJointSource"]:
        if not rpi_udp_stream_enabled():
            return None
        return cls(port=_udp_port(), stale_sec=_stale_sec())

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        logger.info(
            "RPI UDP 关节流已启用：端口 %d，超时 %.2fs（ARM_CONTROL_RPI_UDP_PORT / STALE）",
            self._port,
            self._stale_sec,
        )

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _recv_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self._port))
        except OSError as exc:
            logger.error("RPI UDP bind 0.0.0.0:%d 失败: %s", self._port, exc)
            return
        sock.settimeout(0.5)
        last_seq: Optional[int] = None
        while not self._stop.is_set():
            try:
                data, _ = sock.recvfrom(256)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue
            if len(data) != PACKET_SIZE:
                continue
            try:
                pkt = unpack_packet(data)
            except struct.error:
                continue
            seq = int(pkt["seq"])
            with self._lock:
                if last_seq is not None and seq != last_seq + 1:
                    self._drops += seq - last_seq - 1
                last_seq = seq
                self._seq = seq
                self._p_deg = [float(x) for x in pkt["p_rel_deg"]]
                self._omega = [float(x) for x in pkt["omega_rad_s"]]
                self._recv_mono = time.monotonic()
        sock.close()

    def snapshot_raw(self) -> Optional[tuple[list[float], list[float], float]]:
        """若包未过期则返回 (p_rel_deg×5, omega×5, recv_mono)；否则 None。"""
        now = time.monotonic()
        with self._lock:
            if self._seq is None:
                return None
            if now - self._recv_mono > self._stale_sec:
                return None
            return (
                list(self._p_deg),
                list(self._omega),
                self._recv_mono,
            )

    def get_motor_p_v_rad(
        self, calibration_rad_4: Sequence[float]
    ) -> Optional[tuple[list[float], list[float]]]:
        snap = self.snapshot_raw()
        if snap is None:
            return None
        p_deg, omega, _ = snap
        return pc_deg_omega_to_motor_p_v_rad(p_deg, omega, calibration_rad_4)
