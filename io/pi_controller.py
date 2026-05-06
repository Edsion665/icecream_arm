"""树莓派控制层：motor + servoMotor（适配 PC<->RPi UDP V2.1）。"""

from __future__ import annotations

import socket
import struct
import time
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ..calculator import JointFrame
from ..config import CONFIG
from ..exceptions import UDPTransportError

# 协议文档：PC_RPI_UDP_PROTOCOL.md
# 固定 108B: =Id + d*12  (seq, ts, p_rel_deg[6], omega_rad_s[6])
_FMT_V2 = "=Id" + "d" * 12
PACKET_V2_SIZE = struct.calcsize(_FMT_V2)


class RPiUDPStreamer:
    """向树莓派发送 V2.1 固定长度 UDP 帧。"""

    def __init__(
        self,
        rpi_ip: str,
        port: int = CONFIG.default_udp_port,
        *,
        strict_udp: bool = True,
    ) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.addr = (rpi_ip, port)
        self._seq = 0
        self._strict_udp = strict_udp
        self._ps = PACKET_V2_SIZE
        approx_kbps = self._ps * CONFIG.control_hz / 1024
        print(
            f"[RPiUDPStreamer] udp://{rpi_ip}:{port} | protocol_v2.1_108B | "
            f"{self._ps} B/帧 @ {CONFIG.control_hz:g} Hz ≈ {approx_kbps:.1f} KB/s"
        )

    def send(
        self,
        p_rel_deg: Sequence[float],
        omega_rad_s: Sequence[float],
        servo_deg: Sequence[float] | None = None,
    ) -> None:
        """打包并 ``sendto``；``strict_udp`` 为真时失败抛出 ``UDPTransportError``。"""
        del servo_deg  # 协议 V2.1 帧内已含 6 维 p/w；保留参数兼容旧调用。
        self._seq += 1
        ts = time.monotonic()
        p = [float(p_rel_deg[i]) for i in range(6)]
        w = [float(omega_rad_s[i]) for i in range(6)]
        payload = struct.pack(_FMT_V2, self._seq, ts, *p, *w)
        try:
            self.sock.sendto(payload, self.addr)
        except OSError as exc:
            if self._strict_udp:
                raise UDPTransportError(f"UDP sendto {self.addr} failed: {exc}") from exc

    def close(self) -> None:
        self.sock.close()
        print("[RPiUDPStreamer] 已关闭。")


@dataclass
class RpiProtocolAdapter:
    """将 ``JointFrame`` 映射为 UDP 发送。"""

    streamer: RPiUDPStreamer

    def send_frame(self, frame: JointFrame) -> None:
        p6 = np.zeros(6, dtype=float)
        w6 = np.zeros(6, dtype=float)
        p6[:4] = frame.arm_rel_deg[:4]
        w6[:4] = frame.arm_omega_rad_s[:4]
        p6[4] = float(getattr(frame, "wrist_rel_deg", 0.0))
        w6[4] = float(getattr(frame, "wrist_omega_rad_s", 0.0))
        p6[5] = float(getattr(frame, "grip_state", 0.0))
        w6[5] = 0.0
        self.streamer.send(p6, w6)

    def close(self) -> None:
        self.streamer.close()


class motor:
    """主臂 + 爪通道 UDP 下发。"""

    def __init__(self, adapter: RpiProtocolAdapter) -> None:
        self._adapter = adapter

    def send(self, frame: JointFrame) -> None:
        self._adapter.send_frame(frame)


class servoMotor:
    """仅爪/腕通道时使用（构造最小 ``JointFrame``）。"""

    def __init__(self, adapter: RpiProtocolAdapter) -> None:
        self._adapter = adapter

    def send(self, servo_deg: Sequence[float]) -> None:
        sd = np.asarray(servo_deg, dtype=float).ravel()
        dummy = JointFrame(
            arm_rel_deg=np.zeros(4, dtype=float),
            joint5_rel_deg=0.0,
            arm_omega_rad_s=np.zeros(4, dtype=float),
            servo_deg=np.array(servo_deg, dtype=float),
            wrist_rel_deg=float(sd[0]) if len(sd) >= 1 else 0.0,
            wrist_omega_rad_s=0.0,
            grip_state=1.0 if (len(sd) >= 2 and float(sd[1]) >= 0.5) else 0.0,
            mode="servo_only",
            timestamp=0.0,
        )
        self._adapter.send_frame(dummy)
