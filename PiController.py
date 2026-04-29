"""树莓派控制层：motor + servoMotor（适配 PC<->RPi UDP V2.1）。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import socket
import struct
import time
from typing import Sequence

from .calculator import JointFrame

# 协议文档：PC_RPI_UDP_PROTOCOL.md
# 固定 108B: =Id + d*12  (seq, ts, p_rel_deg[6], omega_rad_s[6])
_FMT_V2 = "=Id" + "d" * 12
PACKET_V2_SIZE = struct.calcsize(_FMT_V2)


class RPiUDPStreamer:
    def __init__(
        self,
        rpi_ip: str,
        port: int = 9870,
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.addr = (rpi_ip, port)
        self._seq = 0
        # 固定使用协议 V2.1 的 108B 帧。
        self._ps = PACKET_V2_SIZE
        print(
            f"[RPiUDPStreamer] udp://{rpi_ip}:{port} | protocol_v2.1_108B | "
            f"{self._ps} B/帧 @ 25Hz ≈ {self._ps * 25 / 1024:.1f} KB/s"
        )

    def send(
        self,
        p_rel_deg: Sequence[float],
        omega_rad_s: Sequence[float],
        servo_deg: Sequence[float] | None = None,
    ) -> None:
        self._seq += 1
        ts = time.monotonic()
        p = [float(p_rel_deg[i]) for i in range(6)]
        w = [float(omega_rad_s[i]) for i in range(6)]
        payload = struct.pack(_FMT_V2, self._seq, ts, *p, *w)
        try:
            self.sock.sendto(payload, self.addr)
        except OSError:
            pass

    def close(self) -> None:
        self.sock.close()
        print("[RPiUDPStreamer] 已关闭。")


@dataclass
class RpiProtocolAdapter:
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
    def __init__(self, adapter: RpiProtocolAdapter):
        self._adapter = adapter

    def send(self, frame: JointFrame) -> None:
        self._adapter.send_frame(frame)


class servoMotor:
    def __init__(self, adapter: RpiProtocolAdapter):
        self._adapter = adapter

    def send(self, servo_deg) -> None:
        dummy = JointFrame(
            arm_rel_deg=np.zeros(4, dtype=float),
            joint5_rel_deg=0.0,
            arm_omega_rad_s=np.zeros(4, dtype=float),
            servo_deg=np.array(servo_deg, dtype=float),
            wrist_rel_deg=float(servo_deg[0]) if len(servo_deg) >= 1 else 0.0,
            wrist_omega_rad_s=0.0,
            grip_state=1.0 if (len(servo_deg) >= 2 and float(servo_deg[1]) >= 0.5) else 0.0,
            mode="servo_only",
            timestamp=0.0,
        )
        self._adapter.send_frame(dummy)

