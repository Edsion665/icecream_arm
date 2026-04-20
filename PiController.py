"""树莓派控制层：motor + servoMotor（适配 PC<->RPi UDP V2）。"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import socket
import struct
import time
from typing import Literal, Sequence

from .calculator import JointFrame

# 协议文档：doc/bridge2pi.md
# 固定 92B: =Idddddddddd  (seq, ts, p_rel_deg[5], omega_rad_s[5])
_FMT_V2 = "=Id" + "d" * 10
PACKET_V2_SIZE = struct.calcsize(_FMT_V2)


class RPiUDPStreamer:
    def __init__(
        self,
        rpi_ip: str,
        port: int = 9870,
        fmt: Literal["v1", "v2"] = "v2",
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.addr = (rpi_ip, port)
        self.fmt = fmt
        self._seq = 0
        # 兼容参数名：当前 v1/v2 都按文档 V2 的 92B 帧发送。
        self._ps = PACKET_V2_SIZE
        print(
            f"[RPiUDPStreamer] udp://{rpi_ip}:{port} | 格式={fmt}->protocol_v2_92B | "
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
        p = [float(p_rel_deg[i]) for i in range(5)]
        w = [float(omega_rad_s[i]) for i in range(5)]
        # 注意：协议 V2 不含 servo 字段；servo 通道由独立路径处理。
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
    _last_servo: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

    def send_frame(self, frame: JointFrame) -> None:
        p5 = np.zeros(5, dtype=float)
        w5 = np.zeros(5, dtype=float)
        p5[:4] = frame.arm_rel_deg[:4]
        w5[:4] = frame.arm_omega_rad_s[:4]
        p5[4] = float(getattr(frame, "joint5_rel_deg", 0.0))
        if frame.servo_deg is not None and len(frame.servo_deg) >= 2:
            self._last_servo = np.array([float(frame.servo_deg[0]), float(frame.servo_deg[1])], dtype=float)
        self.streamer.send(p5, w5, servo_deg=self._last_servo)

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
            mode="servo_only",
            timestamp=0.0,
        )
        self._adapter.send_frame(dummy)

