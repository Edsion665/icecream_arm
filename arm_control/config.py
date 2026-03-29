"""全局配置：串口、网络、相机等参数集中管理。"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class TauFfRuntimeConfig:
    """力矩前馈模式可调参数（可在运行时改 `calibration_rad` 等）。"""

    # 四轴标定零位（弧度），与 STM32 上报 FB 弧度同含义；默认全 0
    calibration_rad: list[float] = field(default_factory=lambda: [1.57416, 1.42462,2.42676,0.47208])
    # TAU: 下发频率（Hz）；过高可能占满 CPU / 串口带宽
    send_hz: float = 20.0
    # Pinocchio 算出的四轴重力前馈 (Nm) 乘以该系数再下发 TAU:（1.0 为不缩放）
    gain: float = 1.0


def _env_mode() -> str:
    v = os.environ.get("ARM_CONTROL_MODE", "tau_ff").strip().lower()
    return v if v in ("data", "tau_ff") else "tau_ff"


# 控制模式：data = 仅 WebSocket set_joint 发 DATA；tau_ff = 握手后按 FB 算重力并发 TAU
CONTROL_MODE: str = _env_mode()

TAU_FF = TauFfRuntimeConfig()
try:
    _hz = float(os.environ.get("ARM_CONTROL_TAU_HZ", "20"))
    TAU_FF.send_hz = max(1.0, min(500.0, _hz))
except ValueError:
    TAU_FF.send_hz = 20.0
try:
    TAU_FF.gain = float(os.environ.get("ARM_CONTROL_TAU_GAIN", "1.0"))
except ValueError:
    TAU_FF.gain = 1.0


@dataclass(frozen=True)
class SerialConfig:
    port: str = "/dev/ttyAMA2"
    baudrate: int = 115200
    bytesize: int = 8
    stopbits: int = 1
    parity: str = "N"  # N/E/O
    reconnect_interval_sec: float = 2.0
    # 握手相关配置
    handshake_enabled: bool = True
    handshake_request: str = "start\r\n"
    handshake_expect: str = "ok"
    handshake_timeout_sec: float = 2.0

    # 发送关节指令后的回传等待（用于判断是否断链并重新握手）
    res_expect_prefix: str = "RES"
    res_wait_timeout_sec: float = 1.5
    # 超时后是否重连并重发（最多尝试次数 = res_retry_count + 1）
    res_retry_count: int = 1
    reconnect_wait_timeout_sec: float = 5.0

    # 若握手连续失败次数超过一次（默认 2 次失败），则即使握手最终成功也不再发送 DATA
    res_handshake_fail_stop_threshold: int = 0


@dataclass(frozen=True)
class NetworkConfig:
    host: str = "0.0.0.0"
    port: int = 8765
    state_push_interval_sec: float = 0.05  # 20Hz
    camera_push_interval_sec: float = 0.1  # 10Hz


@dataclass(frozen=True)
class CameraConfig:
    enabled: bool = False
    device_index: int = 0
    jpeg_quality: int = 80


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = SerialConfig()
    network: NetworkConfig = NetworkConfig()
    camera: CameraConfig = CameraConfig()
    log_dir: Path = Path("/home/phoenix/icecream/logs")


CONFIG = AppConfig()


def set_tau_calibration_rad(r0: float, r1: float, r2: float, r3: float) -> None:
    """四轴标定零位（弧度），供外部调用。"""
    TAU_FF.calibration_rad[:] = [float(r0), float(r1), float(r2), float(r3)]


def set_tau_gain(gain: float) -> None:
    """四轴力矩前馈总增益（与 `TAU_FF.gain` 相同）。"""
    TAU_FF.gain = float(gain)

