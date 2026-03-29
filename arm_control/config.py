"""全局配置：串口、网络、相机等参数集中管理。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


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

