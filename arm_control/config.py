"""全局配置：串口、网络、相机等参数集中管理。"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class TauFfRuntimeConfig:
    """力矩前馈模式可调参数（可在运行时改 `calibration_rad` 等）。"""

    # 四轴标定零位（弧度），与力矩前馈所用角度源（MIT 电机 p 或 FB）同含义
    calibration_rad: list[float] = field(default_factory=lambda: [1.57416, 1.360151,2.380980,0.490005])
    # MIT 35B 下发频率（Hz）；过高可能占满 CPU / 串口带宽
    send_hz: float = 20.0
    # Pinocchio 算出的四轴重力前馈 (Nm) 乘以该系数再下发（1.0 为不缩放）
    gain: float = 1.0
    # 每轴 MIT 命令中的 p、v（弧度、rad/s）；kp/kd 下发时由 ``MIT_CMD_*`` 固定；力矩 t 由 Pinocchio 填入
    mit_motor_cmd: list[dict[str, float]] = field(
        default_factory=lambda: [
            {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0},
            {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0},
            {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0},
            {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0},
        ]
    )


def _env_mode() -> str:
    v = os.environ.get("ARM_CONTROL_MODE", "tau_ff").strip().lower()
    return v if v in ("data", "tau_ff") else "tau_ff"


# 控制模式：data = 仅 WebSocket set_joint 发 DATA；tau_ff = 重力前馈并发 MIT 35B 二进制（角度源见 TAU_FF_INPUT）
CONTROL_MODE: str = _env_mode()

TAU_FF = TauFfRuntimeConfig()
try:
    _hz = float(os.environ.get("ARM_CONTROL_TAU_HZ", "20"))
    TAU_FF.send_hz = max(1.0, min(500.0, _hz))
except ValueError:
    TAU_FF.send_hz = 25.0
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
    # 握手相关（当前 serial_manager._open_serial 中 start/ok 逻辑已注释；若恢复该段代码再启用）
    handshake_enabled: bool = False
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


def set_mit_motor_cmd_params(motors: list[dict[str, Any]]) -> None:
    """更新 MIT 命令中每轴的 p、v；实际下发 kp/kd 见 ``MIT_CMD_KP_FLOAT_MODE`` / ``MIT_CMD_FIXED_*``。"""
    if len(motors) != 4:
        raise ValueError("须恰好 4 个电机，每项含 p、v")
    for i, m in enumerate(motors):
        cur = TAU_FF.mit_motor_cmd[i]
        for k in ("p", "v"):
            if k in m:
                cur[k] = float(m[k])


def mit_uplink_mode_from_env() -> str:
    """STM32 MIT 上行：`none` | `hex68` | `binary`（34 字节帧，见 RPI_STM32_PROTOCOL.md）。"""
    v = os.environ.get("ARM_CONTROL_MIT_UPLINK", "none").strip().lower()
    if v in ("", "0", "false", "off", "none"):
        return "none"
    # hex68 = 一行 68 个十六进制字符（34 字节，含 0xAA 0x55）；hex44 等为旧名，等同 hex68
    if v in ("hex68", "hex44", "hex40", "hex", "line"):
        return "hex68"
    if v in ("binary", "raw", "bin"):
        return "binary"
    return "none"


MIT_UPLINK_MODE: str = mit_uplink_mode_from_env()


def tau_ff_input_from_env() -> str:
    """力矩前馈四轴弧度来源：`mit`=34B/hex 解码的电机 p；`fb`= FB 行四浮点。"""
    v = os.environ.get("ARM_CONTROL_TAU_FF_INPUT", "mit").strip().lower()
    return v if v in ("mit", "fb") else "mit"


TAU_FF_INPUT: str = tau_ff_input_from_env()


def mit_cmd_kp_float_mode_from_env() -> bool:
    """True=浮游模式：MIT 下发四轴 kp=0；False=正常模式：kp 为 ``MIT_CMD_FIXED_KP_NORMAL``。"""
    v = os.environ.get("ARM_CONTROL_MIT_CMD_KP_FLOAT", "0").strip().lower()
    return v in ("1", "true", "yes", "on", "float")


# --- kp：浮游(kp=0) / 正常( MIT_CMD_FIXED_KP_NORMAL ) ---
# 环境变量 ARM_CONTROL_MIT_CMD_KP_FLOAT=1 → 浮游；未设或 0 → 正常。
# 代码内写死：将下一行改为 ``MIT_CMD_KP_FLOAT_MODE = True`` 或 ``False``，并删掉右侧 ``mit_cmd_kp_float_mode_from_env()``。
MIT_CMD_KP_FLOAT_MODE: bool = mit_cmd_kp_float_mode_from_env()

# MIT 下行四轴固定 kp（0~500，RPI_MIT_CMD_BINARY_ENCODE.md）；正常模式、电机 1~4 → 索引 0~3
MIT_CMD_FIXED_KP_NORMAL: tuple[float, float, float, float] = (10.0, 25.0, 28.0, 15)
#MIT_CMD_FIXED_KP_NORMAL: tuple[float, float, float, float] = (10.0, 16.0, 20.0, 10.0)
# MIT 下行四轴固定 kd（0~5）；电机 1~4 → 索引 0~3
MIT_CMD_FIXED_KD: tuple[float, float, float, float] = (1.8, 2.0, 1.8, 0.3)



def set_kp_float_mode(enabled: bool) -> None:
    """动态切换 kp 浮游模式（True=kp=0，False=正常kp）。"""
    global MIT_CMD_KP_FLOAT_MODE
    MIT_CMD_KP_FLOAT_MODE = bool(enabled)
