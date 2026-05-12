"""V2 runtime configuration."""

from __future__ import annotations

import os
from dataclasses import dataclass


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name, "").strip().lower()
    if not raw:
        return default
    return raw in ("1", "true", "yes", "on")


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, str(default)).strip())
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, str(default)).strip())
    except ValueError:
        return default


@dataclass(frozen=True)
class InitPremoveConfig:
    """启动四轴预置位（premove）：从当前反馈角按角速度上限趋近 ``标定 + rel_deg``。

    环境变量接口（与 ``scripts/start.sh`` 中 export 一致）：
    - ``ARM_CONTROL_RPI_PREMOVE_SKIP``：1/true 跳过；未设时默认真（须显式 0 启用）
    - ``ARM_CONTROL_RPI_PREMOVE_VMAX``：限速 rad/s（每步 ``±vmax*dt``）
    - ``ARM_CONTROL_RPI_PREMOVE_TOL_RAD``：到位判据（rad）
    - ``ARM_CONTROL_RPI_PREMOVE_REL_DEG_1`` … ``_4``：相对标定目标（度）
    - ``ARM_CONTROL_RPI_PREMOVE_MAX_SEC``：超时秒数（默认 180）
    """

    skip: bool = _env_bool("ARM_CONTROL_RPI_PREMOVE_SKIP", True)
    vmax_rad_s: float = max(1e-6, _env_float("ARM_CONTROL_RPI_PREMOVE_VMAX", 0.4))
    tol_rad: float = max(1e-4, _env_float("ARM_CONTROL_RPI_PREMOVE_TOL_RAD", 0.008))
    rel_deg: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_RPI_PREMOVE_REL_DEG_1", 18.99),
        _env_float("ARM_CONTROL_RPI_PREMOVE_REL_DEG_2", -18.64),
        _env_float("ARM_CONTROL_RPI_PREMOVE_REL_DEG_3", -121.12),
        _env_float("ARM_CONTROL_RPI_PREMOVE_REL_DEG_4", -77.51),
    )
    max_sec: float = max(5.0, _env_float("ARM_CONTROL_RPI_PREMOVE_MAX_SEC", 180.0))


@dataclass(frozen=True)
class SerialConfig:
    port: str = os.environ.get("ARM_CONTROL_SERIAL_PORT", "/dev/ttyAMA2")
    baudrate: int = _env_int("ARM_CONTROL_SERIAL_BAUD", 115200)
    reconnect_interval_sec: float = _env_float("ARM_CONTROL_SERIAL_RECONNECT_SEC", 2.0)


@dataclass(frozen=True)
class UdpConfig:
    enabled: bool = _env_bool("ARM_CONTROL_RPI_UDP", False)
    host: str = "0.0.0.0"
    port: int = _env_int("ARM_CONTROL_RPI_UDP_PORT", 9870)
    stale_sec: float = max(0.05, _env_float("ARM_CONTROL_RPI_UDP_STALE_SEC", 0.35))


@dataclass(frozen=True)
class ServerConfig:
    host: str = os.environ.get("ARM_CONTROL_WS_HOST", "0.0.0.0")
    port: int = _env_int("ARM_CONTROL_WS_PORT", 8765)
    state_push_interval_sec: float = max(
        0.01, _env_float("ARM_CONTROL_STATE_PUSH_SEC", 0.05)
    )


@dataclass(frozen=True)
class ControlConfig:
    tau_hz: float = max(1.0, min(500.0, _env_float("ARM_CONTROL_TAU_HZ", 25.0)))
    tau_gain: float = _env_float("ARM_CONTROL_TAU_GAIN", 1.0)
    calibration_rad: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_CAL_R0", 1.57416),
        _env_float("ARM_CONTROL_CAL_R1", 1.29151),
        _env_float("ARM_CONTROL_CAL_R2", 2.400980),
        _env_float("ARM_CONTROL_CAL_R3", 0.490005),
    )
    hold_kp: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_HOLD_KP_1", 10.0),
        _env_float("ARM_CONTROL_HOLD_KP_2", 60.0),
        _env_float("ARM_CONTROL_HOLD_KP_3", 50.0),
        _env_float("ARM_CONTROL_HOLD_KP_4", 25.0),
    )

    # hold_kp: tuple[float, float, float, float] = (
    #     _env_float("ARM_CONTROL_HOLD_KP_1", 0.0),
    #     _env_float("ARM_CONTROL_HOLD_KP_2", 0.0),
    #     _env_float("ARM_CONTROL_HOLD_KP_3", 0.0),
    #     _env_float("ARM_CONTROL_HOLD_KP_4", 0.0),
    # )
    
    hold_kd: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_HOLD_KD_1", 1.8),
        _env_float("ARM_CONTROL_HOLD_KD_2", 13.0),
        _env_float("ARM_CONTROL_HOLD_KD_3", 10),
        _env_float("ARM_CONTROL_HOLD_KD_4", 2.0),
    )

    # hold_kd: tuple[float, float, float, float] = (
    #     _env_float("ARM_CONTROL_HOLD_KD_1", 1.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_2", 3.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_3", 1.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_4", 0.3),
    # )
    
    kp_float_mode: bool = _env_bool("ARM_CONTROL_MIT_CMD_KP_FLOAT", False)
    init_premove: InitPremoveConfig = InitPremoveConfig()
    feedback_source: str = os.environ.get("ARM_CONTROL_TAU_FF_INPUT", "mit").strip().lower()
    # 启动后等待首帧关节反馈（秒）。0=不等待（与旧行为一致）。与 arm_control 在无反馈时不发 MIT 的思路一致，避免未收到 STM32 上行就进入主环发空指令。
    init_feedback_wait_sec: float = max(0.0, _env_float("ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC", 10.0))
    # 1=开启重力前馈；0=关闭（MIT 帧中 t 全为 0，且不调用 Pinocchio）
    gravity_ff_enabled: bool = _env_bool("ARM_CONTROL_GRAVITY_FF", True)
    max_cmd_speed_rad_s: tuple[float, float, float, float] = (
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M1", 0.5)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M2", 0.5)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M3", 0.5)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M4", 0.5)),
    )
    cold_hold_sec: float = max(0.0, _env_float("ARM_CONTROL_COLD_HOLD_SEC", 0.0))
    boot_move_enabled: bool = _env_bool("ARM_CONTROL_BOOT_MOVE_ENABLED", False)
    boot_move_rel_deg: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_BOOT_MOVE_DEG_1", 2.2839),
        _env_float("ARM_CONTROL_BOOT_MOVE_DEG_2", -53.5752),
        _env_float("ARM_CONTROL_BOOT_MOVE_DEG_3", 140.9516),
        _env_float("ARM_CONTROL_BOOT_MOVE_DEG_4", 92.6236),
    )
    boot_move_vmax_rad_s: float = max(
        1e-4, _env_float("ARM_CONTROL_BOOT_MOVE_VMAX", 0.2)
    )
    boot_move_tol_rad: float = max(
        1e-4, _env_float("ARM_CONTROL_BOOT_MOVE_TOL_RAD", 0.01)
    )


@dataclass(frozen=True)
class PiCameraUdpConfig:
    """Pi → camera JSON on UDP (``docs/pi2camera.md`` §4)."""

    enabled: bool = _env_bool("ARM_CONTROL_CAMERA_UDP_BROADCAST", False)
    host: str = os.environ.get("ARM_CONTROL_CAMERA_UDP_HOST", "255.255.255.255")
    port: int = _env_int("ARM_CONTROL_CAMERA_UDP_PORT", 9982)
    hz: float = max(1.0, min(60.0, _env_float("ARM_CONTROL_CAMERA_UDP_HZ", 20.0)))
    broadcast: bool = _env_bool("ARM_CONTROL_CAMERA_UDP_SO_BROADCAST", True)


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = SerialConfig()
    udp: UdpConfig = UdpConfig()
    server: ServerConfig = ServerConfig()
    control: ControlConfig = ControlConfig()
    camera_udp: PiCameraUdpConfig = PiCameraUdpConfig()


CONFIG = AppConfig()

