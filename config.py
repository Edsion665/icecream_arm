"""V2 runtime configuration."""

from __future__ import annotations

import os
from dataclasses import dataclass


#
# ---- 统一静态调参区 ----
# 这一段集中存放跨模块共享的协议与映射常量。将这些“只读超参数”统一放在这里，
# 便于后续只改 config 就能完成行为调整，避免在 UDP、映射、重力补偿等文件中分散修改。
#

# MOTOR_AXIS_SIGN 定义四个电机轴在“电机坐标系”和“关节坐标系”之间的方向关系，
# 顺序固定为 motor1..motor4。取值 +1 表示正方向一致，-1 表示方向相反。
# 该符号会被目标映射、反馈反算、关节力矩到电机力矩转换等链路共同复用，
# 用于保证整条控制链在符号约定上的一致性。
MOTOR_AXIS_SIGN: tuple[float, float, float, float] = (-1.0, -1.0, -1.0, 1.0)

# pi2camera 下行 ``motor_rad``：在 ``motor_rad_to_joint_rel_rad`` 之后对 M3/M4（下标 2、3）再取反，
# 与相机解算端关节正方向约定对齐（bridge/控制链仍用 MOTOR_AXIS_SIGN，不改）。
PI2CAMERA_JOINT_REL_SIGN: tuple[float, float, float, float] = (-1.0, 1.0, -1.0, -1.0)

# 以下常量定义 bridge2pi 的 UDP 二进制帧格式。当前协议为（与 ``docs/bridge2pi.md`` v3 一致）：
# seq(uint32) + ts(float64) + p_rel_deg[8](float64) + omega_rad_s[8](float64)。
# UDP_PACKET_FMT/UDP_PACKET_SIZE/UDP_VECTOR_DIM 必须保持一致，修改任一项时需同步检查其余项。
UDP_VECTOR_DIM: int = 8
UDP_PACKET_FMT: str = "=Id" + "d" * (UDP_VECTOR_DIM * 2)
UDP_PACKET_SIZE: int = 4 + 8 + UDP_VECTOR_DIM * 8 * 2

# WRIST_MIN_DEG 与 WRIST_MAX_DEG 定义腕部舵机角度语义的物理范围（单位：度），
# 对应 bridge2pi 在 p_rel_deg[4] 中传输的 wrist 角度。
WRIST_MIN_DEG: float = -180.0
WRIST_MAX_DEG: float = 180.0

# p_rel_deg[6]/[7] 映射至 Pi→STM32 42B 帧的步进增量角与传送带启停（见 ``docs/pi2stm.md``）。
# 夹爪状态语义通过 UDP 的 p_rel_deg[5] 传输。约定 0.0 为闭合、1.0 为张开，
# 中间值可通过阈值转换为离散开合指令。
GRIP_CLOSED_STATE: float = 0.0
GRIP_OPEN_STATE: float = 1.0

# GRIP_THRESHOLD 为夹爪状态离散化阈值：大于等于阈值判定为 open，否则判定为 closed。
GRIP_THRESHOLD: float = 0.5

# 以下脉宽映射定义夹爪舵机在 closed/open 两种状态下的目标脉宽（单位：us）。
GRIP_CLOSED_US: int = 600
GRIP_OPEN_US: int = 1200

# GRIP_MIT39_INIT_US 是 MIT39 启动阶段使用的夹爪初始脉宽（单位：us），
# 仅用于 boot/init，故与运行期基于 UDP 状态的开合映射解耦。
GRIP_MIT39_INIT_US: int = 1500


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


def _env_init_feedback_wait_sec() -> float:
    """>0 为有限超时；<=0 或 inf 表示阻塞直到首帧串口关节反馈（无跳过）。"""
    raw = os.environ.get("ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC", "").strip().lower()
    if not raw:
        return float("inf")
    if raw in ("inf", "infinity", "none"):
        return float("inf")
    try:
        v = float(raw)
    except ValueError:
        return float("inf")
    if v <= 0:
        return float("inf")
    return v


# ---- 重力前馈实机修正（Pinocchio → 电机符号映射之后、写入 MIT ``t`` 之前）----
#
# motor1：URDF 转轴与重力平行，Pinocchio 对第 1 关节理论 τ≈0，不做缩放或偏置。
# motor2..motor4：对 Pinocchio 电机空间力矩逐轴缩放。
#   export ARM_CONTROL_GRAVITY_SCALE_M2=1.3
#   export ARM_CONTROL_GRAVITY_SCALE_M3=1.2
#   export ARM_CONTROL_GRAVITY_SCALE_M4=1.2
GRAVITY_AXIS_SCALE_M234: tuple[float, float, float] = (
    _env_float("ARM_CONTROL_GRAVITY_SCALE_M2", 1.3),
    _env_float("ARM_CONTROL_GRAVITY_SCALE_M3", 1.2),
    _env_float("ARM_CONTROL_GRAVITY_SCALE_M4", 1.2),
)


def apply_gravity_motor_output(
    tau_motor_nm: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """重力前馈电机空间输出修正：M1 直通，M2–M4 乘缩放。

    输入 ``tau_motor_nm`` 为 ``joint_tau_to_motor_tau`` 之后、尚未修正的四轴力矩 (Nm)，
    顺序 motor1..motor4，与 ``controller`` 写入 MIT 的 ``t`` 一致。
    """
    t1, t2, t3, t4 = tau_motor_nm
    s2, s3, s4 = GRAVITY_AXIS_SCALE_M234
    return (
        float(t1),
        float(t2) * s2,
        float(t3) * s3,
        float(t4) * s4,
    )


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


class ControlConfig:
    tau_hz: float = max(1.0, min(500.0, _env_float("ARM_CONTROL_TAU_HZ", 25.0)))
    tau_gain: float = _env_float("ARM_CONTROL_TAU_GAIN", 1.0)
    calibration_rad: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_CAL_R0", 2.055016),
        _env_float("ARM_CONTROL_CAL_R1", 3.6232),
        _env_float("ARM_CONTROL_CAL_R2", -4.1500),
        _env_float("ARM_CONTROL_CAL_R3", 0.378),
    )
    
    hold_kp: tuple[float, float, float, float] = (
        _env_float("ARM_CONTROL_HOLD_KP_1", 20.0),
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
        _env_float("ARM_CONTROL_HOLD_KD_1", 1.7),
        _env_float("ARM_CONTROL_HOLD_KD_2", 13.0),
        _env_float("ARM_CONTROL_HOLD_KD_3", 10),
        _env_float("ARM_CONTROL_HOLD_KD_4", 2.0),
    )

    # hold_kd: tuple[float, float, float, foat] = (
    #     _env_float("ARM_CONTROL_HOLD_KD_1", 1.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_2", 3.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_3", 1.0),
    #     _env_float("ARM_CONTROL_HOLD_KD_4", 0.3),
    # )
    
    kp_float_mode: bool = _env_bool("ARM_CONTROL_MIT_CMD_KP_FLOAT", False)
    feedback_source: str = os.environ.get("ARM_CONTROL_TAU_FF_INPUT", "mit").strip().lower()
    # 启动后等待首帧串口关节反馈：默认无限阻塞；设为正数则为最长等待秒数（超时后仍继续启动，仅用于调试）。
    init_feedback_wait_sec: float = _env_init_feedback_wait_sec()
    # 超过该时间未收到新的 STM32 串口反馈帧则视为掉线，进入 frozen（不发 MIT/舵机占位帧），恢复后按 safe gate 重新锁存首帧 UDP。
    serial_feedback_stale_sec: float = max(
        0.02, _env_float("ARM_CONTROL_SERIAL_FEEDBACK_STALE_SEC", 1.5)
    )
    # 1=开启重力前馈；0=关闭（MIT 帧中 t 全为 0，且不调用 Pinocchio）
    gravity_ff_enabled: bool = _env_bool("ARM_CONTROL_GRAVITY_FF", True)
    max_cmd_speed_rad_s: tuple[float, float, float, float] = (
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M1", 0.8)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M2", 0.6)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M3", 0.8)),
        max(1e-4, _env_float("ARM_CONTROL_MAX_SPEED_M4", 1.5)),
    )
    # 启动安全门：在收到首帧 UDP 后，先冻结后续 UDP 输入，按限速从当前位姿追到首帧目标，
    # 到达后再恢复正常 UDP 逐帧跟踪，以避免“晚启动+首帧跳变”带来的突变风险。
    startup_safe_gate_enabled: bool = _env_bool("ARM_CONTROL_STARTUP_SAFE_GATE", True)
    startup_safe_gate_tol_rad: float = max(
        1e-4, _env_float("ARM_CONTROL_STARTUP_SAFE_GATE_TOL_RAD", 0.01)
    )
    startup_safe_gate_timeout_sec: float = max(
        0.5, _env_float("ARM_CONTROL_STARTUP_SAFE_GATE_TIMEOUT_SEC", 20.0)
    )
    cold_hold_sec: float = max(0.0, _env_float("ARM_CONTROL_COLD_HOLD_SEC", 0.0))



@dataclass(frozen=True)
class SwitchGateConfig:
    """ttyAMA4 环路开关闭合时向 head 发 pi2head start（``docs/pi2head.md``）。"""

    head_host: str = "192.168.31.71"
    head_port: int = 8778
    port: str = "/dev/ttyAMA4"
    baudrate: int = 115200
    poll_interval_sec: float = 0.5


@dataclass(frozen=True)
class PiCameraUdpConfig:
    """Pi → camera JSON on UDP (``docs/pi2camera.md`` §4)."""

    enabled: bool = _env_bool("ARM_CONTROL_CAMERA_UDP_BROADCAST", True)
    host: str = os.environ.get("ARM_CONTROL_CAMERA_UDP_HOST", "255.255.255.255")
    port: int = _env_int("ARM_CONTROL_CAMERA_UDP_PORT", 9982)
    hz: float = _env_float("ARM_CONTROL_CAMERA_UDP_HZ", 50.0)
    broadcast: bool = _env_bool("ARM_CONTROL_CAMERA_UDP_SO_BROADCAST", True)


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig = SerialConfig()
    udp: UdpConfig = UdpConfig()
    server: ServerConfig = ServerConfig()
    control: ControlConfig = ControlConfig()
    switch_gate: SwitchGateConfig = SwitchGateConfig()
    camera_udp: PiCameraUdpConfig = PiCameraUdpConfig()


CONFIG = AppConfig()

