"""默认网络、控制参数与标定加载。

模块级常量保留为对外稳定 API；结构化参数见 ``ControlConfig`` / ``IKConfig`` /
``SimulationConfig`` / ``BridgeRuntimeConfig``。
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class ControlConfig:
    """控制周期、限速与到位判定参数。"""

    control_hz: float = 25.0
    default_tcp_port: int = 9888
    default_udp_port: int = 9870
    approach_gain_scale: float = 0.1  # APPROACH_GAIN = control_hz * scale
    max_joint_vel_rad_s: float = 0.6
    max_target_rate_rad_s: float = 1.5
    pose_vel_max_m_s: float = 0.1
    reached_joints_tol_deg: float = 5.0
    reached_pose_tol_m: float = 0.005
    reached_wrist_tol_deg: float = 5.0
    reached_claw_delay_s: float = 2.0
    reached_timeout_s: float = 10.0
    reached_stable_frames: int = 5
    rpi_ws_port: int = 8765
    pi_feedback_reconnect_interval_s: float = 3.0

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_hz

    @property
    def approach_gain(self) -> float:
        return self.control_hz * self.approach_gain_scale


@dataclass(frozen=True)
class IKConfig:
    """几何解耦与数值 IK 参数。"""

    q4_geometric_offset_deg: float = -180.0
    q4_geometric_q23_coeff: float = -1.0
    pose_q5_extra_deg: float = 0.0
    q4_blend_time_s: float = 2.0
    ik_damping: float = 1e-2
    ik_max_iter: int = 80
    ik_pos_tol: float = 1e-5
    q4_safe_min: float = -2.8
    q4_safe_max: float = 2.8
    sv_warn: float = 0.025
    sv_crit: float = 0.005


@dataclass(frozen=True)
class SimulationConfig:
    """Isaac Sim 场景与关节 PD 默认值。"""

    sim_track_snap_threshold_rad: float = 0.35
    arm_prim_path: str = "/World/IceCreamArm"
    target_marker_path: str = "/World/TargetJoint4Marker"
    marker_z_lift_m: float = 0.04
    default_usd_candidates: Tuple[str, ...] = (
        "ice_cream_v8_arm.usd",
        "ice_cream_single_arm.usd",
        "ice_cream_arm.usd",
    )
    v8_payload_basename: str = "ice_cream_v8_arm.usd"
    v8_payload_children: Tuple[str, ...] = (
        "ice_cream_v8_arm_physics.usd",
        "ice_cream_v8_arm_sensor.usd",
        "ice_cream_v8_arm_base.usd",
    )
    pd_kp_base: float = 1e4
    pd_kd_base: float = 1e3


@dataclass(frozen=True)
class BridgeRuntimeConfig:
    """运行期开关（CLI / 测试注入）。"""

    udp_strict: bool = True


# 默认单例（与下方模块常量一致）
DEFAULT_CONTROL_CONFIG = ControlConfig()
DEFAULT_IK_CONFIG = IKConfig()
DEFAULT_SIMULATION_CONFIG = SimulationConfig()
DEFAULT_BRIDGE_RUNTIME = BridgeRuntimeConfig()

# --- 模块级别名（历史导入路径） ---
CONTROL_HZ = DEFAULT_CONTROL_CONFIG.control_hz
CONTROL_DT = DEFAULT_CONTROL_CONFIG.control_dt
DEFAULT_TCP_PORT = DEFAULT_CONTROL_CONFIG.default_tcp_port
DEFAULT_UDP_PORT = DEFAULT_CONTROL_CONFIG.default_udp_port
APPROACH_GAIN = DEFAULT_CONTROL_CONFIG.approach_gain
MAX_JOINT_VEL_RAD_S = DEFAULT_CONTROL_CONFIG.max_joint_vel_rad_s
MAX_TARGET_RATE_RAD_S = DEFAULT_CONTROL_CONFIG.max_target_rate_rad_s
POSE_VEL_MAX_M_S = DEFAULT_CONTROL_CONFIG.pose_vel_max_m_s

Q4_GEOMETRIC_OFFSET_DEG = DEFAULT_IK_CONFIG.q4_geometric_offset_deg
Q4_GEOMETRIC_Q23_COEFF = DEFAULT_IK_CONFIG.q4_geometric_q23_coeff
POSE_Q5_EXTRA_DEG = DEFAULT_IK_CONFIG.pose_q5_extra_deg
Q4_BLEND_TIME_S = int(DEFAULT_IK_CONFIG.q4_blend_time_s)
IK_DAMPING = DEFAULT_IK_CONFIG.ik_damping
IK_MAX_ITER = DEFAULT_IK_CONFIG.ik_max_iter
IK_POS_TOL = DEFAULT_IK_CONFIG.ik_pos_tol
Q4_SAFE_MIN, Q4_SAFE_MAX = DEFAULT_IK_CONFIG.q4_safe_min, DEFAULT_IK_CONFIG.q4_safe_max
SV_WARN = DEFAULT_IK_CONFIG.sv_warn
SV_CRIT = DEFAULT_IK_CONFIG.sv_crit

SIM_TRACK_SNAP_THRESHOLD_RAD = DEFAULT_SIMULATION_CONFIG.sim_track_snap_threshold_rad

REACHED_JOINTS_TOL_DEG = DEFAULT_CONTROL_CONFIG.reached_joints_tol_deg
REACHED_POSE_TOL_M = DEFAULT_CONTROL_CONFIG.reached_pose_tol_m
REACHED_WRIST_TOL_DEG = DEFAULT_CONTROL_CONFIG.reached_wrist_tol_deg
REACHED_CLAW_DELAY_S = DEFAULT_CONTROL_CONFIG.reached_claw_delay_s
REACHED_TIMEOUT_S = DEFAULT_CONTROL_CONFIG.reached_timeout_s
REACHED_STABLE_FRAMES = DEFAULT_CONTROL_CONFIG.reached_stable_frames

RPI_WS_PORT = DEFAULT_CONTROL_CONFIG.rpi_ws_port
PI_FEEDBACK_RECONNECT_INTERVAL_S = DEFAULT_CONTROL_CONFIG.pi_feedback_reconnect_interval_s

DEFAULT_Q_CALIB_DEG_LIST = [0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_INITIAL_JOINT_REL_DEG_4 = [0.0, 90.0, -180.0, -20.0]


def frontend_pose_to_internal_m(x: float, y: float, z: float) -> tuple[float, float, float]:
    """前端输入的 link4 目标 (x,y,z) 米（父节点 local）→ IK local。

    约定：
    - 前端发送的是机械臂父节点坐标系（local）下的位置；
    - IK 也使用同一 local（URDF 基座 link0）求解；
    - 这里不做坐标变换，保持数值一一对应。

    Args:
        x: 目标 x（米）。
        y: 目标 y（米）。
        z: 目标 z（米）。

    Returns:
        与输入一致的三元组（显式 float）。
    """
    return (float(x), float(y), float(z))


def load_calibration_deg(calib_file: str | None) -> List[float]:
    """从 initial_position.md 或指定文件解析标定关节角（度）。

    Args:
        calib_file: 标定文件路径；空或不可读时返回默认列表。

    Returns:
        长度 5 的关节角列表（度）。
    """
    if not calib_file:
        return list(DEFAULT_Q_CALIB_DEG_LIST)
    try:
        if not os.path.isfile(calib_file):
            return list(DEFAULT_Q_CALIB_DEG_LIST)
        with open(calib_file, "r", encoding="utf-8") as f:
            text = f.read()
    except OSError:
        return list(DEFAULT_Q_CALIB_DEG_LIST)
    pattern = re.compile(r"joint(\d)\s*=\s*([+-]?\d+\.?\d*)\s*°", re.IGNORECASE)
    found = {}
    for m in pattern.finditer(text):
        idx = int(m.group(1))
        if 1 <= idx <= 5:
            found[idx] = float(m.group(2))
    if len(found) == 5:
        return [found[i] for i in range(1, 6)]
    return list(DEFAULT_Q_CALIB_DEG_LIST)
