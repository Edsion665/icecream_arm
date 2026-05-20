"""默认网络、控制参数与标定加载。

运行时使用单例 ``CONFIG`` / ``IK_CONFIG`` / ``SIM_CONFIG`` / ``RUNTIME``（见文件底部）。
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from typing import List, Optional


@dataclass(frozen=True)
class ControlConfig:
    """控制周期与网络参数。

    速度规划完全由 Pi 侧 ramp 负责，Bridge 侧只计算目标角度并以固定频率发送。
    """

    control_hz: float = 25.0
    default_tcp_port: int = 9888
    default_udp_port: int = 9870
    listen_host: str = "0.0.0.0"
    rpi_ip: Optional[str] = None
    web_test_host: str = "0.0.0.0"
    web_test_port: int = 8877
    calibration_md_relpath: str = "initial_position.md"
    # 手腕关节5固定角（度），IK 求解时作为约束传入
    q5_fixed_deg: float = 0.0
    # 到位判定容差（度），用于 ReachTracker 判断命令是否执行完成
    reached_joints_tol_deg: float = 5.0
    # Pi udp.p_rel_deg 与 bridge 指令相差超过此值(deg)时，到位判定只用 bridge 指令角
    reached_udp_sync_tol_deg: float = 1.0
    reached_pose_tol_m: float = 0.005
    reached_wrist_tol_deg: float = 5.0
    reached_claw_delay_s: float = 2.0
    reached_timeout_s: float = 10.0
    reached_stable_frames: int = 5
    # 无 Pi 反馈时禁止判到位（避免用 q_cmd 自比导致 head 提前关爪）
    require_pi_feedback_for_reached: bool = True
    # pi2camera v2：Pi→PC camera_state UDP 监听（motor_rad）
    camera_udp_listen_host: str = "0.0.0.0"
    camera_udp_listen_port: int = 9982
    # 距上次收到 camera_state 超过此毫秒则不判到位（None=不检查）
    max_camera_packet_age_ms_for_reached: Optional[float] = 500.0
    # 旧 WebSocket 回传（已弃用到位判定，仅 teleop 等可能仍用）
    rpi_ws_port: int = 8765
    pi_feedback_reconnect_interval_s: float = 3.0

    @property
    def control_dt(self) -> float:
        return 1.0 / self.control_hz


@dataclass(frozen=True)
class IKConfig:
    """几何解耦与数值 IK 参数。

    q4_geometric_offset_deg / q4_geometric_q23_coeff 定义 joint4 的几何约束：
        q4 = offset + coeff * (q2 + q3)
    由 urdf_kinematics.py 读取为模块级常量，IK 求解和 engine 的 _pose_to_joints 均依赖。
    """

    # joint4 几何约束系数
    q4_geometric_offset_deg: float = -180.0
    q4_geometric_q23_coeff: float = -1.0
    # IK 数值求解参数
    pose_q5_extra_deg: float = 0.0
    ik_damping: float = 1e-2
    ik_max_iter: int = 80
    ik_pos_tol: float = 1e-5
    # joint4 安全角度范围（rad）
    q4_safe_min: float = -2.8
    q4_safe_max: float = 2.8
    # 奇异性检测阈值
    sv_warn: float = 0.025
    sv_crit: float = 0.005


@dataclass(frozen=True)
class SimulationConfig:
    """Isaac Sim 场景与关节 PD 默认值。"""

    sim_track_snap_threshold_rad: float = 0.001
    arm_prim_path: str = "/World/IceCreamArm"
    target_marker_path: str = "/World/TargetJoint4Marker"
    # 沿父空间 +Z 平移手臂根 prim（米）；父为 /World 时即世界 +Z 抬高整臂（含 link0）。例：0.1 = 抬高 10 cm。
    arm_world_z_offset_m: float = 0.2
    marker_z_lift_m: float = 0.04
    arm_usd_relpath: str = "v12/ice_cream_v12_arm.usd"
    arm_urdf_relpath: str = "v12/ice_cream_v12.SLDASM.urdf"
    articulation_prim_path: Optional[str] = None
    pd_kp_base: float = 1e4
    pd_kd_base: float = 1e3
    sim_headless: bool = False
    sim_web_host: str = "0.0.0.0"
    sim_web_port: int = 8877
    sim_gain_scale: float = 1.0
    sim_resync: bool = True


@dataclass(frozen=True)
class BridgeRuntimeConfig:
    """运行期开关（CLI / 测试注入）。"""

    udp_strict: bool = True


CONFIG = ControlConfig()
IK_CONFIG = IKConfig()
SIM_CONFIG = SimulationConfig()
RUNTIME = BridgeRuntimeConfig()

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
