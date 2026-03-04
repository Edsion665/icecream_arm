#!/usr/bin/env python3
"""
雅可比矩阵验证：在 Isaac Sim 中让机械臂末端走直线
====================================================
ice_cream_0208.SLDASM  5-DOF 机械臂

功能
----
1. 从 Isaac Sim Articulation 实时读取关节角（而非脚本内部积分值）
2. 计算解析雅可比，将笛卡尔速度映射为关节速度，积分后下发位置控制
3. 用 USD DebugDraw / 球体标记 在场景中实时绘制：
      • 绿球 = 末端实际轨迹
      • 红球 = 理想直线轨迹
4. 定期打印偏离直线距离与位置误差
5. 验证完成后打印汇总报告

HOME 位置（由 URDF 工作空间分析确定，所有连杆 Z > 0.1m，不入地）
  q_home = [0, -1.271, 1.5, -0.5, 0]  rad
  末端位置 ≈ [0.494, 0.166, 0.482] m（基座系）
  σ_min = 0.2055（远离奇异）

轨迹模式
  line  - 直线+Z向下：方向、长度、速度、--speed-z
  circle - 先回 HOME，再以 HOME 为圆上一点逆时针画圆（XOY、同高）：--radius、--circle-speed
           未到 HOME 不会开始画圆；可调 --stabilize-max-steps 放宽等待时间（默认 3000 步≈50s）
  均可通过绿/红球描述轨迹，并打印偏离与误差报告

用法
----
  python icecream_jacobian_line.py
  python icecream_jacobian_line.py --direction 0 0 1 --length 0.05 --speed 0.015
  python icecream_jacobian_line.py --direction 0 1 0 --length 0.06 --speed 0.02
  python icecream_jacobian_line.py --headless   # 无界面批量验证
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# ── 解析命令行（必须在 SimulationApp 之前）─────────────────────────────────
parser = argparse.ArgumentParser(
    description="Isaac Sim 雅可比直线验证（ice_cream_0208）",
    formatter_class=argparse.RawDescriptionHelpFormatter,
)
parser.add_argument("--usd",       type=str,   default=None,
                    help="机械臂 USD 路径（默认同目录 ice_cream_arm.usd）")
parser.add_argument("--urdf",      type=str,   default=None,
                    help="URDF 路径（可选，缺省用内置精确参数）")
parser.add_argument("--direction", type=float, nargs=3, default=[0., 1., 0.],
                    help="直线方向，默认 Y 轴 [0,1,0]")
parser.add_argument("--length",    type=float, default=0.08,
                    help="直线长度（m），默认 0.08")
parser.add_argument("--speed",     type=float, default=0.015,
                    help="末端沿直线方向速度（m/s），默认 0.015")
parser.add_argument("--speed-z",   type=float, default=0.005,
                    help="沿 Z 轴向下速度（m/s），与直线叠加，默认 0.005")
parser.add_argument("--damping",   type=float, default=0.01,
                    help="阻尼伪逆 λ，默认 0.01")
parser.add_argument("--stabilize-steps", type=int, default=120,
                    help="运动前至少稳定帧数，默认 120（≈2s）；画圆时会等到达 HOME 或超时")
parser.add_argument("--stabilize-max-steps", type=int, default=3000,
                    help="画圆时最多等待步数，超时未到 HOME 则报错退出，默认 3000（≈50s）")
parser.add_argument("--stabilize-kp", type=float, default=12.0,
                    help="回 HOME 时关节空间前馈增益，越大回得越快，默认 12")
parser.add_argument("--stabilize-max-dq", type=float, default=0.05,
                    help="回 HOME 时每步最大关节角变化(rad)，默认 0.05")
parser.add_argument("--settle-steps", type=int, default=100,
                    help="[circle] 到达 HOME 后再静置步数，待速度衰减后再画圆，默认 100（≈1.7s）")
parser.add_argument("--circle-ramp-steps", type=int, default=30,
                    help="[circle] 画圆起步线速度缓升步数，前几步低于满速以利收敛，默认 30")
parser.add_argument("--headless",  action="store_true",
                    help="无界面模式")
parser.add_argument("--mode",      type=str,   default="line",
                    choices=["line", "circle"],
                    help="轨迹模式: line=直线+Z向下, circle=HOME高度XOY平面画圆")
parser.add_argument("--radius",    type=float, default=0.03,
                    help="[circle] 圆半径（m），默认 0.03")
parser.add_argument("--circle-speed", type=float, default=0.01,
                    help="[circle] 沿圆线速度（m/s），默认 0.01")
parser.add_argument("--track-kp",    type=float, default=0.0,
                    help="轨迹跟踪位置误差增益；圆模式建议 3~8，0=纯前馈")
parser.add_argument("--track-kp-z",   type=float, default=0.0,
                    help="[circle] 高度 z 的跟踪增益，默认 1.0 保持同高；0=不修正 z")
parser.add_argument("--track-kp-ori", type=float, default=0.5,
                    help="[circle] 仅保持末端向下时的角速度增益，默认 0.5；不跟踪姿态轨迹")
args, _unknown = parser.parse_known_args()

# ── 启动 Isaac Sim ─────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": args.headless})

# ── 后续导入（依赖 SimulationApp 已启动）──────────────────────────────────
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
import omni.usd

# ── 同目录模块 ─────────────────────────────────────────────────────────────
_CODE_DIR = os.path.dirname(os.path.abspath(__file__))
if _CODE_DIR not in sys.path:
    sys.path.insert(0, _CODE_DIR)

from icecream_kinematics import (
    URDFKinematics,
    NUM_JOINTS,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    _rpy_to_matrix,
)

# ═══════════════════════════════════════════════════════════════════════════
# HOME 配置（URDF 工作空间分析结果）
# ═══════════════════════════════════════════════════════════════════════════

# 所有连杆 Z ≥ 0.105m（不碰地面）
# EE ≈ [0.494, 0.166, 0.482] m，σ_min=0.2055，条件数<2.1
Q_HOME = np.array([0.0, -1.271, 1.5, -0.5, 0.0], dtype=float)

SINGULARITY_THR = 0.01   # 最小奇异值低于此为奇异
PASS_DEV_TOL    = 0.001  # 1 mm 偏离容差（直线）
PASS_ERR_TOL    = 0.001  # 1 mm 位置误差容差（直线）
# 画圆通过容差（放宽）：偏离圆、位置误差低于此即通过
PASS_DEV_TOL_CIRCLE  = 0.015   # 15 mm 偏离圆
PASS_ERR_TOL_CIRCLE  = 0.015   # 15 mm 与理想圆周位置误差
# 画圆可选：关节角与 Q_HOME 误差低于此才允许开始（若需严格从 HOME 起画可保留）
STABILIZE_JOINT_TOL = 0.03

# 控制安全相关：接近奇异时自动减速 + 关节速度限幅（避免高速/大半径条件下数值发散）
NEAR_SINGULAR_THR = 3.0 * SINGULARITY_THR  # 小于此认为“接近奇异”，逐步缩放任务空间速度
MAX_JOINT_VEL = 1.0                         # 雅可比反解得到的关节速度范数上限（rad/s）
MIN_NEAR_SINGULAR_SCALE = 0.2              # 接近奇异时任务空间速度最小缩放系数


# ═══════════════════════════════════════════════════════════════════════════
# 工具：在场景中放置球体作为轨迹标记
# ═══════════════════════════════════════════════════════════════════════════

class TrajectoryMarker:
    """
    在 USD Stage 中创建小球标记实际轨迹（绿色）和理想轨迹（红色）。
    每隔 N 步放置一个球。
    """

    def __init__(self, stage, interval: int = 10):
        self._stage    = stage
        self._interval = interval
        self._step     = 0
        self._actual_count = 0
        self._ideal_count  = 0

    def _add_sphere(self, path: str, position: np.ndarray,
                    color: tuple, radius: float = 0.005) -> None:
        from pxr import UsdGeom, Gf
        sphere = UsdGeom.Sphere.Define(self._stage, path)
        sphere.CreateRadiusAttr(radius)
        sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xf = UsdGeom.Xformable(sphere)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))

    def tick(self, p_actual: np.ndarray, p_ideal: np.ndarray) -> None:
        self._step += 1
        if self._step % self._interval != 0:
            return
        self._add_sphere(
            f"/World/TrajectoryActual/m{self._actual_count}",
            p_actual, (0.1, 0.9, 0.2),   # 绿色
        )
        self._add_sphere(
            f"/World/TrajectoryIdeal/m{self._ideal_count}",
            p_ideal, (0.9, 0.2, 0.1),    # 红色
        )
        self._actual_count += 1
        self._ideal_count  += 1

    def add_start_end_markers(
        self, p_start: np.ndarray, p_end: np.ndarray
    ) -> None:
        """放置起点（蓝色大球）和理想终点（黄色大球）。"""
        self._add_sphere("/World/LineStart", p_start, (0.1, 0.3, 1.0), 0.010)
        self._add_sphere("/World/LineEnd",   p_end,   (1.0, 0.9, 0.1), 0.010)


# ═══════════════════════════════════════════════════════════════════════════
# 点到直线距离
# ═══════════════════════════════════════════════════════════════════════════

def point_to_line_dist(p, origin, direction):
    d = np.asarray(direction, float); d /= (np.linalg.norm(d) + 1e-12)
    v = np.asarray(p, float) - np.asarray(origin, float)
    return float(np.linalg.norm(v - np.dot(v, d) * d))


# 末端期望姿态：与 Z 轴平行、垂直于 XOY 向下（基座系下 EE z 轴 = -Z）
R_EE_DESIRED_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]], dtype=float)

# 基座系下“向下”方向
EE_DOWN_DIR = np.array([0.0, 0.0, -1.0], dtype=float)


def _angular_velocity_keep_down(R_cur: np.ndarray, kp: float) -> np.ndarray:
    """
    仅保持末端一根轴对准基座 -Z（向下），不约束整姿态。
    R_cur 为当前末端旋转矩阵 (3,3)，取与 [0,0,-1] 最接近的那一列为“指向轴”，
    返回使该轴对准向下的角速度 (3,)（基座系）。
    """
    # 取当前姿态中“最朝下”的那一列作为要约束的轴（避免依赖 URDF 具体是第几列）
    down = EE_DOWN_DIR
    dots = R_cur[2, :]  # 各列在基座 Z 上的分量，越负越朝下
    col = int(np.argmin(dots))  # 最朝下的列
    tool_axis = R_cur[:, col].ravel().astype(float)
    tool_axis = tool_axis / (np.linalg.norm(tool_axis) + 1e-12)
    dot_nd = float(np.dot(tool_axis, down))
    if dot_nd > 0.999:
        return np.zeros(3, dtype=float)
    if dot_nd < -0.999:
        # 几乎朝上，绕基座 X 转 180
        return kp * np.array([1.0, 0.0, 0.0], dtype=float)
    return kp * np.cross(tool_axis, down)


def _orientation_error_axis_angle(R_des: np.ndarray, R_cur: np.ndarray) -> np.ndarray:
    """R_des, R_cur 为 3x3，返回轴角表示的姿态误差 (3,)，单位 rad。"""
    R_err = R_des @ R_cur.T
    angle = np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-8:
        return np.zeros(3, dtype=float)
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ], dtype=float)
    return (axis / (2.0 * np.sin(angle) + 1e-12)) * angle


def point_to_circle_dist(p, cx: float, cy: float, z0: float, r: float) -> float:
    """点到 XOY 平面内圆的距离。圆: 中心 (cx, cy), 平面 z=z0, 半径 r。"""
    p = np.asarray(p, float).ravel()[:3]
    dx = p[0] - cx
    dy = p[1] - cy
    d_xy = np.sqrt(dx * dx + dy * dy) + 1e-12
    # 圆上最近点在该射线 (cx,cy) -> (px,py) 上
    closest_x = cx + r * (dx / d_xy)
    closest_y = cy + r * (dy / d_xy)
    return float(np.sqrt((p[0] - closest_x) ** 2 + (p[1] - closest_y) ** 2 + (p[2] - z0) ** 2))


# ═══════════════════════════════════════════════════════════════════════════
# 主逻辑
# ═══════════════════════════════════════════════════════════════════════════

def main():
    # ── 运动学（解析雅可比）────────────────────────────────────────────────
    urdf_path = args.urdf or _find_urdf()
    kin = URDFKinematics(urdf_path=urdf_path)
    print(f"[kinematics] 参数来源: {kin._source} | {kin._urdf_path}")

    # ── 方向向量与合成速度（直线模式）────────────────────────────────────────
    direction = np.array(args.direction, dtype=float)
    direction = direction / (np.linalg.norm(direction) + 1e-12)
    v_ee_nominal = args.speed * direction + args.speed_z * np.array([0.0, 0.0, -1.0])
    direction_line = v_ee_nominal / (np.linalg.norm(v_ee_nominal) + 1e-12)
    duration_move_line = args.length / (args.speed + 1e-12)

    # ── 预检：路径奇异性 ────────────────────────────────────────────────────
    print(f"\n[precheck] HOME 关节角: {Q_HOME}")
    p_home = kin.forward_kinematics_position(Q_HOME)
    sv_home_min, cond_home = kin.manipulability(Q_HOME)
    print(f"[precheck] HOME 末端位置(基座系): {p_home}")
    print(f"[precheck] HOME σ_min={sv_home_min:.4f}  κ={cond_home:.2f}")
    # 记录 HOME 时刻的末端姿态，后续画圆时以此姿态为基准（始终保持该姿态）
    R_home = kin.forward_kinematics_rotation(Q_HOME)

    if args.mode == "line":
        path_sv_min, path_ok = _check_path_singularity(
            kin, Q_HOME, v_ee_nominal, duration_move_line
        )
    else:
        # 圆：HOME 在圆边上，圆心在 HOME 左侧 (cx = p_home[0]-r)，逆时针
        circle_cx_pre = p_home[0] - args.radius
        circle_cy_pre = p_home[1]
        circle_z0_pre = p_home[2]
        path_sv_min, path_ok = _check_circle_singularity(
            kin, Q_HOME, circle_cx_pre, circle_cy_pre, circle_z0_pre,
            args.radius, args.circle_speed,
        )
    if not path_ok:
        print(f"[precheck] ⚠ 路径包含奇异点！σ_min={path_sv_min:.4f}，请换参数。")
    else:
        print(f"[precheck] ✓ 路径无奇异  σ_min={path_sv_min:.4f}")

    # ── 自动调参：根据速度/半径/路径可操纵性调整阻尼与轨迹 Kp（用户显式传参则不覆盖）──
    damping_used = args.damping
    track_kp_line = args.track_kp if args.track_kp > 0.0 else 0.0
    track_kp_circle_xy = args.track_kp if args.track_kp > 0.0 else 0.0
    track_kp_circle_z = args.track_kp_z if args.track_kp_z > 0.0 else 0.0
    track_kp_ori = args.track_kp_ori
    circle_ramp_steps_used = args.circle_ramp_steps

    # 等效速度标度：直线用合成 v_ee，圆用 circle-speed
    if args.mode == "line":
        speed_eff = float(np.linalg.norm(v_ee_nominal))
        speed_ref = 0.015  # 直线默认速度参考
    else:
        speed_eff = float(abs(args.circle_speed))
        speed_ref = 0.01   # 圆默认速度参考

    # 1) 阻尼 λ 自适应：默认值 0.01 时，随着速度增大和路径 σ_min 变小自动增大阻尼
    if abs(args.damping - 0.01) < 1e-9:
        speed_scale = max(1.0, speed_eff / (speed_ref + 1e-12))
        sv_scale = 1.0
        if path_sv_min > 1e-4:
            # σ_min 小 → sv_scale 大 → 阻尼更大
            sv_scale = min(3.0, max(0.5, 0.05 / path_sv_min))
        factor = min(3.0, max(1.0, float(np.sqrt(speed_scale * sv_scale))))
        damping_used = args.damping * factor

    # 2) 轨迹 Kp 自适应（仅在用户未显式传参时自动选择）
    if args.mode == "line":
        if args.track_kp <= 0.0:
            # 速度越大，Kp 适当减小以增强稳定性
            speed_scale = max(0.5, min(3.0, speed_eff / (speed_ref + 1e-12)))
            track_kp_line = 4.0 / speed_scale
    else:
        if args.track_kp <= 0.0:
            speed_scale = max(0.5, min(3.0, speed_eff / (speed_ref + 1e-12)))
            k_xy = 5.0 / speed_scale
            if path_sv_min > 1e-4:
                # 路径越接近奇异点（σ_min 小），位置环 Kp 适度减小
                k_xy *= min(1.0, path_sv_min / 0.08)
            track_kp_circle_xy = float(np.clip(k_xy, 2.0, 8.0))
        if args.track_kp_z <= 0.0:
            speed_scale = max(0.5, min(3.0, speed_eff / (speed_ref + 1e-12)))
            k_z = 1.0 / speed_scale
            track_kp_circle_z = float(np.clip(k_z, 0.5, 2.0))
        # 姿态 Kp：仅在保持默认 0.5 时，根据速度略作缩放
        if abs(args.track_kp_ori - 0.5) < 1e-9:
            speed_scale = max(0.5, min(3.0, speed_eff / (speed_ref + 1e-12)))
            k_ori = 0.5 / speed_scale
            track_kp_ori = float(np.clip(k_ori, 0.3, 0.8))

        # 3) 圆轨迹加速段步数自适应：默认值 30 时，按整圈步数的 5%~15% 估计
        if args.circle_ramp_steps == 30:
            duration_circle_est = 2.0 * np.pi * args.radius / (abs(args.circle_speed) + 1e-12)
            approx_steps = max(1, int(duration_circle_est / (1.0 / 60.0)))
            circle_ramp_steps_used = int(np.clip(0.1 * approx_steps, 10, 200))

    # ── 构建 Isaac Sim 场景 ─────────────────────────────────────────────────
    physics_dt = 1.0 / 60.0
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=physics_dt,
        rendering_dt=physics_dt,
    )
    world.scene.add_default_ground_plane()

    # 加载机械臂 USD
    usd_path = _resolve_usd(args.usd)
    arm_prim_path = "/World/IceCreamArm"
    add_reference_to_stage(usd_path=usd_path, prim_path=arm_prim_path)
    arm = world.scene.add(
        SingleArticulation(arm_prim_path, name="ice_cream_arm")
    )

    world.reset()

    # PD 增益
    ctrl = arm.get_articulation_controller()
    try:
        ctrl.set_gains(
            kps=np.full(NUM_JOINTS, 1e5, dtype=float),
            kds=np.full(NUM_JOINTS, 1e4, dtype=float),
        )
    except Exception:
        pass

    # ── 轨迹标记器 ─────────────────────────────────────────────────────────
    stage = omni.usd.get_context().get_stage()
    marker = TrajectoryMarker(stage, interval=8)

    # ── 阶段状态机 ─────────────────────────────────────────────────────────
    PHASE_STABILIZE  = "STABILIZE"   # 运动到 HOME 并稳定
    PHASE_MOVE       = "MOVE"        # 沿直线运动
    PHASE_DONE       = "DONE"        # 完成，保持并打印报告

    phase       = PHASE_STABILIZE
    stab_step   = 0
    settle_step = 0   # 到达 HOME 后连续静置步数（仅画圆用）
    move_step   = 0

    # 轨迹参数（从仿真读取的起点）
    p_line_start  = None
    p_ideal_final = None
    total_steps   = None
    dt_move       = None
    v_ee          = None
    # [circle] 圆心 (cx,cy)、平面高度 z0、半径、线速度
    circle_cx = circle_cy = circle_z0 = circle_r = circle_speed = None

    # 统计数据
    deviations  = []
    pos_errors  = []
    sv_mins     = []
    q_sim_log   = []   # 从仿真实际读取的关节角
    p_sim_log   = []   # 从仿真实际读取的末端位置（FK）

    # 当前控制目标关节角（用于积分）
    q_cmd = Q_HOME.copy()

    if args.mode == "line":
        print(f"\n[sim] 轨迹=直线 | 方向={direction} | 长度={args.length}m | "
              f"速度={args.speed}m/s" + (f" | Z向下={args.speed_z}m/s" if args.speed_z != 0 else ""))
        if track_kp_line > 0.0:
            print(f"      线段位置 Kp(auto)={track_kp_line:.3f}")
    else:
        print(f"\n[sim] 轨迹=圆(XOY平面) | 半径={args.radius*100:.1f}cm | "
              f"线速度={args.circle_speed*100:.1f}cm/s")
        print(f"      画圆前先回 HOME（前馈 kp={args.stabilize_kp}），到 HOME 后再静置 {args.settle_steps} 步再画圆")
        print(f"      圆轨迹 Kp_xy(auto)={track_kp_circle_xy:.3f}  Kp_z(auto)={track_kp_circle_z:.3f}  Kp_ori={track_kp_ori:.3f}")
        print(f"      圆轨迹加速步数 circle_ramp_steps={circle_ramp_steps_used}")
    print(f"      阻尼λ={damping_used:.4f} | 最少稳定帧数={args.stabilize_steps}")
    print()

    # ── 主循环 ─────────────────────────────────────────────────────────────
    while simulation_app.is_running():
        world.step(render=not args.headless)

        if world.is_stopped():
            world.reset()
            phase       = PHASE_STABILIZE
            stab_step   = 0
            settle_step = 0
            move_step   = 0
            q_cmd       = Q_HOME.copy()
            deviations.clear(); pos_errors.clear()
            sv_mins.clear(); q_sim_log.clear(); p_sim_log.clear()
            continue

        if not world.is_playing():
            continue

        # ── 从仿真读取当前关节角（真实物理状态）──────────────────────────
        q_sim_raw = arm.get_joint_positions()
        if q_sim_raw is None:
            continue
        q_sim = np.array(q_sim_raw, dtype=float).ravel()[:NUM_JOINTS]

        # 用 FK 计算当前末端位置（基座系）
        p_sim = kin.forward_kinematics_position(q_sim)

        # ══════════════════════════════════════════════════════════════════
        # PHASE 1: 稳定到 HOME
        # ══════════════════════════════════════════════════════════════════
        if phase == PHASE_STABILIZE:
            stab_step += 1
            q_err = np.linalg.norm(q_sim - Q_HOME)
            at_home = q_err < STABILIZE_JOINT_TOL

            if not at_home:
                settle_step = 0
                # 关节空间前馈：每步朝 Q_HOME 移动，加快回 HOME
                dq = np.clip(
                    args.stabilize_kp * (Q_HOME - q_sim) * physics_dt,
                    -args.stabilize_max_dq, args.stabilize_max_dq,
                )
                q_cmd = np.clip(q_sim + dq, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
                ctrl.apply_action(ArticulationAction(joint_positions=q_cmd.tolist()))
            else:
                # 已到 HOME：画圆模式先静置若干步再动，避免残余速度叠加
                settle_step += 1
                ctrl.apply_action(ArticulationAction(joint_positions=Q_HOME.tolist()))

            if stab_step % 30 == 0:
                p_cur = kin.forward_kinematics_position(q_sim)
                print(f"[stabilize] step={stab_step:4d} | "
                      f"关节误差={q_err:.4f} rad | "
                      f"EE=[{p_cur[0]:.3f},{p_cur[1]:.3f},{p_cur[2]:.3f}]"
                      + (f" | 静置={settle_step}/{args.settle_steps}" if at_home and args.mode == "circle" else ""))

            # 直线：满步数即开始；画圆：必须先到 HOME 且静置满 settle_steps 才允许开始
            min_steps_done = stab_step >= args.stabilize_steps
            can_start_line = min_steps_done
            can_start_circle = min_steps_done and at_home and (settle_step >= args.settle_steps)
            if args.mode == "circle" and not at_home and stab_step >= args.stabilize_max_steps:
                print(f"\n[stabilize] 在 {args.stabilize_max_steps} 步内未到达 HOME（当前误差 {q_err:.4f} rad）")
                print(f"           画圆必须以 HOME 为起点，请增大 --stabilize-max-steps 或检查关节控制/PD 增益。")
                phase = PHASE_DONE
                continue

            if (args.mode == "line" and can_start_line) or (args.mode == "circle" and can_start_circle):
                p_line_start  = kin.forward_kinematics_position(q_sim).copy()
                dt_move       = physics_dt
                q_cmd         = q_sim.copy()

                if args.mode == "line":
                    p_ideal_final = p_line_start + duration_move_line * v_ee_nominal
                    total_steps   = max(1, int(duration_move_line / dt_move))
                    v_ee          = v_ee_nominal.copy()
                    marker.add_start_end_markers(p_line_start, p_ideal_final)
                    print(f"\n[move] 开始直线+Z向下运动")
                    print(f"       起点: {p_line_start}  理想终点: {p_ideal_final}")
                    print(f"       v_ee = 直线 {args.speed}m/s + Z向下 {args.speed_z}m/s")
                else:
                    # 圆：以 HOME 为圆上一点（圆心在 HOME 左侧 radius），逆时针画整圆，无多余动作
                    circle_r      = args.radius
                    circle_speed  = args.circle_speed
                    circle_cx     = float(p_home[0] - circle_r)  # HOME 在圆上 = (cx+r, cy) => cx = p_home[0]-r
                    circle_cy     = float(p_home[1])
                    circle_z0     = float(p_home[2])
                    duration_circle = 2.0 * np.pi * circle_r / (circle_speed + 1e-12)
                    total_steps   = 1 + max(0, int(duration_circle / dt_move))  # 步 1 为 angle=0（HOME）
                    p_ideal_final = p_home.copy()
                    v_ee          = np.array([0.0, circle_speed, 0.0], dtype=float)
                    marker.add_start_end_markers(p_home, np.array([circle_cx + circle_r * np.cos(0.5*np.pi), circle_cy + circle_r * np.sin(0.5*np.pi), circle_z0]))
                    print(f"\n[move] 已到 HOME，以 HOME 为圆上一点逆时针画圆（半径 {circle_r*100:.1f}cm）")
                    print(f"       圆心: ({circle_cx:.4f}, {circle_cy:.4f})  z={circle_z0:.3f}m  线速度: {circle_speed*100:.1f}cm/s")
                print(f"       总步数: {total_steps} | dt={dt_move*1000:.2f}ms")
                print(f"       {'步':>5s} | {'偏离(mm)':>10s} | {'误差(mm)':>10s} | "
                      f"{'σ_min':>8s} | {'EE实际(m)':>32s}")
                print(f"       {'-'*75}")
                phase = PHASE_MOVE

        # ══════════════════════════════════════════════════════════════════
        # PHASE 2: 雅可比直线运动
        # ══════════════════════════════════════════════════════════════════
        elif phase == PHASE_MOVE:
            move_step += 1

            J_full = kin.jacobian_analytical(q_sim)
            J_pos  = J_full[:3, :]

            # 奇异性检查
            sv    = np.linalg.svd(J_pos, compute_uv=False)
            sv_min_cur = float(sv.min())
            sv_mins.append(sv_min_cur)

            if sv_min_cur < SINGULARITY_THR:
                print(f"\n[move] ⚠ 步 {move_step}: 接近奇异！σ_min={sv_min_cur:.4f}")

            if args.mode == "circle":
                # 逆时针：步 1 对应 angle=0（HOME），angle = omega * (move_step-1) * dt
                angle   = (circle_speed / (circle_r + 1e-12)) * ((move_step - 1) * dt_move)
                p_ideal = np.array([
                    circle_cx + circle_r * np.cos(angle),
                    circle_cy + circle_r * np.sin(angle),
                    circle_z0,
                ], dtype=float)
                # 起步缓升：前 circle_ramp_steps 步线速度从 0.3 倍升到 1 倍，减轻残余速度叠加
                ramp = circle_ramp_steps_used
                if move_step <= ramp and ramp > 0:
                    speed_scale = 0.3 + 0.7 * (move_step - 1) / max(1, ramp - 1)
                else:
                    speed_scale = 1.0
                v_linear = np.array([
                    -speed_scale * circle_speed * np.sin(angle),
                    speed_scale * circle_speed * np.cos(angle),
                    0.0,
                ], dtype=float)
                v_linear = v_linear + track_kp_circle_xy * np.array(
                    [p_ideal[0] - p_sim[0], p_ideal[1] - p_sim[1], 0.0], dtype=float
                )
                v_linear = v_linear + track_kp_circle_z * np.array(
                    [0.0, 0.0, circle_z0 - p_sim[2]], dtype=float
                )
                # 姿态：不重新规划姿态轨迹，而是始终保持 HOME 时的末端姿态 R_home
                R_cur = kin.forward_kinematics_rotation(q_sim)
                err_ori = _orientation_error_axis_angle(R_home, R_cur)
                v_angular = track_kp_ori * err_ori
                twist = np.concatenate([v_linear, v_angular])
                dev = point_to_circle_dist(p_sim, circle_cx, circle_cy, circle_z0, circle_r)
            else:
                s       = move_step * dt_move
                p_ideal = p_line_start + s * v_ee
                dev     = point_to_line_dist(p_sim, p_line_start, direction_line)
                if track_kp_line > 0.0:
                    v_ee = v_ee + track_kp_line * (p_ideal - p_sim)
                twist = np.asarray(v_ee, dtype=float)
            err = float(np.linalg.norm(p_sim - p_ideal))

            # 构造用于 DLS 的雅可比与阻尼矩阵
            if args.mode == "circle":
                J_use = J_full
                M = J_use @ J_use.T + (damping_used ** 2) * np.eye(6)
            else:
                J_use = J_pos
                M = J_use @ J_use.T + (damping_used ** 2) * np.eye(3)

            # 接近奇异时自动缩放任务空间速度，避免在小奇异值方向给出过大的关节速度
            twist_used = twist
            if sv_min_cur < NEAR_SINGULAR_THR:
                # α 从 1.0 线性降到 MIN_NEAR_SINGULAR_SCALE
                alpha = (sv_min_cur - SINGULARITY_THR) / max(
                    1e-6, (NEAR_SINGULAR_THR - SINGULARITY_THR)
                )
                alpha = float(np.clip(alpha, MIN_NEAR_SINGULAR_SCALE, 1.0))
                twist_used = twist * alpha

            dq = J_use.T @ np.linalg.solve(M, twist_used)

            # 关节速度范数限幅（rad/s），在高线速度/大半径参数下保持数值稳定
            dq_norm = float(np.linalg.norm(dq))
            if dq_norm > MAX_JOINT_VEL:
                dq *= MAX_JOINT_VEL / (dq_norm + 1e-12)

            # 积分更新命令角（以 q_cmd 为基础，保持平滑）
            q_cmd = np.clip(q_cmd + dq * dt_move,
                            JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

            # 下发位置控制
            ctrl.apply_action(
                ArticulationAction(joint_positions=q_cmd.tolist())
            )

            deviations.append(dev)
            pos_errors.append(err)
            q_sim_log.append(q_sim.copy())
            p_sim_log.append(p_sim.copy())
            marker.tick(p_sim, p_ideal)

            if move_step % 30 == 0 or move_step == total_steps:
                print(f"       {move_step:>5d} | {dev*1000:>10.4f} | {err*1000:>10.4f} | "
                      f"{sv_min_cur:>8.4f} | "
                      f"[{p_sim[0]:.4f},{p_sim[1]:.4f},{p_sim[2]:.4f}]")

            if move_step >= total_steps:
                phase = PHASE_DONE

        # ══════════════════════════════════════════════════════════════════
        # PHASE 3: 完成，打印报告
        # ══════════════════════════════════════════════════════════════════
        elif phase == PHASE_DONE:
            # 保持最后位置
            ctrl.apply_action(
                ArticulationAction(joint_positions=q_cmd.tolist())
            )

            # 只打印一次报告
            if len(deviations) > 0:
                if args.mode == "circle":
                    _print_report_circle(
                        deviations, pos_errors, sv_mins,
                        circle_cx, circle_cy, circle_z0, circle_r, circle_speed,
                        kin.forward_kinematics_position(q_sim),
                    )
                else:
                    seg_len = float(np.linalg.norm(p_ideal_final - p_line_start))
                    _print_report(
                        deviations, pos_errors, sv_mins,
                        p_line_start, p_ideal_final,
                        kin.forward_kinematics_position(q_sim),
                        direction_line, seg_len, args.speed, args.speed_z,
                    )
                deviations.clear()   # 清空避免重复打印

    simulation_app.close()


# ═══════════════════════════════════════════════════════════════════════════
# 汇总报告
# ═══════════════════════════════════════════════════════════════════════════

def _print_report(
    deviations, pos_errors, sv_mins,
    p_start, p_ideal_end, p_actual_end,
    direction, length, speed, speed_z=0.0,
):
    dev  = np.array(deviations)
    err  = np.array(pos_errors)
    sv   = np.array(sv_mins)

    passed_dev = float(dev.max()) < PASS_DEV_TOL
    passed_err = float(err.max()) < PASS_ERR_TOL

    print()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   Isaac Sim 雅可比直线+Z向下验证报告                        ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"  合成方向: {direction}  路径长: {length*100:.1f}cm")
    print(f"  速度: 直线 {speed*100:.1f}cm/s" + (f"  Z向下 {speed_z*100:.1f}cm/s" if speed_z != 0 else ""))
    print(f"  起点(从仿真读取): [{p_start[0]:.4f},{p_start[1]:.4f},{p_start[2]:.4f}]")
    print(f"  理想终点:         [{p_ideal_end[0]:.4f},{p_ideal_end[1]:.4f},{p_ideal_end[2]:.4f}]")
    print(f"  实际终点(仿真FK): [{p_actual_end[0]:.4f},{p_actual_end[1]:.4f},{p_actual_end[2]:.4f}]")
    print()
    print(f"  ── 偏离直线（点到直线距离） ──")
    print(f"     最大: {dev.max()*1000:.4f} mm   "
          f"{'✓ OK' if passed_dev else '✗ FAIL'}（阈值 {PASS_DEV_TOL*1000:.0f} mm）")
    print(f"     平均: {dev.mean()*1000:.4f} mm")
    print(f"     终点: {dev[-1]*1000:.4f} mm")
    print()
    print(f"  ── 与理想直线位置误差 ──")
    print(f"     最大: {err.max()*1000:.4f} mm   "
          f"{'✓ OK' if passed_err else '✗ FAIL'}（阈值 {PASS_ERR_TOL*1000:.0f} mm）")
    print(f"     平均: {err.mean()*1000:.4f} mm")
    print(f"     终点: {err[-1]*1000:.4f} mm")
    print()
    print(f"  ── 路径可操纵性 ──")
    print(f"     最小奇异值: {sv.min():.4f}   "
          f"{'✓ 无奇异' if sv.min() > SINGULARITY_THR else '⚠ 接近奇异'}")
    print(f"     平均奇异值: {sv.mean():.4f}")
    print()

    overall = passed_dev and passed_err
    if overall:
        print("  ════════════════════════════════════════")
        print("  ✓ 验证通过：雅可比矩阵正确，末端走直线")
        print("  ════════════════════════════════════════")
    else:
        print("  ✗ 验证未完全通过，偏差超出容差。")
        print("    建议：减小 --speed，增大 --stabilize-steps，")
        print("    或调小 --damping（当前可能过阻尼）。")
    print()


def _print_report_circle(
    deviations, pos_errors, sv_mins,
    cx, cy, z0, radius, circle_speed,
    p_actual_end,
):
    """画圆轨迹验证报告。"""
    dev = np.array(deviations)
    err = np.array(pos_errors)
    sv  = np.array(sv_mins)
    passed_dev = float(dev.max()) < PASS_DEV_TOL_CIRCLE
    passed_err = float(err.max()) < PASS_ERR_TOL_CIRCLE

    print()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   Isaac Sim 雅可比画圆验证报告（XOY 平面、末端垂直向下同高）║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"  圆心: ({cx:.4f}, {cy:.4f})  平面高度 z={z0:.4f}m  半径: {radius*100:.1f}cm")
    print(f"  线速度: {circle_speed*100:.1f}cm/s")
    print(f"  实际终点(仿真FK): [{p_actual_end[0]:.4f},{p_actual_end[1]:.4f},{p_actual_end[2]:.4f}]")
    print()
    print(f"  ── 偏离圆（点到圆距离） ──")
    print(f"     最大: {dev.max()*1000:.4f} mm   "
          f"{'✓ OK' if passed_dev else '✗ FAIL'}（阈值 {PASS_DEV_TOL_CIRCLE*1000:.0f} mm）")
    print(f"     平均: {dev.mean()*1000:.4f} mm")
    print(f"     终点: {dev[-1]*1000:.4f} mm")
    print()
    print(f"  ── 与理想圆周位置误差 ──")
    print(f"     最大: {err.max()*1000:.4f} mm   "
          f"{'✓ OK' if passed_err else '✗ FAIL'}（阈值 {PASS_ERR_TOL_CIRCLE*1000:.0f} mm）")
    print(f"     平均: {err.mean()*1000:.4f} mm")
    print()
    print(f"  ── 路径可操纵性 ──")
    print(f"     最小奇异值: {sv.min():.4f}   "
          f"{'✓ 无奇异' if sv.min() > SINGULARITY_THR else '⚠ 接近奇异'}")
    print(f"     平均奇异值: {sv.mean():.4f}")
    print()
    if passed_dev and passed_err:
        print("  ════════════════════════════════════════")
        print("  ✓ 验证通过：雅可比正确，末端沿圆轨迹")
        print("  ════════════════════════════════════════")
    else:
        print("  ✗ 验证未完全通过。可调 --track-kp / --track-kp-z / --track-kp-ori 或减小 --circle-speed/--radius。")
    print()


# ═══════════════════════════════════════════════════════════════════════════
# 辅助
# ═══════════════════════════════════════════════════════════════════════════

def _find_urdf() -> str | None:
    candidates = [
        os.path.join(_CODE_DIR, "ice_cream_0208_SLDASM.urdf"),
        os.path.join(os.path.dirname(_CODE_DIR),
                     "ice_cream_0208.SLDASM", "urdf",
                     "ice_cream_0208.SLDASM.urdf"),
    ]
    return next((p for p in candidates if os.path.isfile(p)), None)


def _resolve_usd(usd_arg: str | None) -> str:
    if usd_arg is not None:
        path = os.path.abspath(usd_arg)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"USD 文件不存在: {path}")
        return path
    # 自动查找同目录的 ice_cream_arm.usd
    candidates = [
        os.path.join(_CODE_DIR, "ice_cream_arm.usd"),
        os.path.join(_CODE_DIR, "ice_cream_0208.SLDASM", "ice_cream_arm.usd"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "未找到 ice_cream_arm.usd，请用 --usd 指定路径，"
        "或先运行 urdf_to_usd.py 生成 USD 文件。"
    )


def _check_path_singularity(kin, q_start, v_ee, duration,
                             dt=1/200.0, damping=0.01):
    """离线预检路径奇异性。v_ee 为合成笛卡尔速度，duration 为运动时长(s)。"""
    n   = max(1, int(duration / dt))
    q   = q_start.copy()
    min_sv = 1e9
    for _ in range(n):
        J  = kin.jacobian_analytical(q)[:3, :]
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv = min(min_sv, float(sv.min()))
        if sv.min() < SINGULARITY_THR:
            return min_sv, False
        M  = J @ J.T + damping ** 2 * np.eye(3)
        dq = J.T @ np.linalg.solve(M, v_ee)
        q  = np.clip(q + dq * dt, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    return min_sv, True


def _check_circle_singularity(kin, q_start, cx: float, cy: float, z0: float,
                              radius: float, circle_speed: float,
                              dt=1/200.0, damping=0.01):
    """离线预检圆周路径奇异性。圆在平面 z=z0，中心(cx,cy)，半径 radius。"""
    duration = 2.0 * np.pi * radius / (circle_speed + 1e-12)
    n = max(1, int(duration / dt))
    q = q_start.copy()
    min_sv = 1e9
    for i in range(n):
        t = i * dt
        angle = (circle_speed / (radius + 1e-12)) * t
        v_ee = np.array([
            -circle_speed * np.sin(angle),
            circle_speed * np.cos(angle),
            0.0,
        ], dtype=float)
        J = kin.jacobian_analytical(q)[:3, :]
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv = min(min_sv, float(sv.min()))
        if sv.min() < SINGULARITY_THR:
            return min_sv, False
        M = J @ J.T + damping ** 2 * np.eye(3)
        dq = J.T @ np.linalg.solve(M, v_ee)
        q = np.clip(q + dq * dt, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
    return min_sv, True


if __name__ == "__main__":
    main()