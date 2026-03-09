#!/usr/bin/env python3
"""
3-DOF 直线跟踪测试（几何解耦策略）
==================================
ice_cream_0208.SLDASM  5-DOF 机械臂，仅用前 3 关节控位置；q4 = -(q2+q3) 使末节垂直 XOY；q5 固定。

Geometric Decoupling Strategy
-----------------------------
  • 任务空间：仅 3D 位置 (x,y,z)，跟踪点为 joint4 电机中心。
  • 雅可比：J ∈ R^{3×3}，该点位置对 q1,q2,q3 的雅可比。
  • q4 几何约束：每步发送 q4 = -(q2+q3)，保证最后一节与 XOY 平面垂直；q5 固定于 HOME。

功能
----
1. 从 Isaac Sim 读取关节角，用 link4 点位置 + J_link4_first3 做 DLS 速度控制
2. 仅前 3 轴更新；q4、q5 保持 HOME（或强拉回），实现几何解耦
3. 绿/红球标记轨迹，打印偏离与误差报告

HOME 位置
  q_home = [0, -1.271, 1.5, -0.5, 0]  rad
  跟踪点（与 link4 重合）目标 z = TRACK_HEIGHT + LINK5_LENGTH（末节向下时末端在 TRACK_HEIGHT）

轨迹模式
  line - 直线：回 HOME 后跟踪点沿直线到地面目标点上方；绿/红球标记并打印报告

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
parser.add_argument("--speed",     type=float, default=0.1,
                    help="末端沿直线方向速度（m/s），默认 0.015")
parser.add_argument("--speed-z",   type=float, default=0.01,
                    help="沿 Z 轴向下速度（m/s），与直线叠加，默认 0.005")
parser.add_argument("--damping",   type=float, default=0.01,
                    help="阻尼伪逆 λ，默认 0.01")
parser.add_argument("--stabilize-steps", type=int, default=120,
                    help="运动前至少稳定帧数，默认 120（≈2s）")
parser.add_argument("--stabilize-max-steps", type=int, default=3000,
                    help="回 HOME 最多等待步数，超时则进入 DONE，默认 3000")
parser.add_argument("--stabilize-kp", type=float, default=12.0,
                    help="回 HOME 时关节空间前馈增益，默认 12")
parser.add_argument("--stabilize-max-dq", type=float, default=0.05,
                    help="回 HOME 时每步最大关节角变化(rad)，默认 0.05")
parser.add_argument("--headless",  action="store_true",
                    help="无界面模式")
parser.add_argument("--mode",      type=str,   default="line",
                    choices=["line"],
                    help="轨迹模式: line=直线（HOME 到地面目标点上方）")
parser.add_argument("--track-kp",    type=float, default=0.0,
                    help="轨迹跟踪位置误差增益，0=纯前馈")
parser.add_argument("--track-kp-z",   type=float, default=0.0,
                    help="高度 z 的跟踪增益，0=不修正")
parser.add_argument("--track-kp-ori", type=float, default=0.5,
                    help="保持末端向下时的角速度增益，默认 0.5")
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

# ── 路径：脚本所在目录与项目根（便于解析 USD/URDF）────────────────────────────
_CODE_DIR = os.path.dirname(os.path.abspath(os.path.realpath(__file__)))
_PROJECT_ROOT = os.path.dirname(_CODE_DIR)
if _CODE_DIR not in sys.path:
    sys.path.insert(0, _CODE_DIR)

from icecream_kinematics import (
    URDFKinematics,
    NUM_JOINTS,
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    LINK5_LENGTH,
    JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4,
    q4_geometric_decouple,
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
STABILIZE_JOINT_TOL = 0.03  # 关节角与 Q_HOME 误差低于此视为到达 HOME

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
        """p_actual: 实际跟随点（电机中心），绿色球。p_ideal: 理想轨迹点，红色球。"""
        self._step += 1
        if self._step % self._interval != 0:
            return
        self._add_sphere(
            f"/World/TrajectoryActual/m{self._actual_count}",
            p_actual, (0.1, 0.9, 0.2),   # 绿色 = 电机中心
        )
        self._add_sphere(
            f"/World/TrajectoryIdeal/m{self._ideal_count}",
            p_ideal, (0.9, 0.2, 0.1),   # 红色 = 理想轨迹
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

# 基座系下“向下”方向（初始化为 -Z，运行时会用 HOME 姿态更新）
EE_DOWN_DIR = np.array([0.0, 0.0, -1.0], dtype=float)

# 跟踪用目标高度（相对于地面的末端高度）
TRACK_HEIGHT = 0.3

# 重新规划直线的时间间隔（秒）及对应步数
TRACK_INTERVAL_SEC = 1.0


def _angular_velocity_keep_down(R_cur: np.ndarray, kp: float) -> np.ndarray:
    """
    保持末端 z 轴（R 第三列）对准世界 -Z（竖直向下）。
    R_cur 为当前末端旋转矩阵 (3,3)，返回使该轴对准 [0,0,-1] 的角速度 (3,)（基座系）。
    """
    # 固定用末端 z 轴（第三列），目标为世界 -Z，保证 link5 朝下而非朝上
    tool_axis = R_cur[:, 2].ravel().astype(float)
    tool_axis = tool_axis / (np.linalg.norm(tool_axis) + 1e-12)
    down = np.array([0.0, 0.0, -1.0], dtype=float)
    dot_nd = float(np.dot(tool_axis, down))
    if dot_nd > 0.999:
        return np.zeros(3, dtype=float)
    if dot_nd < -0.999:
        # 几乎朝上（+Z），绕基座 X 转 180°
        return kp * np.array([1.0, 0.0, 0.0], dtype=float)
    return kp * np.cross(tool_axis, down)


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

    # ── 预检：路径奇异性（3-DOF 跟踪点 = joint4 电机中心，q4=-(q2+q3)，q5 固定）────────────────
    print(f"\n[precheck] HOME 关节角: {Q_HOME}")
    p_home = kin.forward_kinematics_position(Q_HOME)
    p_link4_origin = kin.forward_kinematics_position_link4(Q_HOME)
    p_track_home = kin.forward_kinematics_position_joint4_motor_center(Q_HOME)
    sv_home_min, cond_home = kin.manipulability(Q_HOME)
    print(f"[precheck] joint4 电机中心偏移(link4系)= {JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4} m")
    print(f"[precheck] HOME link4 原点(转轴)位置: {p_link4_origin}")
    print(f"[precheck] HOME 跟踪点(电机中心)位置:  {p_track_home}")
    print(f"[precheck] HOME 末端(link5)位置:      {p_home}")
    print(f"[precheck] LINK5_LENGTH={LINK5_LENGTH:.4f}m（目标 z = TRACK_HEIGHT + LINK5_LENGTH）")
    print(f"[precheck] HOME σ_min={sv_home_min:.4f}  κ={cond_home:.2f}")
    R_home = kin.forward_kinematics_rotation(Q_HOME)
    # 末端“向下”固定为世界系 -Z，不随 HOME 姿态改（保证 link5 相对世界 z 轴平行）
    EE_DOWN_DIR[:] = np.array([0.0, 0.0, -1.0], dtype=float)

    path_sv_min, path_ok = _check_path_singularity_track_point(
        kin, Q_HOME, v_ee_nominal, duration_move_line
    )
    if not path_ok:
        print(f"[precheck] ⚠ 路径包含奇异点！σ_min={path_sv_min:.4f}，请换参数。")
    else:
        print(f"[precheck] ✓ 路径无奇异  σ_min={path_sv_min:.4f}")

    # ── 轨迹控制增益
    # 直线段位置 Kp：高速时纯前馈易漂移，用 Kp 把末端拉回理想直线；0.01 m/s 可不用，≥0.05 建议有
    damping_used = args.damping
    if args.track_kp > 0.0:
        track_kp_line = args.track_kp
    else:
        # 速度高时默认加一点位置反馈，减少重规划次数、加快收敛
        track_kp_line = 4.0 if args.speed >= 0.05 else 0.0
    track_kp_ori = args.track_kp_ori

    # ── 构建 Isaac Sim 场景 ─────────────────────────────────────────────────
    physics_dt = 1.0 / 60.0
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=physics_dt,
        rendering_dt=physics_dt,
    )
    # 地面：仅用固定 isaacsim_assets 路径；不存在则回退到程序建地面
    stage_for_scene = omni.usd.get_context().get_stage()
    if os.path.isfile(DEFAULT_GROUND_USD_PATH):
        add_reference_to_stage(usd_path=DEFAULT_GROUND_USD_PATH, prim_path=LOCAL_GROUND_PRIM_PATH)
        print(f"[ground] 使用地面 USD: {DEFAULT_GROUND_USD_PATH} -> {LOCAL_GROUND_PRIM_PATH}")
    else:
        print(f"[ground] 地面 USD 不存在: {DEFAULT_GROUND_USD_PATH}，回退到程序建地面")
        _add_local_ground_plane(stage_for_scene)


    # 加载机械臂 USD
    usd_path = _resolve_usd(args.usd)
    arm_prim_path = "/World/IceCreamArm"
    add_reference_to_stage(usd_path=usd_path, prim_path=arm_prim_path)
    arm = world.scene.add(
        SingleArticulation(arm_prim_path, name="ice_cream_arm")
    )

    world.reset()

    # PD 增益：关节 4、5 略大，配合几何解耦时强保持 q4/q5 于 HOME
    ctrl = arm.get_articulation_controller()
    try:
        kps = np.array([1e5, 1e5, 1e5, 2e5, 2e5], dtype=float)
        kds = np.array([1e4, 1e4, 1e4, 3e4, 3e4], dtype=float)  # 关节 4、5 略增 kd 抑制末节摆动
        ctrl.set_gains(kps=kps, kds=kds)
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
    move_step   = 0
    track_step_counter = 0  # PHASE_DONE 中用于每隔一段时间重新规划直线

    # 轨迹参数（从仿真读取的起点）
    p_line_start  = None
    p_ideal_final = None
    total_steps   = None
    dt_move       = None
    v_ee          = None

    # HOME 下方目标点（矩形中心向上 TRACK_HEIGHT）
    target_pos = None

    # 统计数据
    deviations  = []
    pos_errors  = []
    sv_mins     = []
    q_sim_log   = []   # 从仿真实际读取的关节角
    p_sim_log   = []   # 从仿真实际读取的末端位置（FK）

    # 当前控制目标关节角（用于积分）
    q_cmd = Q_HOME.copy()

    print(f"\n[sim] 3-DOF 几何解耦 | 跟踪点=与 link4 重合 | 方向={direction} | "
          f"速度={args.speed}m/s" + (f" | Z向下={args.speed_z}m/s" if args.speed_z != 0 else ""))
    if track_kp_line > 0.0:
        print(f"      线段位置 Kp={track_kp_line:.2f}（沿直线纠偏，高速时减少漂移）")
    print(f"      阻尼λ={damping_used:.4f} | 最少稳定帧数={args.stabilize_steps}")
    print()

    # ── 主循环 ─────────────────────────────────────────────────────────────
    while simulation_app.is_running():
        world.step(render=not args.headless)

        if world.is_stopped():
            world.reset()
            phase       = PHASE_STABILIZE
            stab_step   = 0
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

        # 跟踪点：与 link4 重合的空间点（几何解耦：仅位置，无姿态约束）
        p_sim = kin.forward_kinematics_position_joint4_motor_center(q_sim)

        # ══════════════════════════════════════════════════════════════════
        # PHASE 1: 稳定到 HOME
        # ══════════════════════════════════════════════════════════════════
        if phase == PHASE_STABILIZE:
            stab_step += 1
            q_err = np.linalg.norm(q_sim - Q_HOME)
            at_home = q_err < STABILIZE_JOINT_TOL

            if not at_home:
                # 关节空间前馈：每步朝 Q_HOME 移动，加快回 HOME
                dq = np.clip(
                    args.stabilize_kp * (Q_HOME - q_sim) * physics_dt,
                    -args.stabilize_max_dq, args.stabilize_max_dq,
                )
                q_cmd = np.clip(q_sim + dq, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)
                ctrl.apply_action(ArticulationAction(joint_positions=q_cmd.tolist()))
            else:
                ctrl.apply_action(ArticulationAction(joint_positions=Q_HOME.tolist()))

            if stab_step % 30 == 0:
                p_cur = kin.forward_kinematics_position(q_sim)
                print(f"[stabilize] step={stab_step:4d} | "
                      f"关节误差={q_err:.4f} rad | "
                      f"EE=[{p_cur[0]:.3f},{p_cur[1]:.3f},{p_cur[2]:.3f}]")

            min_steps_done = stab_step >= args.stabilize_steps
            can_start_line = min_steps_done and at_home

            if can_start_line:
                p_line_start  = kin.forward_kinematics_position_joint4_motor_center(q_sim).copy()
                dt_move       = physics_dt
                q_cmd         = q_sim.copy()
                # 几何解耦：q1,q2,q3 控位置；q4 = -(q2+q3) 使末节垂直 XOY；q5 固定
                q_cmd[3] = q4_geometric_decouple(q_cmd)
                q_cmd[4] = Q_HOME[4]

                # 在跟踪点正下方地面生成小矩形；目标点 z = TRACK_HEIGHT + LINK5_LENGTH（末节向下时末端在 TRACK_HEIGHT）
                try:
                    from pxr import UsdGeom, Gf
                    home_box_center = np.array([p_line_start[0], p_line_start[1], 0.0], dtype=float)
                    target_pos = np.array(
                        [home_box_center[0], home_box_center[1], TRACK_HEIGHT + LINK5_LENGTH], dtype=float
                    )
                    print(
                        "[home_box] center=[%.4f, %.4f, %.4f], target_pos=[%.4f, %.4f, %.4f]"
                        % (
                            home_box_center[0],
                            home_box_center[1],
                            home_box_center[2],
                            target_pos[0],
                            target_pos[1],
                            target_pos[2],
                        )
                    )
                    if stage is not None:
                        box_prim = UsdGeom.Cube.Define(stage, "/World/HomeBox")
                        box_prim.CreateSizeAttr(0.05)  # 5cm 立方体
                        xf_box = UsdGeom.Xformable(box_prim)
                        xf_box.ClearXformOpOrder()
                        xf_box.AddTranslateOp().Set(
                            Gf.Vec3d(
                                float(home_box_center[0]),
                                float(home_box_center[1]),
                                float(home_box_center[2]),
                            )
                        )
                        color_attr = box_prim.CreateDisplayColorAttr()
                        color_attr.Set([Gf.Vec3f(0.9, 0.4, 0.1)])  # 橙色小方块
                except Exception:
                    home_box_center = np.array([p_line_start[0], p_line_start[1], 0.0], dtype=float)
                    target_pos = np.array(
                        [home_box_center[0], home_box_center[1], TRACK_HEIGHT + LINK5_LENGTH], dtype=float
                    )

                # 以当前位置为起点、以 target_pos 为终点，重新规划一条直线段
                delta = target_pos - p_line_start
                seg_len = float(np.linalg.norm(delta))
                if seg_len < 1e-6:
                    # 目标点与当前点几乎重合，不再规划直线
                    p_ideal_final = target_pos.copy()
                    total_steps   = 1
                    v_ee          = np.zeros(3, dtype=float)
                else:
                    direction = delta / seg_len
                    v_ee = args.speed * direction
                    duration_move_line = seg_len / (args.speed + 1e-12)
                    total_steps = max(1, int(duration_move_line / dt_move))
                    p_ideal_final = target_pos.copy()

                marker.add_start_end_markers(p_line_start, p_ideal_final)

                print(f"\n[move] 开始直线运动（当前位置 -> 目标点）")
                print(f"       起点: {p_line_start}  理想终点: {p_ideal_final}")
                print(f"       v_ee = 直线 {args.speed}m/s （自动按当前位置→目标点方向投影）")
                print(f"       总步数: {total_steps} | dt={dt_move*1000:.2f}ms")
                print(f"       [调试] 跟踪点=joint4电机中心 | q4=-(q2+q3) 末节垂直XOY | q5 固定 | q1,q2,q3 控位置")
                print(f"       {'步':>5s} | {'偏离(mm)':>10s} | {'误差(mm)':>10s} | "
                      f"{'σ_min':>8s} | {'跟踪点(电机中心)(m)':>28s}")
                print(f"       {'-'*75}")
                phase = PHASE_MOVE

        # ══════════════════════════════════════════════════════════════════
        # PHASE 2: 几何解耦 — 仅 3×3 位置 DLS，无姿态约束
        # ══════════════════════════════════════════════════════════════════
        elif phase == PHASE_MOVE:
            move_step += 1
            p_link4_origin = kin.forward_kinematics_position_link4(q_sim)

            # 几何解耦：仅用与 link4 重合点的位置对 q1,q2,q3 的雅可比 J ∈ R^{3×3}，determined 系统
            J_link4_3 = kin.jacobian_joint4_motor_center_first3(q_sim)

            sv    = np.linalg.svd(J_link4_3, compute_uv=False)
            sv_min_cur = float(sv.min())
            sv_mins.append(sv_min_cur)

            if sv_min_cur < SINGULARITY_THR:
                print(f"\n[move] ⚠ 步 {move_step}: 接近奇异！σ_min={sv_min_cur:.4f}")

            s       = move_step * dt_move
            p_ideal = p_line_start + s * v_ee
            dev     = point_to_line_dist(p_sim, p_line_start, direction_line)
            dev_joint = point_to_line_dist(p_link4_origin, p_line_start, direction_line)
            # 任务空间：仅 3D 线速度，无角速度（姿态由 q4=-(q2+q3) 与 q5 固定保证）
            if track_kp_line > 0.0:
                twist_linear = v_ee + track_kp_line * (p_ideal - p_sim)
                v_lin_norm = float(np.linalg.norm(twist_linear))
                if v_lin_norm > 0.5:
                    twist_linear *= 0.5 / v_lin_norm
                twist = np.asarray(twist_linear, dtype=float)
            else:
                twist = np.asarray(v_ee, dtype=float)
            err = float(np.linalg.norm(p_sim - p_ideal))

            # 3×3 DLS：J_link4_3 @ dq3 = twist，无过定、无姿态行
            J_use = J_link4_3
            M = J_use @ J_use.T + (damping_used ** 2) * np.eye(3)

            twist_used = twist
            if sv_min_cur < NEAR_SINGULAR_THR:
                alpha = (sv_min_cur - SINGULARITY_THR) / max(
                    1e-6, (NEAR_SINGULAR_THR - SINGULARITY_THR)
                )
                alpha = float(np.clip(alpha, MIN_NEAR_SINGULAR_SCALE, 1.0))
                twist_used = twist * alpha

            dq3 = J_use.T @ np.linalg.solve(M, twist_used)  # (3,)

            dq_norm = float(np.linalg.norm(dq3))
            if dq_norm > MAX_JOINT_VEL:
                dq3 *= MAX_JOINT_VEL / (dq_norm + 1e-12)

            # q1,q2,q3：位置跟踪（电机中心）；q4 = -(q2+q3) 使末节垂直 XOY；q5 固定
            q_cmd[:3] = np.clip(
                q_cmd[:3] + dq3 * dt_move,
                JOINT_LIMITS_LOWER[:3], JOINT_LIMITS_UPPER[:3]
            )
            q_cmd[3] = q4_geometric_decouple(q_cmd)
            q_cmd[4] = Q_HOME[4]  # joint5 固定

            ctrl.apply_action(
                ArticulationAction(joint_positions=q_cmd.tolist())
            )

            deviations.append(dev)
            pos_errors.append(err)
            q_sim_log.append(q_sim.copy())
            p_sim_log.append(p_sim.copy())
            marker.tick(p_sim, p_ideal)

            # 调试：每 30 步输出「谁在跟线」— 电机中心 vs 转轴 到直线的偏离，以及末端与 -Z 夹角
            if move_step % 30 == 0 or move_step == total_steps:
                dist_mm = float(np.linalg.norm(p_sim - p_link4_origin)) * 1000.0
                R_ee = kin.forward_kinematics_rotation(q_sim)
                ee_z = R_ee[:, 2]  # 末端 z 轴（沿末节指向）在世界系下的方向
                cos_angle = np.clip(float(np.dot(ee_z, np.array([0.0, 0.0, -1.0]))), -1.0, 1.0)
                angle_with_neg_z_deg = np.degrees(np.arccos(cos_angle))
                print(f"       {move_step:>5d} | {dev*1000:>10.4f} | {err*1000:>10.4f} | "
                      f"{sv_min_cur:>8.4f} | "
                      f"[{p_sim[0]:.4f},{p_sim[1]:.4f},{p_sim[2]:.4f}]")
                print(f"       [dbg] 电机中心偏离直线= {dev*1000:.4f} mm  |  转轴偏离直线= {dev_joint*1000:.4f} mm  "
                      f"| 两点距离= {dist_mm:.2f} mm  (若为电机中心跟线，前者应更小)")
                print(f"            跟踪点(电机中心)= [{p_sim[0]:.4f},{p_sim[1]:.4f},{p_sim[2]:.4f}]  "
                      f"link4原点(转轴)= [{p_link4_origin[0]:.4f},{p_link4_origin[1]:.4f},{p_link4_origin[2]:.4f}]")
                print(f"            [姿态] 末端 z 轴与世界 -Z 夹角= {angle_with_neg_z_deg:.4f}° (0°=垂直向下)")

            if move_step >= total_steps:
                phase = PHASE_DONE

        # ══════════════════════════════════════════════════════════════════
        # PHASE 3: 完成，打印报告 & 周期性重新规划直线
        # ══════════════════════════════════════════════════════════════════
        elif phase == PHASE_DONE:
            # 保持最后位置
            ctrl.apply_action(
                ArticulationAction(joint_positions=q_cmd.tolist())
            )

            if len(deviations) > 0:
                seg_len = float(np.linalg.norm(p_ideal_final - p_line_start))
                _print_report(
                    deviations, pos_errors, sv_mins,
                    p_line_start, p_ideal_final,
                    kin.forward_kinematics_position_joint4_motor_center(q_sim),
                    direction_line, seg_len, args.speed, args.speed_z,
                )
                deviations.clear()   # 清空避免重复打印

            # 每隔一定时间重新规划：高速时缩短间隔，更快纠正方向、减少多次规划才收敛
            if stage is not None:
                track_step_counter += 1
                track_interval_sec = 0.5 if args.speed >= 0.05 else TRACK_INTERVAL_SEC
                TRACK_INTERVAL_STEPS = max(1, int(track_interval_sec / physics_dt))
                if track_step_counter >= TRACK_INTERVAL_STEPS:
                    track_step_counter = 0
                    # 读取当前小矩形中心位置，并据此定义新的目标点
                    try:
                        from pxr import UsdGeom
                        box_prim = stage.GetPrimAtPath("/World/HomeBox")
                        if not box_prim:
                            continue
                        box_xf = UsdGeom.Xformable(box_prim)
                        world_xf = box_xf.ComputeLocalToWorldTransform(0.0)
                        t = world_xf.ExtractTranslation()
                        # 小矩形中心坐标
                        box_center = np.array([t[0], t[1], t[2]], dtype=float)
                        # 跟踪点目标 z = 地面 + TRACK_HEIGHT + LINK5_LENGTH
                        target_pos = np.array(
                            [box_center[0], box_center[1], box_center[2] + TRACK_HEIGHT + LINK5_LENGTH],
                            dtype=float,
                        )
                    except Exception:
                        continue

                    p_line_start = kin.forward_kinematics_position_joint4_motor_center(q_sim).copy()
                    delta = target_pos - p_line_start
                    seg_len = float(np.linalg.norm(delta))
                    if seg_len > 1e-6:
                        direction = delta / seg_len
                        v_ee = args.speed * direction
                        duration_move_line = seg_len / (args.speed + 1e-12)
                        total_steps = max(1, int(duration_move_line / physics_dt))
                        p_ideal_final = target_pos.copy()
                        dt_move = physics_dt
                        move_step = 0
                        direction_line = direction.copy()
                        deviations.clear()
                        pos_errors.clear()
                        sv_mins.clear()
                        q_sim_log.clear()
                        p_sim_log.clear()
                        print(
                            "\n[retrack] 重新规划直线: 跟踪点=[%.4f, %.4f, %.4f] -> 目标点=[%.4f, %.4f, %.4f]"
                            % (
                                p_line_start[0], p_line_start[1], p_line_start[2],
                                p_ideal_final[0], p_ideal_final[1], p_ideal_final[2],
                            )
                        )
                        print(
                            "[retrack] target_pos=[%.4f, %.4f, %.4f], 跟踪点_now=[%.4f, %.4f, %.4f]"
                            % (target_pos[0], target_pos[1], target_pos[2],
                               p_line_start[0], p_line_start[1], p_line_start[2])
                        )
                        phase = PHASE_MOVE

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
    print("║   3-DOF 几何解耦 直线验证报告（跟踪点=与 link4 重合）       ║")
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


# ═══════════════════════════════════════════════════════════════════════════
# 辅助
# ═══════════════════════════════════════════════════════════════════════════

# 本地地面：prim 路径与固定 USD 路径（Isaac 4.5 Grid 默认环境）
LOCAL_GROUND_PRIM_PATH = "/World/LocalGround"
DEFAULT_GROUND_USD_PATH = "/home/huangjianan/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments/Grid/default_environment.usd"


def _add_local_ground_plane(stage=None):
    """在当前 Stage 创建本地地面（可见 + 碰撞），不依赖 Nucleus。
    使用与 World 相同的 stage 可避免路径/上下文不一致。
    """
    from pxr import UsdGeom, Gf
    if stage is None:
        stage = omni.usd.get_context().get_stage()
    path = LOCAL_GROUND_PRIM_PATH
    # 优先用 PhysicsSchemaTools：自带碰撞与网格（部分 Isaac 版本在 pxr 中提供）
    try:
        from pxr import PhysicsSchemaTools
        # 常见签名: (stage, path, normal_axis, size, position, color_or_scale)
        PhysicsSchemaTools.addGroundPlane(
            stage, path, "Z",
            15.0, Gf.Vec3f(0, 0, 0), Gf.Vec3f(0.5, 0.5, 0.5)
        )
        print(f"[ground] 使用本地地面: {path} (PhysicsSchemaTools)")
        return
    except Exception as e:
        # 便于排查：是导入路径问题还是 API 签名问题
        print(f"[ground] PhysicsSchemaTools 不可用: {type(e).__name__}: {e}")
    # 回退：大扁立方体 + 显示颜色，并尽量加碰撞
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.ClearXformOpOrder()
    xf.AddScaleOp().Set(Gf.Vec3d(25.0, 25.0, 0.02))
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.01))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.5, 0.5, 0.55)])
    try:
        from pxr import UsdPhysics
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    except Exception:
        pass
    print(f"[ground] 使用本地地面: {path} (Cube，上表面 z=0)")


def _find_urdf() -> str | None:
    """按优先级在 sim_code、icecream_model/urdf 下查找 URDF。"""
    candidates = [
        os.path.join(_CODE_DIR, "ice_cream_0208_SLDASM.urdf"),
        os.path.join(_CODE_DIR, "ice_cream_0208.SLDASM.urdf"),
        os.path.join(_PROJECT_ROOT, "icecream_model", "urdf", "ice_cream_0208.SLDASM.urdf"),
        os.path.join(_PROJECT_ROOT, "icecream_model", "urdf", "ice_cream_0208_SLDASM.urdf"),
        os.path.join(_PROJECT_ROOT, "ice_cream_0208.SLDASM", "urdf", "ice_cream_0208.SLDASM.urdf"),
    ]
    return next((p for p in candidates if os.path.isfile(p)), None)


def _resolve_usd(usd_arg: str | None) -> str:
    """解析机械臂 USD：--usd 指定则用该路径，否则在 sim_code / 项目根下查找。"""
    if usd_arg is not None:
        path = os.path.abspath(usd_arg)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"USD 文件不存在: {path}")
        return path
    candidates = [
        os.path.join(_CODE_DIR, "ice_cream_single_arm.usd"),
        os.path.join(_PROJECT_ROOT, "sim_code", "ice_cream_single_arm.usd"),

    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        "未找到 ice_cream_arm.usd，请用 --usd 指定路径，"
        "或将其放在 sim_code/ 下，或先运行 urdf_to_usd.py 生成。"
    )


def _check_path_singularity(kin, q_start, v_ee, duration,
                             dt=1/200.0, damping=0.01):
    """离线预检路径奇异性（5-DOF 末端位置）。"""
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


def _check_path_singularity_track_point(kin, q_start, v_ee, duration,
                                        dt=1/200.0, damping=0.01):
    """离线预检跟踪点（joint4 电机中心）路径奇异性（几何解耦：q4=-(q2+q3)，q5 固定）。"""
    n   = max(1, int(duration / dt))
    q   = q_start.copy()
    min_sv = 1e9
    for _ in range(n):
        J  = kin.jacobian_joint4_motor_center_first3(q)
        sv = np.linalg.svd(J, compute_uv=False)
        min_sv = min(min_sv, float(sv.min()))
        if sv.min() < SINGULARITY_THR:
            return min_sv, False
        M   = J @ J.T + damping ** 2 * np.eye(3)
        dq3 = J.T @ np.linalg.solve(M, v_ee)
        q[:3] = np.clip(q[:3] + dq3 * dt, JOINT_LIMITS_LOWER[:3], JOINT_LIMITS_UPPER[:3])
        q[3] = np.clip(np.pi / 2.0 - (q[0] + q[1]), JOINT_LIMITS_LOWER[3], JOINT_LIMITS_UPPER[3])
        q[4] = q_start[4]
    return min_sv, True


if __name__ == "__main__":
    main()