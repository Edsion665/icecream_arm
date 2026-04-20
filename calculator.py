"""统一计算层：IK_calculator 与 mover。"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# 运动学（仅 link4 几何解耦 IK / FK 所需；sim_test 独立，不依赖 sim_code）
# ---------------------------------------------------------------------------
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
NUM_JOINTS = 5

JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS, dtype=float)
JOINT_LIMITS_UPPER = np.array([3.14] * NUM_JOINTS, dtype=float)

from .config import (
    APPROACH_GAIN,
    CONTROL_DT,
    DEFAULT_INITIAL_JOINT_REL_DEG_4,
    IK_DAMPING,
    IK_MAX_ITER,
    IK_POS_TOL,
    MAX_JOINT_VEL_RAD_S,
    MAX_TARGET_RATE_RAD_S,
    POSE_VEL_MAX_M_S,
    POSE_Q5_EXTRA_DEG,
    Q4_GEOMETRIC_OFFSET_DEG,
    Q4_GEOMETRIC_Q23_COEFF,
    Q4_SAFE_MAX,
    Q4_SAFE_MIN,
    SV_CRIT,
    SV_WARN,
    frontend_pose_to_internal_m,
)

Q4_OFFSET_DEG = float(Q4_GEOMETRIC_OFFSET_DEG)
Q4_OFFSET_RAD = float(np.deg2rad(Q4_OFFSET_DEG))
Q4_Q23_COEFF = float(Q4_GEOMETRIC_Q23_COEFF)

_HARDCODED_JOINTS: List[dict] = [
    {
        "name": "joint1",
        "xyz": np.array([0.124999999999995, 0.125000000000034, 0.1055]),
        "rpy": np.array([0.0, 0.0, 0.0]),
        "axis": np.array([0.0, 0.0, -1.0]),
    },
    {
        "name": "joint2",
        "xyz": np.array([0.0225803798520988, -0.0220337569588361, 0.0489000000003376]),
        "rpy": np.array([1.5707963267949, 0.0, 0.111028214146299]),
        "axis": np.array([0.0, 0.0, -1.0]),
    },
    {
        "name": "joint3",
        "xyz": np.array([0.31372987976612, 0.0556095676499454, 0.0]),
        "rpy": np.array([0.0, 0.0, -1.39536543135007]),
        "axis": np.array([0.0, 0.0, -1.0]),
    },
    {
        "name": "joint4",
        "xyz": np.array([-0.119739543983627, 0.269212264652694, 0.0]),
        "rpy": np.array([0.0, 0.0, -1.39405537845928]),
        "axis": np.array([0.0, 0.0, -1.0]),
    },
    {
        "name": "joint5",
        "xyz": np.array([0.0250377384169529, 0.0971256101915153, -0.0244999999998483]),
        "rpy": np.array([1.22364489414134, -1.5707963267949, 0.0]),
        "axis": np.array([0.0, 0.0, -1.0]),
    },
]


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def _axis_angle_to_matrix(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).ravel()[:3]
    n = np.linalg.norm(axis)
    if n < 1e-10:
        return np.eye(3)
    axis = axis / n
    kx, ky, kz = axis
    c, s = np.cos(theta), np.sin(theta)
    return np.array(
        [
            [c + kx * kx * (1 - c), kx * ky * (1 - c) - kz * s, kx * kz * (1 - c) + ky * s],
            [ky * kx * (1 - c) + kz * s, c + ky * ky * (1 - c), ky * kz * (1 - c) - kx * s],
            [kz * kx * (1 - c) - ky * s, kz * ky * (1 - c) + kx * s, c + kz * kz * (1 - c)],
        ],
        dtype=float,
    )


def _pose_to_matrix(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation
    T[:3, 3] = np.asarray(position, dtype=float).ravel()[:3]
    return T


def q4_geometric_decouple(q: np.ndarray) -> float:
    """几何解耦：q4 = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q2+q3)。"""
    q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
    return float(
        np.clip(
            Q4_OFFSET_RAD + Q4_Q23_COEFF * (q[1] + q[2]),
            JOINT_LIMITS_LOWER[3],
            JOINT_LIMITS_UPPER[3],
        )
    )


class URDFKinematics:
    """link0→link5：仅实现 link4 原点 FK 与几何解耦 IK（与 sim_test 控制一致）。"""

    def __init__(self, urdf_path: Optional[str] = None):
        self._joints: List[dict] = []
        if urdf_path is not None and os.path.isfile(urdf_path):
            self._urdf_path = os.path.abspath(urdf_path)
            self._parse_urdf()
            self._source = "urdf_file"
        else:
            self._urdf_path = urdf_path or "<hardcoded>"
            self._joints = [dict(j) for j in _HARDCODED_JOINTS]
            self._source = "hardcoded"

    def _parse_urdf(self) -> None:
        tree = ET.parse(self._urdf_path)
        root = tree.getroot()
        order = []
        for j in root.findall(".//joint"):
            name = j.get("name")
            if name not in JOINT_NAMES:
                continue
            origin = j.find("origin")
            xyz = np.zeros(3, dtype=float)
            rpy = np.zeros(3, dtype=float)
            if origin is not None:
                if origin.get("xyz"):
                    xyz = np.array([float(v) for v in origin.get("xyz").split()])
                if origin.get("rpy"):
                    rpy = np.array([float(v) for v in origin.get("rpy").split()])
            axis_el = j.find("axis")
            axis = np.array([0.0, 0.0, -1.0])
            if axis_el is not None and axis_el.get("xyz"):
                axis = np.array([float(v) for v in axis_el.get("xyz").split()])
            if np.linalg.norm(axis) < 1e-10:
                axis = np.array([0.0, 0.0, -1.0])
            order.append(
                (
                    JOINT_NAMES.index(name),
                    {"name": name, "xyz": xyz, "rpy": rpy, "axis": axis},
                )
            )
        order.sort(key=lambda x: x[0])
        self._joints = [x[1] for x in order]
        if len(self._joints) != NUM_JOINTS:
            raise RuntimeError(f"URDF 中找到 {len(self._joints)} 个关节，期望 {NUM_JOINTS}")

    def _joint_transform(self, idx: int, q: float) -> np.ndarray:
        j = self._joints[idx]
        R_origin = _rpy_to_matrix(j["rpy"])
        R_q = _axis_angle_to_matrix(j["axis"], q)
        return _pose_to_matrix(j["xyz"], R_origin @ R_q)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
        return T

    def forward_kinematics_link4(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        T = np.eye(4, dtype=float)
        for i in range(4):
            T = T @ self._joint_transform(i, q[i])
        return T

    def forward_kinematics_position_link4(self, q: np.ndarray) -> np.ndarray:
        return self.forward_kinematics_link4(q)[:3, 3].copy()

    def q_from_q3_geometric_decouple(
        self, q3: np.ndarray, q5_fixed: float = 0.0
    ) -> np.ndarray:
        q3 = np.asarray(q3, dtype=float).ravel()[:3]
        q = np.zeros(NUM_JOINTS, dtype=float)
        q[:3] = q3
        q[3] = q4_geometric_decouple(q)
        q[4] = float(q5_fixed)
        return q

    def forward_kinematics_link4_geometric_decouple(
        self, q3: np.ndarray, q5_fixed: float = 0.0
    ) -> Tuple[np.ndarray, np.ndarray]:
        q = self.q_from_q3_geometric_decouple(q3, q5_fixed)
        p_joint4 = self.forward_kinematics_position_link4(q)
        return q, p_joint4

    def jacobian_link4_origin_full5(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J = np.zeros((3, NUM_JOINTS), dtype=float)
        T_accum = [np.eye(4, dtype=float)]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
            T_accum.append(T.copy())
        p_link4 = T_accum[4][:3, 3]
        for i in range(4):
            j = self._joints[i]
            R_parent = T_accum[i][:3, :3]
            p_parent = T_accum[i][:3, 3]
            p_joint_i = p_parent + R_parent @ j["xyz"]
            z_i = R_parent @ (_rpy_to_matrix(j["rpy"]) @ j["axis"])
            z_i = z_i / (np.linalg.norm(z_i) + 1e-10)
            J[:, i] = np.cross(z_i, p_link4 - p_joint_i)
        return J

    def jacobian_link4_geometric_decouple(
        self, q3: np.ndarray, q5_fixed: float = 0.0
    ) -> np.ndarray:
        q = self.q_from_q3_geometric_decouple(q3, q5_fixed)
        J_link4_5 = self.jacobian_link4_origin_full5(q)
        M = np.zeros((5, 3), dtype=float)
        M[0, 0], M[1, 1], M[2, 2] = 1.0, 1.0, 1.0
        c = float(Q4_Q23_COEFF)
        M[3, 1], M[3, 2] = c, c
        return (J_link4_5 @ M).copy()

    def inverse_kinematics_link4_geometric_decouple(
        self,
        target_position: np.ndarray,
        q3_init: Optional[np.ndarray] = None,
        q5_fixed: float = 0.0,
        max_iter: int = 80,
        pos_tol: float = 1e-5,
        damping: float = 1e-2,
        joint_limits_lower: Optional[np.ndarray] = None,
        joint_limits_upper: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool]:
        target_position = np.asarray(target_position, dtype=float).ravel()[:3]
        ll = (
            JOINT_LIMITS_LOWER[:3]
            if joint_limits_lower is None
            else np.asarray(joint_limits_lower).ravel()[:3]
        )
        ul = (
            JOINT_LIMITS_UPPER[:3]
            if joint_limits_upper is None
            else np.asarray(joint_limits_upper).ravel()[:3]
        )
        q3 = (
            np.zeros(3, dtype=float)
            if q3_init is None
            else np.asarray(q3_init, dtype=float).ravel()[:3].copy()
        )
        q3 = np.clip(q3, ll, ul)

        for _ in range(max_iter):
            q_full, p_j4 = self.forward_kinematics_link4_geometric_decouple(q3, q5_fixed)
            ep = target_position - p_j4
            if np.linalg.norm(ep) < pos_tol:
                return self.q_from_q3_geometric_decouple(q3, q5_fixed), True
            J = self.jacobian_link4_geometric_decouple(q3, q5_fixed)
            Jt = J.T
            JJt = J @ Jt + (damping**2) * np.eye(3)
            dq3 = Jt @ np.linalg.solve(JJt, ep)
            q3 = np.clip(q3 + dq3, ll, ul)

        q_full = self.q_from_q3_geometric_decouple(q3, q5_fixed)
        return q_full, False


from .listener import MotionCommand4Axis

ARM_AXES = 4


class MotionMode(str, Enum):
    POSE = "pose"
    JOINTS = "joints"


@dataclass
class JointFrame:
    arm_rel_deg: np.ndarray
    arm_omega_rad_s: np.ndarray
    servo_deg: np.ndarray
    mode: str
    timestamp: float
    # 第 5 轴相对标定角（度）；笛卡尔下含 POSE_Q5_EXTRA_DEG，供仿真/UDP 与 q_cmd[4] 一致
    joint5_rel_deg: float = 0.0


@dataclass
class CalculatorState:
    mode: MotionMode = MotionMode.JOINTS
    q_calib_deg: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q_calib_rad: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q5_fixed_rad: float = 0.0
    pose_xyz: np.ndarray = field(default_factory=lambda: np.array([0.35, 0.2, 0.25], dtype=float))
    prev_pose_xyz: Optional[np.ndarray] = None
    joint_rel_deg_4: np.ndarray = field(default_factory=lambda: np.zeros(ARM_AXES, dtype=float))
    servo_deg: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    q_full: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    q_cmd: np.ndarray = field(default_factory=lambda: np.zeros(NUM_JOINTS, dtype=float))
    initialized: bool = False
    sing_hold: bool = False

    def reset_command(self) -> None:
        # 标定零点不变（q_calib）；启动姿态由 DEFAULT_INITIAL_JOINT_REL_DEG_4 给出相对标定的前四轴角。
        self.joint_rel_deg_4 = np.asarray(DEFAULT_INITIAL_JOINT_REL_DEG_4, dtype=float).reshape(ARM_AXES).copy()
        self.q_full[:ARM_AXES] = self.q_calib_rad[:ARM_AXES] + np.deg2rad(self.joint_rel_deg_4)
        self.q_full[4] = float(self.q5_fixed_rad)
        self.q_cmd[:NUM_JOINTS] = self.q_full[:NUM_JOINTS].copy()
        self.mode = MotionMode.JOINTS
        self.initialized = True
        self.prev_pose_xyz = None

    def sync_from_articulation_rad(self, q_joint_rad: np.ndarray, n_dof: int) -> None:
        """用仿真/实机读到的关节角（弧度）覆盖 q_full/q_cmd/joint_rel，使控制层与物理姿态一致。"""
        q = np.asarray(q_joint_rad, dtype=float).ravel()
        n = min(int(n_dof), NUM_JOINTS, len(q))
        if n <= 0:
            return
        self.q_full[:n] = q[:n]
        self.q_cmd[:n] = q[:n]
        self.joint_rel_deg_4 = (np.rad2deg(self.q_full[:ARM_AXES]) - self.q_calib_deg[:ARM_AXES]).copy()
        if n >= 5:
            self.q5_fixed_rad = float(q[4])
        self.mode = MotionMode.JOINTS
        self.initialized = True
        self.prev_pose_xyz = None


class IK_calculator:
    def __init__(self, kin: URDFKinematics):
        self._kin = kin

    def step(self, state: CalculatorState, dt: float = CONTROL_DT) -> None:
        pose = state.pose_xyz.copy()
        if state.prev_pose_xyz is not None:
            max_step = POSE_VEL_MAX_M_S * dt
            d = pose - state.prev_pose_xyz
            n = float(np.linalg.norm(d))
            if n > max_step and n >= 1e-12:
                pose = state.prev_pose_xyz + d * (max_step / n)
        state.prev_pose_xyz = pose.copy()

        q_curr_3 = state.q_full[:3]
        J3 = self._kin.jacobian_link4_geometric_decouple(q_curr_3, state.q5_fixed_rad)
        sv = np.linalg.svd(J3, compute_uv=False)
        sv_min = float(sv.min())
        if sv_min < SV_CRIT:
            state.sing_hold = True
            return

        state.sing_hold = False
        damp = (
            min(IK_DAMPING * (SV_WARN / (sv_min + 1e-8)), IK_DAMPING * 20.0)
            if sv_min < SV_WARN
            else IK_DAMPING
        )
        q_sol, ok = self._kin.inverse_kinematics_link4_geometric_decouple(
            pose,
            q3_init=q_curr_3,
            q5_fixed=state.q5_fixed_rad,
            max_iter=IK_MAX_ITER,
            pos_tol=IK_POS_TOL,
            damping=damp,
        )
        if not ok:
            return
        q4c = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2])
        if not (Q4_SAFE_MIN <= q4c <= Q4_SAFE_MAX):
            return
        delta = q_sol[:NUM_JOINTS] - state.q_full[:NUM_JOINTS]
        dn = float(np.linalg.norm(delta))
        max_d = MAX_TARGET_RATE_RAD_S * dt
        if dn > max_d:
            delta *= max_d / dn
        state.q_full[:NUM_JOINTS] += delta
        state.q_full[3] = np.clip(
            Q4_OFFSET_RAD + Q4_Q23_COEFF * (state.q_full[1] + state.q_full[2]),
            -3.14,
            3.14,
        )


class mover:
    def step(self, state: CalculatorState, dt: float = CONTROL_DT) -> None:
        """关节模式：目标角直接写入 q_full（与 cartesian 仅一段 q_cmd 趋近一致，避免 mover+趋近双层限速）。"""
        q_tgt = state.q_full.copy()
        q_tgt[:ARM_AXES] = state.q_calib_rad[:ARM_AXES] + np.deg2rad(state.joint_rel_deg_4)
        state.q_full[:ARM_AXES] = q_tgt[:ARM_AXES]


class CalculatorEngine:
    """统一 step 接口：step(command, state, dt) -> JointFrame"""

    def __init__(self, kin: URDFKinematics):
        self._kin = kin
        self._ik = IK_calculator(kin)
        self._mover = mover()

    def apply_command(self, cmd: MotionCommand4Axis, state: CalculatorState) -> None:
        p = cmd.payload
        if cmd.kind == "pose":
            state.mode = MotionMode.POSE
            xi, yi, zi = frontend_pose_to_internal_m(float(p["x"]), float(p["y"]), float(p["z"]))
            state.pose_xyz = np.array([xi, yi, zi], dtype=float)
            # 始终从当前末端 FK 作为限速起点；若在 POSE 下再次发绝对 pose，旧逻辑 prev=None
            # 会跳过笛卡尔渐变，IK 每帧对准「很远」的目标，q3 等轴会像一步飞过去。
            state.prev_pose_xyz = self._kin.forward_kinematics_position_link4(state.q_full).copy()
        elif cmd.kind == "pose_delta":
            state.mode = MotionMode.POSE
            state.pose_xyz[0] += float(p["dx"])
            state.pose_xyz[1] += float(p["dy"])
            state.pose_xyz[2] += float(p["dz"])
            state.prev_pose_xyz = None
        elif cmd.kind == "joints":
            state.mode = MotionMode.JOINTS
            arr = p["axes_rel_deg"]
            state.joint_rel_deg_4 = np.array([float(arr[i]) for i in range(ARM_AXES)], dtype=float)
        elif cmd.kind == "joints_delta":
            state.mode = MotionMode.JOINTS
            arr = p["deltas_rel_deg"]
            for i in range(min(len(arr), ARM_AXES)):
                state.joint_rel_deg_4[i] += float(arr[i])

    def step(self, command: Optional[MotionCommand4Axis], state: CalculatorState, dt: float = CONTROL_DT) -> JointFrame:
        if not state.initialized:
            state.reset_command()
        if command is not None:
            self.apply_command(command, state)

        if state.mode == MotionMode.POSE:
            self._ik.step(state, dt=dt)
        else:
            self._mover.step(state, dt=dt)

        # 笛卡尔：q5 不参与 link4 位置 IK，叠加偏置使末节（link5）朝向可调（默认 180° 朝下相对原朝向）
        q5_pose_tgt: float | None = None
        if state.mode == MotionMode.POSE:
            q5_pose_tgt = float(
                np.clip(
                    state.q5_fixed_rad + np.deg2rad(POSE_Q5_EXTRA_DEG),
                    float(JOINT_LIMITS_LOWER[4]),
                    float(JOINT_LIMITS_UPPER[4]),
                )
            )
            state.q_full[4] = q5_pose_tgt

        # 参考 sim_code/cartesian_ik_verify.py:
        # q_error -> dq_desired -> 仅向量范数限幅 -> q_cmd 递推（与 cartesian 一致，无逐轴 clip）
        q_err = state.q_full[:ARM_AXES] - state.q_cmd[:ARM_AXES]
        dq_des = q_err * APPROACH_GAIN

        dq_n = float(np.linalg.norm(dq_des))
        if dq_n > MAX_JOINT_VEL_RAD_S:
            dq_des *= MAX_JOINT_VEL_RAD_S / dq_n

        state.q_cmd[:ARM_AXES] += dq_des * dt
        if state.mode == MotionMode.POSE:
            state.q_cmd[3] = np.clip(
                Q4_OFFSET_RAD + Q4_Q23_COEFF * (state.q_cmd[1] + state.q_cmd[2]),
                Q4_SAFE_MIN,
                Q4_SAFE_MAX,
            )
            if q5_pose_tgt is not None:
                state.q_cmd[4] = q5_pose_tgt

        # 与 cartesian 一致：下发 ω 用本步 dq_des；POSE 下 q4 与 q2+q3 耦合（∂q4/∂q2=∂q4/∂q3=Q4_Q23_COEFF）
        omega_arm = np.zeros(ARM_AXES, dtype=float)
        omega_arm[:ARM_AXES] = dq_des
        if state.mode == MotionMode.POSE:
            omega_arm[3] = Q4_Q23_COEFF * (dq_des[1] + dq_des[2])

        p_rel_deg = np.rad2deg(state.q_cmd[:ARM_AXES]) - state.q_calib_deg[:ARM_AXES]
        j5_rel = float(np.rad2deg(state.q_cmd[4]) - state.q_calib_deg[4])
        return JointFrame(
            arm_rel_deg=p_rel_deg.copy(),
            arm_omega_rad_s=omega_arm.copy(),
            servo_deg=state.servo_deg.copy(),
            mode=state.mode.value,
            timestamp=time.monotonic(),
            joint5_rel_deg=j5_rel,
        )
