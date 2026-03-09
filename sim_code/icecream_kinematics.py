#!/usr/bin/env python3
"""
Ice Cream 机械臂运动学 (ice_cream_0208.SLDASM)
============================================================
坐标系约定：
  base  ── link0 固连，机械臂底座系（URDF 根节点）
  world ── Isaac Sim 场景世界系；机械臂 prim 位姿给出 T_world_base
  ee    ── link5 原点（末端执行器）
  cam   ── 安装在 link5 上的相机，由 icecream_camera 的 offset+euler 定义

变换链：T_world_ee = T_world_base · T_base_ee(q)
        T_world_cam = T_world_ee · T_ee_cam

工作空间（q 在 ±1.2 rad 时采样）：
  X ∈ [0.10, 0.71] m   Y ∈ [-0.33, 0.65] m   Z ∈ [-0.34, 0.71] m
"""
from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import numpy as np

# --------------------------------------------------------------------------
# 常量
# --------------------------------------------------------------------------
JOINT_NAMES   = ["joint1", "joint2", "joint3", "joint4", "joint5"]
NUM_JOINTS    = 5
EE_LINK_NAME  = "link5"
BASE_LINK_NAME = "link0"

JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS, dtype=float)
JOINT_LIMITS_UPPER = np.array([ 3.14] * NUM_JOINTS, dtype=float)

# 第五节（link5）长度：link4 原点到 link5 原点的距离（joint5 在 link4 系下的 xyz 的模）
# 用于「末节始终向下」时：link4 目标 z = 期望末端高度 + LINK5_LENGTH，则末端恰在期望高度
_LINK5_XYZ = np.array([0.0250377384169529, 0.0971256101915153, -0.0244999999998483], dtype=float)
LINK5_LENGTH = float(np.linalg.norm(_LINK5_XYZ))

# joint4 电机几何中心在 link4 系下的偏移（m）
# - link4 原点 = joint4 转轴位置（URDF 中 link4 的 origin），不是电机外壳的几何中心。
# - 本向量 = 在 link4 坐标系下，从 link4 原点指向 joint4 所在电机几何中心的向量。
# 从 URDF/CAD 获取方法：
#   1. 在 URDF 中 link4 的原点即 joint4 的转动中心；
#   2. 在 CAD 中量取该关节电机的几何中心（外壳中心或质心），再换算到 link4 系下；
#   3. 或：若 URDF 中 link4 有子 visual/collision 表示电机体，取该 mesh 中心在 link4 系下的坐标。
# 若暂无准确值可填 [0,0,0]，则跟踪点退化为 joint4 转轴。
# 当前值：跟踪点 = joint4 原点沿 link4 系 -z 方向 20 mm（电机中心）；置零则退化为转轴。
JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4 = np.array([0.0, 0.0, -0.02], dtype=float)


# 几何解耦时 q4 的固定偏移：q4 = Q4_OFFSET_DEG - (q2+q3)，单位弧度
Q4_OFFSET_DEG = 120.0  # 度
Q4_OFFSET_RAD = float(np.deg2rad(Q4_OFFSET_DEG))


def q4_geometric_decouple(q: np.ndarray) -> float:
    """
    几何解耦时 joint4 目标角：q4 = 120° - (q2+q3)（弧度：Q4_OFFSET_RAD - (q2+q3)）。
    使末节按给定偏移保持姿态；与 ice_cream_single_test 一致。
    """
    q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
    return float(np.clip(Q4_OFFSET_RAD - (q[1] + q[2]), JOINT_LIMITS_LOWER[3], JOINT_LIMITS_UPPER[3]))


def _load_joint4_motor_offset_from_urdf(urdf_path: str) -> Optional[np.ndarray]:
    """
    从 URDF 中读取 link4 的 joint4 电机中心偏移（若存在）。
    若 link4 内包含自定义标签 <motor_center xyz="x y z" />（单位 m），则返回 (3,) 向量，否则返回 None。
    可在 URDF 的 link4 中增加：<motor_center xyz="0 0 -0.02" /> 表示电机中心在 link4 系下相对原点的偏移（z 轴反方向 20 mm）。
    """
    if not os.path.isfile(urdf_path):
        return None
    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        for link in root.findall(".//link"):
            if link.get("name") != "link4":
                continue
            mc = link.find("motor_center")
            if mc is not None and mc.get("xyz"):
                parts = mc.get("xyz").split()
                if len(parts) >= 3:
                    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=float)
            break
    except Exception:
        pass
    return None


# 若存在 URDF 且 link4 定义了 motor_center 且为非零向量，则覆盖默认偏移（0 0 0 视为未定义，不覆盖）
def _apply_joint4_motor_offset_from_urdf() -> None:
    """保留函数以兼容旧代码，如需从 URDF 中读取 motor_center，可在此处启用。当前默认使用硬编码偏移 [0, 0, -0.02]。"""
    return

# --------------------------------------------------------------------------
# URDF 精确参数（硬编码 fallback，无需 URDF 文件也可运行）
# --------------------------------------------------------------------------
_HARDCODED_JOINTS: List[dict] = [
    {   # joint1: link0 → link1
        "name": "joint1",
        "xyz":  np.array([ 0.124999999999995,  0.125000000000034,  0.1055             ]),
        "rpy":  np.array([ 0.0,                0.0,                0.0                ]),
        "axis": np.array([ 0.0,                0.0,               -1.0               ]),
    },
    {   # joint2: link1 → link2
        "name": "joint2",
        "xyz":  np.array([ 0.0225803798520988, -0.0220337569588361, 0.0489000000003376]),
        "rpy":  np.array([ 1.5707963267949,     0.0,                0.111028214146299 ]),
        "axis": np.array([ 0.0,                 0.0,               -1.0              ]),
    },
    {   # joint3: link2 → link3
        "name": "joint3",
        "xyz":  np.array([ 0.31372987976612,    0.0556095676499454, 0.0               ]),
        "rpy":  np.array([ 0.0,                 0.0,               -1.39536543135007  ]),
        "axis": np.array([ 0.0,                 0.0,               -1.0              ]),
    },
    {   # joint4: link3 → link4
        "name": "joint4",
        "xyz":  np.array([-0.119739543983627,   0.269212264652694,  0.0               ]),
        "rpy":  np.array([ 0.0,                 0.0,               -1.39405537845928  ]),
        "axis": np.array([ 0.0,                 0.0,               -1.0              ]),
    },
    {   # joint5: link4 → link5
        "name": "joint5",
        "xyz":  np.array([ 0.0250377384169529,  0.0971256101915153,-0.0244999999998483]),
        "rpy":  np.array([ 1.22364489414134,    -1.5707963267949,   0.0               ]),
        "axis": np.array([ 0.0,                  0.0,              -1.0              ]),
    },
]


# --------------------------------------------------------------------------
# 基础旋转工具
# --------------------------------------------------------------------------

def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """固定轴 XYZ RPY（弧度）→ 3×3 旋转矩阵。"""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,           cp*sr,              cp*cr  ],
    ], dtype=float)


def _axis_angle_to_matrix(axis: np.ndarray, theta: float) -> np.ndarray:
    """Rodrigues 公式：绕单位向量 axis 旋转 theta 弧度。"""
    axis = np.asarray(axis, dtype=float).ravel()[:3]
    n = np.linalg.norm(axis)
    if n < 1e-10:
        return np.eye(3)
    axis = axis / n
    kx, ky, kz = axis
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c + kx*kx*(1-c),   kx*ky*(1-c) - kz*s,  kx*kz*(1-c) + ky*s],
        [ky*kx*(1-c) + kz*s, c + ky*ky*(1-c),    ky*kz*(1-c) - kx*s],
        [kz*kx*(1-c) - ky*s, kz*ky*(1-c) + kx*s,  c + kz*kz*(1-c)  ],
    ], dtype=float)


def _pose_to_matrix(position: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """(3,) position + (3,3) rotation → 4×4 齐次变换。"""
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation
    T[:3,  3] = np.asarray(position, dtype=float).ravel()[:3]
    return T


def _matrix_to_pose(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """4×4 齐次变换 → position (3,), rotation (3,3)。"""
    return T[:3, 3].copy(), T[:3, :3].copy()


def _quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    """四元数 [x,y,z,w] → 3×3 旋转矩阵。"""
    x, y, z, w = np.asarray(quat_xyzw, dtype=float).ravel()[:4]
    return np.array([
        [1 - 2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)  ],
        [  2*(x*y+z*w),  1 - 2*(x*x+z*z),  2*(y*z-x*w)  ],
        [  2*(x*z-y*w),    2*(y*z+x*w),  1 - 2*(x*x+y*y) ],
    ], dtype=float)


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → 四元数 [x,y,z,w]。"""
    R = np.asarray(R, dtype=float)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    return q / (np.linalg.norm(q) + 1e-10)


def _matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    """3×3 旋转矩阵 → RPY (roll, pitch, yaw) 弧度。"""
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll  = np.arctan2( R[2, 1],  R[2, 2])
        pitch = np.arctan2(-R[2, 0],  sy)
        yaw   = np.arctan2( R[1, 0],  R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2],  R[1, 1])
        pitch = np.arctan2(-R[2, 0],  sy)
        yaw   = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


# --------------------------------------------------------------------------
# 运动学核心
# --------------------------------------------------------------------------

class URDFKinematics:
    """
    link0 → link5 运动学链：FK、解析雅可比（含数值验证）、数值 IK。

    使用优先级：
      1. 若传入有效的 urdf_path 则从 URDF 文件解析；
      2. 否则使用硬编码的精确参数（从 ice_cream_0208.SLDASM.urdf 提取），
         无需文件也可运行，保证与 URDF 完全一致。
    """

    def __init__(self, urdf_path: Optional[str] = None):
        self._joints: List[dict] = []
        if urdf_path is not None and os.path.isfile(urdf_path):
            self._urdf_path = os.path.abspath(urdf_path)
            self._parse_urdf()
            self._source = "urdf_file"
        else:
            # 使用内置精确参数
            self._urdf_path = urdf_path or "<hardcoded>"
            self._joints = [dict(j) for j in _HARDCODED_JOINTS]  # deep-copy
            self._source = "hardcoded"

    def _parse_urdf(self) -> None:
        """从 URDF XML 文件解析关节参数（与硬编码完全一致）。"""
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
            axis = np.array([0., 0., -1.])
            if axis_el is not None and axis_el.get("xyz"):
                axis = np.array([float(v) for v in axis_el.get("xyz").split()])
            # 部分 URDF（如 SolidWorks 导出的 SINGLE）关节轴为 "0 0 0"，导致 FK 无关节旋转、与仿真/实物严重偏差
            if np.linalg.norm(axis) < 1e-10:
                axis = np.array([0., 0., -1.])
            order.append((JOINT_NAMES.index(name), {
                "name": name, "xyz": xyz, "rpy": rpy, "axis": axis,
            }))
        order.sort(key=lambda x: x[0])
        self._joints = [x[1] for x in order]
        if len(self._joints) != NUM_JOINTS:
            raise RuntimeError(
                f"URDF 中找到 {len(self._joints)} 个关节，期望 {NUM_JOINTS}"
            )

    # ------------------------------------------------------------------
    # 单关节变换
    # ------------------------------------------------------------------

    def _joint_transform(self, idx: int, q: float) -> np.ndarray:
        """
        第 idx 个关节的 4×4 变换（父 link → 子 link）。
        T = T_origin · T_axis_rotation(q)
        """
        j = self._joints[idx]
        R_origin = _rpy_to_matrix(j["rpy"])
        R_q      = _axis_angle_to_matrix(j["axis"], q)
        return _pose_to_matrix(j["xyz"], R_origin @ R_q)

    # ------------------------------------------------------------------
    # 正向运动学
    # ------------------------------------------------------------------

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """base → ee 的 4×4 齐次变换 T_base_ee。"""
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
        return T

    def forward_kinematics_position(self, q: np.ndarray) -> np.ndarray:
        """末端在基座系下的位置 (3,)。"""
        return self.forward_kinematics(q)[:3, 3].copy()

    def forward_kinematics_rotation(self, q: np.ndarray) -> np.ndarray:
        """末端在基座系下的旋转矩阵 (3,3)。"""
        return self.forward_kinematics(q)[:3, :3].copy()

    def forward_kinematics_link4(self, q: np.ndarray) -> np.ndarray:
        """Base → link4 的 4×4 齐次变换（仅前 4 个关节）。link4 原点为关节转动中心。"""
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        T = np.eye(4, dtype=float)
        for i in range(4):
            T = T @ self._joint_transform(i, q[i])
        return T

    def forward_kinematics_position_link4(self, q: np.ndarray) -> np.ndarray:
        """
        与 link4 原点重合的空间点在基座系下的位置 (3,)。
        仅表示该点的坐标，与 link4 的姿态无关；末节朝向由机构与 q4、q5 决定。
        """
        return self.forward_kinematics_link4(q)[:3, 3].copy()

    def get_link5_length(self) -> float:
        """第五节（link4 原点 → link5 原点）长度（m）。末节向下时末端 z = link4_z - LINK5_LENGTH。"""
        return float(np.linalg.norm(self._joints[4]["xyz"]))

    def jacobian_link4_position_first3(self, q: np.ndarray) -> np.ndarray:
        """
        与 link4 原点重合的空间点位置对前 3 个关节的雅可比 J ∈ R^{3×3}。
        该点只是空间坐标，旋转与 link4 无关；仅用 joint1~3 控制该点位置（q4、q5 不参与）。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J = np.zeros((3, 3), dtype=float)
        T_accum = [np.eye(4, dtype=float)]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
            T_accum.append(T.copy())
        p_link4 = T_accum[4][:3, 3]
        for i in range(3):
            j = self._joints[i]
            R_parent = T_accum[i][:3, :3]
            p_parent = T_accum[i][:3, 3]
            p_joint_i = p_parent + R_parent @ j["xyz"]
            z_i = R_parent @ (_rpy_to_matrix(j["rpy"]) @ j["axis"])
            z_i = z_i / (np.linalg.norm(z_i) + 1e-10)
            J[:, i] = np.cross(z_i, p_link4 - p_joint_i)
        return J

    def forward_kinematics_position_joint4_motor_center(self, q: np.ndarray) -> np.ndarray:
        """
        joint4 电机几何中心在基座系下的位置 (3,)。
        = link4 原点位置 + R_link4 @ JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4。
        """
        T4 = self.forward_kinematics_link4(q)
        p_link4 = T4[:3, 3]
        R_link4 = T4[:3, :3]
        offset = JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4
        return (p_link4 + R_link4 @ offset).copy()

    def jacobian_joint4_motor_center_first3(self, q: np.ndarray) -> np.ndarray:
        """
        joint4 电机中心位置对前 3 个关节的雅可比 J ∈ R^{3×3}。
        含旋转贡献：dp/dq_i = J_link4[:,i] + omega_i × (R_link4 @ offset)。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J_link4 = self.jacobian_link4_position_first3(q)
        T_accum = [np.eye(4, dtype=float)]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
            T_accum.append(T.copy())
        R_link4 = T_accum[4][:3, :3]
        r = R_link4 @ JOINT4_MOTOR_CENTER_OFFSET_IN_LINK4
        for i in range(3):
            j = self._joints[i]
            R_parent = T_accum[i][:3, :3]
            z_i = R_parent @ (_rpy_to_matrix(j["rpy"]) @ j["axis"])
            z_i = z_i / (np.linalg.norm(z_i) + 1e-10)
            J_link4[:, i] += np.cross(z_i, r)
        return J_link4

    # ------------------------------------------------------------------
    # 几何雅可比（解析推导 + 数值差分双重实现）
    # ------------------------------------------------------------------

    def jacobian_analytical(self, q: np.ndarray) -> np.ndarray:
        """
        解析几何雅可比 J ∈ R^{6×5}。

        对于所有关节轴均为 [0,0,-1]（局部 z 轴旋转）的 URDF，
        列 i 由以下公式给出：
            J_v[:,i] = z_i × (p_ee - p_i)     ← 线速度部分
            J_w[:,i] = z_i                      ← 角速度部分
        其中 z_i 为关节 i 轴线在基座系中的方向，
             p_i  为关节 i 原点在基座系中的位置，
             p_ee 为末端在基座系中的位置。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J = np.zeros((6, NUM_JOINTS), dtype=float)

        # 计算每个关节坐标系相对于基座系的变换（累积 T）
        T_accum = [np.eye(4, dtype=float)]
        T = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            T = T @ self._joint_transform(i, q[i])
            T_accum.append(T.copy())

        p_ee = T_accum[NUM_JOINTS][:3, 3]   # 末端位置（基座系）

        for i in range(NUM_JOINTS):
            # 关节 i 的坐标系 = T_accum[i]（第 i 个关节变换前的累积变换，
            # 即父 link 在基座系下的坐标系）
            # 但 joint_transform 已将 origin 平移包含进去，所以关节 i 轴
            # 的原点在 T_accum[i+1] 的平移部分（也可在 T_accum[i] 加上 origin 平移）
            # 精确做法：关节 i 的轴方向 z_i 是 T_accum[i] 的旋转部分 × 局部轴方向
            # 而关节 i 的原点 p_i 是 T_accum[i+1] 的平移（含本关节 origin xyz 但不含旋转）
            # 
            # 等价地，考虑到 T_accum[i+1] = T_accum[i] @ T_joint_i：
            #   p_i = T_accum[i] @ [xyz_i; 1]  → 这是关节 i 轴原点在基座系中的位置
            #   z_i = T_accum[i][:3,:3] @ axis_i_local  → 轴方向在基座系中

            # 关节 i 轴原点 = T_accum[i] 施加到 origin xyz 后的位置
            # 注意：T_accum[i] 是累积到第 i 个关节「之前」的变换
            # 关节 i 的 origin 相对于父 link 的平移 = j["xyz"]
            # 因此关节 i 轴在基座系中的原点 p_i = T_accum[i] @ (j["xyz"],1)
            j = self._joints[i]
            R_parent = T_accum[i][:3, :3]   # 父 link（i-1 后）在基座系的旋转
            p_parent = T_accum[i][:3,  3]   # 父 link 原点在基座系的位置

            p_joint_i = p_parent + R_parent @ j["xyz"]   # 关节 i 轴原点（基座系）
            z_i       = R_parent @ (_rpy_to_matrix(j["rpy"]) @ j["axis"])  # 轴方向（基座系）
            z_i       = z_i / (np.linalg.norm(z_i) + 1e-10)

            J[:3, i] = np.cross(z_i, p_ee - p_joint_i)   # 线速度
            J[3:, i] = z_i                                  # 角速度

        return J

    def jacobian(self, q: np.ndarray, delta: float = 1e-7) -> np.ndarray:
        """
        数值雅可比 J ∈ R^{6×5}（中心差分，精度 O(δ²)）。
        与 jacobian_analytical 结果应高度一致（偏差 < 1e-6）。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]
        J  = np.zeros((6, NUM_JOINTS), dtype=float)

        # 使用中心差分提高精度
        for i in range(NUM_JOINTS):
            qp = q.copy(); qp[i] += delta
            qm = q.copy(); qm[i] -= delta
            Tp = self.forward_kinematics(qp)
            Tm = self.forward_kinematics(qm)

            # 线速度列（中心差分）
            J[:3, i] = (Tp[:3, 3] - Tm[:3, 3]) / (2 * delta)

            # 角速度列：通过旋转误差提取
            Rd = Tp[:3, :3] @ Tm[:3, :3].T   # ΔR ≈ R(q+δ) · R(q-δ)^T
            angle = np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1.0, 1.0))
            if angle > 1e-9:
                ax = np.array([
                    Rd[2, 1] - Rd[1, 2],
                    Rd[0, 2] - Rd[2, 0],
                    Rd[1, 0] - Rd[0, 1],
                ], dtype=float)
                J[3:, i] = (ax / (2 * np.sin(angle))) * (angle / (2 * delta))

        return J

    def check_jacobian_consistency(
        self,
        q: np.ndarray,
        tol: float = 1e-5,
    ) -> Tuple[float, float, bool]:
        """
        验证解析雅可比与数值雅可比的一致性。
        返回 (max_abs_error, max_rel_error, passed)。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J_anal = self.jacobian_analytical(q)
        J_num  = self.jacobian(q)
        diff   = np.abs(J_anal - J_num)
        scale  = np.maximum(np.abs(J_anal), np.abs(J_num)) + 1e-8
        max_abs = float(diff.max())
        max_rel = float((diff / scale).max())
        return max_abs, max_rel, max_abs < tol

    def manipulability(self, q: np.ndarray) -> Tuple[float, float]:
        """
        可操纵性指标：(最小奇异值, 条件数)。
        使用解析雅可比，最小奇异值 > 0 表示非奇异；值越大可操纵性越好。
        典型奇异阈值 < 0.01。
        """
        q = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        J_pos = self.jacobian_analytical(q)[:3, :]
        sv    = np.linalg.svd(J_pos, compute_uv=False)
        cond  = sv.max() / (sv.min() + 1e-12)
        return float(sv.min()), float(cond)

    # ------------------------------------------------------------------
    # 逆运动学（数值 IK，阻尼最小二乘）
    # ------------------------------------------------------------------

    def inverse_kinematics(
        self,
        target_position: np.ndarray,
        target_orientation: Optional[np.ndarray] = None,
        q_init: Optional[np.ndarray] = None,
        max_iter: int = 150,
        pos_tol: float = 1e-4,
        ori_tol: float = 1e-3,
        damping: float = 1e-2,
        position_only: bool = False,
        joint_limits_lower: Optional[np.ndarray] = None,
        joint_limits_upper: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, bool]:
        """
        阻尼最小二乘数值 IK（DLS）。目标位姿均在基座系下。
        target_orientation: 3×3 旋转矩阵（可选）。
        返回 (q_solution, success)。
        """
        target_position = np.asarray(target_position, dtype=float).ravel()[:3]
        q = np.zeros(NUM_JOINTS, dtype=float) if q_init is None else \
            np.asarray(q_init, dtype=float).ravel()[:NUM_JOINTS].copy()

        ll = JOINT_LIMITS_LOWER if joint_limits_lower is None else np.asarray(joint_limits_lower)
        ul = JOINT_LIMITS_UPPER if joint_limits_upper is None else np.asarray(joint_limits_upper)

        for _ in range(max_iter):
            T   = self.forward_kinematics(q)
            p   = T[:3, 3]
            R   = T[:3, :3]
            ep  = target_position - p

            if position_only:
                err = ep
                J   = self.jacobian(q)[:3, :]
            else:
                if target_orientation is not None:
                    R_des = np.asarray(target_orientation).reshape(3, 3)
                    R_err = R_des @ R.T
                    angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0))
                    if angle > 1e-8:
                        ax  = np.array([R_err[2,1]-R_err[1,2],
                                        R_err[0,2]-R_err[2,0],
                                        R_err[1,0]-R_err[0,1]], dtype=float)
                        eo  = ax / (2 * np.sin(angle)) * angle
                    else:
                        eo = np.zeros(3)
                    err = np.concatenate([ep, eo])
                    J   = self.jacobian(q)
                else:
                    err = ep
                    J   = self.jacobian(q)[:3, :]

            # 收敛判断
            if np.linalg.norm(ep) < pos_tol:
                if position_only or target_orientation is None:
                    return np.clip(q, ll, ul), True
                if np.linalg.norm(err[3:]) < ori_tol:
                    return np.clip(q, ll, ul), True

            # DLS 更新
            Jt  = J.T
            JJt = J @ Jt + (damping ** 2) * np.eye(J.shape[0])
            dq  = Jt @ np.linalg.solve(JJt, err)
            q   = np.clip(q + dq, ll, ul)

        return np.clip(q, ll, ul), False


# --------------------------------------------------------------------------
# 模块级便捷接口
# --------------------------------------------------------------------------

_default_kin: Optional[URDFKinematics] = None


def get_kinematics(urdf_path: Optional[str] = None) -> URDFKinematics:
    """获取（或创建）全局默认 URDFKinematics 实例。"""
    global _default_kin
    if _default_kin is None:
        _default_kin = URDFKinematics(urdf_path)
    return _default_kin


def base_to_ee_pose(q: np.ndarray, urdf_path: Optional[str] = None
                    ) -> Tuple[np.ndarray, np.ndarray]:
    """基座系下末端位姿：position (3,), rotation (3,3)。"""
    return _matrix_to_pose(get_kinematics(urdf_path).forward_kinematics(q))


def ik_base_frame(
    target_position: np.ndarray,
    target_orientation: Optional[np.ndarray] = None,
    q_init: Optional[np.ndarray] = None,
    position_only: bool = False,
    **kwargs,
) -> Tuple[np.ndarray, bool]:
    """基座系下目标位姿的逆解。"""
    return get_kinematics(kwargs.pop("urdf_path", None)).inverse_kinematics(
        target_position,
        target_orientation=target_orientation,
        q_init=q_init,
        position_only=position_only,
        **kwargs,
    )


# --------------------------------------------------------------------------
# 独立自测（无需 Isaac Sim）
# --------------------------------------------------------------------------

if __name__ == "__main__":
    print("=" * 60)
    print("URDFKinematics 自测（ice_cream_0208.SLDASM）")
    print("=" * 60)

    # 尝试从上传路径加载；否则用硬编码参数
    import sys
    _possible_urdf = [
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "ice_cream_0208_SLDASM.urdf"),
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "ice_cream_0208.SLDASM", "urdf",
                     "ice_cream_0208.SLDASM.urdf"),
    ]
    _urdf = next((p for p in _possible_urdf if os.path.isfile(p)), None)
    kin = URDFKinematics(urdf_path=_urdf)
    print(f"参数来源: {kin._source}  ({kin._urdf_path})")

    # 1. FK(q=0)
    q0 = np.zeros(NUM_JOINTS)
    T0 = kin.forward_kinematics(q0)
    p0, R0 = _matrix_to_pose(T0)
    print(f"\n[FK] q=0 末端位置: {p0}")
    print(f"     末端旋转矩阵:\n{R0}")

    # 2. 解析 vs 数值雅可比一致性
    q_test = np.array([0.5301, 0.5483, -1.1999, 1.0894, 0.7541])
    err_abs, err_rel, ok = kin.check_jacobian_consistency(q_test)
    print(f"\n[Jacobian 一致性] q_test={q_test}")
    print(f"  解析 vs 数值最大绝对误差: {err_abs:.2e}")
    print(f"  解析 vs 数值最大相对误差: {err_rel:.2e}")
    print(f"  通过(< 1e-5): {ok}")

    # 3. IK 验证
    p_target = np.array([0.47, 0.03, 0.22])
    q_ik, success = kin.inverse_kinematics(p_target, q_init=np.zeros(NUM_JOINTS),
                                           position_only=True)
    p_ik = kin.forward_kinematics_position(q_ik)
    print(f"\n[IK] 目标: {p_target}")
    print(f"     解: {q_ik}  收敛={success}")
    print(f"     FK(q_ik): {p_ik}  误差={np.linalg.norm(p_ik-p_target)*1000:.3f}mm")

    # 4. 可操纵性
    sv_min, cond = kin.manipulability(q_test)
    print(f"\n[可操纵性] q_test: 最小奇异值={sv_min:.4f}, 条件数={cond:.2f}")