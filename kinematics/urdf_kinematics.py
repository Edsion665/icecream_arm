"""URDF 解析与 link4 几何解耦 FK/IK。"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from typing import List, Optional, Tuple

import numpy as np

from ..config import Q4_GEOMETRIC_OFFSET_DEG, Q4_GEOMETRIC_Q23_COEFF

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
NUM_JOINTS = 5

JOINT_LIMITS_LOWER = np.array([-3.14] * NUM_JOINTS, dtype=float)
JOINT_LIMITS_UPPER = np.array([3.14] * NUM_JOINTS, dtype=float)

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
    t = np.eye(4, dtype=float)
    t[:3, :3] = rotation
    t[:3, 3] = np.asarray(position, dtype=float).ravel()[:3]
    return t


def q4_geometric_decouple(q: np.ndarray) -> float:
    """几何解耦：q4 = Q4_OFFSET_RAD + Q4_Q23_COEFF * (q2+q3)，并夹在 joint4 限位内。"""
    qv = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
    return float(
        np.clip(
            Q4_OFFSET_RAD + Q4_Q23_COEFF * (qv[1] + qv[2]),
            JOINT_LIMITS_LOWER[3],
            JOINT_LIMITS_UPPER[3],
        )
    )


class URDFKinematics:
    """link0→link5：link4 原点 FK 与几何解耦 IK（与控制桥约定一致）。"""

    def __init__(self, urdf_path: Optional[str] = None) -> None:
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
        r_origin = _rpy_to_matrix(j["rpy"])
        r_q = _axis_angle_to_matrix(j["axis"], q)
        return _pose_to_matrix(j["xyz"], r_origin @ r_q)

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        qv = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        t = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            t = t @ self._joint_transform(i, qv[i])
        return t

    def forward_kinematics_link4(self, q: np.ndarray) -> np.ndarray:
        qv = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        t = np.eye(4, dtype=float)
        for i in range(4):
            t = t @ self._joint_transform(i, qv[i])
        return t

    def forward_kinematics_position_link4(self, q: np.ndarray) -> np.ndarray:
        return self.forward_kinematics_link4(q)[:3, 3].copy()

    def q_from_q3_geometric_decouple(self, q3: np.ndarray, q5_fixed: float = 0.0) -> np.ndarray:
        q3v = np.asarray(q3, dtype=float).ravel()[:3]
        q = np.zeros(NUM_JOINTS, dtype=float)
        q[:3] = q3v
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
        qv = np.asarray(q, dtype=float).ravel()[:NUM_JOINTS]
        jac = np.zeros((3, NUM_JOINTS), dtype=float)
        t_accum = [np.eye(4, dtype=float)]
        t = np.eye(4, dtype=float)
        for i in range(NUM_JOINTS):
            t = t @ self._joint_transform(i, qv[i])
            t_accum.append(t.copy())
        p_link4 = t_accum[4][:3, 3]
        for i in range(4):
            j = self._joints[i]
            r_parent = t_accum[i][:3, :3]
            p_parent = t_accum[i][:3, 3]
            p_joint_i = p_parent + r_parent @ j["xyz"]
            z_i = r_parent @ (_rpy_to_matrix(j["rpy"]) @ j["axis"])
            z_i = z_i / (np.linalg.norm(z_i) + 1e-10)
            jac[:, i] = np.cross(z_i, p_link4 - p_joint_i)
        return jac

    def jacobian_link4_geometric_decouple(self, q3: np.ndarray, q5_fixed: float = 0.0) -> np.ndarray:
        q = self.q_from_q3_geometric_decouple(q3, q5_fixed)
        j_link4_5 = self.jacobian_link4_origin_full5(q)
        m = np.zeros((5, 3), dtype=float)
        m[0, 0], m[1, 1], m[2, 2] = 1.0, 1.0, 1.0
        c = float(Q4_Q23_COEFF)
        m[3, 1], m[3, 2] = c, c
        return (j_link4_5 @ m).copy()

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
            _, p_j4 = self.forward_kinematics_link4_geometric_decouple(q3, q5_fixed)
            ep = target_position - p_j4
            if np.linalg.norm(ep) < pos_tol:
                return self.q_from_q3_geometric_decouple(q3, q5_fixed), True
            j = self.jacobian_link4_geometric_decouple(q3, q5_fixed)
            jt = j.T
            jjt = j @ jt + (damping**2) * np.eye(3)
            dq3 = jt @ np.linalg.solve(jjt, ep)
            q3 = np.clip(q3 + dq3, ll, ul)

        q_full = self.q_from_q3_geometric_decouple(q3, q5_fixed)
        return q_full, False
