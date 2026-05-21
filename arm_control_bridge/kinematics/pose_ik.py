"""笛卡尔 pose 鲁棒 IK：多初值 + 可选 y 镜像回退。"""

from __future__ import annotations

from typing import List, Optional, Tuple

import numpy as np

from ..config import CONFIG, IK_CONFIG
from .urdf_kinematics import (
    JOINT_LIMITS_LOWER,
    JOINT_LIMITS_UPPER,
    Q4_OFFSET_RAD,
    Q4_Q23_COEFF,
    URDFKinematics,
)

# 与关节限位一致，避免 q4 几何角略超 ±160° 仍可达时被误拒
_Q4_MIN = float(JOINT_LIMITS_LOWER[3])
_Q4_MAX = float(JOINT_LIMITS_UPPER[3])


def _q4_in_range(q2: float, q3: float) -> bool:
    q4c = float(Q4_OFFSET_RAD + Q4_Q23_COEFF * (q2 + q3))
    return _Q4_MIN <= q4c <= _Q4_MAX


def _try_ik_once(
    kin: URDFKinematics,
    target: np.ndarray,
    q3_init: np.ndarray,
    *,
    q5_fixed: float,
    max_iter: int,
    pos_tol: float,
    damping: float,
) -> Tuple[Optional[np.ndarray], float]:
    """单次 IK；返回 (q_full, fk_err) 或 (None, inf)。"""
    q_sol, ok = kin.inverse_kinematics_link4_geometric_decouple(
        target,
        q3_init=q3_init,
        q5_fixed=q5_fixed,
        max_iter=max_iter,
        pos_tol=pos_tol,
        damping=damping,
    )
    if not ok:
        return None, float("inf")
    if not _q4_in_range(float(q_sol[1]), float(q_sol[2])):
        return None, float("inf")
    # 保持 IK 解出的 j1；勿强行改为 -atan2(y,x)，否则与 q2/q3 不匹配（obs2 抓取常见 7cm 误差）
    q4c = float(Q4_OFFSET_RAD + Q4_Q23_COEFF * (q_sol[1] + q_sol[2]))
    q_sol[3] = float(np.clip(q4c, _Q4_MIN, _Q4_MAX))
    p = kin.forward_kinematics_position_link4(q_sol)
    err = float(np.linalg.norm(p - target))
    return q_sol, err


def _seed_list(q_full_seed: np.ndarray, target: np.ndarray) -> List[np.ndarray]:
    seeds: List[np.ndarray] = []
    q3 = np.asarray(q_full_seed, dtype=float).ravel()[:3].copy()
    seeds.append(q3)
    seeds.append(np.zeros(3, dtype=float))
    j1 = -np.arctan2(float(target[1]), float(target[0]))
    for base in (
        q3,
        np.array([j1, q3[1], q3[2]], dtype=float),
        np.deg2rad([0.0, 90.0, -180.0]),
        np.deg2rad([-90.0, 10.0, -60.0]),
        np.deg2rad([0.0, 20.0, -80.0]),
    ):
        seeds.append(np.asarray(base, dtype=float).ravel()[:3].copy())
    # 去重（粗略）
    uniq: List[np.ndarray] = []
    for s in seeds:
        if not any(np.allclose(s, u, atol=1e-3) for u in uniq):
            uniq.append(s)
    return uniq


def solve_pose_link4(
    kin: URDFKinematics,
    xyz: np.ndarray,
    q_full_seed: np.ndarray,
    *,
    q5_fixed: float = 0.0,
) -> Tuple[bool, Optional[np.ndarray], np.ndarray, str]:
    """求解 link4 目标位姿。

    Returns:
        (ok, q_full, xyz_used, reason)
    """
    xyz = np.asarray(xyz, dtype=float).ravel()[:3]
    candidates: List[Tuple[np.ndarray, str]] = [(xyz, "direct")]
    if IK_CONFIG.pose_ik_try_mirror_y and abs(float(xyz[1])) > 1e-4:
        mirrored = xyz.copy()
        mirrored[1] = -mirrored[1]
        candidates.append((mirrored, "mirror_y"))

    dampings = (IK_CONFIG.ik_damping, IK_CONFIG.ik_damping * 2.0, 0.15)
    max_iters = (IK_CONFIG.ik_max_iter, IK_CONFIG.ik_max_iter * 2)

    best_q: Optional[np.ndarray] = None
    best_err = float("inf")
    best_tag = ""
    best_xyz = xyz.copy()

    for target, tag in candidates:
        for q3_init in _seed_list(q_full_seed, target):
            for damp in dampings:
                for mi in max_iters:
                    q_sol, err = _try_ik_once(
                        kin,
                        target,
                        q3_init,
                        q5_fixed=q5_fixed,
                        max_iter=mi,
                        pos_tol=IK_CONFIG.ik_pos_tol,
                        damping=damp,
                    )
                    if q_sol is None:
                        continue
                    if err < best_err:
                        best_err = err
                        best_q = q_sol
                        best_tag = tag
                        best_xyz = target.copy()

    accept_tol = max(IK_CONFIG.ik_pos_tol * 50.0, 0.002)
    if best_q is None or best_err > accept_tol:
        return False, None, xyz, "no_converge"
    reason = best_tag if best_tag != "direct" else "ok"
    return True, best_q, best_xyz, reason
