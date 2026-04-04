"""基于 URDF + Pinocchio 的广义重力项 τ_g(q)。"""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Sequence

import numpy as np

_ROBOTARM_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = (
    _ROBOTARM_ROOT
    / "icecream_arm_model"
    / "ice_cream_v8.SLDASM"
    / "urdf"
    / "ice_cream_v8.SLDASM.urdf"
)
# package://ice_cream_v8.SLDASM/... 在 icecream_arm_model/ice_cream_v8.SLDASM/ 下解析
_PACKAGE_DIRS = [str(_ROBOTARM_ROOT / "icecream_arm_model")]


def _pin():
    import pinocchio as pin

    if not hasattr(pin, "buildModelsFromUrdf"):
        raise ImportError(
            "当前 `pinocchio` 不是 INRIA Pinocchio：PyPI 上同名包是 nose 测试插件。\n"
            "请二选一：\n"
            "  pip uninstall pinocchio -y && pip install pin\n"
            "  conda install -c conda-forge pinocchio"
        )
    return pin


@lru_cache(maxsize=1)
def _model_and_data():
    pin = _pin()
    if not URDF_PATH.is_file():
        raise FileNotFoundError(f"URDF 不存在: {URDF_PATH}")
    model, _, _ = pin.buildModelsFromUrdf(str(URDF_PATH), package_dirs=_PACKAGE_DIRS)
    return model, model.createData()


def get_model():
    """返回 Pinocchio Model（首次调用时加载 URDF）。"""
    m, _ = _model_and_data()
    return m


def q_joint_labels() -> tuple[str, ...]:
    """Pinocchio 中各关节名（不含 universe），顺序与 nv / τ 一致。"""
    model, _ = _model_and_data()
    return tuple(model.names[1 : model.njoints])


def _q_from_arm_joint_rad(
    joint_rad: np.ndarray,
    finger_L_m: float,
    finger_R_m: float,
) -> np.ndarray:
    """内部：5 个臂关节弧度 + 两指米 → 完整 q（按 URDF：continuous 用 cos/sin，revolute 用弧度）。"""
    pin = _pin()
    model, _ = _model_and_data()
    q = pin.neutral(model)
    joint_rad = np.asarray(joint_rad, dtype=float).reshape(5)
    for arm_i in range(5):
        jid = arm_i + 1
        j = model.joints[jid]
        th = float(joint_rad[arm_i])
        if j.nq == 2:
            q[j.idx_q] = np.cos(th)
            q[j.idx_q + 1] = np.sin(th)
        elif j.nq == 1:
            q[j.idx_q] = th
        else:
            raise RuntimeError(
                f"关节 {model.names[jid]} 的 nq={j.nq} 未支持，请检查 URDF / Pinocchio 模型"
            )
    jl = model.joints[model.getJointId("FINGER_L")]
    jr = model.joints[model.getJointId("FINGER_RIGHT")]
    q[jl.idx_q] = float(finger_L_m)
    q[jr.idx_q] = float(finger_R_m)
    return q


def q_from_arm_joint_deg_x100(
    joint_deg_x100: Sequence[float],
    finger_L_m: float = 0.0,
    finger_R_m: float = 0.0,
) -> np.ndarray:
    """
    由 5 个臂关节「角度×100」与两指位移（米）构造完整广义坐标 q（长度 model.nq）。

    joint_deg_x100 与固件/telemetry 常见约定一致：物理角度(度) × 100，例如 9000 表示 90°。
    v7：joint1..joint3 为 continuous（各 2 维 cos/sin），joint4/joint5 为 revolute（各 1 维）；
    两指 prismatic 各占 1 维（model.nq 一般为 10）。
    """
    joint_rad = np.deg2rad(np.asarray(joint_deg_x100, dtype=float).reshape(5) / 100.0)
    return _q_from_arm_joint_rad(joint_rad, finger_L_m, finger_R_m)


def get_gravity_torques(q: Sequence[float] | np.ndarray) -> np.ndarray:
    """
    计算广义重力项 g(q)，与 RNEA 中 τ_g 一致（无速度加速度时）。

    Args:
        q: 广义坐标，长度须为 model.nq（v7 当前为 10：joint1–3 各 2 维 cos/sin，joint4–5 各 1 维，
           两指各 1 维），内部单位为 Pinocchio 约定（转动为弧度等）。
           若要从「角度×100」构造 q，请用 q_from_arm_joint_deg_x100(...)，
           或直接用 get_gravity_torques_arm_deg_x100。

    Returns:
        shape (model.nv,) 的 τ_g（当前 nv=7）；转动关节为 Nm，移动关节为沿轴广义力。
    """
    pin = _pin()
    model, data = _model_and_data()
    q = np.asarray(q, dtype=float).reshape(model.nq)
    pin.computeGeneralizedGravity(model, data, q)
    # data.g 每帧由 Pinocchio 覆写；返回副本，调用方无跨帧累积或「缓冲溢出」
    return data.g.copy()


def get_gravity_torques_arm_deg_x100(
    joint_deg_x100: Sequence[float],
    finger_L_m: float = 0.0,
    finger_R_m: float = 0.0,
) -> np.ndarray:
    """由臂关节「角度×100」5 个数与可选夹指位移（米）计算 τ_g，返回长度 nv 的向量。"""
    return get_gravity_torques(q_from_arm_joint_deg_x100(joint_deg_x100, finger_L_m, finger_R_m))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="计算 v8 机械臂重力项 τ_g。关节角为「度×100」（与固件一致），例如 9000=90°。"
    )
    parser.add_argument(
        "joint_deg_x100",
        nargs="*",
        type=float,
        metavar="DEGx100",
        help="5 个数：joint1～joint5 的角度×100；不写则全 0",
    )
    parser.add_argument(
        "--finger-l",
        type=float,
        default=0.0,
        metavar="M",
        help="左指 prismatic 位移（米），默认 0",
    )
    parser.add_argument(
        "--finger-r",
        type=float,
        default=0.0,
        metavar="M",
        help="右指 prismatic 位移（米），默认 0",
    )
    args = parser.parse_args()
    if len(args.joint_deg_x100) not in (0, 5):
        parser.error("关节角须为 0 个（全零）或 5 个「度×100」数值")
    joints = list(args.joint_deg_x100) if len(args.joint_deg_x100) == 5 else [0.0] * 5

    q_test = q_from_arm_joint_deg_x100(joints, args.finger_l, args.finger_r)
    tau = get_gravity_torques(q_test)
    print("URDF:", URDF_PATH)
    print("nq =", get_model().nq, "nv =", get_model().nv)
    print("关节顺序 (与 τ 分量对应):", q_joint_labels())
    print("输入 joint1..5 (度×100):", joints, "| finger_L/R (m):", args.finger_l, args.finger_r)
    print("τ_g (Nm / 广义力):", tau)
