"""Gravity compensation calculator for V2."""

from __future__ import annotations

import math
from functools import lru_cache
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from .config import GRAVITY_AXIS_SCALE, MOTOR_AXIS_SIGN
from .domain.mapping import (
    joint_tau_to_motor_tau,
    motor_delta_rad_to_joint_deg_x100,
)
from .gravity.mlp_gravity import run_mlp


_HERE = Path(__file__).resolve()
_PROJECT_ROOT = _HERE.parent
_MODEL_ROOT = _PROJECT_ROOT / "robotarm"

# Path to the residual MLP model trained in gravity/
_RESIDUAL_MODEL_PATH = _PROJECT_ROOT / "gravity" / "gravity_residual_model.npz"

# ---------------------------------------------------------------------------
# link0→link5 analytic FK (must match test_sim.py for WebSocket / Isaac Sim)
# Joint origins/axes from ice_cream_v12.SLDASM.urdf — same chain as test_sim.fk_link5.
# Fixed origin rotations use R = Rx·Ry·Rz(rpy) (not Rz·Ry·Rx). If Pinocchio/URDF disagree,
# numeric ``rpy`` in this table may need re-export for this convention.
# ---------------------------------------------------------------------------

_LINK5_ANALYTIC_JOINTS: tuple[dict[str, np.ndarray], ...] = (
    {
        "xyz": np.array([0.0, 0.0, 0.067], dtype=float),
        "rpy": np.array([0.0, 0.0, 0.0], dtype=float),
        "axis": np.array([0.0, 0.0, -1.0], dtype=float),
    },
    {
        "xyz": np.array([0.02, -0.03115, 0.0585], dtype=float),
        "rpy": np.array([1.5707963267949, 0.0, 1.5707963267949], dtype=float),
        "axis": np.array([0.0, 0.0, 1.0], dtype=float),
    },
    {
        "xyz": np.array([0.31862, 0.0, 0.0], dtype=float),
        "rpy": np.array([0.0, 0.0, 0.0], dtype=float),
        "axis": np.array([0.0, 0.0, 1.0], dtype=float),
    },
    {
        "xyz": np.array([0.29035, 0.076833, -0.00365], dtype=float),
        "rpy": np.array([0.0, 0.0, 0.0], dtype=float),
        "axis": np.array([0.0, 0.0, 1.0], dtype=float),
    },
    {
        "xyz": np.array([0.09985, 0.0, -0.028], dtype=float),
        "rpy": np.array([1.5707963267948966, 1.5707963267948966, 1.5707963267948966*2], dtype=float),
        "axis": np.array([0.0, 0.0, 1.0], dtype=float),
    },
)


def _link5_fk_rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    """Fixed joint origin rotation: R = Rx(roll) @ Ry(pitch) @ Rz(yaw), rpy = (roll,pitch,yaw) rad."""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rx @ ry @ rz


def _link5_fk_axis_angle_to_matrix(axis: np.ndarray, theta: float) -> np.ndarray:
    axis = axis / (np.linalg.norm(axis) + 1e-10)
    kx, ky, kz = float(axis[0]), float(axis[1]), float(axis[2])
    c, s = math.cos(theta), math.sin(theta)
    return np.array(
        [
            [c + kx * kx * (1 - c), kx * ky * (1 - c) - kz * s, kx * kz * (1 - c) + ky * s],
            [ky * kx * (1 - c) + kz * s, c + ky * ky * (1 - c), ky * kz * (1 - c) - kx * s],
            [kz * kx * (1 - c) - ky * s, kz * ky * (1 - c) + kx * s, c + kz * kz * (1 - c)],
        ],
        dtype=float,
    )


def _link5_fk_joint_T(idx: int, q: float) -> np.ndarray:
    j = _LINK5_ANALYTIC_JOINTS[idx]
    t = np.eye(4, dtype=float)
    t[:3, :3] = _link5_fk_rpy_to_matrix(j["rpy"]) @ _link5_fk_axis_angle_to_matrix(j["axis"], q)
    t[:3, 3] = j["xyz"]
    return t


def _link5_fk_chain(q: np.ndarray) -> np.ndarray:
    """Homogeneous ^0T_5: link5 frame expressed in link0 (URDF base), same as test_sim.fk_link5."""
    t = np.eye(4, dtype=float)
    for i in range(5):
        t = t @ _link5_fk_joint_T(i, float(q[i]))
    return t


def _link5_fk_T_np(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
) -> np.ndarray:
    """URDF analytic chain (numpy): ^0T_5, link5 frame in link0. Not serialized on WebSocket."""
    if len(arm4_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm4_rad and calibration_rad must contain 4 values")
    q = _q_feedback_to_link5_fk_sim(arm4_rad, joint5_rel_deg, calibration_rad)
    return _link5_fk_chain(q)


def compute_link5_fk_T_np(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
) -> np.ndarray:
    """Public alias for offline / test_sim: same ^0T_5 as analytic URDF chain."""
    return _link5_fk_T_np(arm4_rad, joint5_rel_deg, calibration_rad)


def compute_link5_rpy_xyz_link0(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
) -> tuple[list[float], list[float]]:
    """link5 relative to link0: roll,pitch,yaw rad + translation (m).

    Euler satisfies ^0R_5 ≈ Rx(roll)Ry(pitch)Rz(yaw) via ``link5_rpy_from_R_rxryrz_np``;
    position is ^0t_5 = first three components of T's last column.
    """
    T = _link5_fk_T_np(arm4_rad, joint5_rel_deg, calibration_rad)
    R = T[:3, :3]
    roll, pitch, yaw = link5_rpy_from_R_rxryrz_np(R)
    xyz = [float(T[0, 3]), float(T[1, 3]), float(T[2, 3])]
    return [roll, pitch, yaw], xyz


def compute_link_chain_rpy_xyz_steps_link0(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
) -> list[dict[str, Any]]:
    """Cumulative pose in link0 after each joint (debug / per-link tuning).

    After applying joint ``i`` (0..4), returns ^0T for that child link frame as
    ``xyz_m`` + ``rpy_rad`` from ``link5_rpy_from_R_rxryrz_np`` (same as link5 broadcast),
    plus ``q_rad`` used on that joint and URDF-style 1-based ``joint`` index (1..5).
    """
    if len(arm4_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm4_rad and calibration_rad must contain 4 values")
    q = _q_feedback_to_link5_fk_sim(arm4_rad, joint5_rel_deg, calibration_rad)
    steps: list[dict[str, Any]] = []
    t = np.eye(4, dtype=float)
    for i in range(5):
        t = t @ _link5_fk_joint_T(i, float(q[i]))
        R = t[:3, :3]
        roll, pitch, yaw = link5_rpy_from_R_rxryrz_np(R)
        steps.append(
            {
                "joint": i + 1,
                "q_rad": float(q[i]),
                "xyz_m": [float(t[0, 3]), float(t[1, 3]), float(t[2, 3])],
                "rpy_rad": [roll, pitch, yaw],
            }
        )
    return steps


def link5_rpy_from_R_rxryrz_np(R: np.ndarray) -> tuple[float, float, float]:
    """Inverse of R = Rx(roll) @ Ry(pitch) @ Rz(yaw). Returns (roll, pitch, yaw) rad.

    Matches ``_link5_fk_rpy_to_matrix`` / WebSocket ``link5_rpy_rad`` convention.
    """
    r = np.asarray(R, dtype=float).reshape(3, 3)
    r00, r01 = float(r[0, 0]), float(r[0, 1])
    r02 = float(r[0, 2])
    r10, r11 = float(r[1, 0]), float(r[1, 1])
    r12 = float(r[1, 2])
    r22 = float(r[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, r02)))
    cp = math.cos(pitch)
    if abs(cp) < 1e-8:
        roll = math.atan2(r10, r11)
        yaw = 0.0
    else:
        yaw = math.atan2(-r01, r00)
        roll = math.atan2(-r12, r22)
    return roll, pitch, yaw


def link5_rpy_intrinsic_zyx_from_R_np(R: np.ndarray) -> tuple[float, float, float]:
    """Deprecated alias: use ``link5_rpy_from_R_rxryrz_np`` (project now uses Rx·Ry·Rz)."""
    return link5_rpy_from_R_rxryrz_np(R)


def link5_hmat_row_major_list_from_rpy_xyz_rxryrz(
    rpy_rad: Sequence[float],
    xyz_m: Sequence[float],
) -> list[list[float]]:
    """^0T_5: rotation R = Rx·Ry·Rz(roll,pitch,yaw), translation (x,y,z) in link0 (m).

    Row-major 4×4 nested lists for JSON / ``pi2camera`` ``link5_hmat``. Matches the
    convention used to derive ``link5_rpy_rad`` / ``link5_xyz_m`` from full FK.
    """
    rpy = np.asarray(rpy_rad, dtype=float).reshape(3)
    xyz = np.asarray(xyz_m, dtype=float).reshape(3)
    R = _link5_fk_rpy_to_matrix(rpy)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return [[float(T[r, c]) for c in range(4)] for r in range(4)]


def _q_feedback_to_link5_fk_sim(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
) -> np.ndarray:
    """Same joint vector as test_sim.fb_to_q (sign * (motor - cal), joint5 in rad)."""
    q = np.zeros(5, dtype=float)
    for i in range(4):
        q[i] = float(MOTOR_AXIS_SIGN[i]) * (float(arm4_rad[i]) - float(calibration_rad[i]))
    q[4] = math.radians(float(joint5_rel_deg))
    return q


def _resolve_model_root() -> Path:
    urdf = _MODEL_ROOT / "icecream_arm_model" / "ice_cream_v12.SLDASM" / "urdf" / "ice_cream_v12.SLDASM.urdf"
    if urdf.is_file():
        return _MODEL_ROOT
    raise FileNotFoundError(
        "Cannot find robot model assets under "
        "`icecreamPi/robotarm/icecream_arm_model/...`."
    )


def _pin():
    import pinocchio as pin

    if not hasattr(pin, "buildModelsFromUrdf"):
        raise ImportError(
            "Installed `pinocchio` is not INRIA Pinocchio. Use one of:\n"
            "  pip uninstall pinocchio -y && pip install pin\n"
            "  conda install -c conda-forge pinocchio"
        )
    return pin


@lru_cache(maxsize=1)
def _model_and_data():
    pin = _pin()
    model_root = _resolve_model_root()
    urdf_path = model_root / "icecream_arm_model" / "ice_cream_v12.SLDASM" / "urdf" / "ice_cream_v12.SLDASM.urdf"
    package_dirs = [str(model_root / "icecream_arm_model")]
    model, _, _ = pin.buildModelsFromUrdf(str(urdf_path), package_dirs=package_dirs)
    return model, model.createData()


@lru_cache(maxsize=1)
def _load_residual_model() -> dict | None:
    """Lazy-load the residual MLP weights once; return None if file missing."""
    if not _RESIDUAL_MODEL_PATH.is_file():
        return None
    d = np.load(_RESIDUAL_MODEL_PATH)
    return {k: d[k] for k in d.files}


def _residual_delta(arm_rad: Sequence[float]) -> tuple[float, float, float, float]:
    """Run residual MLP and return delta_tau (4,) to add to pinocchio output."""
    model = _load_residual_model()
    if model is None:
        return (0.0, 0.0, 0.0, 0.0)
    x = (np.asarray(arm_rad, dtype=np.float64).reshape(1, 4) - model["x_mean"]) / model["x_std"]
    h = np.tanh(x @ model["w1"] + model["b1"])
    y_n = h @ model["w2"] + model["b2"]
    y = (y_n * model["y_std"] + model["y_mean"]).reshape(4)
    return (float(y[0]), float(y[1]), float(y[2]), float(y[3]))


def _q_from_arm_joint_rad(
    joint_rad: np.ndarray,
    finger_l_m: float,
    finger_r_m: float,
) -> np.ndarray:
    pin = _pin()
    model, _ = _model_and_data()
    q = pin.neutral(model)
    joint_rad = np.asarray(joint_rad, dtype=float).reshape(5)
    for arm_i in range(5):
        jid = arm_i + 1
        j = model.joints[jid]
        theta = float(joint_rad[arm_i])
        if j.nq == 2:
            q[j.idx_q] = np.cos(theta)
            q[j.idx_q + 1] = np.sin(theta)
        elif j.nq == 1:
            q[j.idx_q] = theta
        else:
            raise RuntimeError(f"Unsupported joint nq={j.nq} for {model.names[jid]}")

    joint_l = model.joints[model.getJointId("FINGER_L")]
    joint_r = model.joints[model.getJointId("FINGER_RIGHT")]
    q[joint_l.idx_q] = float(finger_l_m)
    q[joint_r.idx_q] = float(finger_r_m)
    return q


def _get_gravity_torques_arm_deg_x100(
    joint_deg_x100: Sequence[float],
    finger_l_m: float = 0.0,
    finger_r_m: float = 0.0,
) -> np.ndarray:
    pin = _pin()
    model, data = _model_and_data()
    joint_rad = np.deg2rad(np.asarray(joint_deg_x100, dtype=float).reshape(5) / 100.0)
    q = _q_from_arm_joint_rad(joint_rad, finger_l_m, finger_r_m)
    q = np.asarray(q, dtype=float).reshape(model.nq)
    pin.computeGeneralizedGravity(model, data, q)
    return data.g.copy()


def _arm_rad_to_q_with_calibration(
    arm_rad: Sequence[float],
    calibration_rad: Sequence[float],
    finger_l_m: float = 0.0,
    finger_r_m: float = 0.0,
) -> np.ndarray:
    if len(arm_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm_rad and calibration_rad must contain 4 values")
    delta = [float(arm_rad[i]) - float(calibration_rad[i]) for i in range(4)]
    joint_deg_x100 = motor_delta_rad_to_joint_deg_x100(delta)
    joint_rad = np.deg2rad(np.asarray(joint_deg_x100, dtype=float).reshape(5) / 100.0)
    return _q_from_arm_joint_rad(joint_rad, finger_l_m, finger_r_m)


def _arm4_and_joint5_rel_deg_to_q(
    arm4_rad: Sequence[float],
    joint5_rel_deg: float,
    calibration_rad: Sequence[float],
    finger_l_m: float = 0.0,
    finger_r_m: float = 0.0,
) -> np.ndarray:
    """Build q from first 4 feedback joints + UDP joint5 relative degree."""
    if len(arm4_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm4_rad and calibration_rad must contain 4 values")
    delta = [float(arm4_rad[i]) - float(calibration_rad[i]) for i in range(4)]
    joint_deg_x100 = motor_delta_rad_to_joint_deg_x100(delta)
    # joint5 uses UDP relative degree directly by bridge2pi convention.
    joint_deg_x100[4] = float(joint5_rel_deg) * 100.0
    joint_rad = np.deg2rad(np.asarray(joint_deg_x100, dtype=float).reshape(5) / 100.0)
    return _q_from_arm_joint_rad(joint_rad, finger_l_m, finger_r_m)


def warmup_gravity_model_pinocchio() -> None:
    """Block until Pinocchio model/data are loaded and one gravity pass succeeds."""
    pin = _pin()
    model, data = _model_and_data()
    q = pin.neutral(model)
    pin.computeGeneralizedGravity(model, data, q)


def compute_gravity_tau_nm(
    arm_rad: Sequence[float],
    calibration_rad: Sequence[float],
    gain: float,
) -> tuple[float, float, float, float]:
    """Return motor-space gravity feedforward torque from pinocchio only."""
    if len(arm_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm_rad and calibration_rad must contain 4 values")

    delta = [float(arm_rad[i]) - float(calibration_rad[i]) for i in range(4)]
    joint_deg_x100 = motor_delta_rad_to_joint_deg_x100(delta)
    tau_joint = _get_gravity_torques_arm_deg_x100(joint_deg_x100, 0.0, 0.0)
    if len(tau_joint) < 4:
        raise ValueError("gravity solver returned less than 4 dof")

    tau_pin = joint_tau_to_motor_tau(tau_joint, gain)
    return (
        float(tau_pin[0]) * GRAVITY_AXIS_SCALE[0],
        float(tau_pin[1]) * GRAVITY_AXIS_SCALE[1],
        float(tau_pin[2]) * GRAVITY_AXIS_SCALE[2],
        float(tau_pin[3]) * GRAVITY_AXIS_SCALE[3],
    )


def compute_gravity_tau_nm_mlp(
    arm_rad: Sequence[float],
    calibration_rad: Sequence[float],
    gain: float,
    model_path: Path | None = None,
) -> tuple[float, float, float, float]:
    """Return motor-space gravity torque using MLP model backend."""
    if len(arm_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm_rad and calibration_rad must contain 4 values")

    if model_path is None:
        model_path = _PROJECT_ROOT / "gravity" / "gravity_mlp_model.npz"
    if not model_path.is_file():
        raise FileNotFoundError(f"MLP model not found: {model_path}")

    return run_mlp(
        model_path=model_path,
        arm_rad=[float(x) for x in arm_rad[:4]],
        calibration_rad=[float(x) for x in calibration_rad[:4]],
        gain=float(gain),
    )
