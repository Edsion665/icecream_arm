"""Gravity compensation calculator for V2."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Sequence

import numpy as np

from .domain.mapping import joint_tau_to_motor_tau, motor_delta_rad_to_joint_deg_x100


_HERE = Path(__file__).resolve()
_PROJECT_ROOT = _HERE.parent
_MODEL_ROOT = _PROJECT_ROOT / "robotarm"


def _resolve_model_root() -> Path:
    urdf = _MODEL_ROOT / "icecream_arm_model" / "ice_cream_v8.SLDASM" / "urdf" / "ice_cream_v8.SLDASM.urdf"
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
    urdf_path = model_root / "icecream_arm_model" / "ice_cream_v8.SLDASM" / "urdf" / "ice_cream_v8.SLDASM.urdf"
    package_dirs = [str(model_root / "icecream_arm_model")]
    model, _, _ = pin.buildModelsFromUrdf(str(urdf_path), package_dirs=package_dirs)
    return model, model.createData()


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


def compute_gravity_tau_nm(
    arm_rad: Sequence[float],
    calibration_rad: Sequence[float],
    gain: float,
) -> tuple[float, float, float, float]:
    """Return motor-space gravity feedforward torque for 4 motors."""
    if len(arm_rad) != 4 or len(calibration_rad) != 4:
        raise ValueError("arm_rad and calibration_rad must contain 4 values")

    delta = [float(arm_rad[i]) - float(calibration_rad[i]) for i in range(4)]
    joint_deg_x100 = motor_delta_rad_to_joint_deg_x100(delta)
    tau_joint = _get_gravity_torques_arm_deg_x100(joint_deg_x100, 0.0, 0.0)
    if len(tau_joint) < 4:
        raise ValueError("gravity solver returned less than 4 dof")

    return joint_tau_to_motor_tau(tau_joint, gain)

