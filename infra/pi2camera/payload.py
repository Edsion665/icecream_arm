"""Build ``camera_state`` JSON body per ``docs/pi2camera.md`` §4."""

from __future__ import annotations

import logging
from typing import Any

from ...calculator import compute_link5_rpy_xyz_link0, link5_hmat_row_major_list_from_rpy_xyz_rxryrz
from ...config import GRIP_THRESHOLD
from ...domain.mapping import motor_rad_to_joint_rel_rad_for_camera
from ...state_store import StateStore

LOGGER = logging.getLogger(__name__)


def build_camera_state_payload(store: StateStore) -> dict[str, Any] | None:
    """Return a dict with ``type``, ``motor_rad``, ``wrist_deg``, ``grip_state``, ``link5_hmat``.

    ``motor_rad`` 为关节相对标定零位角（rad）：``MOTOR_AXIS_SIGN`` + 标定零位，
    并对 M3/M4 乘 ``PI2CAMERA_JOINT_REL_SIGN``（默认下标 2、3 取反）以匹配相机解算约定。
    ``link5_hmat`` is ^0T_5 from ``link5_rpy_rad``/``link5_xyz_m`` convention (Rx·Ry·Rz + t).
    Returns ``None`` when motor feedback is unavailable. ``link5_hmat`` may be ``null`` if FK fails.
    """
    udp = store.snapshot_udp()
    fb = store.snapshot_feedback()
    cal = store.get_calibration_rad()
    pose_source = fb.fb_arm_rad or fb.mit_arm_rad
    if pose_source is None or len(pose_source) < 4:
        return None
    if udp.recv_mono <= 0 or len(udp.p_rel_deg) < 5:
        return None

    joint_rel_rad = motor_rad_to_joint_rel_rad_for_camera(pose_source, cal)
    motor_rad = [float(joint_rel_rad[i]) for i in range(4)]
    wrist_deg = float(udp.p_rel_deg[4])
    grip_state = 0
    if len(udp.p_rel_deg) >= 6:
        grip_state = 1 if float(udp.p_rel_deg[5]) >= GRIP_THRESHOLD else 0

    link5_hmat: list[list[float]] | None = None
    try:
        rpy, xyz = compute_link5_rpy_xyz_link0(
            arm4_rad=pose_source,
            joint5_rel_deg=wrist_deg,
            calibration_rad=cal,
        )
        link5_hmat = link5_hmat_row_major_list_from_rpy_xyz_rxryrz(rpy, xyz)
    except Exception as exc:  # noqa: BLE001
        LOGGER.warning("camera_state link5_hmat compose failed: %s", exc)

    return {
        "type": "camera_state",
        "motor_rad": motor_rad,
        "wrist_deg": wrist_deg,
        "grip_state": grip_state,
        "link5_hmat": link5_hmat,
    }
