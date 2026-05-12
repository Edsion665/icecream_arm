"""State payload for WebSocket: link5 pose + ``link5_hmat`` (Rx·Ry·Rz + t), optional chain steps."""

from __future__ import annotations

import logging
from typing import Any

from ...calculator import (
    compute_link5_rpy_xyz_link0,
    compute_link_chain_rpy_xyz_steps_link0,
    link5_hmat_row_major_list_from_rpy_xyz_rxryrz,
)
from ...state_store import StateStore

LOGGER = logging.getLogger(__name__)


def build_state_payload(store: StateStore) -> dict[str, Any]:
    """Broadcast link5 in link0 plus per-joint cumulative chain (tuning).

    ``link_chain_steps``: after each joint i=1..5, pose of that child frame in link0
    as ``xyz_m``, ``rpy_rad`` (same Rx·Ry·Rz decomposition as link5), and ``q_rad``.
    """
    udp = store.snapshot_udp()
    fb = store.snapshot_feedback()
    cal = store.get_calibration_rad()
    link5_rpy_rad: list[float] | None = None
    link5_xyz_m: list[float] | None = None
    link5_hmat: list[list[float]] | None = None
    link_chain_steps: list[dict[str, Any]] | None = None
    pose_source = fb.fb_arm_rad or fb.mit_arm_rad
    has_udp_joint5 = udp.recv_mono > 0 and len(udp.p_rel_deg) >= 5
    if pose_source is not None and has_udp_joint5:
        try:
            link5_rpy_rad, link5_xyz_m = compute_link5_rpy_xyz_link0(
                arm4_rad=pose_source,
                joint5_rel_deg=float(udp.p_rel_deg[4]),
                calibration_rad=cal,
            )
            link5_hmat = link5_hmat_row_major_list_from_rpy_xyz_rxryrz(link5_rpy_rad, link5_xyz_m)
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("link5 FK (rpy/xyz/hmat) failed: %s", exc)
        try:
            link_chain_steps = compute_link_chain_rpy_xyz_steps_link0(
                arm4_rad=pose_source,
                joint5_rel_deg=float(udp.p_rel_deg[4]),
                calibration_rad=cal,
            )
        except Exception as exc:  # noqa: BLE001
            LOGGER.warning("link_chain_steps failed: %s", exc)

    return {
        "link5_rpy_rad": link5_rpy_rad,
        "link5_xyz_m": link5_xyz_m,
        "link5_hmat": link5_hmat,
        "link_chain_steps": link_chain_steps,
    }
