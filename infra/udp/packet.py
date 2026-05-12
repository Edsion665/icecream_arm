"""UDP packet format helpers."""

from __future__ import annotations

import struct
from typing import Any, Sequence

from ...config import (
    GRIP_CLOSED_US,
    GRIP_MIT39_INIT_US,
    GRIP_OPEN_US,
    GRIP_THRESHOLD,
    UDP_PACKET_FMT,
    UDP_PACKET_SIZE,
    UDP_VECTOR_DIM,
    WRIST_MAX_DEG,
    WRIST_MIN_DEG,
)
from ..serial.codec import SERVO_CENTER_US, SERVO_MAX_US

assert UDP_PACKET_SIZE == struct.calcsize(UDP_PACKET_FMT)


def unpack_udp_packet(data: bytes) -> dict[str, Any]:
    seq, ts, *rest = struct.unpack(UDP_PACKET_FMT, data)
    return {
        "seq": int(seq),
        "ts": float(ts),
        "p_rel_deg": tuple(float(v) for v in rest[:UDP_VECTOR_DIM]),
        "omega_rad_s": tuple(float(v) for v in rest[UDP_VECTOR_DIM : UDP_VECTOR_DIM * 2]),
    }


def wrist_deg_to_us(wrist_deg: float) -> int:
    """Map wrist angle -180..180 deg to 2500..500 us (reversed), 0 deg at SERVO_CENTER_US."""
    deg = max(WRIST_MIN_DEG, min(WRIST_MAX_DEG, float(wrist_deg)))
    return int(
        round(SERVO_CENTER_US - deg * (SERVO_MAX_US - SERVO_CENTER_US) / WRIST_MAX_DEG)
    )


def grip_state_to_us(grip_state: float) -> int:
    """Map gripper state to pulse width: 0=closed, 1=open."""
    return GRIP_OPEN_US if float(grip_state) >= GRIP_THRESHOLD else GRIP_CLOSED_US


def decode_udp_servo(p_rel_deg: Sequence[float]) -> tuple[int, int]:
    """Decode bridge2pi v2.1 p_rel_deg[4:6] into (wrist_us, gripper_us)."""
    if len(p_rel_deg) < 6:
        raise ValueError("p_rel_deg must contain wrist_deg and grip_state")
    return wrist_deg_to_us(float(p_rel_deg[4])), grip_state_to_us(float(p_rel_deg[5]))


def mit39_init_pulse_us() -> tuple[int, int]:
    """MIT39 init 腕/爪下行脉宽 (us)：腕=codec 中性；爪=``GRIP_MIT39_INIT_US``（与 UDP 语义分立）。"""
    return (SERVO_CENTER_US, GRIP_MIT39_INIT_US)
