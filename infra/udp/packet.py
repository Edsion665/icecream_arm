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
from ..serial.codec import SERVO_CENTER_US, SERVO_MAX_US, STEPPER_DEG_RANGE

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
    """Decode bridge2pi ``p_rel_deg[4:6]`` into (wrist_us, gripper_us)."""
    if len(p_rel_deg) < 6:
        raise ValueError("p_rel_deg must contain wrist_deg and grip_state")
    return wrist_deg_to_us(float(p_rel_deg[4])), grip_state_to_us(float(p_rel_deg[5]))


def decode_udp_pi2stm_aux(p_rel_deg: Sequence[float]) -> tuple[int, int]:
    """Map bridge2pi v3 ``p_rel_deg[6], p_rel_deg[7]`` to ``encode_mit_cmd_42`` 参数。

    - ``stepper_deg``：与 ``docs/pi2stm.md`` 下行一致，**增量角**（度），Pi 侧 round 后限幅到 ``STEPPER_DEG_RANGE``。
    - ``conveyor_run``：float 语义 ``0≈停 / 1≈转``，用 ``GRIP_THRESHOLD`` 同一阈值离散化为 ``0|1``。
    """
    if len(p_rel_deg) < 8:
        raise ValueError("bridge2pi v3 requires len(p_rel_deg) >= 8 for stepper_deg and conveyor_run")
    raw_s = int(round(float(p_rel_deg[6])))
    lo, hi = STEPPER_DEG_RANGE
    stepper_deg = max(lo, min(hi, raw_s))
    conveyor_run = 1 if float(p_rel_deg[7]) >= GRIP_THRESHOLD else 0
    return stepper_deg, conveyor_run


def mit39_init_pulse_us() -> tuple[int, int]:
    """MIT39 init 腕/爪下行脉宽 (us)：腕=codec 中性；爪=``GRIP_MIT39_INIT_US``（与 UDP 语义分立）。"""
    return (SERVO_CENTER_US, GRIP_MIT39_INIT_US)
