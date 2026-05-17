"""Thread-safe register-style state storage for V2."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from threading import Lock
from time import monotonic
from typing import Any, Optional

from .config import UDP_VECTOR_DIM


def _udp_vector_zeros() -> tuple[float, ...]:
    return (0.0,) * UDP_VECTOR_DIM


@dataclass
class UdpRegister:
    seq: int = -1
    recv_mono: float = 0.0
    p_rel_deg: tuple[float, ...] = field(default_factory=_udp_vector_zeros)
    omega_rad_s: tuple[float, ...] = field(default_factory=_udp_vector_zeros)


@dataclass
class FeedbackRegister:
    mit_arm_rad: Optional[tuple[float, float, float, float]] = None
    fb_arm_rad: Optional[tuple[float, float, float, float]] = None
    motors: dict[int, dict[str, Any]] = field(default_factory=dict)
    crc_error_count: int = 0
    # STM32 上行 v3（docs/pi2stm.md）：步进逻辑位置（°）、传送带 PB6 状态（0/1）。
    stm_stepper_deg: Optional[int] = None
    stm_conveyor_run: Optional[int] = None
    # 最近一次成功解析的 STM32 串口关节反馈（MIT 上行整帧或 FB 行）的本地 monotonic 时间戳。
    serial_feedback_mono: float = 0.0


@dataclass
class RuntimeRegister:
    control_source: str = "hold"
    safety_reason: str = "boot"
    last_tau_nm: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    servo_command: dict[str, Any] = field(default_factory=dict)


class StateStore:
    """Single source of truth used by serial/listener/controller/server."""

    def __init__(
        self,
        calibration_rad: tuple[float, float, float, float] | None = None,
    ) -> None:
        self._lock = Lock()
        self._udp = UdpRegister()
        self._feedback = FeedbackRegister()
        self._runtime = RuntimeRegister()
        if calibration_rad is None:
            self._calibration_rad = (1.57416, 1.360151, 2.380980, 0.490005)
        else:
            self._calibration_rad = tuple(float(x) for x in calibration_rad)

    def get_calibration_rad(self) -> tuple[float, float, float, float]:
        """四轴标定零位（rad），与仿真 p_rel_deg=0 对齐；UDP 映射与重力补偿均使用此值。"""
        with self._lock:
            return tuple(self._calibration_rad)

    def set_calibration_rad(self, r0: float, r1: float, r2: float, r3: float) -> None:
        """绝对设置四轴标定零位（rad）。"""
        with self._lock:
            self._calibration_rad = (float(r0), float(r1), float(r2), float(r3))

    def adjust_calibration_deg(self, d0: float, d1: float, d2: float, d3: float) -> None:
        """在现有标定上叠加微调（度），用于实机与仿真零位对齐。"""
        with self._lock:
            c = list(self._calibration_rad)
            for i, d in enumerate((d0, d1, d2, d3)):
                c[i] += math.radians(float(d))
            self._calibration_rad = (c[0], c[1], c[2], c[3])

    def update_udp(
        self,
        seq: int,
        p_rel_deg: tuple[float, ...],
        omega_rad_s: tuple[float, ...],
    ) -> None:
        with self._lock:
            self._udp.seq = int(seq)
            self._udp.recv_mono = monotonic()
            self._udp.p_rel_deg = p_rel_deg
            self._udp.omega_rad_s = omega_rad_s

    def snapshot_udp(self) -> UdpRegister:
        with self._lock:
            return UdpRegister(
                seq=self._udp.seq,
                recv_mono=self._udp.recv_mono,
                p_rel_deg=tuple(self._udp.p_rel_deg),
                omega_rad_s=tuple(self._udp.omega_rad_s),
            )

    def update_mit_feedback(self, idx: int, motor: dict[str, Any]) -> None:
        with self._lock:
            current = dict(self._feedback.motors.get(idx, {"id": idx}))
            current.update(motor)
            self._feedback.motors[idx] = current
            mit = (
                self._feedback.motors.get(0, {}).get("p"),
                self._feedback.motors.get(1, {}).get("p"),
                self._feedback.motors.get(2, {}).get("p"),
                self._feedback.motors.get(3, {}).get("p"),
            )
            if all(v is not None for v in mit):
                self._feedback.mit_arm_rad = (
                    float(mit[0]),
                    float(mit[1]),
                    float(mit[2]),
                    float(mit[3]),
                )

    def set_fb_arm_rad(self, rad: tuple[float, float, float, float]) -> None:
        with self._lock:
            self._feedback.fb_arm_rad = rad
            self._feedback.serial_feedback_mono = monotonic()

    def touch_serial_feedback_recv(self) -> None:
        with self._lock:
            self._feedback.serial_feedback_mono = monotonic()

    def update_stm_aux_feedback(self, stepper_deg: int, conveyor_run: int) -> None:
        with self._lock:
            self._feedback.stm_stepper_deg = int(stepper_deg)
            self._feedback.stm_conveyor_run = int(conveyor_run)

    def inc_crc_error(self) -> None:
        with self._lock:
            self._feedback.crc_error_count += 1

    def snapshot_feedback(self) -> FeedbackRegister:
        with self._lock:
            return FeedbackRegister(
                mit_arm_rad=self._feedback.mit_arm_rad,
                fb_arm_rad=self._feedback.fb_arm_rad,
                motors={k: dict(v) for k, v in self._feedback.motors.items()},
                crc_error_count=self._feedback.crc_error_count,
                stm_stepper_deg=self._feedback.stm_stepper_deg,
                stm_conveyor_run=self._feedback.stm_conveyor_run,
                serial_feedback_mono=float(self._feedback.serial_feedback_mono),
            )

    def set_runtime(
        self,
        *,
        control_source: str | None = None,
        safety_reason: str | None = None,
        last_tau_nm: tuple[float, float, float, float] | None = None,
    ) -> None:
        with self._lock:
            if control_source is not None:
                self._runtime.control_source = control_source
            if safety_reason is not None:
                self._runtime.safety_reason = safety_reason
            if last_tau_nm is not None:
                self._runtime.last_tau_nm = last_tau_nm

    def set_servo_command(self, payload: dict[str, Any]) -> None:
        with self._lock:
            self._runtime.servo_command = dict(payload)

    def snapshot_runtime(self) -> RuntimeRegister:
        with self._lock:
            return RuntimeRegister(
                control_source=self._runtime.control_source,
                safety_reason=self._runtime.safety_reason,
                last_tau_nm=tuple(self._runtime.last_tau_nm),
                servo_command=dict(self._runtime.servo_command),
            )

    def to_payload(self) -> dict[str, Any]:
        # Compatibility wrapper: payload presentation moved out of store internals.
        from .infra.state.presenter import build_state_payload

        return build_state_payload(self)

