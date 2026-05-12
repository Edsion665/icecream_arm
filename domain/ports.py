"""Domain-level interface boundaries."""

from __future__ import annotations

from typing import Any, Protocol

from ..state_store import FeedbackRegister, RuntimeRegister, UdpRegister


class StateReadPort(Protocol):
    def snapshot_feedback(self) -> FeedbackRegister: ...
    def snapshot_udp(self) -> UdpRegister: ...
    def get_calibration_rad(self) -> tuple[float, float, float, float]: ...


class StateWritePort(Protocol):
    def set_runtime(
        self,
        *,
        control_source: str | None = None,
        safety_reason: str | None = None,
        last_tau_nm: tuple[float, float, float, float] | None = None,
    ) -> None: ...

    def set_servo_command(self, payload: dict[str, Any]) -> None: ...
    def set_calibration_rad(self, r0: float, r1: float, r2: float, r3: float) -> None: ...
    def adjust_calibration_deg(self, d0: float, d1: float, d2: float, d3: float) -> None: ...


class MotorCommandSink(Protocol):
    def send_mit_cmd(self, motors: list[dict[str, float]]) -> None: ...
    def send_mit_cmd_with_servo(self, motors: list[dict[str, float]], wrist_us: int, gripper_us: int) -> None: ...


class StatePort(StateReadPort, StateWritePort, Protocol):
    pass
