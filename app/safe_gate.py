"""Startup UDP first-frame safe gate."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from time import monotonic
from typing import Sequence

LOGGER = logging.getLogger(__name__)


@dataclass(frozen=True)
class SafeGateConfig:
    enabled: bool
    vmax_rad_s: tuple[float, float, float, float]
    tol_rad: float
    timeout_sec: float
    nominal_dt: float


class StartupSafeGate:
    """Freeze UDP to first frame and ramp from current pose with speed limit."""

    def __init__(self, cfg: SafeGateConfig) -> None:
        self._cfg = cfg
        self._state: str = "waiting_first_udp"
        self._target_p: list[float] | None = None
        self._ramp_p: list[float] | None = None
        self._last_mono: float = 0.0
        self._start_mono: float = 0.0

    def active(self) -> bool:
        return self._cfg.enabled and self._state == "ramping"

    def reset(self) -> None:
        """STM32 断链后恢复时调用：重新等待「下一帧 UDP」作为限速 ramp 起点。"""
        self._state = "waiting_first_udp"
        self._target_p = None
        self._ramp_p = None
        self._last_mono = 0.0
        self._start_mono = 0.0

    def apply(
        self,
        arm_rad: Sequence[float],
        udp_p_cmd: Sequence[float],
    ) -> tuple[list[float], list[float]] | None:
        """Return gated (p, v) while ramping, else None for normal UDP."""
        if not self._cfg.enabled:
            return None
        if self._state == "done":
            return None

        now = monotonic()
        if self._state == "waiting_first_udp":
            self._target_p = [float(udp_p_cmd[i]) for i in range(4)]
            self._ramp_p = [float(arm_rad[i]) for i in range(4)]
            self._state = "ramping"
            self._start_mono = now
            self._last_mono = now
            LOGGER.info(
                "safe gate latched first UDP target: current=%s target=%s vmax=%s tol=%.5f",
                [round(float(v), 4) for v in self._ramp_p],
                [round(float(v), 4) for v in self._target_p],
                [round(float(v), 4) for v in self._cfg.vmax_rad_s],
                float(self._cfg.tol_rad),
            )

        if self._state != "ramping" or self._target_p is None or self._ramp_p is None:
            return None

        dt = now - self._last_mono
        if dt <= 0.0:
            dt = self._cfg.nominal_dt
        dt = max(0.001, min(0.2, dt))
        self._last_mono = now

        out_v = [0.0, 0.0, 0.0, 0.0]
        done = True
        for i in range(4):
            err = self._target_p[i] - self._ramp_p[i]
            vmax = max(1e-5, float(self._cfg.vmax_rad_s[i]))
            step = max(-vmax * dt, min(vmax * dt, err))
            self._ramp_p[i] += step
            out_v[i] = step / dt
            if abs(self._target_p[i] - self._ramp_p[i]) > self._cfg.tol_rad:
                done = False

        if now - self._start_mono >= self._cfg.timeout_sec:
            self._ramp_p = [float(self._target_p[i]) for i in range(4)]
            out_v = [0.0, 0.0, 0.0, 0.0]
            done = True
            LOGGER.warning(
                "safe gate timeout reached (%.2fs), force finish to first target",
                float(self._cfg.timeout_sec),
            )

        if done:
            self._state = "done"
            LOGGER.info(
                "safe gate completed: target=%s elapsed=%.3fs",
                [round(float(v), 4) for v in self._target_p],
                now - self._start_mono,
            )
            return ([float(self._target_p[i]) for i in range(4)], [0.0, 0.0, 0.0, 0.0])

        return ([float(self._ramp_p[i]) for i in range(4)], out_v)

