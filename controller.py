"""Control law module: MIT command generation and safety handling."""

from __future__ import annotations

import logging
from time import monotonic
from typing import TYPE_CHECKING

from .calculator import compute_gravity_tau_nm
from .config import ControlConfig, UdpConfig
from .domain.mapping import udp_rel_to_motor_pv
from .domain.ports import StatePort

if TYPE_CHECKING:
    from .app.safe_gate import StartupSafeGate

LOGGER = logging.getLogger(__name__)


class ArmController:
    """Build one cycle of motor command from registers."""

    def __init__(
        self,
        cfg: ControlConfig,
        udp_cfg: UdpConfig,
        store: StatePort,
        safe_gate: "StartupSafeGate | None" = None,
    ) -> None:
        self._cfg = cfg
        self._udp_cfg = udp_cfg
        self._store = store
        self._safe_gate = safe_gate
        self._last_cmd_p: list[float] | None = None
        self._hold_p_latch: list[float] | None = None
        # 持续限速 ramp：safe_gate 完成后接管，每帧向最新 UDP 目标步进
        self._ramp_p: list[float] | None = None
        self._ramp_last_mono: float = 0.0

    def _select_feedback_rad(self) -> tuple[float, float, float, float] | None:
        fb = self._store.snapshot_feedback()
        if self._cfg.feedback_source == "fb":
            return fb.fb_arm_rad
        return fb.mit_arm_rad or fb.fb_arm_rad

    def has_arm_feedback(self) -> bool:
        return self._select_feedback_rad() is not None

    def prime_hold_latch_from_feedback(self) -> None:
        arm_rad = self._select_feedback_rad()
        if arm_rad is None:
            return
        if self._hold_p_latch is None:
            self._hold_p_latch = [float(v) for v in arm_rad]

    def set_hold_latch_rad(self, rad: tuple[float, float, float, float]) -> None:
        self._hold_p_latch = [float(rad[i]) for i in range(4)]
        self._last_cmd_p = [float(rad[i]) for i in range(4)]

    def _udp_fresh(self) -> tuple[bool, float]:
        udp = self._store.snapshot_udp()
        if udp.recv_mono <= 0:
            return (False, float("inf"))
        age = monotonic() - udp.recv_mono
        return (age <= self._udp_cfg.stale_sec, age)

    def _udp_to_motor_pv(self) -> tuple[list[float], list[float]]:
        udp = self._store.snapshot_udp()
        cal = self._store.get_calibration_rad()
        return udp_rel_to_motor_pv(cal, udp.p_rel_deg, udp.omega_rad_s)

    def _apply_tracking_ramp(
        self, target_p: list[float]
    ) -> tuple[list[float], list[float]]:
        """从 _ramp_p 向 target_p 以 tracking_speed_rad_s 限速步进。
        target_p 每帧取最新 UDP 目标，途中更新终点立即生效。
        """
        now = monotonic()
        dt = now - self._ramp_last_mono
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1.0, self._cfg.tau_hz)
        self._ramp_last_mono = now

        if self._ramp_p is None:
            self._ramp_p = [float(v) for v in target_p]
            return (list(self._ramp_p), [0.0, 0.0, 0.0, 0.0])

        vmax = self._cfg.tracking_speed_rad_s
        out_v = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            err = target_p[i] - self._ramp_p[i]
            step = max(-vmax[i] * dt, min(vmax[i] * dt, err))
            self._ramp_p[i] += step
            out_v[i] = step / dt
        return (list(self._ramp_p), out_v)

    def build_motor_commands(self) -> list[dict[str, float]]:
        arm_rad = self._select_feedback_rad()
        if arm_rad is None:
            self._store.set_runtime(
                control_source="hold",
                safety_reason="no_feedback",
                last_tau_nm=(0.0, 0.0, 0.0, 0.0),
            )
            last_hold = self._build_hold_from_last_command()
            if last_hold is not None:
                return last_hold
            return []

        if not self._cfg.gravity_ff_enabled:
            tau = (0.0, 0.0, 0.0, 0.0)
        else:
            try:
                tau = compute_gravity_tau_nm(
                    arm_rad=arm_rad,
                    calibration_rad=self._store.get_calibration_rad(),
                    gain=self._cfg.tau_gain,
                )
            except Exception as exc:  # noqa: BLE001
                LOGGER.warning("gravity calculation failed, switch to hold: %s", exc)
                self._store.set_runtime(
                    control_source="hold",
                    safety_reason="pinocchio_error",
                    last_tau_nm=(0.0, 0.0, 0.0, 0.0),
                )
                return self._hold_with_zero_tau(arm_rad)

        udp_ok, _age = self._udp_fresh()
        if self._udp_cfg.enabled and udp_ok:
            p_cmd, v_cmd = self._udp_to_motor_pv()
            if self._safe_gate is not None:
                gated = self._safe_gate.apply(arm_rad=arm_rad, udp_p_cmd=p_cmd)
            else:
                gated = None
            if gated is not None:
                # safe_gate 启动阶段：用 gate 输出，同步 ramp_p 跟随，避免 gate 完成后跳变
                p_cmd, v_cmd = gated
                self._ramp_p = [float(x) for x in p_cmd]
                self._ramp_last_mono = monotonic()
                source = "safe_gate"
                reason = "startup_first_frame_ramp"
            else:
                # 正常跟踪阶段：持续 ramp 向最新 UDP 目标步进
                p_cmd, v_cmd = self._apply_tracking_ramp(p_cmd)
                source = "tracking_ramp"
                reason = "ok"
            self._hold_p_latch = [float(x) for x in p_cmd]
        else:
            if self._hold_p_latch is not None:
                p_cmd = [float(x) for x in self._hold_p_latch]
            else:
                p_cmd = [float(v) for v in arm_rad]
                self._hold_p_latch = [float(v) for v in arm_rad]
            v_cmd = [0.0, 0.0, 0.0, 0.0]
            source = "hold"
            reason = "udp_timeout" if self._udp_cfg.enabled else "udp_disabled"

        kp_list = [0.0, 0.0, 0.0, 0.0] if self._cfg.kp_float_mode else list(self._cfg.hold_kp)
        kd_list = list(self._cfg.hold_kd)
        self._last_cmd_p = [float(x) for x in p_cmd]
        self._store.set_runtime(
            control_source=source,
            safety_reason=reason,
            last_tau_nm=tau,
        )
        cmds: list[dict[str, float]] = []
        for i in range(4):
            cmds.append(
                {
                    "p": float(p_cmd[i]),
                    "v": float(v_cmd[i]),
                    "kp": float(kp_list[i]),
                    "kd": float(kd_list[i]),
                    "t": float(tau[i]),
                }
            )
        return cmds

    def _hold_with_zero_tau(self, arm_rad: tuple[float, float, float, float]) -> list[dict[str, float]]:
        if self._hold_p_latch is None:
            self._hold_p_latch = [float(arm_rad[i]) for i in range(4)]
        p_use = self._hold_p_latch
        self._last_cmd_p = [float(x) for x in p_use]
        kp_list = [0.0, 0.0, 0.0, 0.0] if self._cfg.kp_float_mode else list(self._cfg.hold_kp)
        kd_list = list(self._cfg.hold_kd)
        return [
            {"p": float(p_use[i]), "v": 0.0, "kp": float(kp_list[i]), "kd": float(kd_list[i]), "t": 0.0}
            for i in range(4)
        ]

    def _build_hold_from_last_command(self) -> list[dict[str, float]] | None:
        if self._last_cmd_p is None:
            return None
        kp_list = [0.0, 0.0, 0.0, 0.0] if self._cfg.kp_float_mode else list(self._cfg.hold_kp)
        kd_list = list(self._cfg.hold_kd)
        return [
            {
                "p": float(self._last_cmd_p[i]),
                "v": 0.0,
                "kp": float(kp_list[i]),
                "kd": float(kd_list[i]),
                "t": 0.0,
            }
            for i in range(4)
        ]
