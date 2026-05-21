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
        self._prev_serial_alive: bool = False
        self._prev_udp_frozen: bool = False
        self._udp_latch_p: tuple[float, ...] | None = None
        self._udp_latch_omega: tuple[float, ...] | None = None
        self._udp_latch_valid: bool = False
        self._last_build_udp_latched: bool = False
        # 持续限速 ramp：safe_gate 完成后接管，每帧向最新 UDP 目标步进
        self._ramp_p: list[float] | None = None
        self._ramp_last_mono: float = 0.0
        self._last_ramp_vmax_rad_s: tuple[float, float, float, float] | None = None

    def _select_feedback_rad(self) -> tuple[float, float, float, float] | None:
        fb = self._store.snapshot_feedback()
        if self._cfg.feedback_source == "fb":
            return fb.fb_arm_rad
        return fb.mit_arm_rad or fb.fb_arm_rad

    def has_arm_feedback(self) -> bool:
        return self._select_feedback_rad() is not None

    def serial_feedback_alive(self) -> bool:
        """串口关节反馈在时效内（见 ``serial_feedback_stale_sec``）。"""
        if not self.has_arm_feedback():
            return False
        fb = self._store.snapshot_feedback()
        if fb.serial_feedback_mono <= 0.0:
            return False
        return monotonic() - fb.serial_feedback_mono <= self._cfg.serial_feedback_stale_sec

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
        return self._udp_to_motor_pv_from(udp.p_rel_deg, udp.omega_rad_s)

    def _udp_to_motor_pv_from(
        self,
        p_rel_deg: tuple[float, ...],
        omega_rad_s: tuple[float, ...],
    ) -> tuple[list[float], list[float]]:
        cal = self._store.get_calibration_rad()
        return udp_rel_to_motor_pv(cal, p_rel_deg, omega_rad_s)

    def _udp_stale_using_latch(self) -> bool:
        if not self._udp_cfg.enabled or not self._udp_latch_valid:
            return False
        ok, _ = self._udp_fresh()
        return not ok

    def effective_udp_p_rel_deg(self) -> tuple[float, ...]:
        """供舵机 UDP 解码：新鲜帧或 PC 断链时的锁存帧。"""
        if self._udp_cfg.enabled and self._udp_latch_valid:
            assert self._udp_latch_p is not None
            return self._udp_latch_p
        return self._store.snapshot_udp().p_rel_deg

    def udp_pc_link_latched(self) -> bool:
        """上一控制周期是否处于 PC UDP 锁存冻结（非 STM32 串口冻结）。"""
        return self._last_build_udp_latched

    def udp_pc_link_state_label(self) -> str:
        """PC UDP 在日志中的状态：避免从未收包却显示「正常」。"""
        if not self._udp_cfg.enabled:
            return "已关闭"
        ok, _ = self._udp_fresh()
        if ok:
            return "跟踪"
        if self._udp_latch_valid and not ok:
            return "锁存冻结"
        udp = self._store.snapshot_udp()
        if udp.recv_mono <= 0:
            return "无帧"
        return "过期无锁存"

    def udp_motor_target_p_rad(self) -> list[float] | None:
        """锁存 UDP 映射后的四轴目标角（rad），未经 ramp。"""
        if not self._udp_latch_valid or self._udp_latch_p is None:
            return None
        o = self._udp_latch_omega if self._udp_latch_omega is not None else (0.0,) * 8
        p_cmd, _ = self._udp_to_motor_pv_from(self._udp_latch_p, o)
        return [float(p_cmd[i]) for i in range(4)]

    def last_tracking_ramp_vmax_rad_s(self) -> list[float] | None:
        """上一周期 tracking_ramp 使用的每轴速度上限 (rad/s)，来自 config.max_cmd_speed_rad_s。"""
        if self._last_ramp_vmax_rad_s is None:
            return None
        return [float(v) for v in self._last_ramp_vmax_rad_s]

    def _tracking_ramp_vmax_rad_s(self) -> tuple[float, float, float, float]:
        """跟踪时 cmd_p 向 UDP 目标靠拢的每轴最大角速度 (rad/s)。
        若 UDP omega_rad_s[0:4] 任一轴非零，则用 UDP 值覆盖 config 默认值。
        """
        if self._udp_latch_omega is not None:
            udp_v = self._udp_latch_omega[:4]
            if any(float(v) > 0 for v in udp_v):
                return tuple(max(1e-4, abs(float(v))) for v in udp_v)
        return tuple(max(1e-4, float(v)) for v in self._cfg.max_cmd_speed_rad_s)

    def _apply_tracking_ramp(
        self, target_p: list[float], vmax: tuple[float, ...]
    ) -> tuple[list[float], list[float]]:
        """从 _ramp_p 向 target_p 以 vmax 限速步进（rad/s）。
        target_p 每帧取最新 UDP 映射目标，途中更新终点立即生效。
        """
        now = monotonic()
        dt = now - self._ramp_last_mono
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / max(1.0, self._cfg.tau_hz)
        self._ramp_last_mono = now

        if self._ramp_p is None:
            self._ramp_p = [float(v) for v in target_p]
            return (list(self._ramp_p), [0.0, 0.0, 0.0, 0.0])

        out_v = [0.0, 0.0, 0.0, 0.0]
        for i in range(4):
            err = target_p[i] - self._ramp_p[i]
            v = float(vmax[i])
            step = max(-v * dt, min(v * dt, err))
            self._ramp_p[i] += step
            out_v[i] = step / dt
        return (list(self._ramp_p), out_v)

    def build_motor_commands(self) -> list[dict[str, float]]:
        self._last_build_udp_latched = False
        udp_ok, _age = self._udp_fresh()
        if self._udp_cfg.enabled and udp_ok:
            snap = self._store.snapshot_udp()
            self._udp_latch_p = tuple(float(x) for x in snap.p_rel_deg)
            self._udp_latch_omega = tuple(float(x) for x in snap.omega_rad_s)
            self._udp_latch_valid = True

        alive = self.serial_feedback_alive()
        lost_edge = self._prev_serial_alive and not alive
        if not alive:
            if lost_edge:
                if self._safe_gate is not None:
                    self._safe_gate.reset()
                udp_note = (
                    "锁存(末帧有效)"
                    if self._udp_stale_using_latch()
                    else ("跟踪中" if udp_ok else "无有效帧")
                )
                LOGGER.info(
                    "[link] STM32关节反馈=冻结 MIT/舵机串口下发已停 | PC-UDP=%s",
                    udp_note,
                )
            self._prev_serial_alive = False
            self._prev_udp_frozen = bool(
                self._udp_cfg.enabled and self._udp_latch_valid and not udp_ok
            )
            self._last_build_udp_latched = self._udp_stale_using_latch()
            self._store.set_runtime(
                control_source="frozen",
                safety_reason="serial_feedback_stale",
                last_tau_nm=(0.0, 0.0, 0.0, 0.0),
            )
            return []
        self._prev_serial_alive = True

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

        udp_frozen = (
            self._udp_cfg.enabled and self._udp_latch_valid and not udp_ok
        )

        if self._udp_cfg.enabled and self._safe_gate is not None:
            udp_frozen_edge = udp_frozen and not self._prev_udp_frozen
            udp_recover_edge = self._prev_udp_frozen and udp_ok
            if udp_frozen_edge or udp_recover_edge:
                self._safe_gate.reset()
            if udp_frozen_edge:
                LOGGER.info(
                    "[link] STM32关节反馈=正常 PC-UDP=冻结(锁存上一帧); "
                    "继续 MIT 下发，safe gate 约束串口目标角过渡"
                )
            if udp_recover_edge:
                LOGGER.info(
                    "[link] STM32关节反馈=正常 PC-UDP=恢复跟踪; "
                    "已收到新 UDP，safe gate 已重新武装"
                )
        self._prev_udp_frozen = bool(udp_frozen)

        if self._udp_cfg.enabled and self._udp_latch_valid and (udp_ok or udp_frozen):
            p_deg = self._udp_latch_p
            o_rad = self._udp_latch_omega
            p_cmd, v_cmd = self._udp_to_motor_pv_from(p_deg, o_rad)
            if self._safe_gate is not None:
                gated = self._safe_gate.apply(arm_rad=arm_rad, udp_p_cmd=p_cmd)
            else:
                gated = None
            if gated is not None:
                p_cmd, v_cmd = gated
                self._ramp_p = [float(x) for x in p_cmd]
                self._ramp_last_mono = monotonic()
                source = "safe_gate"
                reason = "udp_latch_gate_ramp" if udp_frozen else "startup_first_frame_ramp"
            else:
                vmax = self._tracking_ramp_vmax_rad_s()
                self._last_ramp_vmax_rad_s = vmax
                p_cmd, v_cmd = self._apply_tracking_ramp(p_cmd, vmax)
                source = "tracking_ramp"
                reason = "udp_stale_latched" if udp_frozen else "ok"
            self._hold_p_latch = [float(x) for x in p_cmd]
            if udp_frozen:
                self._last_build_udp_latched = True
        else:
            if self._hold_p_latch is not None:
                p_cmd = [float(x) for x in self._hold_p_latch]
            else:
                p_cmd = [float(v) for v in arm_rad]
                self._hold_p_latch = [float(v) for v in arm_rad]
            v_cmd = [0.0, 0.0, 0.0, 0.0]
            source = "hold"
            if not self._udp_cfg.enabled:
                reason = "udp_disabled"
            elif not self._udp_latch_valid:
                reason = "udp_wait_first_packet"
            else:
                reason = "udp_timeout"

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
