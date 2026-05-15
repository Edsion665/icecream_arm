"""Runtime assembly and lifecycle orchestration."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import logging
import math
import time

from .safe_gate import SafeGateConfig, StartupSafeGate
from ..calculator import warmup_gravity_model_pinocchio
from ..config import CONFIG
from ..controller import ArmController
from ..domain.ports import MotorCommandSink, StatePort
from ..infra.udp.packet import decode_udp_servo, grip_state_to_us, mit39_init_pulse_us, wrist_deg_to_us
from ..listener import UdpListener
from ..serial import SerialManager
from ..infra.pi2camera.broadcast import run_pi2camera_udp_broadcast
from ..server import StateServer
from ..state_store import StateStore


SERVO_ONLY_MOTOR_CMDS = [
    {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0},
    {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0},
    {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0},
    {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0},
]


@dataclass
class RuntimeComponents:
    store: StateStore
    serial_mgr: SerialManager
    udp_listener: UdpListener
    state_server: StateServer
    controller: ArmController


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    )


async def _wait_for_initial_arm_feedback(controller: ArmController, timeout_sec: float) -> None:
    """阻塞直到首帧串口关节反馈；可选有限超时（调试用）。"""
    logger = logging.getLogger(__name__)
    if math.isfinite(timeout_sec) and timeout_sec > 0:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_sec:
            if controller.has_arm_feedback():
                logger.info("initial arm feedback ready after %.3fs", time.monotonic() - t0)
                return
            await asyncio.sleep(0.01)
        logger.warning(
            "still no arm feedback after %.1fs (serial/STM32?); continuing without feedback",
            timeout_sec,
        )
        return
    t0 = time.monotonic()
    while True:
        if controller.has_arm_feedback():
            logger.info("initial arm feedback ready after %.3fs", time.monotonic() - t0)
            return
        await asyncio.sleep(0.01)


async def run_control_loop(
    controller: ArmController,
    serial_mgr: MotorCommandSink,
    store: StatePort,
) -> None:
    logger = logging.getLogger(__name__)
    hz = max(1.0, CONFIG.control.tau_hz)
    dt = 1.0 / hz
    next_tick = time.monotonic()
    w0, g0 = mit39_init_pulse_us()
    _wrist_us: int = w0
    _gripper_us: int = g0
    _last_trace_log_mono: float = 0.0
    while True:
        next_tick += dt
        cmds = controller.build_motor_commands()
        udp = store.snapshot_udp()
        p_eff = controller.effective_udp_p_rel_deg()
        udp_servo_ok = False
        if CONFIG.udp.enabled:
            try:
                _wrist_us, _gripper_us = decode_udp_servo(p_eff)
                udp_servo_ok = True
            except ValueError:
                logger.warning("UDP servo decode failed: p_rel_deg=%s", p_eff)
        rt = store.snapshot_runtime()
        sc = rt.servo_command
        if sc:
            data = sc.get('data') or {}
            if 'wrist_deg' in data:
                _wrist_us = wrist_deg_to_us(float(data['wrist_deg']))
            if 'grip' in data:
                _gripper_us = grip_state_to_us(float(data['grip']))
            if 'wrist_us' in data:
                _wrist_us = max(500, min(2500, int(data['wrist_us'])))
            if 'gripper_us' in data:
                _gripper_us = max(500, min(2500, int(data['gripper_us'])))
        if cmds:
            serial_mgr.send_mit_cmd_with_servo(cmds, _wrist_us, _gripper_us)
        elif controller.serial_feedback_alive() and udp_servo_ok:
            serial_mgr.send_mit_cmd_with_servo(
                [dict(m) for m in SERVO_ONLY_MOTOR_CMDS],
                _wrist_us,
                _gripper_us,
            )
        fb = store.snapshot_feedback()
        rt = store.snapshot_runtime()
        fb_t = [
            round(float(fb.motors.get(i, {}).get("t", 0.0)), 4)
            for i in range(4)
        ] if fb.motors else None
        now = time.monotonic()
        if now - _last_trace_log_mono >= 1.0:
            _last_trace_log_mono = now
            stm32_fz = not controller.serial_feedback_alive()
            udp_pc_fz = controller.udp_pc_link_latched()
            logger.info(
                "[trace] freeze STM32=%s PC_UDP=%s | seq=%s src=%s reason=%s udp_p=%s cmd_p=%s "
                "cmd_kd=%s fb_p=%s fb_t=%s",
                "冻结" if stm32_fz else "正常",
                "锁存冻结" if udp_pc_fz else "正常",
                udp.seq,
                rt.control_source,
                rt.safety_reason,
                [round(v, 4) for v in p_eff[:4]],
                [round(float(m.get("p", 0.0)), 4) for m in cmds] if cmds else None,
                [round(float(m.get("kd", 0.0)), 4) for m in cmds] if cmds else None,
                [round(float(v), 4) for v in fb.mit_arm_rad] if fb.mit_arm_rad else None,
                fb_t,
            )
        sleep_for = next_tick - time.monotonic()
        if sleep_for <= 0:
            next_tick = time.monotonic()
            continue
        await asyncio.sleep(sleep_for)


def build_components() -> RuntimeComponents:
    store = StateStore(calibration_rad=CONFIG.control.calibration_rad)
    hz = max(1.0, CONFIG.control.tau_hz)
    safe_gate = StartupSafeGate(
        SafeGateConfig(
            enabled=bool(CONFIG.control.startup_safe_gate_enabled),
            vmax_rad_s=tuple(float(v) for v in CONFIG.control.max_cmd_speed_rad_s),
            tol_rad=float(CONFIG.control.startup_safe_gate_tol_rad),
            timeout_sec=float(CONFIG.control.startup_safe_gate_timeout_sec),
            nominal_dt=1.0 / hz,
        )
    )
    return RuntimeComponents(
        store=store,
        serial_mgr=SerialManager(CONFIG.serial, store),
        udp_listener=UdpListener(CONFIG.udp, store),
        state_server=StateServer(CONFIG.server, store),
        controller=ArmController(CONFIG.control, CONFIG.udp, store, safe_gate=safe_gate),
    )


async def run_runtime(components: RuntimeComponents) -> None:
    logger = logging.getLogger(__name__)
    logger.info("gravity model warmup start (pinocchio)")
    t0 = time.monotonic()
    warmup_gravity_model_pinocchio()
    logger.info("gravity model warmup done in %.3fs", time.monotonic() - t0)
    components.serial_mgr.start()
    components.udp_listener.start()
    await _wait_for_initial_arm_feedback(
        components.controller,
        float(CONFIG.control.init_feedback_wait_sec),
    )
    components.controller.prime_hold_latch_from_feedback()
    boot_cmds = components.controller.build_motor_commands()
    if boot_cmds:
        w0, g0 = mit39_init_pulse_us()
        components.serial_mgr.send_mit_cmd_with_servo(boot_cmds, w0, g0)

    tasks = [
        asyncio.create_task(
            run_control_loop(
                components.controller,
                components.serial_mgr,
                components.store,
            )
        ),
        asyncio.create_task(components.state_server.run()),
    ]
    if CONFIG.camera_udp.enabled:
        tasks.append(
            asyncio.create_task(
                run_pi2camera_udp_broadcast(components.store, CONFIG.camera_udp),
            ),
        )
    try:
        await asyncio.gather(*tasks)
    finally:
        components.udp_listener.stop()
        components.serial_mgr.stop()
        components.serial_mgr.join(timeout=2.0)
        for t in tasks:
            t.cancel()
        logger.info("icecreamPi V2 stopped")


async def run_async() -> None:
    setup_logging()
    await run_runtime(build_components())
