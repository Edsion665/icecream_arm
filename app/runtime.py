"""Runtime assembly and lifecycle orchestration."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import logging
import math
import time

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


async def _run_premove(
    store: StatePort,
    serial_mgr: MotorCommandSink,
    controller: ArmController,
) -> None:
    ip = CONFIG.control.init_premove
    logger = logging.getLogger(__name__)
    if ip.skip:
        logger.info(
            "premove 已跳过：当前配置为跳过（环境变量未设置时默认跳过）。"
            "若要启用预旋转：export ARM_CONTROL_RPI_PREMOVE_SKIP=0"
        )
        return
    if not controller.has_arm_feedback():
        logger.warning("premove skipped: no arm feedback (unsafe to step without p)")
        return
    cal = store.get_calibration_rad()
    target = [cal[i] + math.radians(ip.rel_deg[i]) for i in range(4)]
    hz = max(1.0, CONFIG.control.tau_hz)
    dt = 1.0 / hz
    deadline = time.monotonic() + float(ip.max_sec)
    logger.info("premove started: target_rel_deg=%s", ip.rel_deg)

    # 从当前反馈角出发，维护独立的 p_cmd 状态，避免每帧从 hold_p_latch 重读导致步进无法累积
    arm_rad = controller._select_feedback_rad()
    p_cmd = [float(arm_rad[i]) for i in range(4)]
    w0, g0 = mit39_init_pulse_us()

    while time.monotonic() < deadline:
        cmds = controller.build_motor_commands()
        if not cmds:
            await asyncio.sleep(dt)
            continue
        for i in range(4):
            err = target[i] - p_cmd[i]
            step = max(-ip.vmax_rad_s * dt, min(ip.vmax_rad_s * dt, err))
            p_cmd[i] += step
            cmds[i]["p"] = p_cmd[i]
            cmds[i]["v"] = step / dt
        serial_mgr.send_mit_cmd_with_servo(cmds, w0, g0)

        done = max(abs(target[i] - p_cmd[i]) for i in range(4)) <= ip.tol_rad
        if done:
            logger.info("premove completed")
            # 与 build_motor_commands 的 hold 共用 _hold_p_latch；否则 premove 后仍锁在启动瞬间姿态。
            controller.set_hold_latch_rad(
                (float(target[0]), float(target[1]), float(target[2]), float(target[3])),
            )
            return
        await asyncio.sleep(dt)

    logger.warning("premove timeout, continue normal loop")
    controller.set_hold_latch_rad(
        (float(p_cmd[0]), float(p_cmd[1]), float(p_cmd[2]), float(p_cmd[3])),
    )


async def _wait_for_initial_arm_feedback(controller: ArmController, timeout_sec: float) -> None:
    """与 arm_control._tau_ff_loop 一致：有 MIT/FB 角度后再可靠下发 hold/UDP，避免 cmds=[] 的空窗。"""
    if timeout_sec <= 0:
        return
    logger = logging.getLogger(__name__)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        if controller.has_arm_feedback():
            logger.info("initial arm feedback ready after %.3fs", time.monotonic() - t0)
            return
        await asyncio.sleep(0.01)
    logger.warning(
        "still no arm feedback after %.1fs (serial/STM32?); main loop will start anyway",
        timeout_sec,
    )


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
        udp_servo_ok = False
        if (
            CONFIG.udp.enabled
            and udp.recv_mono > 0
            and time.monotonic() - udp.recv_mono <= CONFIG.udp.stale_sec
        ):
            try:
                _wrist_us, _gripper_us = decode_udp_servo(udp.p_rel_deg)
                udp_servo_ok = True
            except ValueError:
                logger.warning("UDP servo decode failed: p_rel_deg=%s", udp.p_rel_deg)
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
        elif udp_servo_ok:
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
            logger.info(
                "[trace] seq=%s src=%s reason=%s udp_p=%s cmd_p=%s cmd_kd=%s fb_p=%s fb_t=%s",
                udp.seq,
                rt.control_source,
                rt.safety_reason,
                [round(v, 4) for v in udp.p_rel_deg[:4]],
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
    return RuntimeComponents(
        store=store,
        serial_mgr=SerialManager(CONFIG.serial, store),
        udp_listener=UdpListener(CONFIG.udp, store),
        state_server=StateServer(CONFIG.server, store),
        controller=ArmController(CONFIG.control, CONFIG.udp, store),
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
    await _run_premove(components.store, components.serial_mgr, components.controller)

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
