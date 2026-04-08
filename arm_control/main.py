"""程序入口：启动串口线程、相机采集与 WebSocket 服务。"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from asyncio import Queue
from typing import Any, Dict, Optional

from .camera_manager import camera_loop
from .config import (
    CONTROL_MODE,
    MIT_CMD_FIXED_KD,
    MIT_CMD_FIXED_KP_NORMAL,
    MIT_CMD_KP_FLOAT_MODE,
    TAU_FF,
    TAU_FF_INPUT,
    set_mit_motor_cmd_params,
    set_tau_calibration_rad,
    set_tau_gain,
    set_kp_float_mode,
)
from .gravity_feedforward import compute_tau_ff_nm
from .m23_gravity_traj import M23GravityTraj
from .mit_stm32_codec import encode_mit_cmd_frame_35
from .protocol import ParsedFrame
from .init_pose_actions import (
    InitPoseController,
    boot_pose_mode_from_env,
)
from .rpi_udp_joint_source import RpiUdpJointSource
from .rpi_udp_premove import rpi_premove_should_run, run_rpi_udp_premove
from .serial_manager import SerialManager
from .state_store import StateStore
from .ws_server import start_ws_server


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] %(levelname)s %(name)s: %(message)s",
    )


def handle_frame(state_store: StateStore, frame: ParsedFrame) -> None:
    """处理从 STM32 收到的帧，根据 cmd 更新状态。

    这里只实现简单示例，具体解析逻辑需按实际协议扩展。
    """
    cmd = frame.cmd
    payload = frame.payload

    # 示例：0x10 电机状态上报（假定每个电机 2 字节 position）
    if cmd == 0x10 and payload:
        for idx, pos_byte in enumerate(payload):
            motor_id = idx
            position = float(pos_byte)  # 仅示意
            state_store.update_motor(motor_id, position=position)


def _build_data_frame_from_cmd(data: Dict[str, Any]) -> bytes:
    """根据 WebSocket command 中的 a0..a5 构造 `DATA:a0,a1,...,a5*XX\\r\\n` 帧。"""
    # 默认值都设为 0，支持缺少某几个字段时仍能工作
    a0 = int(data.get("a0", 0))
    a1 = int(data.get("a1", 0))
    a2 = int(data.get("a2", 0))
    a3 = int(data.get("a3", 0))
    a4 = int(data.get("a4", 0))
    a5 = int(data.get("a5", 0))

    # XOR 校验字段基于「帧头 + 6 整数」（不包含 '*' 和校验码本身）
    payload_str = f"DATA:{a0},{a1},{a2},{a3},{a4},{a5}"
    cs = 0
    for b in payload_str.encode("ascii"):
        cs ^= b
    cs_str = f"{cs:02X}"
    frame = f"{payload_str}*{cs_str}\r\n".encode("ascii")
    return frame


def _build_tau_frame(t0: float, t1: float, t2: float, t3: float) -> bytes:
    """TAU:t0,t1,t2,t3*XX\\r\\n，XOR 与 DATA: 相同（对 TAU:... 不含 * 部分逐字节异或）。"""
    payload_str = f"TAU:{t0:.6f},{t1:.6f},{t2:.6f},{t3:.6f}"
    cs = 0
    for b in payload_str.encode("ascii"):
        cs ^= b
    line = f"{payload_str}*{cs & 0xFF:02X}\r\n"
    return line.encode("ascii")


async def _tau_ff_loop(
    state_store: StateStore,
    serial_mgr: SerialManager,
    m23_traj: M23GravityTraj,
    rpi_udp: Optional[RpiUdpJointSource],
    pose_ctrl: InitPoseController,
) -> None:
    """力矩前馈：按四轴弧度差 + Pinocchio 算 τ，按 send_hz 发 MIT 35B（p/v 见 mit_motor_cmd；kp 见 MIT_CMD_KP_FLOAT_MODE / MIT_CMD_FIXED_KP_NORMAL；kd 见 MIT_CMD_FIXED_KD）。

    可选：``ARM_CONTROL_M23_GRAVITY_TRAJ=1`` 时 M2/M3 目标角走离散步进，力矩 t 仍由当前反馈弧度算重力。

    可选：``ARM_CONTROL_RPI_UDP=1`` 时 p/v 来自 PC UDP（与 rpi_receiver 同包）；关节 5 不用；1/3/4 位置与速度取反、2 不变。
    无包或包过期时不回退 ``mit_motor_cmd``（其默认 p=0 会导致甩动），而是 **p=当前反馈、v=0** 保持位姿。不与 M23 轨迹叠加。
    """
    logger = logging.getLogger(__name__)
    logged_ff_err: str | None = None
    next_tick = time.monotonic()
    while True:
        if CONTROL_MODE != "tau_ff":
            await asyncio.sleep(0.05)
            next_tick = time.monotonic()
            continue

        interval = 1.0 / max(1.0, float(TAU_FF.send_hz))
        next_tick += interval

        if TAU_FF_INPUT == "mit":
            arm_rad = state_store.get_mit_arm_rad()
            if arm_rad is None:
                arm_rad = state_store.get_fb_arm_rad()
        else:
            arm_rad = state_store.get_fb_arm_rad()
        if arm_rad is None:
            next_tick = time.monotonic()
            await asyncio.sleep(interval)
            continue
        cal = TAU_FF.calibration_rad
        if len(cal) != 4:
            logger.error("TAU_FF.calibration_rad 须为 4 个浮点数")
            next_tick = time.monotonic()
            await asyncio.sleep(interval)
            continue
        delta = tuple(arm_rad[i] - cal[i] for i in range(4))
        try:
            t0, t1, t2, t3 = compute_tau_ff_nm(delta)
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if msg != logged_ff_err:
                logged_ff_err = msg
                logger.warning("重力前馈计算失败（缺 numpy/pinocchio 或 URDF？）：%s", exc)
            next_tick = time.monotonic()
            await asyncio.sleep(interval)
            continue
        logged_ff_err = None
        g = float(TAU_FF.gain)
        t0, t1, t2, t3 = t0 * g, t1 * g, t2 * g, t3 * g
        taus = (t0, t1, t2, t3)

        pose_pv = pose_ctrl.step(arm_rad, interval)

        traj_active = (
            pose_pv is None
            and
            rpi_udp is None
            and m23_traj.enabled
            and m23_traj.tick(interval, arm_rad)
        )
        p1_traj = p2_traj = v1_traj = v2_traj = 0.0
        if traj_active:
            p1_traj, p2_traj, v1_traj, v2_traj = m23_traj.p_v_for_motor_1_2(interval)

        udp_pv = (
            rpi_udp.get_motor_p_v_rad(TAU_FF.calibration_rad)
            if rpi_udp is not None
            else None
        )

        cmds: list[dict[str, float]] = []
        for i in range(4):
            m = TAU_FF.mit_motor_cmd[i]
            import arm_control.config as _cfg; kp_send = 0.0 if _cfg.MIT_CMD_KP_FLOAT_MODE else MIT_CMD_FIXED_KP_NORMAL[i]
            p_cmd = float(m["p"])
            v_cmd = float(m["v"])
            if pose_pv is not None:
                p_cmd = float(pose_pv[0][i])
                v_cmd = float(pose_pv[1][i])
            elif udp_pv is not None:
                p_cmd = float(udp_pv[0][i])
                v_cmd = float(udp_pv[1][i])
            elif rpi_udp is not None:
                # 已启用 UDP 流但尚未收到或已超时：勿用 mit_motor_cmd 默认 p=0（会猛甩到错误绝对角）
                p_cmd = float(arm_rad[i])
                v_cmd = 0.0
            elif not traj_active:
                # 无任何主动控制源时，跟随当前反馈角保持位姿，避免 p=0 默认值导致甩动
                p_cmd = float(arm_rad[i])
                v_cmd = 0.0
            elif traj_active:
                if i in (0, 3):
                    p_cmd = float(arm_rad[i])
                    v_cmd = 0.0
                elif i == 1:
                    p_cmd, v_cmd = p1_traj, v1_traj
                elif i == 2:
                    p_cmd, v_cmd = p2_traj, v2_traj
            cmds.append(
                {
                    "p": p_cmd,
                    "v": v_cmd,
                    "kp": kp_send,
                    "kd": MIT_CMD_FIXED_KD[i],
                    "t": float(taus[i]),
                }
            )
        raw = encode_mit_cmd_frame_35(cmds)
        serial_mgr.send_raw(raw)
        # 原 TAU 文本下发（已改为 MIT 35 字节二进制，见 RPI_MIT_CMD_BINARY_ENCODE.md）：
        # raw = _build_tau_frame(t0, t1, t2, t3)
        # serial_mgr.send_raw(raw)

        sleep_for = next_tick - time.monotonic()
        if sleep_for < 0:
            logger.debug("tau_ff 周期滞后 %.3fs，重置调度基准（避免长期欠账）", -sleep_for)
            next_tick = time.monotonic()
        else:
            await asyncio.sleep(sleep_for)


async def _process_commands(
    state_store: StateStore,
    serial_mgr: SerialManager,
    command_queue: "Queue[Dict[str, Any]]",
    pose_ctrl: InitPoseController,
) -> None:
    """处理从 WebSocket 收到的控制指令，并转发到串口。"""
    logger = logging.getLogger(__name__)
    while True:
        cmd_msg = await command_queue.get()
        try:
            if cmd_msg.get("type") != "command":
                continue
            cmd_name = cmd_msg.get("cmd")
            data = cmd_msg.get("data") or {}

            if cmd_name == "set_joint":
                # 仅在 data 模式下转发 DATA:（力矩前馈模式下关闭，避免与 TAU 流冲突）
                if CONTROL_MODE != "data":
                    logger.debug("set_joint 已忽略（当前为 tau_ff 模式，不发 DATA）")
                    continue
                frame = _build_data_frame_from_cmd(data)
                logger.info("下发关节指令到 STM32：%r", frame)
                ok = await asyncio.to_thread(serial_mgr.send_raw_and_wait_for_res, frame)
                if not ok:
                    logger.error(
                        "未收到 STM32 回传 RES：重连/重发失败（frame=%r）", frame
                    )
            elif cmd_name == "set_tau_calibration":
                # 四轴标定零位（弧度）：{"r0":0,"r1":0,"r2":0,"r3":0}
                r0 = float(data.get("r0", 0.0))
                r1 = float(data.get("r1", 0.0))
                r2 = float(data.get("r2", 0.0))
                r3 = float(data.get("r3", 0.0))
                set_tau_calibration_rad(r0, r1, r2, r3)
                logger.info("已更新力矩前馈标定零位 (rad): %s", TAU_FF.calibration_rad)
            elif cmd_name == "set_tau_gain":
                set_tau_gain(float(data.get("gain", 1.0)))
                logger.info("已更新力矩前馈增益: %s", TAU_FF.gain)
            elif cmd_name == "set_mit_motor_cmd":
                motors = data.get("motors")
                if not isinstance(motors, list) or len(motors) != 4:
                    logger.warning(
                        "set_mit_motor_cmd 需要 data.motors 为长度 4 的列表，每项 p/v（kp/kd 下发固定见 config）"
                    )
                    continue
                try:
                    set_mit_motor_cmd_params(motors)
                except ValueError as exc:
                    logger.warning("%s", exc)
                    continue
                logger.info("已更新 MIT 命令 p/v: %s", TAU_FF.mit_motor_cmd)
            elif cmd_name == "goto_home_pose":
                set_kp_float_mode(False)
                pose_ctrl.request_home()
                logger.info("已受理 goto_home_pose，切换正常kp（在 tau_ff 主循环内执行）")
            elif cmd_name == "goto_work_pose":
                set_kp_float_mode(False)
                pose_ctrl.request_work()
                logger.info("已受理 goto_work_pose，切换正常kp（在 tau_ff 主循环内执行并保持）")
            else:
                raw = json.dumps({"cmd": cmd_name, "data": data}).encode("utf-8")
                logger.info(
                    "下发通用命令（二进制封帧）：cmd=%s, payload=%r", cmd_name, raw
                )
                serial_mgr.send_command(0x11, raw)
        finally:
            command_queue.task_done()


async def _run_async() -> None:
    setup_logging()
    logger = logging.getLogger(__name__)

    state_store = StateStore()
    command_queue: "Queue[Dict[str, Any]]" = Queue()

    def on_frame(frame: ParsedFrame) -> None:
        handle_frame(state_store, frame)

    serial_mgr = SerialManager(state_store=state_store, on_frame=on_frame)
    serial_mgr.start()
    m23_traj = M23GravityTraj.from_env()
    rpi_udp = RpiUdpJointSource.from_env()
    pose_ctrl = InitPoseController()
    logger.info("串口管理线程已启动，控制模式=%s", CONTROL_MODE)
    if m23_traj.enabled:
        logger.info(
            "M2/M3 重力协同轨迹已启用：M2 %+g° M3 %+g°，v_max=%g rad/s（ARM_CONTROL_M23_DEG_M2/M3、M23_V_MAX）",
            m23_traj.deg_m2,
            m23_traj.deg_m3,
            m23_traj.v_max,
        )

    if CONTROL_MODE == "tau_ff":
        logger.info(
            "预热 Pinocchio 重力模型（首次加载 URDF，树莓派上可能需数秒）…"
        )
        try:
            await asyncio.to_thread(
                lambda: compute_tau_ff_nm((0.0, 0.0, 0.0, 0.0))
            )
            logger.info(
                "Pinocchio 预热完成，MIT 命令下发目标频率 %.1f Hz",
                float(TAU_FF.send_hz),
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning("Pinocchio 预热失败（运行中仍会重试）：%s", exc)

        boot_pose = boot_pose_mode_from_env()
        if boot_pose == "home":
            pose_ctrl.request_home()
            logger.info("启动姿态：归位请求已下发到主循环（ARM_CONTROL_BOOT_POSE=home）")
        elif boot_pose == "work":
            pose_ctrl.request_work()
            logger.info("启动姿态：工作位请求已下发到主循环（ARM_CONTROL_BOOT_POSE=work）")
        elif rpi_premove_should_run():
            logger.info("RPI UDP 预移动：到位后再 bind 接收线程…")
            await run_rpi_udp_premove(state_store, serial_mgr)

    if rpi_udp is not None:
        rpi_udp.start()

    def on_camera_frame(frame_b64: str) -> None:
        logger.debug("收到相机帧（长度=%d）", len(frame_b64))

    tasks = [
        asyncio.create_task(start_ws_server(state_store, command_queue)),
        asyncio.create_task(_process_commands(state_store, serial_mgr, command_queue, pose_ctrl)),
        asyncio.create_task(camera_loop(on_camera_frame)),
        asyncio.create_task(
            _tau_ff_loop(state_store, serial_mgr, m23_traj, rpi_udp, pose_ctrl)
        ),
    ]

    try:
        await asyncio.gather(*tasks)
    finally:
        if rpi_udp is not None:
            rpi_udp.stop()
        serial_mgr.stop()
        logger.info("程序退出，已请求串口线程停止")


def main() -> None:
    asyncio.run(_run_async())


if __name__ == "__main__":
    main()
