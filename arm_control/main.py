"""程序入口：启动串口线程、相机采集与 WebSocket 服务。"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from asyncio import Queue
from typing import Any, Dict

from .camera_manager import camera_loop
from .config import CONTROL_MODE, TAU_FF, set_tau_calibration_rad, set_tau_gain
from .gravity_feedforward import compute_tau_ff_nm
from .protocol import ParsedFrame
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


async def _tau_ff_loop(state_store: StateStore, serial_mgr: SerialManager) -> None:
    """力矩前馈：按 FB 弧度差 + Pinocchio 计算四轴 τ，按 send_hz 发 TAU:（不等待 RES）。"""
    logger = logging.getLogger(__name__)
    logged_ff_err: str | None = None
    while True:
        if CONTROL_MODE != "tau_ff":
            await asyncio.sleep(0.05)
            continue

        interval = 1.0 / max(1.0, float(TAU_FF.send_hz))
        iter_start = time.monotonic()

        fb = state_store.get_fb_arm_rad()
        if fb is None:
            await asyncio.sleep(interval)
            continue
        cal = TAU_FF.calibration_rad
        if len(cal) != 4:
            logger.error("TAU_FF.calibration_rad 须为 4 个浮点数")
            await asyncio.sleep(interval)
            continue
        delta = tuple(fb[i] - cal[i] for i in range(4))
        try:
            t0, t1, t2, t3 = compute_tau_ff_nm(delta)
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if msg != logged_ff_err:
                logged_ff_err = msg
                logger.warning("重力前馈计算失败（缺 numpy/pinocchio 或 URDF？）：%s", exc)
            await asyncio.sleep(interval)
            continue
        logged_ff_err = None
        g = float(TAU_FF.gain)
        t0, t1, t2, t3 = t0 * g, t1 * g, t2 * g, t3 * g
        raw = _build_tau_frame(t0, t1, t2, t3)
        serial_mgr.send_raw(raw)

        elapsed = time.monotonic() - iter_start
        remain = interval - elapsed
        if remain > 0:
            await asyncio.sleep(remain)


async def _process_commands(
    state_store: StateStore,
    serial_mgr: SerialManager,
    command_queue: "Queue[Dict[str, Any]]",
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
    logger.info("串口管理线程已启动，控制模式=%s", CONTROL_MODE)

    if CONTROL_MODE == "tau_ff":
        logger.info(
            "预热 Pinocchio 重力模型（首次加载 URDF，树莓派上可能需数秒）…"
        )
        try:
            await asyncio.to_thread(
                lambda: compute_tau_ff_nm((0.0, 0.0, 0.0, 0.0))
            )
            logger.info(
                "Pinocchio 预热完成，TAU 目标频率 %.1f Hz",
                float(TAU_FF.send_hz),
            )
        except Exception as exc:  # noqa: BLE001
            logger.warning("Pinocchio 预热失败（运行中仍会重试）：%s", exc)

    def on_camera_frame(frame_b64: str) -> None:
        logger.debug("收到相机帧（长度=%d）", len(frame_b64))

    tasks = [
        asyncio.create_task(start_ws_server(state_store, command_queue)),
        asyncio.create_task(_process_commands(state_store, serial_mgr, command_queue)),
        asyncio.create_task(camera_loop(on_camera_frame)),
        asyncio.create_task(_tau_ff_loop(state_store, serial_mgr)),
    ]

    try:
        await asyncio.gather(*tasks)
    finally:
        serial_mgr.stop()
        logger.info("程序退出，已请求串口线程停止")


def main() -> None:
    asyncio.run(_run_async())


if __name__ == "__main__":
    main()
