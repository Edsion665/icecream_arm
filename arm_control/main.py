"""程序入口：启动串口线程、相机采集与 WebSocket 服务。"""

from __future__ import annotations

import asyncio
import json
import logging
from asyncio import Queue
from typing import Any, Dict

from .camera_manager import camera_loop
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

    # XOR 校验字段基于“帧头 + 6 整数”（不包含 '*' 和校验码本身）
    payload_str = f"DATA:{a0},{a1},{a2},{a3},{a4},{a5}"
    cs = 0
    for b in payload_str.encode("ascii"):
        cs ^= b
    cs_str = f"{cs:02X}"
    frame = f"{payload_str}*{cs_str}\r\n".encode("ascii")
    return frame


async def _process_commands(
    state_store: StateStore,
    serial_mgr: SerialManager,
    command_queue: "Queue[Dict[str, Any]]",
) -> None:
    """处理从 WebSocket 收到的控制指令，并转发到串口。"""
    while True:
        cmd_msg = await command_queue.get()
        try:
            if cmd_msg.get("type") != "command":
                continue
            cmd_name = cmd_msg.get("cmd")
            data = cmd_msg.get("data") or {}

            if cmd_name == "set_joint":
                # 将上位机的 set_joint 命令转换为 `DATA:...*XX\r\n` 文本协议
                frame = _build_data_frame_from_cmd(data)
                logging.getLogger(__name__).info("下发关节指令到 STM32：%r", frame)
                ok = await asyncio.to_thread(serial_mgr.send_raw_and_wait_for_res, frame)
                if not ok:
                    logging.getLogger(__name__).error(
                        "未收到 STM32 回传 RES：重连/重发失败（frame=%r）", frame
                    )
            else:
                # 其他命令暂时仍用原有二进制封帧方式（如后续扩展）
                raw = json.dumps({"cmd": cmd_name, "data": data}).encode("utf-8")
                logging.getLogger(__name__).info(
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
    logger.info("串口管理线程已启动")

    def on_camera_frame(frame_b64: str) -> None:
        # TODO: 结合 WebSocket 推送相机帧，这里先占位
        logger.debug("收到相机帧（长度=%d）", len(frame_b64))

    tasks = [
        asyncio.create_task(start_ws_server(state_store, command_queue)),
        asyncio.create_task(_process_commands(state_store, serial_mgr, command_queue)),
        asyncio.create_task(camera_loop(on_camera_frame)),
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

