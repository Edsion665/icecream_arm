"""相机采集管理：封装具体相机 SDK，当前提供占位实现。"""

from __future__ import annotations

import asyncio
import base64
import logging
from typing import AsyncIterator, Optional

from .config import CONFIG

LOGGER = logging.getLogger(__name__)


async def dummy_camera_frames() -> AsyncIterator[str]:
    """占位相机实现：周期性输出固定字符串，方便联调。"""
    interval = CONFIG.network.camera_push_interval_sec
    count = 0
    while True:
        # 这里只是简单返回一段 base64 文本，真实项目中应替换为相机 SDK 采集 JPEG
        fake_jpeg = f"dummy_frame_{count}".encode("utf-8")
        encoded = base64.b64encode(fake_jpeg).decode("ascii")
        yield encoded
        count += 1
        await asyncio.sleep(interval)


async def camera_loop(
    frame_callback: callable[[str], None],
) -> None:
    """持续采集相机帧并通过回调上报。

    真正接入相机时，应在此处调用 SDK 获取图像并编码为 base64 JPEG。
    """
    if not CONFIG.camera.enabled:
        LOGGER.info("相机功能已禁用")
        return

    LOGGER.info("启动相机采集循环（占位实现）")
    async for frame in dummy_camera_frames():
        frame_callback(frame)

