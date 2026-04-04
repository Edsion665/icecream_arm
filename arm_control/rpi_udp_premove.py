"""RPI UDP 开启前：四轴先运动到相对标定角的预设位（按 MIT 帧率限速），再启动 UDP 接收。"""

from __future__ import annotations

import asyncio
import logging
import math
import os
import time
from typing import Tuple

from .config import (
    MIT_CMD_FIXED_KD,
    MIT_CMD_FIXED_KP_NORMAL,
    MIT_CMD_KP_FLOAT_MODE,
    TAU_FF,
    TAU_FF_INPUT,
)
from .gravity_feedforward import compute_tau_ff_nm
from .rpi_udp_joint_source import rpi_udp_stream_enabled
from .mit_stm32_codec import encode_mit_cmd_frame_35
from .serial_manager import SerialManager
from .state_store import StateStore

logger = logging.getLogger(__name__)

# 相对 TAU_FF.calibration_rad 的四电机目标角（度），电机空间（与 UDP 的 PC 关节符号无关）
DEFAULT_REL_DEG_DEG: Tuple[float, float, float, float] = (
    18.99,
    -18.64,
    -121.12,
    -77.51,
)


def rpi_premove_should_run() -> bool:
    """与 PC UDP 配套：启用 UDP 流时默认先预到位，可用 ARM_CONTROL_RPI_PREMOVE_SKIP=1 跳过。"""
    if not rpi_udp_stream_enabled():
        return False
    v = os.environ.get("ARM_CONTROL_RPI_PREMOVE_SKIP", "").strip().lower()
    return v not in ("1", "true", "yes", "on")


def _rel_deg_from_env() -> Tuple[float, float, float, float]:
    raw = os.environ.get("ARM_CONTROL_RPI_PREMOVE_REL_DEG", "").strip()
    if not raw:
        return DEFAULT_REL_DEG_DEG
    parts = [p.strip() for p in raw.split(",")]
    if len(parts) != 4:
        logger.warning(
            "ARM_CONTROL_RPI_PREMOVE_REL_DEG 须为 4 个逗号分隔浮点数，已用默认 %s",
            DEFAULT_REL_DEG_DEG,
        )
        return DEFAULT_REL_DEG_DEG
    try:
        return tuple(float(x) for x in parts)  # type: ignore[return-value]
    except ValueError:
        logger.warning(
            "ARM_CONTROL_RPI_PREMOVE_REL_DEG 解析失败，已用默认 %s",
            DEFAULT_REL_DEG_DEG,
        )
        return DEFAULT_REL_DEG_DEG


def _v_max() -> float:
    try:
        return max(1e-6, float(os.environ.get("ARM_CONTROL_RPI_PREMOVE_VMAX", "0.4")))
    except ValueError:
        return 0.4


def _tol_rad() -> float:
    try:
        return max(1e-4, float(os.environ.get("ARM_CONTROL_RPI_PREMOVE_TOL_RAD", "0.008")))
    except ValueError:
        return 0.008


def _wait_fb_sec() -> float:
    try:
        return max(1.0, float(os.environ.get("ARM_CONTROL_RPI_PREMOVE_WAIT_FB_SEC", "45")))
    except ValueError:
        return 45.0


def _max_move_sec() -> float:
    try:
        return max(5.0, float(os.environ.get("ARM_CONTROL_RPI_PREMOVE_MAX_SEC", "180")))
    except ValueError:
        return 180.0


def _arm_rad_snapshot(state_store: StateStore) -> Tuple[float, float, float, float] | None:
    if TAU_FF_INPUT == "mit":
        arm_rad = state_store.get_mit_arm_rad()
        if arm_rad is None:
            arm_rad = state_store.get_fb_arm_rad()
    else:
        arm_rad = state_store.get_fb_arm_rad()
    return arm_rad


async def run_rpi_udp_premove(
    state_store: StateStore,
    serial_mgr: SerialManager,
) -> None:
    """在 tau_ff 主循环与 UDP 监听启动之前调用；按 send_hz 发 MIT，p 限速趋近目标，τ 为重力前馈。"""
    rel_deg = _rel_deg_from_env()
    v_max = _v_max()
    tol = _tol_rad()
    hz = max(1.0, float(TAU_FF.send_hz))
    interval = 1.0 / hz
    wait_fb = _wait_fb_sec()
    max_move = _max_move_sec()

    logger.info(
        "RPI 预移动：相对标定角(°)=%s  v_max=%.3g rad/s  tol=%.4g rad  帧率=%.1f Hz",
        rel_deg,
        v_max,
        tol,
        hz,
    )

    t_deadline_fb = time.monotonic() + wait_fb
    arm_rad: Tuple[float, float, float, float] | None = None
    while time.monotonic() < t_deadline_fb:
        arm_rad = _arm_rad_snapshot(state_store)
        if arm_rad is not None:
            break
        await asyncio.sleep(0.05)
    if arm_rad is None:
        logger.error("预移动：超时未拿到四轴反馈，跳过预移动并仍将启动 UDP（请检查上行 MIT/FB）")
        return

    cal = TAU_FF.calibration_rad
    if len(cal) != 4:
        logger.error("预移动：calibration_rad 长度须为 4，跳过")
        return

    target_p = [
        float(cal[i]) + math.radians(float(rel_deg[i])) for i in range(4)
    ]
    p_cmd = [float(arm_rad[i]) for i in range(4)]

    logged_ff_err: str | None = None
    t0_move = time.monotonic()
    next_tick = time.monotonic()

    while True:
        if time.monotonic() - t0_move > max_move:
            logger.error("预移动：超过 %.0fs 仍未到位，中止并启动 UDP", max_move)
            break

        interval = 1.0 / max(1.0, float(TAU_FF.send_hz))
        next_tick += interval

        arm_rad = _arm_rad_snapshot(state_store)
        if arm_rad is None:
            await asyncio.sleep(interval)
            next_tick = time.monotonic()
            continue

        delta_fb = tuple(arm_rad[i] - cal[i] for i in range(4))
        try:
            t0, t1, t2, t3 = compute_tau_ff_nm(delta_fb)
        except Exception as exc:  # noqa: BLE001
            msg = str(exc)
            if msg != logged_ff_err:
                logged_ff_err = msg
                logger.warning("预移动：重力前馈失败：%s", exc)
            await asyncio.sleep(interval)
            next_tick = time.monotonic()
            continue
        logged_ff_err = None
        g = float(TAU_FF.gain)
        taus = (t0 * g, t1 * g, t2 * g, t3 * g)

        cmds: list[dict[str, float]] = []
        for i in range(4):
            err = target_p[i] - p_cmd[i]
            step_max = v_max * interval
            step = max(-step_max, min(step_max, err))
            p_cmd[i] += step
            v_cmd = step / interval if interval > 0 else 0.0
            kp_send = 0.0 if MIT_CMD_KP_FLOAT_MODE else MIT_CMD_FIXED_KP_NORMAL[i]
            cmds.append(
                {
                    "p": p_cmd[i],
                    "v": v_cmd,
                    "kp": kp_send,
                    "kd": MIT_CMD_FIXED_KD[i],
                    "t": float(taus[i]),
                }
            )

        raw = encode_mit_cmd_frame_35(cmds)
        serial_mgr.send_raw(raw)

        max_err = max(abs(target_p[i] - p_cmd[i]) for i in range(4))
        if max_err <= tol:
            # 到位后再发一帧精确目标、速度置零，减少稳态误差
            settle_cmds: list[dict[str, float]] = []
            for i in range(4):
                kp_send = 0.0 if MIT_CMD_KP_FLOAT_MODE else MIT_CMD_FIXED_KP_NORMAL[i]
                settle_cmds.append(
                    {
                        "p": target_p[i],
                        "v": 0.0,
                        "kp": kp_send,
                        "kd": MIT_CMD_FIXED_KD[i],
                        "t": float(taus[i]),
                    }
                )
            serial_mgr.send_raw(encode_mit_cmd_frame_35(settle_cmds))
            logger.info("预移动完成，即将开启 UDP 关节流")
            break

        sleep_for = next_tick - time.monotonic()
        if sleep_for < 0:
            next_tick = time.monotonic()
        else:
            await asyncio.sleep(sleep_for)
