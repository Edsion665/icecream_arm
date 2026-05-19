#!/usr/bin/env python3
"""沿 docs/tra.txt 分割点采集 MIT 反馈（2/3/4 轴相对角控制）。

- **相对角输入**：tra.txt 为相对标定零位的 deg；下行前由 ``udp_rel_to_motor_pv`` 结合
  ``CONFIG.control.calibration_rad``（或 ``ARM_CONTROL_CAL_R0..R3``）换算为绝对 rad。
  换机械臂时只需改标定零位，同一份 tra.txt 可复用。
- **开环分割点**：切换点后固定时长下发并采样，不做到位判定。
- **结束流程**：J1 反向累计至 -90° 并完成该圈轨迹后 → 限速回 0° → 限速至 +90° → 退出。
- **中断安全**：JSONL 每点即时 ``flush+fsync``，Ctrl+C 后已写入行均保留。

用法::

    python -m icecream.calibration.tra_collect --tra icecream/docs/tra.txt
"""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import os
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

os.environ.setdefault("ARM_CONTROL_RPI_UDP", "1")

from icecream.app.runtime import SERVO_ONLY_MOTOR_CMDS, setup_logging
from icecream.app.safe_gate import SafeGateConfig, StartupSafeGate
from icecream.calculator import warmup_gravity_model_pinocchio
from icecream.config import CONFIG, MOTOR_AXIS_SIGN, UDP_VECTOR_DIM
from icecream.controller import ArmController
from icecream.domain.mapping import udp_rel_to_motor_pv
from icecream.infra.udp.packet import decode_udp_pi2stm_aux, decode_udp_servo, mit39_init_pulse_us
from icecream.serial import SerialManager
from icecream.state_store import StateStore

LOGGER = logging.getLogger(__name__)
_REL_LINE = re.compile(r"^\[rel\] p=\[([-\d. ]+)\]$")
_AXIS1_INDEX = 0


def _now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def count_jsonl_lines(path: Path) -> int:
    if not path.is_file():
        return 0
    n = 0
    with path.open(encoding="utf-8") as f:
        for line in f:
            if line.strip():
                n += 1
    return n


def load_tra_rel234(path: Path) -> list[list[float]]:
    """解析 tra.txt 中每点的 [rel] p（2/3/4 轴，单位 deg）。"""
    points: list[list[float]] = []
    for idx, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw.strip()
        m = _REL_LINE.match(line)
        if not m:
            continue
        vals = [float(x) for x in m.group(1).split()]
        if len(vals) != 3:
            raise ValueError(f"{path}:{idx} [rel] 须为 3 个数值（2/3/4 轴）: {line!r}")
        points.append(vals)
    if not points:
        raise ValueError(f"{path} 中未找到 [rel] p= 行")
    return points


def build_p_rel_deg(axis1_rel_deg: float, rel234: Sequence[float]) -> tuple[float, ...]:
    p = [0.0] * UDP_VECTOR_DIM
    p[0] = float(axis1_rel_deg)
    p[1] = float(rel234[0])
    p[2] = float(rel234[1])
    p[3] = float(rel234[2])
    return tuple(p)


def _mean4(samples: list[list[float]]) -> list[float]:
    n = float(len(samples))
    return [sum(row[i] for row in samples) / n for i in range(4)]


def build_tra_runtime() -> tuple[StateStore, SerialManager, ArmController]:
    store = StateStore(calibration_rad=CONFIG.control.calibration_rad)
    hz = max(1.0, CONFIG.control.tau_hz)
    safe_gate = StartupSafeGate(
        SafeGateConfig(
            enabled=True,
            vmax_rad_s=tuple(float(v) for v in CONFIG.control.max_cmd_speed_rad_s),
            tol_rad=float(CONFIG.control.startup_safe_gate_tol_rad),
            timeout_sec=float(CONFIG.control.startup_safe_gate_timeout_sec),
            nominal_dt=1.0 / hz,
        )
    )
    serial_mgr = SerialManager(CONFIG.serial, store)
    controller = ArmController(CONFIG.control, CONFIG.udp, store, safe_gate=safe_gate)
    return store, serial_mgr, controller


def append_jsonl(path: Path, record: dict[str, Any]) -> None:
    """追加一行并落盘，中断后已写入数据不丢失。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
        f.flush()
        os.fsync(f.fileno())


def _reset_controller_ramp(controller: ArmController) -> None:
    if controller._safe_gate is not None:  # noqa: SLF001
        controller._safe_gate.reset()  # noqa: SLF001
    controller._ramp_p = None  # noqa: SLF001
    controller._ramp_last_mono = 0.0  # noqa: SLF001


def _send_mit_tick(
    store: StateStore,
    serial_mgr: SerialManager,
    controller: ArmController,
    seq: int,
    p_rel: tuple[float, ...],
    omega0: tuple[float, ...],
) -> tuple[int, list[dict[str, float]] | None]:
    seq = (seq + 1) & 0xFFFFFFFF
    store.update_udp(seq, p_rel, omega0)
    cmds = controller.build_motor_commands()
    p_eff = controller.effective_udp_p_rel_deg()
    wrist_us, gripper_us = decode_udp_servo(p_eff)
    stepper_deg, conveyor_run = decode_udp_pi2stm_aux(p_eff)
    if cmds:
        serial_mgr.send_mit_cmd_with_servo(
            cmds,
            wrist_us,
            gripper_us,
            stepper_deg=stepper_deg,
            conveyor_run=conveyor_run,
        )
    elif controller.serial_feedback_alive():
        serial_mgr.send_mit_cmd_with_servo(
            [dict(m) for m in SERVO_ONLY_MOTOR_CMDS],
            wrist_us,
            gripper_us,
            stepper_deg=stepper_deg,
            conveyor_run=conveyor_run,
        )
    return seq, cmds


def _axis1_motor_reached(
    cmds: list[dict[str, float]] | None,
    target_p: Sequence[float],
    tol_rad: float,
) -> bool:
    if not cmds:
        return False
    return abs(float(cmds[_AXIS1_INDEX]["p"]) - float(target_p[_AXIS1_INDEX])) <= tol_rad


def _axis1_ramp_timeout_sec(from_rel_deg: float, to_rel_deg: float) -> float:
    """按轴 1 限速粗算运动时间上限。"""
    delta_motor_rad = abs(
        float(MOTOR_AXIS_SIGN[_AXIS1_INDEX])
        * math.radians(float(to_rel_deg) - float(from_rel_deg))
    )
    vmax = max(1e-4, float(CONFIG.control.max_cmd_speed_rad_s[_AXIS1_INDEX]))
    est = delta_motor_rad / vmax
    return max(
        float(CONFIG.control.startup_safe_gate_timeout_sec),
        est * 2.0 + 3.0,
    )


async def wait_for_feedback(controller: ArmController, timeout_sec: float) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        if controller.has_arm_feedback():
            LOGGER.info("arm feedback ready after %.3fs", time.monotonic() - t0)
            return
        await asyncio.sleep(0.01)
    raise RuntimeError(f"{timeout_sec:.1f}s 内未收到串口关节反馈")


async def open_loop_dwell_and_sample(
    store: StateStore,
    serial_mgr: SerialManager,
    controller: ArmController,
    seq: int,
    p_rel: tuple[float, ...],
    omega0: tuple[float, ...],
    dwell_sec: float,
    sample_hz: float,
) -> tuple[int, list[float], float]:
    """开环持续下发同一分割点目标 ``dwell_sec`` 秒，期间静止采样 MIT 反馈。"""
    hz = max(1.0, CONFIG.control.tau_hz)
    dt = 1.0 / hz
    sample_period = 1.0 / max(5.0, sample_hz)
    p_samples: list[list[float]] = []
    t_samples: list[float] = []
    end_at = time.monotonic() + max(0.0, dwell_sec)
    next_sample = time.monotonic()

    while time.monotonic() < end_at:
        seq, _ = _send_mit_tick(store, serial_mgr, controller, seq, p_rel, omega0)
        now = time.monotonic()
        if now >= next_sample:
            fb = store.snapshot_feedback()
            if fb.mit_arm_rad is not None and 0 in fb.motors:
                p_samples.append([float(v) for v in fb.mit_arm_rad])
                t_samples.append(float(fb.motors[0].get("t", 0.0)))
            next_sample += sample_period
        await asyncio.sleep(dt)

    if not p_samples:
        raise RuntimeError(f"{dwell_sec:.1f}s 窗口内未收到 MIT 反馈")
    avg_p = _mean4(p_samples)
    avg_t0 = sum(t_samples) / float(len(t_samples))
    return seq, avg_p, avg_t0


async def ramp_axis1_to_rel(
    store: StateStore,
    serial_mgr: SerialManager,
    controller: ArmController,
    cal: tuple[float, float, float, float],
    seq: int,
    omega0: tuple[float, ...],
    axis1_rel_deg: float,
    hold_rel234: Sequence[float],
    from_rel_deg: float,
    tol_rad: float,
    label: str,
) -> int:
    """J1 限速趋近目标相对角（保持 2/3/4 轴 rel 不变），用于结束回零段。"""
    p_rel = build_p_rel_deg(axis1_rel_deg, hold_rel234)
    target_p, _ = udp_rel_to_motor_pv(cal, p_rel, omega0)
    _reset_controller_ramp(controller)
    timeout = _axis1_ramp_timeout_sec(from_rel_deg, axis1_rel_deg)
    hz = max(1.0, CONFIG.control.tau_hz)
    dt = 1.0 / hz
    deadline = time.monotonic() + timeout
    LOGGER.info(
        "%s: axis1_rel %.2f -> %.2f deg (motor p0≈%.4f), timeout=%.1fs",
        label,
        from_rel_deg,
        axis1_rel_deg,
        target_p[_AXIS1_INDEX],
        timeout,
    )

    while time.monotonic() < deadline:
        seq, cmds = _send_mit_tick(store, serial_mgr, controller, seq, p_rel, omega0)
        if _axis1_motor_reached(cmds, target_p, tol_rad):
            LOGGER.info("%s: axis1 cmd 到位 (tol=%.4f rad)", label, tol_rad)
            return seq
        await asyncio.sleep(dt)

    raise RuntimeError(f"{label}: 轴 1 在 {timeout:.1f}s 内未到达目标")


async def run_trajectory(
    store: StateStore,
    serial_mgr: SerialManager,
    controller: ArmController,
    waypoints: list[list[float]],
    *,
    dwell_sec: float,
    sample_hz: float,
    out_path: Path,
    axis1_step_deg: float,
    axis1_forward_deg: float,
    axis1_final_deg: float,
    tol_rad: float,
) -> str:
    """主采集循环；正常结束返回 ``completed``，Ctrl+C 返回 ``interrupted``。"""
    cal = store.get_calibration_rad()
    omega0 = (0.0,) * UDP_VECTOR_DIM
    seq = 0
    cycle = 0
    axis1_rel = 0.0
    last_rel234 = list(waypoints[-1])
    step_mag = abs(float(axis1_step_deg))
    if step_mag <= 0:
        raise ValueError("axis1_step_deg 须为正数")
    forward_sign = 1.0 if float(axis1_forward_deg) >= 0.0 else -1.0
    forward_mag = abs(float(axis1_forward_deg))
    max_cycle = int(round(forward_mag / step_mag))

    append_jsonl(
        out_path,
        {
            "record_type": "session_meta",
            "timestamp_utc": _now_utc_iso(),
            "calibration_rad": [round(c, 6) for c in cal],
            "axis1_step_deg": axis1_step_deg,
            "axis1_forward_deg": axis1_forward_deg,
            "axis1_final_deg": axis1_final_deg,
            "waypoint_count": len(waypoints),
            "note": "tra.txt 为相对角；绝对角由本机 calibration_rad 映射",
        },
    )

    try:
        while cycle <= max_cycle:
            axis1_rel = forward_sign * float(cycle) * step_mag
            LOGGER.info(
                "=== trajectory cycle %d/%d (axis1_rel=%.2f deg, sign=%+.0f) ===",
                cycle,
                max_cycle,
                axis1_rel,
                forward_sign,
            )
            for wp_idx, rel234 in enumerate(waypoints):
                last_rel234 = list(rel234)
                p_rel = build_p_rel_deg(axis1_rel, rel234)
                target_p, _ = udp_rel_to_motor_pv(cal, p_rel, omega0)
                _reset_controller_ramp(controller)

                LOGGER.info(
                    "cycle=%d wp=%d/%d dwell %.1fs | target_rel=[%.4f, %.4f, %.4f, %.4f]",
                    cycle,
                    wp_idx + 1,
                    len(waypoints),
                    dwell_sec,
                    p_rel[0],
                    p_rel[1],
                    p_rel[2],
                    p_rel[3],
                )

                seq, avg_p, avg_t0 = await open_loop_dwell_and_sample(
                    store,
                    serial_mgr,
                    controller,
                    seq,
                    p_rel,
                    omega0,
                    dwell_sec,
                    sample_hz,
                )
                append_jsonl(
                    out_path,
                    {
                        "record_type": "waypoint_sample",
                        "timestamp_utc": _now_utc_iso(),
                        "timestamp_mono": round(time.monotonic(), 6),
                        "cycle": cycle,
                        "waypoint_index": wp_idx,
                        "waypoint_label": wp_idx + 1,
                        "dwell_sec": dwell_sec,
                        "target_rel_deg": [round(p_rel[i], 6) for i in range(4)],
                        "target_motor_p_rad": [round(target_p[i], 6) for i in range(4)],
                        "fb_p_rad": [round(x, 6) for x in avg_p],
                        "fb_t_axis1_nm": round(avg_t0, 6),
                    },
                )
                LOGGER.info(
                    "recorded wp %d: fb_p=%s fb_t_axis1=%.4f",
                    wp_idx + 1,
                    [round(x, 4) for x in avg_p],
                    avg_t0,
                )

            if cycle >= max_cycle:
                LOGGER.info(
                    "axis1 已达目标相对角 %.1f deg，开始结束段：回 0° → %.1f°",
                    axis1_rel,
                    axis1_final_deg,
                )
                seq = await ramp_axis1_to_rel(
                    store,
                    serial_mgr,
                    controller,
                    cal,
                    seq,
                    omega0,
                    0.0,
                    last_rel234,
                    axis1_rel,
                    tol_rad,
                    "shutdown_j1_to_zero",
                )
                seq = await ramp_axis1_to_rel(
                    store,
                    serial_mgr,
                    controller,
                    cal,
                    seq,
                    omega0,
                    float(axis1_final_deg),
                    last_rel234,
                    0.0,
                    tol_rad,
                    "shutdown_j1_to_final",
                )
                append_jsonl(
                    out_path,
                    {
                        "record_type": "session_complete",
                        "timestamp_utc": _now_utc_iso(),
                        "axis1_final_rel_deg": float(axis1_final_deg),
                        "status": "completed",
                    },
                )
                LOGGER.info("采集正常结束（J1 已至 %.1f°）", axis1_final_deg)
                return "completed"

            cycle += 1

    except asyncio.CancelledError:
        append_jsonl(
            out_path,
            {
                "record_type": "session_complete",
                "timestamp_utc": _now_utc_iso(),
                "status": "interrupted",
            },
        )
        raise

    return "completed"


async def run_async_main(args: argparse.Namespace) -> None:
    setup_logging()
    cal = CONFIG.control.calibration_rad
    LOGGER.info(
        "calibration_rad (本机标定零位 rad) = %s — 换臂请改 config 或 ARM_CONTROL_CAL_R0..R3",
        [round(c, 6) for c in cal],
    )

    LOGGER.info("gravity model warmup (pinocchio)")
    warmup_gravity_model_pinocchio()

    store, serial_mgr, controller = build_tra_runtime()
    serial_mgr.start()
    lines_before = count_jsonl_lines(args.out)

    try:
        await wait_for_feedback(controller, args.feedback_timeout_sec)
        controller.prime_hold_latch_from_feedback()
        boot = controller.build_motor_commands()
        if boot:
            w0, g0 = mit39_init_pulse_us()
            serial_mgr.send_mit_cmd_with_servo(boot, w0, g0)

        waypoints = load_tra_rel234(args.tra)
        LOGGER.info("loaded %d waypoints from %s", len(waypoints), args.tra)
        if args.out.exists() and args.truncate_out:
            args.out.unlink()
            lines_before = 0

        status = await run_trajectory(
            store,
            serial_mgr,
            controller,
            waypoints,
            dwell_sec=args.dwell_sec,
            sample_hz=args.sample_hz,
            out_path=args.out,
            axis1_step_deg=args.axis1_step_deg,
            axis1_forward_deg=args.axis1_forward_deg,
            axis1_final_deg=args.axis1_final_deg,
            tol_rad=args.tol_rad,
        )
        LOGGER.info("run finished: %s", status)
    finally:
        serial_mgr.stop()
        serial_mgr.join(timeout=2.0)
        total = count_jsonl_lines(args.out)
        added = total - lines_before
        LOGGER.info(
            "输出文件 %s：本次新增 %d 行，文件共 %d 行（中断后已 flush 的数据均保留）",
            args.out,
            added,
            total,
        )


def parse_args() -> argparse.Namespace:
    root = _repo_root()
    parser = argparse.ArgumentParser(
        description="沿 tra.txt 分割点开环采集 MIT fb_p / 轴 1 力矩",
    )
    parser.add_argument(
        "--tra",
        type=Path,
        default=root / "docs" / "tra.txt",
        help="轨迹 txt（每点 [rel] p 为 2/3/4 轴相对角 deg）",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=root / "calibration" / "tra_samples.jsonl",
        help="JSONL 输出（每点一行，追加写入，中断不丢已写行）",
    )
    parser.add_argument(
        "--dwell-sec",
        type=float,
        default=5.0,
        help="切换至分割点后开环下发并静止采样的时长 (s)",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=None,
        help="已废弃，等同 --dwell-sec",
    )
    parser.add_argument("--sample-hz", type=float, default=20.0, help="静止窗口内采样频率")
    parser.add_argument(
        "--feedback-timeout-sec",
        type=float,
        default=30.0,
        help="启动后等待首帧反馈的最长时间",
    )
    parser.add_argument(
        "--axis1-step-deg",
        type=float,
        default=10.0,
        help="每完成一圈轨迹时 J1 相对零位步进幅度 (deg，方向由 --axis1-forward-deg 符号决定)",
    )
    parser.add_argument(
        "--axis1-forward-deg",
        type=float,
        default=-90.0,
        help="J1 相对零位累计目标 (deg，负=反向采集至 -90°)，达到并完成该圈后进入结束段",
    )
    parser.add_argument(
        "--axis1-final-deg",
        type=float,
        default=90.0,
        help="结束段：回零后再限速转到的 J1 相对角 (deg)，默认 +90°（与反向采集对称）",
    )
    parser.add_argument(
        "--tol-rad",
        type=float,
        default=None,
        help="结束段 J1 到位容差 (rad)，默认 config startup_safe_gate_tol",
    )
    parser.add_argument(
        "--truncate-out",
        action="store_true",
        help="运行前清空已有输出文件",
    )
    args = parser.parse_args()
    if args.settle_sec is not None:
        args.dwell_sec = float(args.settle_sec)
    if args.tol_rad is None:
        args.tol_rad = float(CONFIG.control.startup_safe_gate_tol_rad)
    return args


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run_async_main(args))
    except KeyboardInterrupt:
        if args.out.is_file() and count_jsonl_lines(args.out) > 0:
            append_jsonl(
                args.out,
                {
                    "record_type": "session_complete",
                    "timestamp_utc": _now_utc_iso(),
                    "status": "interrupted",
                },
            )
        LOGGER.info("用户中断 (Ctrl+C)；JSONL 中已 flush 的采样行已保留，见上方文件行数统计")


if __name__ == "__main__":
    main()
