#!/usr/bin/env python3
"""每次运行发一帧 MIT v3（42 字节），将 ``conveyor_run`` 相对「上次本脚本下发值」反转一次。

``docs/pi2stm.md``：``conveyor_run`` 为 ``0``（PB6 低 / 停）或 ``1``（PB6 高 / 转）。

无法在单行脚本里读取 MCU 当前真实引脚电平（除非再接上行解析），因此用本地状态文件记录
上次**通过本脚本写出的** ``conveyor_run``；首次运行视为上次为 ``0``，本次下发 ``1``。

**使用前请停止 ``runtime.py`` / ``start.sh``**，独占串口。

示例::

    python3 scripts/toggle_conveyor_mit42.py --port /dev/ttyAMA2
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

try:
    import serial  # type: ignore[import-untyped]
except ImportError as exc:  # pragma: no cover
    raise SystemExit("需要 pyserial：pip install pyserial / uv sync") from exc

_REPO_ROOT = Path(__file__).resolve().parent.parent
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from infra.serial.codec import SERVO_CENTER_US, encode_mit_cmd_42

HOLD_MOTOR = {"p": 0.0, "v": 0.0, "kp": 0.0, "kd": 0.0, "t": 0.0}

DEFAULT_STATE_PATH = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")) / "icecream_conveyor_last_run"


def _read_last(path: Path) -> int:
    try:
        raw = path.read_text(encoding="ascii").strip()
        return 1 if raw == "1" else 0
    except OSError:
        return 0


def _write_last(path: Path, value: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("1" if value else "0", encoding="ascii")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="MIT v3：toggle conveyor_run（42B 下行）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--port", default=None, help="串口（默认 ARM_CONTROL_SERIAL_PORT 或 /dev/ttyAMA2）")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument(
        "--state-file",
        type=Path,
        default=None,
        help=f"记录上次下发 conveyor_run 的文件（默认 {DEFAULT_STATE_PATH}）",
    )
    p.add_argument(
        "--force",
        type=int,
        choices=(0, 1),
        default=None,
        help="不反转：强制下发 0 或 1，并写入状态文件",
    )
    p.add_argument(
        "--reset-state",
        action="store_true",
        help="删除状态文件后退出（下次运行按「首次」逻辑：先发 1）",
    )
    p.add_argument("--wrist-us", type=int, default=None, help=f"腕舵机 us（默认 {SERVO_CENTER_US}）")
    p.add_argument("--gripper-us", type=int, default=None, help=f"夹爪 us（默认 {SERVO_CENTER_US}）")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    state_path = args.state_file if args.state_file is not None else DEFAULT_STATE_PATH

    if args.reset_state:
        try:
            state_path.unlink()
            print(f"已删除状态文件: {state_path}")
        except OSError as exc:
            print(f"删除状态文件失败（可能不存在）: {exc}")
        return

    port = args.port or os.environ.get("ARM_CONTROL_SERIAL_PORT", "/dev/ttyAMA2")
    wrist = int(args.wrist_us if args.wrist_us is not None else SERVO_CENTER_US)
    gripper = int(args.gripper_us if args.gripper_us is not None else SERVO_CENTER_US)
    motors = [dict(HOLD_MOTOR) for _ in range(4)]

    last = _read_last(state_path)
    if args.force is not None:
        new_run = args.force
        note = "（--force）"
    else:
        new_run = 1 - last
        note = "（相对上次脚本下发值反转）"

    frame = encode_mit_cmd_42(
        motors,
        wrist,
        gripper,
        stepper_deg=0,
        conveyor_run=new_run,
    )
    hx = frame.hex().upper()
    hx_spaced = " ".join(hx[j : j + 2] for j in range(0, len(hx), 2))

    level = "低 (停)" if new_run == 0 else "高 (转)"
    print(f"串口 {port} @ {args.baud}")
    print(f"本次下发 conveyor_run={new_run} → PB6 期望为 {level} {note}")
    print(f"上次记录（仅脚本视角）: conveyor_run={last}  @ {state_path}")
    print(f"42B HEX ({len(frame)} 字节):\n  {hx_spaced}")

    with serial.Serial(
        port=port,
        baudrate=args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.2,
    ) as ser:
        ser.write(frame)
        ser.flush()

    _write_last(state_path, new_run)
    print("已发送并更新状态文件。")


if __name__ == "__main__":
    main()
