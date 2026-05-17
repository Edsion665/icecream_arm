"""UDP 下行帧调试日志（与 bridge2pi v3 八维位置/速度布局一致）。"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from ..io.pi_controller import VECTOR_DIM

LogFn = Callable[[str], None]


def log_udp_frame_preview(frame: Any, log: LogFn, *, tag: str = "[bridge][UDP]") -> None:
    """打印即将下发的一帧相对角与角速度（8 维布局）。

    Args:
        frame: ``JointFrame`` 或具备 arm_rel_deg、arm_omega_rad_s 等属性的对象。
        log: 日志回调（通常为 ``print(..., flush=True)`` 的包装）。
        tag: 日志前缀。
    """
    p8 = np.zeros(VECTOR_DIM, dtype=float)
    w8 = np.zeros(VECTOR_DIM, dtype=float)
    p8[:4] = frame.arm_rel_deg[:4]
    w8[:4] = frame.arm_omega_rad_s[:4]
    p8[4] = float(getattr(frame, "wrist_rel_deg", 0.0))
    w8[4] = float(getattr(frame, "wrist_omega_rad_s", 0.0))
    p8[5] = float(getattr(frame, "grip_state", 0.0))
    w8[5] = 0.0
    p8[6] = float(getattr(frame, "stepper_deg_cmd", 0.0))
    w8[6] = 0.0
    p8[7] = float(getattr(frame, "conveyor_run_cmd", 0.0))
    w8[7] = 0.0
    log(
        f"{tag} 即将发送一帧: "
        f"p_rel_deg={np.array2string(p8, precision=3)} "
        f"omega_rad_s={np.array2string(w8, precision=3)}"
    )
