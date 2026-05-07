"""UDP 下行帧调试日志（与协议 V2.1 六维位置/速度布局一致）。"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

LogFn = Callable[[str], None]


def log_udp_frame_preview(frame: Any, log: LogFn, *, tag: str = "[bridge][UDP]") -> None:
    """打印即将下发的一帧相对角与角速度（6 维布局）。

    Args:
        frame: ``JointFrame`` 或具备 arm_rel_deg、arm_omega_rad_s 等属性的对象。
        log: 日志回调（通常为 ``print(..., flush=True)`` 的包装）。
        tag: 日志前缀。
    """
    p6 = np.zeros(6, dtype=float)
    w6 = np.zeros(6, dtype=float)
    p6[:4] = frame.arm_rel_deg[:4]
    w6[:4] = frame.arm_omega_rad_s[:4]
    p6[4] = float(getattr(frame, "wrist_rel_deg", 0.0))
    w6[4] = float(getattr(frame, "wrist_omega_rad_s", 0.0))
    p6[5] = float(getattr(frame, "grip_state", 0.0))
    w6[5] = 0.0
    log(
        f"{tag} 即将发送一帧: "
        f"p_rel_deg={np.array2string(p6, precision=3)} "
        f"omega_rad_s={np.array2string(w6, precision=3)}"
    )
