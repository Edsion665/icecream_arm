"""无重依赖的协议常量（避免 io/control 循环导入）。"""

from __future__ import annotations

# 仅非运动类立即 ack；joints/pose/claw 阻塞至 UDP 判到位（新命令可抢占旧等待）
IMMEDIATE_ACK_KINDS = frozenset({"stepper", "conveyor", "speed"})
