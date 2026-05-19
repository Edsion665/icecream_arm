"""无重依赖的协议常量（避免 io/control 循环导入）。"""

from __future__ import annotations

# 入队即更新目标；HTTP/TCP 立即 ack，到位由 GET /api/reached 查询
IMMEDIATE_ACK_KINDS = frozenset(
    {"pose", "pose_delta", "joints", "joints_delta", "stepper", "conveyor"}
)
