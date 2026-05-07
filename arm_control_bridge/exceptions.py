"""arm_control_bridge 专用异常：便于上层捕获与诊断。"""

from __future__ import annotations


class BridgeError(Exception):
    """控制桥接层错误的基类。"""


class UDPTransportError(BridgeError):
    """UDP 发送失败（网络不可达、权限、已关闭的 socket 等）。"""


class SymlinkSetupError(BridgeError):
    """仿真 USD payload 所需符号链接创建失败。"""


class PiFeedbackDecodeError(BridgeError):
    """树莓派 WebSocket 消息解析或字段不符合预期。"""


class ArticulationCommandError(BridgeError):
    """Isaac 关节指令应用失败（set_joint_positions / apply_action）。"""
