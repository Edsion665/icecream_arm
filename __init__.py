"""
arm_control_bridge: 4 轴主臂 + 爪通道控制桥。

子包：``io``（网络与机载通信）、``sim``（Isaac 仿真）、``runtime``（到位与调试）、
``control`` / ``kinematics``（算法层）。
"""

from .config import CONTROL_HZ, DEFAULT_TCP_PORT, DEFAULT_UDP_PORT, load_calibration_deg

__all__ = ["CONTROL_HZ", "DEFAULT_TCP_PORT", "DEFAULT_UDP_PORT", "load_calibration_deg"]

