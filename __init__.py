"""
arm_control_bridge: 新架构实验目录（4轴主臂 + claw独立通道）。
"""

from .config import CONTROL_HZ, DEFAULT_TCP_PORT, DEFAULT_UDP_PORT, load_calibration_deg

__all__ = ["CONTROL_HZ", "DEFAULT_TCP_PORT", "DEFAULT_UDP_PORT", "load_calibration_deg"]

