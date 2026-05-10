"""遥操作通信层（与上位机/外设的数据收发与协议封装）。"""

from .slave_arm_ws import SlaveArmWsClient

__all__ = [
    "SlaveArmWsClient",
]

