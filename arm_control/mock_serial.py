"""用于本地联调的串口模拟模块，不依赖真实硬件。"""

from __future__ import annotations

import io
from typing import Optional


class MockSerial(io.BytesIO):
    """模仿 pyserial.Serial 的最小接口，便于单元测试。"""

    def __init__(self, initial_bytes: bytes | None = None) -> None:
        super().__init__(initial_bytes or b"")
        self.is_open = True

    # pyserial 兼容方法 -----------------------------------------------------

    def read(self, size: int = 1) -> bytes:  # type: ignore[override]
        return super().read(size)

    def write(self, data: bytes) -> int:  # type: ignore[override]
        pos_before = self.tell()
        super().seek(0, io.SEEK_END)
        written = super().write(data)
        super().seek(pos_before)
        return written

    def close(self) -> None:  # type: ignore[override]
        self.is_open = False
        super().close()

