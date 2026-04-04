"""与 PC 端 rpi_bridge / cartesian_ik_verify 一致的 UDP 关节包（树莓派解析）。"""

from __future__ import annotations

import struct

# seq(I) ts(d) p_rel_deg(5d) omega_rad_s(5d)
_FMT = "=Id" + "d" * 10
PACKET_SIZE = struct.calcsize(_FMT)


def unpack_packet(data: bytes) -> dict:
    f = struct.unpack(_FMT, data)
    return {
        "seq": f[0],
        "ts": f[1],
        "p_rel_deg": list(f[2:7]),
        "omega_rad_s": list(f[7:12]),
    }
