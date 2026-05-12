#!/usr/bin/env python3
"""Listen to icecreamPi WebSocket: decode ``link5_hmat`` -> euler (Rx·Ry·Rz) + translation."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import websockets

# 允许在 ``icecream/icecreamPi`` 下执行 ``python test.py``（仓库根需在 sys.path）
_ROOT = Path(__file__).resolve().parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from icecreamPi.calculator import link5_rpy_from_R_rxryrz_np


def _print_chain(steps: Any) -> None:
    if not isinstance(steps, list) or not steps:
        return
    print("  --- link0 下逐步累积 (joint 与 URDF joint1..5 对应) ---", flush=True)
    for row in steps:
        if not isinstance(row, dict):
            continue
        ji = row.get("joint")
        q = row.get("q_rad")
        xyz = row.get("xyz_m")
        rpy = row.get("rpy_rad")
        if not isinstance(xyz, list) or len(xyz) < 3:
            continue
        if not isinstance(rpy, list) or len(rpy) < 3:
            continue
        r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
        print(
            f"  joint{ji}  q={float(q):+.6f} rad | "
            f"xyz_m=({float(xyz[0]):.6f},{float(xyz[1]):.6f},{float(xyz[2]):.6f}) | "
            f"rpy_deg=({math.degrees(r):+.2f},{math.degrees(p):+.2f},{math.degrees(y):+.2f})",
            flush=True,
        )


def _R_t_from_link5_hmat(hmat: Any) -> tuple[np.ndarray, np.ndarray] | None:
    """Upper 3×3 rotation and 3×1 translation from row-major 4×4."""
    if not isinstance(hmat, list) or len(hmat) < 4:
        return None
    rows = []
    for i in range(4):
        row = hmat[i]
        if not isinstance(row, (list, tuple)) or len(row) < 4:
            return None
        rows.append([float(row[j]) for j in range(4)])
    R = np.asarray(rows, dtype=np.float64)[:3, :3]
    t = np.asarray(rows, dtype=np.float64)[:3, 3].reshape(3)
    return R, t


def _print_from_hmat(hmat: Any) -> bool:
    """Print euler + translation decoded from ``link5_hmat``. Returns True if printed."""
    rt = _R_t_from_link5_hmat(hmat)
    if rt is None:
        return False
    R, t = rt
    roll, pitch, yaw = link5_rpy_from_R_rxryrz_np(R)
    x, y, z = float(t[0]), float(t[1]), float(t[2])
    rd, pd, yd = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    print("  [from link5_hmat] translation (m, link0): " f"x={x:.6f} y={y:.6f} z={z:.6f}", flush=True)
    print(
        "  [from link5_hmat] euler R_x·R_y·R_z (roll,pitch,yaw) rad | deg: "
        f"roll={roll:.6f} ({rd:.3f}°) pitch={pitch:.6f} ({pd:.3f}°) yaw={yaw:.6f} ({yd:.3f}°)",
        flush=True,
    )
    return True


async def run(uri: str, chain_only: bool) -> None:
    async with websockets.connect(uri) as ws:
        print(f"connected: {uri}", flush=True)
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                print("skip: invalid json", flush=True)
                continue
            if msg.get("type") != "state":
                continue
            data = msg.get("data") or {}
            steps = data.get("link_chain_steps")

            if chain_only:
                if isinstance(steps, list) and steps:
                    _print_chain(steps)
                else:
                    print("  waiting link_chain_steps …", flush=True)
                continue

            hmat: Any = data.get("link5_hmat")
            if _print_from_hmat(hmat):
                _print_chain(steps)
                continue

            rpy_ws: Any = data.get("link5_rpy_rad")
            xyz_ws: Any = data.get("link5_xyz_m")
            if (
                isinstance(rpy_ws, list)
                and len(rpy_ws) >= 3
                and isinstance(xyz_ws, list)
                and len(xyz_ws) >= 3
            ):
                roll, pitch, yaw = float(rpy_ws[0]), float(rpy_ws[1]), float(rpy_ws[2])
                x, y, z = float(xyz_ws[0]), float(xyz_ws[1]), float(xyz_ws[2])
                rd, pd, yd = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
                print(
                    "  [from link5_rpy_rad / link5_xyz_m] position (m): "
                    f"x={x:.6f} y={y:.6f} z={z:.6f}",
                    flush=True,
                )
                print(
                    "  [from fields] euler R_x·R_y·R_z rad | deg: "
                    f"roll={roll:.6f} ({rd:.3f}°) pitch={pitch:.6f} ({pd:.3f}°) yaw={yaw:.6f} ({yd:.3f}°)",
                    flush=True,
                )
                _print_chain(steps)
            else:
                print(
                    "  waiting link5_hmat or link5_rpy_rad+link5_xyz_m "
                    "(need arm feedback + UDP joint5)",
                    flush=True,
                )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--uri",
        default="ws://127.0.0.1:8765",
        help="WebSocket URI (default ws://127.0.0.1:8765)",
    )
    p.add_argument(
        "--chain-only",
        action="store_true",
        help="只打印 link_chain_steps，刷屏少、便于逐节对照",
    )
    args = p.parse_args()
    try:
        asyncio.run(run(args.uri, chain_only=args.chain_only))
    except KeyboardInterrupt:
        print("\nexit", file=sys.stderr)


if __name__ == "__main__":
    main()
