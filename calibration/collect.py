#!/usr/bin/env python3
"""Gravity dataset collection utility for icecreamPi.

Workflow:
1. Read waypoint pairs from calibration/root.txt (relative degrees).
2. Interpolate points between every adjacent pair.
3. Keep sending UDP command packets so controller follows each point.
4. Subscribe WebSocket state stream and sample MIT feedback p/t.
5. Save per-point averages to calibration/gravity.json.
"""

from __future__ import annotations

import argparse
import ast
import asyncio
import json
import socket
import sys
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import websockets

UDP_PACKET_FMT = "=Id" + "d" * 10
UDP_PACKET_SIZE = struct.calcsize(UDP_PACKET_FMT)


def _now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _mean4(samples: list[list[float]]) -> list[float]:
    if not samples:
        raise ValueError("empty samples")
    n = float(len(samples))
    return [sum(x[i] for x in samples) / n for i in range(4)]


def load_root_points(path: Path) -> list[list[float]]:
    points: list[list[float]] = []
    for idx, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        try:
            obj = ast.literal_eval(line)
        except (ValueError, SyntaxError) as exc:
            raise ValueError(f"root.txt 第 {idx} 行无法解析: {line!r}") from exc
        if not isinstance(obj, (list, tuple)) or len(obj) < 4:
            raise ValueError(f"root.txt 第 {idx} 行必须至少包含 4 个数值")
        points.append([float(obj[0]), float(obj[1]), float(obj[2]), float(obj[3])])
    if len(points) < 2:
        raise ValueError("root.txt 至少需要 2 个点位")
    return points


@dataclass
class InterpPoint:
    seg_idx: int
    idx_in_seg: int
    target_rel_deg: list[float]


def build_interpolated_path(points: list[list[float]], points_per_segment: int) -> list[InterpPoint]:
    if not 10 <= points_per_segment <= 20:
        raise ValueError("--points-per-segment 必须在 [10, 20]")

    out: list[InterpPoint] = []
    for seg_idx in range(len(points) - 1):
        a = points[seg_idx]
        b = points[seg_idx + 1]
        for i in range(points_per_segment):
            if seg_idx > 0 and i == 0:
                # 邻接段首点与上一段末点重合，跳过避免重复采样。
                continue
            alpha = 0.0 if points_per_segment == 1 else float(i) / float(points_per_segment - 1)
            target = [a[k] + (b[k] - a[k]) * alpha for k in range(4)]
            out.append(InterpPoint(seg_idx=seg_idx, idx_in_seg=i, target_rel_deg=target))
    return out


class UdpCommandSender(threading.Thread):
    def __init__(self, host: str, port: int, hz: float) -> None:
        super().__init__(daemon=True, name="CollectUdpSender")
        self._host = host
        self._port = port
        self._period = 1.0 / max(1.0, hz)
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._target: list[float] = [0.0, 0.0, 0.0, 0.0]
        self._seq = 0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def set_target(self, target_rel_deg: list[float]) -> None:
        with self._lock:
            self._target = [float(x) for x in target_rel_deg[:4]]

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        while not self._stop_event.is_set():
            with self._lock:
                p4 = list(self._target)
            p_rel_deg = p4 + [0.0]
            omega = [0.0] * 5
            payload = struct.pack(
                UDP_PACKET_FMT,
                self._seq & 0xFFFFFFFF,
                time.time(),
                *p_rel_deg,
                *omega,
            )
            if len(payload) != UDP_PACKET_SIZE:
                raise RuntimeError("UDP packet size mismatch")
            self._sock.sendto(payload, (self._host, self._port))
            self._seq += 1
            time.sleep(self._period)
        self._sock.close()


class WsFeedbackBridge(threading.Thread):
    """Subscribe state broadcast and keep latest fb_p/fb_t cache."""

    def __init__(self, ws_url: str) -> None:
        super().__init__(daemon=True, name="CollectWsFeedback")
        self._ws_url = ws_url
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._connected = threading.Event()
        self._latest_monotonic: float = 0.0
        self._latest_fb_p: list[float] | None = None
        self._latest_fb_t: list[float] | None = None
        self._minimal_ws_warned = False

    def stop(self) -> None:
        self._stop_event.set()

    def wait_connected(self, timeout: float) -> bool:
        return self._connected.wait(timeout=timeout)

    def snapshot(self) -> tuple[float, list[float] | None, list[float] | None]:
        with self._lock:
            p = list(self._latest_fb_p) if self._latest_fb_p is not None else None
            t = list(self._latest_fb_t) if self._latest_fb_t is not None else None
            return self._latest_monotonic, p, t

    def run(self) -> None:
        asyncio.run(self._run_async())

    async def _run_async(self) -> None:
        while not self._stop_event.is_set():
            try:
                async with websockets.connect(self._ws_url, ping_interval=20, ping_timeout=20) as ws:
                    self._connected.set()
                    async for raw in ws:
                        if self._stop_event.is_set():
                            return
                        self._consume_state(raw)
            except Exception:
                self._connected.clear()
                await asyncio.sleep(1.0)

    def _consume_state(self, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if msg.get("type") != "state":
            return
        data = msg.get("data")
        if not isinstance(data, dict):
            return
        if "link5_rpy_rad" in data and "feedback" not in data:
            if not self._minimal_ws_warned:
                self._minimal_ws_warned = True
                print(
                    "[collect] WebSocket 为精简 link5 位姿负载，缺少 feedback.mit_arm_rad / motors；"
                    "重力采集无法采样。请恢复完整 state 或使用其它反馈通道。",
                    file=sys.stderr,
                    flush=True,
                )
            return
        fb = data.get("feedback")
        if not isinstance(fb, dict):
            return

        mit_arm = fb.get("mit_arm_rad")
        motors = fb.get("motors")
        if not isinstance(mit_arm, list) or len(mit_arm) < 4:
            return
        if not isinstance(motors, list):
            return

        t_by_id: dict[int, float] = {}
        for m in motors:
            if not isinstance(m, dict):
                continue
            try:
                motor_id = int(m.get("id"))
                torque = float(m.get("t"))
            except (TypeError, ValueError):
                continue
            t_by_id[motor_id] = torque
        if any(i not in t_by_id for i in range(4)):
            return

        fb_p = [float(mit_arm[0]), float(mit_arm[1]), float(mit_arm[2]), float(mit_arm[3])]
        fb_t = [t_by_id[0], t_by_id[1], t_by_id[2], t_by_id[3]]
        with self._lock:
            self._latest_monotonic = time.monotonic()
            self._latest_fb_p = fb_p
            self._latest_fb_t = fb_t


def collect_for_point(
    bridge: WsFeedbackBridge,
    settle_sec: float,
    sample_sec: float,
    sample_hz: float,
) -> tuple[list[float], list[float]]:
    settle_deadline = time.monotonic() + settle_sec
    while time.monotonic() < settle_deadline:
        time.sleep(0.05)

    p_samples: list[list[float]] = []
    t_samples: list[list[float]] = []
    end_at = time.monotonic() + sample_sec
    poll_period = 1.0 / max(5.0, sample_hz)
    last_stamp = -1.0
    while time.monotonic() < end_at:
        stamp, fb_p, fb_t = bridge.snapshot()
        if stamp > last_stamp and fb_p is not None and fb_t is not None:
            last_stamp = stamp
            p_samples.append(fb_p)
            t_samples.append(fb_t)
        time.sleep(poll_period)

    if not p_samples or not t_samples:
        raise RuntimeError("采样窗口内未收到有效反馈数据")
    return _mean4(p_samples), _mean4(t_samples)


def parse_args() -> argparse.Namespace:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="重力补偿数据采集（UDP 控制 + WS 回传采样）")
    parser.add_argument("--root", type=Path, default=base / "root.txt", help="root.txt 路径")
    parser.add_argument("--out", type=Path, default=base / "gravity.json", help="输出 JSON 路径")
    parser.add_argument("--points-per-segment", type=int, default=15, help="每段插值点数，10~20")
    parser.add_argument("--settle-sec", type=float, default=5.0, help="到位后稳定时长（秒）")
    parser.add_argument("--sample-sec", type=float, default=5.0, help="每点采样时长（秒）")
    parser.add_argument("--udp-host", type=str, default="127.0.0.1", help="icecreamPi UDP 主机")
    parser.add_argument("--udp-port", type=int, default=9870, help="icecreamPi UDP 端口")
    parser.add_argument("--udp-hz", type=float, default=25.0, help="UDP 持续发送频率")
    parser.add_argument("--ws-url", type=str, default="ws://127.0.0.1:8765", help="状态广播 WS")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="采样窗口轮询频率")
    parser.add_argument("--ws-connect-timeout", type=float, default=10.0, help="WS 连接等待超时")
    return parser.parse_args()


def write_records_atomic(out_path: Path, records: list[dict[str, Any]]) -> None:
    """Write JSON via temp file + replace to avoid partial/truncated output."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = out_path.with_suffix(out_path.suffix + ".tmp")
    payload = json.dumps(records, ensure_ascii=False, indent=2)
    tmp_path.write_text(payload, encoding="utf-8")
    tmp_path.replace(out_path)


def main() -> None:
    args = parse_args()
    points = load_root_points(args.root)
    path = build_interpolated_path(points, args.points_per_segment)
    if not path:
        raise RuntimeError("未生成采样点，请检查 root.txt 与 points-per-segment")

    bridge = WsFeedbackBridge(args.ws_url)
    sender = UdpCommandSender(args.udp_host, args.udp_port, args.udp_hz)

    print(f"[collect] root points={len(points)} interpolated samples={len(path)}")
    print(f"[collect] udp={args.udp_host}:{args.udp_port} ws={args.ws_url}")

    bridge.start()
    if not bridge.wait_connected(timeout=args.ws_connect_timeout):
        raise RuntimeError(f"WS 连接超时: {args.ws_url}")

    sender.set_target(path[0].target_rel_deg)
    sender.start()

    records: list[dict[str, Any]] = []
    try:
        for idx, pt in enumerate(path, start=1):
            sender.set_target(pt.target_rel_deg)
            print(
                f"[collect] point {idx}/{len(path)} seg={pt.seg_idx} idx={pt.idx_in_seg} "
                f"target_rel_deg={[round(x, 4) for x in pt.target_rel_deg]}"
            )
            avg_p, avg_t = collect_for_point(
                bridge=bridge,
                settle_sec=max(0.0, args.settle_sec),
                sample_sec=max(0.1, args.sample_sec),
                sample_hz=args.sample_hz,
            )
            rec = {
                "target_rel_deg": [round(x, 6) for x in pt.target_rel_deg],
                "avg_fb_p_rad": [round(x, 6) for x in avg_p],
                "avg_fb_t_nm": [round(x, 6) for x in avg_t],
                "timestamp": _now_utc_iso(),
            }
            records.append(rec)
            # Incremental persistence: keep completed points even if interrupted later.
            write_records_atomic(args.out, records)
            print(
                f"[collect] saved {idx}/{len(path)} "
                f"avg_fb_p_rad={[round(x, 4) for x in avg_p]} "
                f"avg_fb_t_nm={[round(x, 4) for x in avg_t]}"
            )
    finally:
        sender.stop()
        bridge.stop()
        sender.join(timeout=2.0)
        bridge.join(timeout=2.0)

    write_records_atomic(args.out, records)
    print(f"[collect] done, wrote {len(records)} records to {args.out}")


if __name__ == "__main__":
    main()

