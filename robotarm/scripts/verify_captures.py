#!/usr/bin/env python3
"""
阶段一 Step 1.1 — 验证 captures.jsonl 中 joints.x.torque 含义
与 gravity_compensation_plan.md「Step 1.1：验证数据格式」一致。

每行 JSON 仅含 motors.*.angle（deg×100）与 joints.*.torque（Nm）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
CAPTURES = ROOT / "captures.jsonl"


def _row(s: dict) -> dict:
    return s if "avg" not in s else s["avg"]


def main() -> int:
    ap = argparse.ArgumentParser(description="Step 1.1 数据格式验证")
    ap.add_argument(
        "path",
        nargs="?",
        type=Path,
        default=CAPTURES,
        help=f"jsonl 路径（默认: {CAPTURES}）",
    )
    args = ap.parse_args()
    path: Path = args.path
    if not path.is_file():
        print(f"找不到文件: {path}", file=sys.stderr)
        return 1

    with open(path, encoding="utf-8") as f:
        samples = [json.loads(line) for line in f if line.strip()]

    j1 = np.array([float(_row(s)["joints.1.torque"]) for s in samples], dtype=np.float64)
    a0 = np.array([float(_row(s)["motors.0.angle"]) for s in samples], dtype=np.float64)

    corr = float(np.corrcoef(a0, j1)[0, 1])
    j2 = [float(_row(s)["joints.2.torque"]) for s in samples]

    print(f"样本数: {len(samples)}  文件: {path}")
    print(f"joints.1.torque 与 motors.0.angle 的 Pearson 相关系数: {corr:.4f}")
    print(f"  （竖直基座关节：若为力矩，|ρ| 通常较小，不应随 a0 强相关）")
    print(f"joints.1.torque 均值: {float(np.mean(j1)):.4f} Nm, 标准差: {float(np.std(j1)):.4f}")
    print(f"joints.2.torque 范围: [{min(j2):.2f}, {max(j2):.2f}] Nm")

    std_ok = float(np.std(j1)) < 0.1
    mean_small = abs(float(np.mean(j1))) < 0.2
    print("\n文档启发式判定（Step 1.1 通过标准）:")
    print(f"  std(joints.1) < 0.1  →  {'✅ 满足' if std_ok else '❌ 不满足'}")
    print(f"  |mean(joints.1)| < 0.2（文档辅助）→  {'✅ 满足' if mean_small else '⚠ 超出'}")
    if std_ok:
        print("\n✅ 可认为 joints.x.torque 为力矩反馈 (Nm)，继续 Step 1.2 拟合。")
    else:
        print("\n⚠️  建议核对采集脚本或参考文档附录 A 重新采集。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
