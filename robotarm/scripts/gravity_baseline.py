#!/usr/bin/env python3
"""
静态重力前馈 baseline（与辨识默认输出一致：偏置 + cos(q+φ)，参数来自 gravity_params.h）

输入协议（与 gravity_compensation_plan.md / motor 侧一致）：
  - 四路关节角 **相对 HOME**（rad）：q0,q1,q2,q3 对应 motor 0~3
  - q0 为竖转基座，模型中 τ_g0 ≈ 0；重力项用 q1,q2,q3（肩/肘/腕）

输出：四路力矩 τ0~τ3（Nm），与 MIT tau_ff 注入顺序一致；可选按 GC_MAX_TFF 限幅。

用法：
  python3 gravity_baseline.py rad 0 0 0 0
  python3 gravity_baseline.py rad 0 0 0 0 --no-clamp
  python3 gravity_baseline.py deg100 -298 8059 13327 2700 --json
  python3 gravity_baseline.py --self-test

其它模块可: from gravity_baseline import GravityBaselineParams, compute_tau_nm, infer_from_motor_deg100
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HEADER = ROOT / "User" / "GravityComp" / "gravity_params.h"

# 与 motor_config.h / gravity_identification 一致（deg×100）
HOME_DEG100_MAP = {0: -298, 1: 8059, 2: 13327, 3: 2700}


@dataclass(frozen=True)
class GravityBaselineParams:
    """由 gravity_params.h 解析；与固件 Gravity_Compute 使用同一组数。"""

    bias_j1: float
    bias_j2: float
    bias_j3: float
    a1: float
    a2: float
    a3: float
    phi1: float
    phi2: float
    phi3: float
    b1: float
    b2: float
    psi1: float
    psi2: float
    c1: float
    zeta1: float
    sign_j0: int
    sign_j1: int
    sign_j2: int
    sign_j3: int
    max_tff: float
    source_path: str = ""

    @classmethod
    def from_header(cls, path: Path) -> GravityBaselineParams:
        text = path.read_text(encoding="utf-8")
        d: dict[str, float | int] = {}
        for line in text.splitlines():
            line = line.strip()
            m = re.match(r"#define\s+(GC_\w+)\s+([-+0-9.eE]+)f?", line)
            if not m:
                continue
            name, val_s = m.group(1), m.group(2)
            if name.startswith("GC_SIGN"):
                d[name] = int(float(val_s))
            else:
                d[name] = float(val_s)

        def req(key: str) -> float:
            if key not in d:
                raise KeyError(f"gravity_params.h 缺少 {key}: {path}")
            return float(d[key])

        def req_i(key: str) -> int:
            return int(d[key])

        return cls(
            bias_j1=req("GC_BIAS_J1"),
            bias_j2=req("GC_BIAS_J2"),
            bias_j3=req("GC_BIAS_J3"),
            a1=req("GC_A1"),
            a2=req("GC_A2"),
            a3=req("GC_A3"),
            phi1=req("GC_PHI1"),
            phi2=req("GC_PHI2"),
            phi3=req("GC_PHI3"),
            b1=req("GC_B1"),
            b2=req("GC_B2"),
            psi1=req("GC_PSI1"),
            psi2=req("GC_PSI2"),
            c1=req("GC_C1"),
            zeta1=req("GC_ZETA1"),
            sign_j0=req_i("GC_SIGN_J0"),
            sign_j1=req_i("GC_SIGN_J1"),
            sign_j2=req_i("GC_SIGN_J2"),
            sign_j3=req_i("GC_SIGN_J3"),
            max_tff=req("GC_MAX_TFF"),
            source_path=str(path.resolve()),
        )


def motor_deg100_to_q_rad(motor_deg100: Sequence[float]) -> tuple[float, float, float, float]:
    """电机绝对位置 (deg×100) → 相对 HOME 的四轴弧度 [q0,q1,q2,q3]。"""
    if len(motor_deg100) != 4:
        raise ValueError("需要 4 个电机位置 (deg×100)")
    out: list[float] = []
    for i in range(4):
        rel = float(motor_deg100[i]) - HOME_DEG100_MAP[i]
        out.append(rel / 100.0 * (math.pi / 180.0))
    return (out[0], out[1], out[2], out[3])


def compute_tau_nm(
    q_rad: Sequence[float],
    p: GravityBaselineParams,
    *,
    clamp: bool = True,
) -> tuple[float, float, float, float]:
    """
    输入四轴相对 HOME 弧度 [q0,q1,q2,q3]；输出四路重力前馈力矩 [τ0,τ1,τ2,τ3]（Nm）。

    q1,q2,q3 为肩/肘/腕；q0 不参与重力式，τ0 恒为 0（再乘 sign_j0）。
    """
    if len(q_rad) != 4:
        raise ValueError("需要 4 个关节角 (rad)")
    _q0, q1, q2, q3 = float(q_rad[0]), float(q_rad[1]), float(q_rad[2]), float(q_rad[3])
    s1 = q1 + q2
    s2 = q1 + q2 + q3

    tau1 = (
        p.bias_j1
        + p.a1 * math.cos(q1 + p.phi1)
        + p.a2 * math.cos(s1 + p.phi2)
        + p.a3 * math.cos(s2 + p.phi3)
    )
    tau2 = p.bias_j2 + p.b1 * math.cos(s1 + p.psi1) + p.b2 * math.cos(s2 + p.psi2)
    tau3 = p.bias_j3 + p.c1 * math.cos(s2 + p.zeta1)

    t0 = 0.0
    t1 = p.sign_j1 * tau1
    t2 = p.sign_j2 * tau2
    t3 = p.sign_j3 * tau3
    t0 = p.sign_j0 * t0

    if clamp:
        m = p.max_tff
        t1 = max(-m, min(m, t1))
        t2 = max(-m, min(m, t2))
        t3 = max(-m, min(m, t3))

    return (t0, t1, t2, t3)


def infer_from_motor_deg100(
    m: Sequence[float],
    p: GravityBaselineParams,
    *,
    clamp: bool = True,
) -> tuple[float, float, float, float]:
    """便捷：直接传入 4 路电机绝对位置 (deg×100)。"""
    return compute_tau_nm(motor_deg100_to_q_rad(m), p, clamp=clamp)


# ---------------------------------------------------------------------------
# 测试 / 对外接口
# ---------------------------------------------------------------------------


def self_test(p: GravityBaselineParams) -> None:
    """HOME 位形下应接近仅偏置 + cos(φ) 的组合；用于快速 sanity check。"""
    q_home = motor_deg100_to_q_rad(
        [HOME_DEG100_MAP[0], HOME_DEG100_MAP[1], HOME_DEG100_MAP[2], HOME_DEG100_MAP[3]]
    )
    tau = compute_tau_nm(q_home, p, clamp=False)
    print("self_test: HOME (q=0) 四轴 rad:", [round(x, 6) for x in q_home])
    print("self_test: τ (Nm, 不限幅):", [round(x, 6) for x in tau])


def main() -> int:
    ap = argparse.ArgumentParser(description="重力 baseline 前馈：四轴角 → 四路力矩 (Nm)")
    ap.add_argument(
        "--header",
        type=Path,
        default=DEFAULT_HEADER,
        help=f"gravity_params.h 路径（默认 {DEFAULT_HEADER}）",
    )
    ap.add_argument("--self-test", action="store_true", help="加载参数并打印 HOME 位形力矩")

    sub = ap.add_subparsers(dest="mode", help="输入方式")

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--json", action="store_true", help="输出一行 JSON（便于其它程序解析）")
    common.add_argument("--no-clamp", action="store_true", help="不使用 GC_MAX_TFF 限幅")

    pr = sub.add_parser("rad", parents=[common], help="四轴相对 HOME 弧度 q0 q1 q2 q3")
    pr.add_argument("q0", type=float)
    pr.add_argument("q1", type=float)
    pr.add_argument("q2", type=float)
    pr.add_argument("q3", type=float)

    pd = sub.add_parser("deg100", parents=[common], help="四电机绝对位置 (deg×100)，与 captures / 协议一致")
    pd.add_argument("m0", type=float)
    pd.add_argument("m1", type=float)
    pd.add_argument("m2", type=float)
    pd.add_argument("m3", type=float)

    args = ap.parse_args()

    if not args.header.is_file():
        print(f"错误: 找不到参数文件 {args.header}，请先运行 scripts/gravity_identification.py", file=sys.stderr)
        return 1

    p = GravityBaselineParams.from_header(args.header)

    if args.self_test:
        print(f"参数来源: {p.source_path}\n")
        self_test(p)
        if args.mode is None:
            return 0

    use_json = getattr(args, "json", False)
    clamp = not getattr(args, "no_clamp", False)

    if args.mode == "rad":
        tau = compute_tau_nm((args.q0, args.q1, args.q2, args.q3), p, clamp=clamp)
        q = (args.q0, args.q1, args.q2, args.q3)
    elif args.mode == "deg100":
        tau = infer_from_motor_deg100((args.m0, args.m1, args.m2, args.m3), p, clamp=clamp)
        q = motor_deg100_to_q_rad((args.m0, args.m1, args.m2, args.m3))
    else:
        if not args.self_test:
            ap.print_help()
            print("\n示例: python3 gravity_baseline.py rad 0 0 0 0", file=sys.stderr)
            print("      python3 gravity_baseline.py deg100 -298 8059 13327 2700", file=sys.stderr)
            print("      python3 gravity_baseline.py --self-test", file=sys.stderr)
        return 0 if args.self_test else 1

    out = {
        "q_rad_relative_home": list(q),
        "tau_nm_motor_order": list(tau),
        "labels": ["motor0_base", "motor1_shoulder", "motor2_elbow", "motor3_wrist"],
        "params_header": p.source_path,
    }
    if use_json:
        print(json.dumps(out, ensure_ascii=False))
    else:
        print("q (rad, rel HOME):", [round(x, 6) for x in q])
        print("τ (Nm):            ", [round(x, 6) for x in tau])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
