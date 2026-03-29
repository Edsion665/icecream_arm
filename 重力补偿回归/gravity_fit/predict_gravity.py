"""加载 coefficients.json，根据 q0..q3（弧度）预测四轴前馈力矩。

输入: 四个浮点数为相对 HOME 的关节角，单位 **弧度 (rad)**，与固件
      Motor_States[i].pos - WORLD_HOME_ABS[i] 一致。
      jsonl 采集数据里是「度×100」，需先换算再传入本脚本。

用法:
  python predict_gravity.py q0_rad q1_rad q2_rad q3_rad
  python predict_gravity.py --coeff out/coefficients_refit_drop8_m2_cross23.json 0 0.1 -0.5 0
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path


def _phi_from_q(q: list[float], nfeat: int) -> list[float]:
    s = [math.sin(q[j]) for j in range(4)]
    c = [math.cos(q[j]) for j in range(4)]
    row = [1.0] + [x for j in range(4) for x in (s[j], c[j])]
    if nfeat == 9:
        return row
    s2 = [math.sin(2.0 * q[j]) for j in range(4)]
    c2 = [math.cos(2.0 * q[j]) for j in range(4)]
    row.extend([x for j in range(4) for x in (s2[j], c2[j])])
    if nfeat == 17:
        return row
    if nfeat == 23:
        pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
        row.extend([s[i] * s[j] for i, j in pairs])
        return row
    raise ValueError(f"unsupported nfeat={nfeat} (expect 9, 17, or 23)")


def predict_tau(q: list[float], coeff_path: Path) -> list[float]:
    data = json.loads(coeff_path.read_text(encoding="utf-8"))
    beta = data["beta"]
    nfeat = len(beta[0])
    phi = _phi_from_q(q, nfeat)
    return [sum(beta[k][f] * phi[f] for f in range(nfeat)) for k in range(4)]


def main() -> None:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description="Predict tau_ff from q (rad)")
    ap.add_argument("--coeff", type=Path, default=None, help="coefficients json")
    ap.add_argument("q", nargs=4, type=float, help="q0 q1 q2 q3 in radians")
    args = ap.parse_args()
    coeff = args.coeff or (here / "out" / "coefficients.json")
    tau = predict_tau(list(args.q), coeff)
    print("tau_ff (N*m):", [round(t, 4) for t in tau])


if __name__ == "__main__":
    main()
