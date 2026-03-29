#!/usr/bin/env python3
"""
用验证集 captures.jsonl 中的编码器角度（deg×100）逐条调用 gravity_torque（Pinocchio），
批量计算 τ_g，并保存为与 captures.jsonl 相同风格的 **JSONL**（每行一条扁平 JSON）。

每行字段：
  motors.0.angle … motors.3.angle — 与输入一致（deg×100）
  joints.1.torque … joints.4.torque — 本行为 Pinocchio 预测的 τ_g（Nm），对应 URDF joint1…joint4

约定：motors.k → 臂关节 joint(k+1)；无 motors.4.angle 时 joint5 用 --joint5-deg100（默认 0），仅参与内部计算，
      不写入输出行（与 captures 键集合一致）。

输出：默认 robotarm/captures_gravity_predictions.jsonl
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gravity_comp.gravity_torque import (  # noqa: E402
    get_gravity_torques_arm_deg_x100,
)

# motor → URDF joint 映射（angle 已是相对 URDF 零点的差值，HOME=0）：
#   motor1 → URDF joint2 (shoulder)，符号 -1（轴方向与实际电机相反）
#   motor2 → URDF joint3 (elbow)，符号 +1
#   motor3 → URDF joint4 (wrist)，符号 +1


def _row(d: dict) -> dict:
    return d if "avg" not in d else d["avg"]


def joint_deg_x100_from_capture(avg: dict) -> list[float]:
    """从一行 captures 构造传给 Pinocchio 的 5 个关节角（deg×100）。

    映射：[j1=0, j2=-m1, j3=+m2, j4=+m3, j5=0]
    """
    m1 = float(avg["motors.1.angle"])
    m2 = float(avg["motors.2.angle"])
    m3 = float(avg["motors.3.angle"])
    return [
        0.0,   # joint1 (base yaw)
        -m1,   # joint2 = -motor1（轴方向相反）
        +m2,   # joint3 = +motor2
        +m3,   # joint4 = +motor3
        0.0,   # joint5
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="对 captures.jsonl 批量计算重力项 τ_g 并保存")
    parser.add_argument(
        "--input",
        type=Path,
        default=ROOT / "captures.jsonl",
        help="验证集 JSONL（默认 robotarm/captures.jsonl）",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "captures_gravity_predictions.jsonl",
        help="输出 JSONL，与 captures.jsonl 同形（默认 captures_gravity_predictions.jsonl）",
    )
    args = parser.parse_args()

    with open(args.input, encoding="utf-8") as f:
        lines = [ln for ln in f if ln.strip()]
    n = 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as out:
        for line in lines:
            raw = json.loads(line)
            avg = _row(raw)
            jdeg = joint_deg_x100_from_capture(avg)
            tau = get_gravity_torques_arm_deg_x100(jdeg, 0.0, 0.0)
            # tau[1]=joint2→joints.2, tau[2]=joint3→joints.3, tau[3]=joint4→joints.4
            # τ2 需要取反（joint2 轴方向与实际电机相反，力矩符号也反）
            row = {
                "motors.0.angle": float(avg["motors.0.angle"]),
                "motors.1.angle": float(avg["motors.1.angle"]),
                "motors.2.angle": float(avg["motors.2.angle"]),
                "motors.3.angle": float(avg["motors.3.angle"]),
                "joints.1.torque": 0.0,
                "joints.2.torque": -float(tau[1]),
                "joints.3.torque": float(tau[2]),
                "joints.4.torque": float(tau[3]),
            }
            out.write(json.dumps(row, ensure_ascii=False) + "\n")
            n += 1
    print(f"已写入 {n} 行（JSONL，与 captures 同形）→ {args.output.resolve()}")


if __name__ == "__main__":
    main()
