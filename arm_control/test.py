#!/usr/bin/env python3
"""
对比 Pinocchio（compute_tau_ff_nm）与 MLP：输入均为「STM32 原始四轴弧度」。

- Pinocchio：delta_pin = raw − config.TAU_FF.calibration_rad（与 main.py 力矩前馈一致），
  在测试中额外显式跑一次 motor_rad_deltas_to_joint_deg_x100（URDF 对齐 / 度×100）。
- MLP：delta_mlp = raw − MLP_STM32_BIAS_RAD（你给定的原始零位/误差，弧度），
  再经 cal_gravity 转为度×100 等特征后推理。

在 icecream/arm_control 目录下执行：
  python test.py --model gravity_compensator.pkl
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_arm_dir = Path(__file__).resolve().parent
if str(_arm_dir) not in sys.path:
    sys.path.insert(0, str(_arm_dir))

from cal_gravity import (  # noqa: E402
    FeatureMode,
    joint_tau_to_motor_ff_nm,
    load_model,
    predict_tau_ml_from_delta_rad,
)
from config import TAU_FF  # noqa: E402
from gravity_feedforward import (  # noqa: E402
    compute_tau_ff_nm,
    compute_tau_joint_nm,
    motor_rad_deltas_to_joint_deg_x100,
)

# 从 STM32 原始弧度中减去该偏置后，作为 MLP 的「相对角」输入（与训练数据集一致）
MLP_STM32_BIAS_RAD = np.array(
    [-1.597047, 1.360151, 2.380980, 0.490005],
    dtype=np.float64,
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "STM32 原始弧度 → Pinocchio（TAU_FF 标定）vs MLP（固定偏置标定），"
            "电机前馈空间对比"
        )
    )
    p.add_argument(
        "--model",
        type=Path,
        default=_arm_dir / "gravity_compensator.pkl",
        help="sklearn/joblib 训练的 MLP 模型路径",
    )
    p.add_argument(
        "--features",
        choices=("motor_deg_x100", "urdf_joint_deg_x100"),
        default="motor_deg_x100",
        help="与 cal_gravity 一致：MLP 输入特征构造方式",
    )
    p.add_argument("--n", type=int, default=10, help="测试组数")
    p.add_argument("--seed", type=int, default=42, help="随机种子")
    p.add_argument(
        "--raw-span",
        type=float,
        default=0.35,
        help="每组在 Pinocchio 标定附近 ±该值（弧度）内随机生成 STM32 原始角",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    model_path = args.model.resolve()
    if not model_path.is_file():
        print(f"未找到模型文件: {model_path}", file=sys.stderr)
        return 1

    try:
        model = load_model(model_path)
    except SystemExit as e:
        print(e, file=sys.stderr)
        return 1

    fm: FeatureMode = args.features  # type: ignore[assignment]
    rng = np.random.default_rng(args.seed)
    pin_cal = np.asarray(TAU_FF.calibration_rad, dtype=np.float64).ravel()
    if pin_cal.size != 4:
        print("TAU_FF.calibration_rad 须为 4 个浮点数", file=sys.stderr)
        return 1

    print(f"模型: {model_path}")
    print(f"特征模式: {fm}")
    print(f"组数: {args.n}，随机种子: {args.seed}")
    print(f"Pinocchio 标定 calibration_rad (rad): {pin_cal.tolist()}")
    print(f"MLP 从原始角减去的偏置 (rad):        {MLP_STM32_BIAS_RAD.tolist()}")
    print(
        "MLP 若输出关节力矩，则经 joint_tau_to_motor_ff_nm 换到电机空间与 Pinocchio 比。"
    )
    print("-" * 72)

    max_l2_motor = 0.0
    max_l2_joint = 0.0
    max_abs_per_motor = np.zeros(4, dtype=np.float64)
    max_abs_per_joint = np.zeros(4, dtype=np.float64)
    span = float(args.raw_span)

    for k in range(args.n):
        noise = rng.uniform(low=-span, high=span, size=4)
        raw_rad = pin_cal + noise

        delta_pin = raw_rad - pin_cal
        d0, d1, d2, d3 = (float(x) for x in delta_pin)
        joint_deg_x100 = motor_rad_deltas_to_joint_deg_x100(d0, d1, d2, d3)

        try:
            delta_pin_t = tuple(delta_pin.tolist())
            tau_pin_j = np.array(compute_tau_joint_nm(delta_pin_t), dtype=np.float64)
            tau_pin_m = np.array(compute_tau_ff_nm(delta_pin_t), dtype=np.float64)
        except Exception as exc:  # noqa: BLE001
            print(f"组 {k + 1}: Pinocchio 计算失败: {exc}", file=sys.stderr)
            return 2

        delta_mlp = raw_rad - MLP_STM32_BIAS_RAD
        try:
            tau_ml_j = predict_tau_ml_from_delta_rad(
                tuple(delta_mlp.tolist()),
                model,
                feature_mode=fm,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"组 {k + 1}: MLP 推理失败: {exc}", file=sys.stderr)
            return 3

        tau_ml_j = np.asarray(tau_ml_j, dtype=np.float64).ravel()
        d_j = tau_ml_j - tau_pin_j
        tau_ml_m = np.array(joint_tau_to_motor_ff_nm(tau_ml_j), dtype=np.float64)
        d_m = tau_ml_m - tau_pin_m
        l2_j = float(np.linalg.norm(d_j))
        l2_m = float(np.linalg.norm(d_m))
        max_l2_joint = max(max_l2_joint, l2_j)
        max_l2_motor = max(max_l2_motor, l2_m)
        max_abs_per_motor = np.maximum(max_abs_per_motor, np.abs(d_m))
        max_abs_per_joint = np.maximum(max_abs_per_joint, np.abs(d_j))

        print(f"#{k + 1:02d}  stm32_raw_rad = {np.array2string(raw_rad, precision=4, separator=', ')}")
        print(
            f"    delta_pin (raw−TAU_FF.cal) = "
            f"{np.array2string(delta_pin, precision=4, separator=', ')}"
        )
        print(f"    URDF joint_deg×100 [j1..j5] = {joint_deg_x100}")
        print(
            f"    delta_mlp (raw−MLP bias)   = "
            f"{np.array2string(delta_mlp, precision=4, separator=', ')}"
        )
        print(f"    τ_joint Pinocchio: {np.array2string(tau_pin_j, precision=4, separator=', ')}")
        print(f"    τ_joint MLP:       {np.array2string(tau_ml_j, precision=4, separator=', ')}")
        print(
            "    Δτ 各关节 j1..j4 (MLP−Pin, Nm): "
            + ", ".join(f"j{i + 1}={d_j[i]:+.6f}" for i in range(4))
        )
        print(f"    Δτ_joint L2:       {l2_j:.6f} Nm")
        print(f"    τ_motor Pinocchio: {np.array2string(tau_pin_m, precision=4, separator=', ')}")
        print(f"    τ_motor MLP:       {np.array2string(tau_ml_m, precision=4, separator=', ')}")
        print(
            "    Δτ 各电机 M0..M3 (MLP−Pin, Nm): "
            + ", ".join(f"M{i}={d_m[i]:+.6f}" for i in range(4))
        )
        print(f"    Δτ_motor L2:       {l2_m:.6f} Nm")
        print("-" * 72)

    print(f"{args.n} 组内 Δτ_joint 最大 L2: {max_l2_joint:.6f} Nm")
    print(f"{args.n} 组内 Δτ_motor 最大 L2: {max_l2_motor:.6f} Nm")
    print(
        "各关节 j1..j4 |Δτ| 最大值 (Nm): "
        + ", ".join(f"j{i + 1}={max_abs_per_joint[i]:.6f}" for i in range(4))
    )
    print(
        "各电机 M0..M3 |Δτ| 最大值 (Nm): "
        + ", ".join(f"M{i}={max_abs_per_motor[i]:.6f}" for i in range(4))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
