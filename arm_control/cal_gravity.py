#!/usr/bin/env python3
"""
重力补偿推理脚本 cal_gravity.py

功能：
- 加载 train_gravity_model.py 训练得到的 gravity_compensator.pkl；
- 输入 4 个关节角（单位：弧度，按 joint1..4 顺序）；
- 内部自动转换为训练时使用的电机位置标度（度×100 ≈ rad*180/pi*100），
  并输出对应 4 个关节的重力补偿力矩（joints.1..4 的 Torque）。

使用示例：
  方式一：命令行一次性计算（输入为弧度）
    python cal_gravity.py  0.5  -1.0  0.8  0.2

  方式二：启动交互模式（不带参数），逐行输入 4 个数：
    python cal_gravity.py
    输入一行 4 个电机位置（用空格分隔，Ctrl+D/Ctrl+Z 结束）:
    > 684.7 -8780 -18102 -2812
    τ = [0.03, -0.89, 3.18, 0.53]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

import joblib
import numpy as np

# 训练数据中 position 近似为「角度(度) × 100」，因此接口接受弧度输入时需要做一次换算。
RAD_TO_INTERNAL = 180.0 / np.pi * 100.0


def load_model(path: Path):
    """加载已经训练好的重力补偿模型 Pipeline。"""
    try:
        model = joblib.load(path)
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"无法加载模型 {path}: {exc}") from exc
    return model


def parse_four_floats(values: List[str]) -> np.ndarray:
    """把 4 个字符串解析为长度为 4 的 float 向量（单位：弧度）。"""
    if len(values) != 4:
        raise ValueError("需要恰好 4 个数，分别对应 motors.0..3.position")
    try:
        nums = [float(v) for v in values]
    except ValueError as exc:
        raise ValueError(f"无法解析为浮点数: {values}") from exc
    return np.asarray(nums, dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(description="机械臂重力补偿推理（cal_gravity）")
    parser.add_argument(
        "positions",
        nargs="*",
        help="4 个关节角（弧度，joint1..4），留空则进入交互模式",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path("gravity_compensator.pkl"),
        help="训练好的重力补偿模型文件路径",
    )
    args = parser.parse_args()

    model = load_model(args.model.resolve())

    # 方式一：命令行直接给 4 个弧度
    if args.positions:
        try:
            q_rad = parse_four_floats(args.positions)
        except ValueError as e:
            print(f"[错误] {e}", file=sys.stderr)
            return 1
        x_internal = q_rad * RAD_TO_INTERNAL
        tau = model.predict(x_internal.reshape(1, -1))[0]
        print("输入关节角 q (rad):", q_rad.tolist())
        print("内部电机位置标度:", x_internal.tolist())
        print("预测重力补偿力矩 τ:", tau.tolist())
        return 0

    # 方式二：交互式多次计算
    print("模型已加载:", args.model.resolve())
    print("输入一行 4 个关节角（单位：弧度，joint1..4，空格分隔，Ctrl+D/Ctrl+Z 结束）:")
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        try:
            q_rad = parse_four_floats(parts)
        except ValueError as e:
            print(f"[错误] {e}", file=sys.stderr)
            continue
        x_internal = q_rad * RAD_TO_INTERNAL
        tau = model.predict(x_internal.reshape(1, -1))[0]
        print(f"q (rad) = {q_rad.tolist()}")
        print(f"x_internal = {x_internal.tolist()}")
        print(f"τ = {tau.tolist()}")
        print("-" * 40)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

