#!/usr/bin/env python3
"""
重力补偿推理脚本 cal_gravity.py

功能：
- 加载 train_gravity_model.py 训练得到的 gravity_compensator.pkl（sklearn Pipeline）；
- 推荐输入：四轴相对标定零位的弧度差 ``delta_rad``，内部先转为与数据集一致的「电机侧度×100」再送入 MLP；
- 默认假定模型输出为 joints.1..4 的力矩 (Nm)，与 captures 标签一致。

使用示例：
  方式一：命令行一次性计算（4 个数为相对标定的弧度差，与 main.py 中 fb−cal 同含义）
    python cal_gravity.py  0.05  -0.02  0.1  0.0

  方式二：交互模式（不带参数）
    python cal_gravity.py
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Literal, Sequence

import joblib
import numpy as np

from gravity_feedforward import motor_rad_deltas_to_joint_deg_x100, pinocchio_tau_to_four_motor_nm

# 弧度差 → 与固件/数据集中 motors.k 一致的「度×100」（逐项，不在电机 1 上额外取负）
RAD_TO_DEG_X100 = 180.0 / np.pi * 100.0

FeatureMode = Literal["motor_deg_x100", "urdf_joint_deg_x100"]


def load_model(path: Path):
    """加载已经训练好的重力补偿模型 Pipeline。"""
    try:
        model = joblib.load(path)
    except Exception as exc:  # noqa: BLE001
        raise SystemExit(f"无法加载模型 {path}: {exc}") from exc
    return model


def parse_four_floats(values: List[str]) -> np.ndarray:
    """把 4 个字符串解析为长度为 4 的 float 向量（相对标定零位的弧度差）。"""
    if len(values) != 4:
        raise ValueError("需要恰好 4 个数：四轴相对标定的弧度差 (delta rad)")
    try:
        nums = [float(v) for v in values]
    except ValueError as exc:
        raise ValueError(f"无法解析为浮点数: {values}") from exc
    return np.asarray(nums, dtype=float)


def motor_delta_rad_to_ml_features_deg_x100(delta_rad: Sequence[float]) -> np.ndarray:
    """四电机相对标定弧度差 → 训练时常用的 4 维特征（每项 = 该轴弧度 × 180/π×100）。

    与 captures 中 ``motors.0..3.angle`` 在「相对同一标定零点」时的标度一致。
    """
    d = np.asarray(list(delta_rad), dtype=np.float64).ravel()
    if d.size != 4:
        raise ValueError("delta_rad 须为长度 4 的序列")
    return d * RAD_TO_DEG_X100


def urdf_joint_deg_x100_features_from_delta_rad(delta_rad: Sequence[float]) -> np.ndarray:
    """四轴弧度差 → Pinocchio/URDF 用关节角（度×100）的前 4 维（含 joint2 对 motor1 取负）。"""
    d = np.asarray(list(delta_rad), dtype=np.float64).ravel()
    if d.size != 4:
        raise ValueError("delta_rad 须为长度 4 的序列")
    full = motor_rad_deltas_to_joint_deg_x100(float(d[0]), float(d[1]), float(d[2]), float(d[3]))
    return np.asarray(full[:4], dtype=np.float64)


def build_ml_features(
    delta_rad: Sequence[float],
    *,
    feature_mode: FeatureMode = "motor_deg_x100",
) -> np.ndarray:
    if feature_mode == "motor_deg_x100":
        return motor_delta_rad_to_ml_features_deg_x100(delta_rad)
    if feature_mode == "urdf_joint_deg_x100":
        return urdf_joint_deg_x100_features_from_delta_rad(delta_rad)
    raise ValueError(f"未知 feature_mode: {feature_mode}")


def predict_tau_ml_from_delta_rad(
    delta_rad: Sequence[float],
    model,
    *,
    feature_mode: FeatureMode = "motor_deg_x100",
) -> np.ndarray:
    """MLP：弧度差 → 度×100 特征 → ``model.predict``，返回形状 (4,) 的预测力矩 (Nm)。"""
    x = build_ml_features(delta_rad, feature_mode=feature_mode)
    y = np.asarray(model.predict(x.reshape(1, -1))[0], dtype=np.float64).ravel()
    if y.size != 4:
        raise ValueError(f"模型输出维数应为 4，got {y.size}")
    return y


def joint_tau_to_motor_ff_nm(tau_joint: Sequence[float]) -> tuple[float, float, float, float]:
    """URDF joint1..4 重力矩 → 与 ``compute_tau_ff_nm`` 一致的四电机前馈 (Nm)。"""
    return pinocchio_tau_to_four_motor_nm(tau_joint)


def main() -> int:
    parser = argparse.ArgumentParser(description="机械臂重力补偿推理（MLP，cal_gravity）")
    parser.add_argument(
        "positions",
        nargs="*",
        help="4 个数：四轴相对标定零位的弧度差（与力矩前馈 loop 中 fb−cal 一致）",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().parent / "gravity_compensator.pkl",
        help="训练好的重力补偿模型文件路径",
    )
    parser.add_argument(
        "--features",
        choices=("motor_deg_x100", "urdf_joint_deg_x100"),
        default="motor_deg_x100",
        help="MLP 输入特征：电机侧度×100（默认）或与 Pinocchio 一致的前四 URDF 关节度×100",
    )
    args = parser.parse_args()

    model = load_model(args.model.resolve())
    fm: FeatureMode = args.features  # type: ignore[assignment]

    if args.positions:
        try:
            delta_rad = parse_four_floats(args.positions)
        except ValueError as e:
            print(f"[错误] {e}", file=sys.stderr)
            return 1
        x_internal = build_ml_features(delta_rad, feature_mode=fm)
        tau_joint = predict_tau_ml_from_delta_rad(delta_rad, model, feature_mode=fm)
        tau_motor = joint_tau_to_motor_ff_nm(tau_joint)
        print("delta_rad (四轴):", delta_rad.tolist())
        print("MLP 输入特征 (度×100):", x_internal.tolist())
        print("预测关节重力矩 τ_joint (Nm, joints.1..4):", tau_joint.tolist())
        print("对应四电机前馈 (Nm):", list(tau_motor))
        return 0

    print("模型已加载:", args.model.resolve())
    print(
        "输入一行 4 个数：相对标定零位的弧度差（空格分隔）；"
        "Ctrl+D / Ctrl+Z 结束。"
    )
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        try:
            delta_rad = parse_four_floats(parts)
        except ValueError as e:
            print(f"[错误] {e}", file=sys.stderr)
            continue
        x_internal = build_ml_features(delta_rad, feature_mode=fm)
        tau_joint = predict_tau_ml_from_delta_rad(delta_rad, model, feature_mode=fm)
        tau_motor = joint_tau_to_motor_ff_nm(tau_joint)
        print(f"delta_rad = {delta_rad.tolist()}")
        print(f"x_internal = {x_internal.tolist()}")
        print(f"τ_joint = {tau_joint.tolist()}")
        print(f"τ_motor = {list(tau_motor)}")
        print("-" * 40)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
