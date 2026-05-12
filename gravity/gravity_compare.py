#!/usr/bin/env python3
"""Compare gravity compensation output against collected calibration data."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Any


def _resolve_import_path() -> None:
    this_file = Path(__file__).resolve()
    icecream_root = this_file.parent.parent.parent
    if str(icecream_root) not in sys.path:
        sys.path.insert(0, str(icecream_root))


_resolve_import_path()

from icecreamPi.calculator import compute_gravity_tau_nm  # noqa: E402
from icecreamPi.config import CONFIG  # noqa: E402
from icecreamPi.gravity.mlp_gravity import run_mlp  # noqa: E402


def _rmse(values: list[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / float(len(values)))


def _joint_stats(errors: list[float]) -> dict[str, float]:
    abs_errors = [abs(x) for x in errors]
    return {
        "mae_nm": statistics.fmean(abs_errors) if abs_errors else 0.0,
        "rmse_nm": _rmse(errors),
        "max_abs_nm": max(abs_errors) if abs_errors else 0.0,
        "bias_nm": statistics.fmean(errors) if errors else 0.0,
    }


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent.parent
    gravity_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="对比模型重力补偿扭矩与采集扭矩误差")
    parser.add_argument(
        "--data",
        type=Path,
        default=root / "calibration" / "gravity.json",
        help="采集数据 JSON 路径",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "gravity_compare_report.json",
        help="对比报告输出路径",
    )
    parser.add_argument("--gain", type=float, default=1.0, help="重力补偿计算增益")
    parser.add_argument(
        "--method",
        type=str,
        choices=("pinocchio", "mlp"),
        default="pinocchio",
        help="预测方法：pinocchio(原始) 或 mlp(学习模型)",
    )
    parser.add_argument(
        "--mlp-model",
        type=Path,
        default=gravity_dir / "gravity_mlp_model.npz",
        help="method=mlp 时使用的模型路径",
    )
    parser.add_argument(
        "--calibration-rad",
        type=float,
        nargs=4,
        metavar=("R0", "R1", "R2", "R3"),
        help="覆盖 calibration_rad；不传则使用 config 默认值",
    )
    return parser.parse_args()


def _to_f4(values: Any, name: str) -> tuple[float, float, float, float]:
    if not isinstance(values, list) or len(values) < 4:
        raise ValueError(f"{name} must be a list of at least 4 values")
    return (float(values[0]), float(values[1]), float(values[2]), float(values[3]))


def main() -> None:
    args = parse_args()
    raw = json.loads(args.data.read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise ValueError("data JSON must be a list")

    calibration = (
        tuple(float(x) for x in args.calibration_rad)
        if args.calibration_rad is not None
        else tuple(float(x) for x in CONFIG.control.calibration_rad)
    )

    if args.method == "mlp" and not args.mlp_model.is_file():
        raise FileNotFoundError(f"MLP model not found: {args.mlp_model}")

    all_errors: list[list[float]] = [[], [], [], []]
    sample_reports: list[dict[str, Any]] = []

    for idx, rec in enumerate(raw, start=1):
        if not isinstance(rec, dict):
            continue
        arm_rad = _to_f4(rec.get("avg_fb_p_rad"), "avg_fb_p_rad")
        measured_tau = _to_f4(rec.get("avg_fb_t_nm"), "avg_fb_t_nm")
        if args.method == "pinocchio":
            predicted_tau = compute_gravity_tau_nm(
                arm_rad=arm_rad,
                calibration_rad=calibration,
                gain=float(args.gain),
            )
        else:
            predicted_tau = run_mlp(
                model_path=args.mlp_model,
                arm_rad=arm_rad,
                calibration_rad=calibration,
                gain=float(args.gain),
            )
        err = [float(predicted_tau[i]) - float(measured_tau[i]) for i in range(4)]
        for j in range(4):
            all_errors[j].append(err[j])
        sample_reports.append(
            {
                "index": idx,
                "target_rel_deg": rec.get("target_rel_deg"),
                "measured_tau_nm": [round(x, 6) for x in measured_tau],
                "predicted_tau_nm": [round(float(x), 6) for x in predicted_tau],
                "error_nm": [round(x, 6) for x in err],
                "timestamp": rec.get("timestamp"),
            }
        )

    joint_summary = {f"joint_{i}": _joint_stats(all_errors[i]) for i in range(4)}
    flat_errors = [x for arr in all_errors for x in arr]
    overall = _joint_stats(flat_errors)

    report = {
        "meta": {
            "data_path": str(args.data),
            "sample_count": len(sample_reports),
            "method": args.method,
            "mlp_model": str(args.mlp_model) if args.method == "mlp" else None,
            "gain": float(args.gain),
            "calibration_rad": [float(x) for x in calibration],
        },
        "overall": overall,
        "per_joint": joint_summary,
        "samples": sample_reports,
    }
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"[compare] samples={len(sample_reports)}")
    print(
        "[compare] overall "
        f"mae={overall['mae_nm']:.6f} "
        f"rmse={overall['rmse_nm']:.6f} "
        f"max_abs={overall['max_abs_nm']:.6f} "
        f"bias={overall['bias_nm']:.6f}"
    )
    for i in range(4):
        s = joint_summary[f"joint_{i}"]
        print(
            f"[compare] joint_{i} "
            f"mae={s['mae_nm']:.6f} rmse={s['rmse_nm']:.6f} "
            f"max_abs={s['max_abs_nm']:.6f} bias={s['bias_nm']:.6f}"
        )
    print(f"[compare] report written to {args.out}")


if __name__ == "__main__":
    main()
