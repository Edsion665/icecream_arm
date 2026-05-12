#!/usr/bin/env python3
"""MLP-based gravity torque estimator for icecreamPi.

This module provides:
- train_mlp: train a small MLP from calibration/gravity.json (90% train split by default)
- run_mlp: load a trained model and infer motor-space gravity torques

It is designed as a drop-in alternative for calculator.compute_gravity_tau_nm.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Any, Sequence

import numpy as np


def _resolve_import_path() -> None:
    this_file = Path(__file__).resolve()
    icecream_root = this_file.parent.parent.parent
    if str(icecream_root) not in sys.path:
        sys.path.insert(0, str(icecream_root))


_resolve_import_path()


@dataclass
class MlpTrainConfig:
    hidden_dim: int = 32
    epochs: int = 4000
    lr: float = 0.01
    train_ratio: float = 0.9
    seed: int = 42
    weight_decay: float = 1e-4
    log_every: int = 200


def _build_features(
    arm_rad: np.ndarray,
    calibration_rad: np.ndarray,
) -> np.ndarray:
    """Feature vector for MLP.

    We use both absolute arm pose and zero-shifted pose:
    [arm0..3, delta0..3], shape=(N, 8)
    """
    delta = arm_rad - calibration_rad
    return np.concatenate([arm_rad, delta], axis=1)


def _load_dataset(path: Path) -> tuple[np.ndarray, np.ndarray]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, list) or not raw:
        raise ValueError("dataset must be a non-empty JSON list")

    x_list: list[list[float]] = []
    y_list: list[list[float]] = []
    for rec in raw:
        if not isinstance(rec, dict):
            continue
        arm = rec.get("avg_fb_p_rad")
        tau = rec.get("avg_fb_t_nm")
        if not isinstance(arm, list) or not isinstance(tau, list):
            continue
        if len(arm) < 4 or len(tau) < 4:
            continue
        x_list.append([float(arm[i]) for i in range(4)])
        y_list.append([float(tau[i]) for i in range(4)])

    if not x_list:
        raise ValueError("no valid samples found in dataset")
    return np.asarray(x_list, dtype=np.float64), np.asarray(y_list, dtype=np.float64)


def _split_indices(n: int, train_ratio: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    idx = np.arange(n)
    rng.shuffle(idx)
    n_train = max(1, min(n - 1, int(round(n * train_ratio))))
    return idx[:n_train], idx[n_train:]


def _standardize_fit(x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    mean = x.mean(axis=0)
    std = x.std(axis=0)
    std = np.where(std < 1e-8, 1.0, std)
    return mean, std


def _standardize_apply(x: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (x - mean) / std


def _init_params(in_dim: int, hidden_dim: int, out_dim: int, seed: int) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0.0, np.sqrt(2.0 / in_dim), size=(in_dim, hidden_dim))
    b1 = np.zeros((1, hidden_dim), dtype=np.float64)
    w2 = rng.normal(0.0, np.sqrt(2.0 / hidden_dim), size=(hidden_dim, out_dim))
    b2 = np.zeros((1, out_dim), dtype=np.float64)
    return {"w1": w1, "b1": b1, "w2": w2, "b2": b2}


def _forward(x: np.ndarray, p: dict[str, np.ndarray]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    z1 = x @ p["w1"] + p["b1"]
    h1 = np.tanh(z1)
    y = h1 @ p["w2"] + p["b2"]
    cache = {"x": x, "z1": z1, "h1": h1}
    return y, cache


def _mse(y_pred: np.ndarray, y_true: np.ndarray) -> float:
    return float(np.mean((y_pred - y_true) ** 2))


def _backward(
    y_pred: np.ndarray,
    y_true: np.ndarray,
    p: dict[str, np.ndarray],
    cache: dict[str, np.ndarray],
    weight_decay: float,
) -> dict[str, np.ndarray]:
    n = y_true.shape[0]
    dy = 2.0 * (y_pred - y_true) / max(1, n)
    h1 = cache["h1"]
    x = cache["x"]

    dw2 = h1.T @ dy + weight_decay * p["w2"]
    db2 = np.sum(dy, axis=0, keepdims=True)
    dh1 = dy @ p["w2"].T
    dz1 = dh1 * (1.0 - h1**2)
    dw1 = x.T @ dz1 + weight_decay * p["w1"]
    db1 = np.sum(dz1, axis=0, keepdims=True)
    return {"w1": dw1, "b1": db1, "w2": dw2, "b2": db2}


def _step(p: dict[str, np.ndarray], g: dict[str, np.ndarray], lr: float) -> None:
    for k in ("w1", "b1", "w2", "b2"):
        p[k] -= lr * g[k]


def train_mlp(
    data_path: Path,
    model_out: Path,
    calibration_rad: Sequence[float],
    cfg: MlpTrainConfig | None = None,
) -> dict[str, float]:
    """Train MLP with 90% train split by default and save model as .npz."""
    cfg = cfg or MlpTrainConfig()
    arm_rad, tau_nm = _load_dataset(data_path)
    calibration = np.asarray(calibration_rad, dtype=np.float64).reshape(1, 4)
    calibration = np.repeat(calibration, arm_rad.shape[0], axis=0)
    x = _build_features(arm_rad, calibration)
    y = tau_nm

    train_idx, test_idx = _split_indices(len(x), cfg.train_ratio, cfg.seed)
    x_train_raw, y_train = x[train_idx], y[train_idx]
    x_test_raw, y_test = x[test_idx], y[test_idx]

    x_mean, x_std = _standardize_fit(x_train_raw)
    y_mean, y_std = _standardize_fit(y_train)
    x_train = _standardize_apply(x_train_raw, x_mean, x_std)
    x_test = _standardize_apply(x_test_raw, x_mean, x_std)
    y_train_n = _standardize_apply(y_train, y_mean, y_std)
    y_test_n = _standardize_apply(y_test, y_mean, y_std)

    p = _init_params(in_dim=x_train.shape[1], hidden_dim=cfg.hidden_dim, out_dim=4, seed=cfg.seed)
    for ep in range(1, cfg.epochs + 1):
        pred, cache = _forward(x_train, p)
        grads = _backward(pred, y_train_n, p, cache, cfg.weight_decay)
        _step(p, grads, cfg.lr)
        if ep % cfg.log_every == 0 or ep == 1 or ep == cfg.epochs:
            train_loss = _mse(pred, y_train_n)
            val_pred, _ = _forward(x_test, p)
            val_loss = _mse(val_pred, y_test_n)
            print(f"[train_mlp] epoch={ep} train_mse={train_loss:.6f} val_mse={val_loss:.6f}")

    train_pred_n, _ = _forward(x_train, p)
    test_pred_n, _ = _forward(x_test, p)
    train_pred = train_pred_n * y_std + y_mean
    test_pred = test_pred_n * y_std + y_mean

    train_rmse = float(np.sqrt(np.mean((train_pred - y_train) ** 2)))
    test_rmse = float(np.sqrt(np.mean((test_pred - y_test) ** 2)))

    model_out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        model_out,
        w1=p["w1"],
        b1=p["b1"],
        w2=p["w2"],
        b2=p["b2"],
        x_mean=x_mean,
        x_std=x_std,
        y_mean=y_mean,
        y_std=y_std,
        calibration=np.asarray(calibration_rad, dtype=np.float64),
    )

    metrics = {
        "samples": float(len(x)),
        "train_samples": float(len(train_idx)),
        "test_samples": float(len(test_idx)),
        "train_rmse_nm": train_rmse,
        "test_rmse_nm": test_rmse,
    }
    print(f"[train_mlp] done: {metrics}")
    return metrics


def run_mlp(
    model_path: Path,
    arm_rad: Sequence[float],
    calibration_rad: Sequence[float] | None = None,
    gain: float = 1.0,
) -> tuple[float, float, float, float]:
    """Run trained MLP and return motor-space torque tuple (4)."""
    d = np.load(model_path)
    w1 = d["w1"]
    b1 = d["b1"]
    w2 = d["w2"]
    b2 = d["b2"]
    x_mean = d["x_mean"]
    x_std = d["x_std"]
    y_mean = d["y_mean"]
    y_std = d["y_std"]
    cal_saved = d["calibration"]

    arm = np.asarray(arm_rad, dtype=np.float64).reshape(1, 4)
    if calibration_rad is None:
        cal = np.asarray(cal_saved, dtype=np.float64).reshape(1, 4)
    else:
        cal = np.asarray(calibration_rad, dtype=np.float64).reshape(1, 4)

    feat = _build_features(arm, cal)
    feat_n = _standardize_apply(feat, x_mean, x_std)
    h = np.tanh(feat_n @ w1 + b1)
    y_n = h @ w2 + b2
    y = y_n * y_std + y_mean
    out = y.reshape(4) * float(gain)
    return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))


def _default_paths() -> tuple[Path, Path]:
    root = Path(__file__).resolve().parent.parent
    data_path = root / "calibration" / "gravity.json"
    model_path = Path(__file__).resolve().parent / "gravity_mlp_model.npz"
    return data_path, model_path


def _parse_cli() -> argparse.Namespace:
    data_default, model_default = _default_paths()
    parser = argparse.ArgumentParser(description="MLP gravity compensation trainer/inferencer")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_train = sub.add_parser("train", help="train MLP from calibration data")
    p_train.add_argument("--data", type=Path, default=data_default)
    p_train.add_argument("--model-out", type=Path, default=model_default)
    p_train.add_argument("--epochs", type=int, default=10000)
    p_train.add_argument("--hidden-dim", type=int, default=32)
    p_train.add_argument("--lr", type=float, default=0.01)
    p_train.add_argument("--train-ratio", type=float, default=0.9)
    p_train.add_argument("--seed", type=int, default=42)
    p_train.add_argument("--weight-decay", type=float, default=1e-4)
    p_train.add_argument("--calibration-rad", type=float, nargs=4, required=False)

    p_run = sub.add_parser("run", help="run MLP inference for one pose")
    p_run.add_argument("--model", type=Path, default=model_default)
    p_run.add_argument("--arm-rad", type=float, nargs=4, required=True)
    p_run.add_argument("--calibration-rad", type=float, nargs=4, required=False)
    p_run.add_argument("--gain", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_cli()
    if args.cmd == "train":
        from icecreamPi.config import CONFIG

        calibration = (
            args.calibration_rad
            if args.calibration_rad is not None
            else tuple(float(x) for x in CONFIG.control.calibration_rad)
        )
        cfg = MlpTrainConfig(
            hidden_dim=int(args.hidden_dim),
            epochs=int(args.epochs),
            lr=float(args.lr),
            train_ratio=float(args.train_ratio),
            seed=int(args.seed),
            weight_decay=float(args.weight_decay),
        )
        train_mlp(
            data_path=args.data,
            model_out=args.model_out,
            calibration_rad=calibration,
            cfg=cfg,
        )
        return

    if args.cmd == "run":
        out = run_mlp(
            model_path=args.model,
            arm_rad=args.arm_rad,
            calibration_rad=args.calibration_rad,
            gain=args.gain,
        )
        print(
            "[run_mlp] tau_nm="
            f"[{out[0]:.6f}, {out[1]:.6f}, {out[2]:.6f}, {out[3]:.6f}]"
        )
        return

    raise RuntimeError(f"unsupported command: {args.cmd}")


if __name__ == "__main__":
    main()
