#!/usr/bin/env python3
"""Residual MLP for gravity compensation correction.

Architecture: arm_rad (4,) → [64, tanh] → delta_tau (4,)
Usage:
  tau_final = tau_pinocchio + infer_residual_mlp(model_path, arm_rad)

Train:
  python3 residual_mlp.py train --data gravity.json --pinocchio-report gravity_compare_report.json
Infer:
  python3 residual_mlp.py infer --model gravity_residual_model.npz --arm-rad 1.57 0.5 1.0 0.4
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Sequence

import numpy as np


# ---------------------------------------------------------------------------
# Model forward
# ---------------------------------------------------------------------------

def _forward(x: np.ndarray, w1: np.ndarray, b1: np.ndarray,
             w2: np.ndarray, b2: np.ndarray) -> np.ndarray:
    """Single hidden layer: x (N,4) -> (N,4)."""
    return np.tanh(x @ w1 + b1) @ w2 + b2


# ---------------------------------------------------------------------------
# Dataset helpers
# ---------------------------------------------------------------------------

def _load_dataset(
    data_path: Path,
    report_path: Path,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (arm_rad, residual) arrays from gravity.json + compare report."""
    raw = json.loads(data_path.read_text())
    report = json.loads(report_path.read_text())

    arm_list, res_list = [], []
    samples = report["samples"]
    for i, rec in enumerate(raw):
        if i >= len(samples):
            break
        arm = [float(rec["avg_fb_p_rad"][j]) for j in range(4)]
        meas = [float(rec["avg_fb_t_nm"][j]) for j in range(4)]
        pred = [float(samples[i]["predicted_tau_nm"][j]) for j in range(4)]
        residual = [meas[j] - pred[j] for j in range(4)]
        arm_list.append(arm)
        res_list.append(residual)

    return np.array(arm_list, dtype=np.float64), np.array(res_list, dtype=np.float64)


def _split(n: int, train_ratio: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(seed)
    idx = np.arange(n)
    rng.shuffle(idx)
    n_train = max(1, min(n - 1, int(round(n * train_ratio))))
    return idx[:n_train], idx[n_train:]


# ---------------------------------------------------------------------------
# Train
# ---------------------------------------------------------------------------

def train(
    data_path: Path,
    report_path: Path,
    model_out: Path,
    hidden: int = 64,
    epochs: int = 5000,
    lr: float = 0.005,
    weight_decay: float = 1e-4,
    train_ratio: float = 0.9,
    seed: int = 42,
    log_every: int = 500,
) -> dict:
    arm, res = _load_dataset(data_path, report_path)
    n = len(arm)

    train_idx, test_idx = _split(n, train_ratio, seed)
    x_tr, y_tr = arm[train_idx], res[train_idx]
    x_te, y_te = arm[test_idx],  res[test_idx]

    # Standardize inputs
    x_mean = x_tr.mean(axis=0)
    x_std  = x_tr.std(axis=0)
    x_std  = np.where(x_std < 1e-8, 1.0, x_std)

    # Standardize targets
    y_mean = y_tr.mean(axis=0)
    y_std  = y_tr.std(axis=0)
    y_std  = np.where(y_std < 1e-8, 1.0, y_std)

    def norm_x(x): return (x - x_mean) / x_std
    def norm_y(y): return (y - y_mean) / y_std
    def denorm_y(y): return y * y_std + y_mean

    x_tr_n = norm_x(x_tr)
    x_te_n = norm_x(x_te)
    y_tr_n = norm_y(y_tr)

    # Init weights (He init)
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0.0, np.sqrt(2.0 / 4),      (4,      hidden))
    b1 = np.zeros((1, hidden))
    w2 = rng.normal(0.0, np.sqrt(2.0 / hidden), (hidden, 4))
    b2 = np.zeros((1, 4))

    # Adam state
    m = {k: np.zeros_like(v) for k, v in [('w1',w1),('b1',b1),('w2',w2),('b2',b2)]}
    v = {k: np.zeros_like(v) for k, v in [('w1',w1),('b1',b1),('w2',w2),('b2',b2)]}
    beta1, beta2, eps_adam = 0.9, 0.999, 1e-8

    best_test_mae = float('inf')
    best_params = None
    history = []

    t_start = time.perf_counter()

    for ep in range(1, epochs + 1):
        # Forward
        z1 = x_tr_n @ w1 + b1          # (N, hidden)
        h1 = np.tanh(z1)                # (N, hidden)
        y_hat_n = h1 @ w2 + b2         # (N, 4)

        # MSE loss + L2
        diff = y_hat_n - y_tr_n
        loss = float(np.mean(diff ** 2))

        # Backward
        n_tr = len(x_tr_n)
        dy = 2.0 * diff / n_tr
        dw2 = h1.T @ dy + weight_decay * w2
        db2 = dy.sum(axis=0, keepdims=True)
        dh1 = dy @ w2.T
        dz1 = dh1 * (1.0 - h1 ** 2)
        dw1 = x_tr_n.T @ dz1 + weight_decay * w1
        db1 = dz1.sum(axis=0, keepdims=True)

        # Adam update
        grads = {'w1': dw1, 'b1': db1, 'w2': dw2, 'b2': db2}
        params = {'w1': w1, 'b1': b1, 'w2': w2, 'b2': b2}
        for k in params:
            m[k] = beta1 * m[k] + (1 - beta1) * grads[k]
            v[k] = beta2 * v[k] + (1 - beta2) * grads[k] ** 2
            m_hat = m[k] / (1 - beta1 ** ep)
            v_hat = v[k] / (1 - beta2 ** ep)
            params[k] -= lr * m_hat / (np.sqrt(v_hat) + eps_adam)
        w1, b1, w2, b2 = params['w1'], params['b1'], params['w2'], params['b2']

        if ep % log_every == 0 or ep == 1 or ep == epochs:
            # Eval on test set (denormalized)
            y_te_hat = denorm_y(_forward(x_te_n, w1, b1, w2, b2))
            test_mae  = float(np.abs(y_te - y_te_hat).mean())
            test_rmse = float(np.sqrt(np.mean((y_te - y_te_hat) ** 2)))

            y_tr_hat = denorm_y(_forward(x_tr_n, w1, b1, w2, b2))
            train_mae = float(np.abs(y_tr - y_tr_hat).mean())

            elapsed = time.perf_counter() - t_start
            print(f"[ep {ep:5d}] loss={loss:.6f}  train_mae={train_mae:.4f}  "
                  f"test_mae={test_mae:.4f}  test_rmse={test_rmse:.4f}  t={elapsed:.1f}s")
            history.append({'ep': ep, 'train_mae': train_mae,
                            'test_mae': test_mae, 'test_rmse': test_rmse})

            if test_mae < best_test_mae:
                best_test_mae = test_mae
                best_params = (w1.copy(), b1.copy(), w2.copy(), b2.copy())

    # Use best checkpoint
    if best_params is not None:
        w1, b1, w2, b2 = best_params

    # Final per-joint eval (full dataset, denormalized)
    x_all_n = norm_x(arm)
    y_all_hat = denorm_y(_forward(x_all_n, w1, b1, w2, b2))
    print("\n[final] per-joint MAE on full dataset (residual):")
    for j in range(4):
        mae_j = float(np.abs(res[:, j] - y_all_hat[:, j]).mean())
        print(f"  joint_{j}: residual_mae={mae_j:.4f} Nm")

    # Save
    model_out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        model_out,
        w1=w1, b1=b1, w2=w2, b2=b2,
        x_mean=x_mean, x_std=x_std,
        y_mean=y_mean, y_std=y_std,
    )
    print(f"\n[saved] {model_out}")

    return {
        'best_test_mae': best_test_mae,
        'history': history,
        'n_train': len(train_idx),
        'n_test':  len(test_idx),
    }


# ---------------------------------------------------------------------------
# Infer
# ---------------------------------------------------------------------------

def infer(
    model_path: Path,
    arm_rad: Sequence[float],
) -> tuple[float, float, float, float]:
    """Return delta_tau (4,) to add on top of pinocchio output."""
    d = np.load(model_path)
    x = (np.asarray(arm_rad, dtype=np.float64).reshape(1, 4) - d['x_mean']) / d['x_std']
    y_n = _forward(x, d['w1'], d['b1'], d['w2'], d['b2'])
    y = y_n * d['y_std'] + d['y_mean']
    out = y.reshape(4)
    return (float(out[0]), float(out[1]), float(out[2]), float(out[3]))


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _default_paths() -> tuple[Path, Path, Path]:
    here = Path(__file__).resolve().parent
    return (
        here / 'gravity.json',
        here / 'gravity_compare_report.json',
        here / 'gravity_residual_model.npz',
    )


def main() -> None:
    data_def, report_def, model_def = _default_paths()
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_tr = sub.add_parser('train')
    p_tr.add_argument('--data',             type=Path, default=data_def)
    p_tr.add_argument('--pinocchio-report', type=Path, default=report_def)
    p_tr.add_argument('--model-out',        type=Path, default=model_def)
    p_tr.add_argument('--hidden',           type=int,  default=64)
    p_tr.add_argument('--epochs',           type=int,  default=5000)
    p_tr.add_argument('--lr',               type=float,default=0.005)
    p_tr.add_argument('--weight-decay',     type=float,default=1e-4)
    p_tr.add_argument('--train-ratio',      type=float,default=0.9)
    p_tr.add_argument('--seed',             type=int,  default=42)
    p_tr.add_argument('--log-every',        type=int,  default=500)

    p_inf = sub.add_parser('infer')
    p_inf.add_argument('--model',   type=Path,  default=model_def)
    p_inf.add_argument('--arm-rad', type=float, nargs=4, required=True)

    args = parser.parse_args()

    if args.cmd == 'train':
        train(
            data_path=args.data,
            report_path=args.pinocchio_report,
            model_out=args.model_out,
            hidden=args.hidden,
            epochs=args.epochs,
            lr=args.lr,
            weight_decay=args.weight_decay,
            train_ratio=args.train_ratio,
            seed=args.seed,
            log_every=args.log_every,
        )
    elif args.cmd == 'infer':
        delta = infer(args.model, args.arm_rad)
        print(f"[infer] delta_tau={[round(x,6) for x in delta]}")


if __name__ == '__main__':
    main()
