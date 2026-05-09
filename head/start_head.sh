#!/usr/bin/env bash
# 一键启动 head（等价于在 head 目录执行 uv run python -m src.run …），
# 实际使用 .venv 内解释器，避免每次走 uv run 包装层。
#
# 用法：
#   ./start_head.sh
#   ./start_head.sh --config other.yaml
#   HEAD_CONFIG=other.yaml ./start_head.sh
#
# 首次无 .venv 时：若已安装 uv，会自动 uv sync（可通过 UV_SYNC_FLAGS 追加，如 --group dev）。

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

ensure_venv() {
  if [[ -x "$ROOT/.venv/bin/python" ]]; then
    return 0
  fi
  if command -v uv >/dev/null 2>&1; then
    # shellcheck disable=SC2086
    uv sync ${UV_SYNC_FLAGS:-}
    return 0
  fi
  echo "start_head.sh: 未找到 $ROOT/.venv ，且系统无 uv 命令。" >&2
  echo "请先执行: cd \"$ROOT\" && uv sync   或手动创建 venv 并安装依赖。" >&2
  return 1
}

ensure_venv

if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  echo "start_head.sh: $ROOT/.venv/bin/python 仍不可用。" >&2
  exit 1
fi

if [[ -n "${HEAD_CONFIG:-}" && "${#@}" -eq 0 ]]; then
  set -- --config "$HEAD_CONFIG"
fi

exec "$ROOT/.venv/bin/python" -m src.run "$@"
