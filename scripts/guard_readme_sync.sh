#!/usr/bin/env bash
set -euo pipefail

# Guard policy:
# If Python source files changed, README.md must be updated in the same diff.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[guard] Not a git repository, skip README sync check."
  exit 0
fi

echo "[guard] Reminder: read README.md before modifying code."

changed_files="$(git diff --name-only HEAD 2>/dev/null || true)"

if [[ -z "$changed_files" ]]; then
  echo "[guard] No changes detected."
  exit 0
fi

py_changed=0
readme_changed=0

while IFS= read -r file; do
  [[ -z "$file" ]] && continue
  if [[ "$file" == *.py ]]; then
    py_changed=1
  fi
  if [[ "$file" == "README.md" ]]; then
    readme_changed=1
  fi
done <<< "$changed_files"

if [[ "$py_changed" -eq 1 && "$readme_changed" -eq 0 ]]; then
  echo "[guard] ERROR: Python code changed but README.md was not updated."
  echo "[guard] Please sync README.md (class/module/behavior changes) before commit."
  exit 1
fi

echo "[guard] PASS: README sync policy satisfied."
