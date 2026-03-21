#!/usr/bin/env bash
# 将 ~/icecream 同步到 GitHub: Edsion665/icecream_arm 分支 feature/raspberryPi-server
set -euo pipefail

REPO_URL="${ICECREAM_GIT_REMOTE:-https://github.com/Edsion665/icecream_arm.git}"
BRANCH="feature/raspberryPi-server"

if ! command -v git >/dev/null 2>&1; then
  echo "错误：未找到 git。请先执行：sudo apt update && sudo apt install -y git"
  exit 1
fi

cd ~/icecream

if [[ ! -d .git ]]; then
  git init
fi

# 避免把虚拟环境、缓存、日志推上去（可按需修改）
if [[ ! -f .gitignore ]]; then
  cat > .gitignore << 'EOF'
icecream/
__pycache__/
*.pyc
.pytest_cache/
.mypy_cache/
*.log
logs/
.env
.venv/
venv/
EOF
  echo "已创建默认 .gitignore（含 icecream/ 虚拟环境目录）"
fi

if ! git remote get-url origin >/dev/null 2>&1; then
  git remote add origin "$REPO_URL"
else
  git remote set-url origin "$REPO_URL"
fi

git fetch origin "$BRANCH" 2>/dev/null || git fetch origin

# 若远程已有该分支：基于远程分支建立本地分支（保留历史），再叠加上传本地文件
if git rev-parse "origin/$BRANCH" >/dev/null 2>&1; then
  git checkout -B "$BRANCH" "origin/$BRANCH"
else
  git checkout -B "$BRANCH" 2>/dev/null || git checkout -B "$BRANCH"
fi

git add -A
if git diff --cached --quiet; then
  echo "没有新的变更需要提交。"
else
  git commit -m "chore: sync raspberry Pi arm_control ($(date -Iseconds))"
fi

echo "推送到 origin $BRANCH ..."
git push -u origin "$BRANCH"

echo "完成。远程：$REPO_URL 分支：$BRANCH"
