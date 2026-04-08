#!/usr/bin/env bash
# 将 Ubuntu 主源与 Docker CE 源切换为清华镜像（Jammy + ubuntu-ports / arm64）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
sudo cp /etc/apt/sources.list "/etc/apt/sources.list.bak.$(date +%Y%m%d%H%M)"
sudo cp "$ROOT/apt-sources-tuna-jammy-ports.list" /etc/apt/sources.list
sudo cp "$ROOT/apt-docker-tuna-jammy.list" /etc/apt/sources.list.d/docker.list
echo "已写入清华源。NodeSource 仍为 deb.nodesource.com（清华无对等镜像，需 Node 可保留或改用 nvm）。"
sudo apt-get update
