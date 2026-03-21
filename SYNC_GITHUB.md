# 将本目录同步到 GitHub：`icecream_arm` 分支 `feature/raspberryPi-server`

仓库：<https://github.com/Edsion665/icecream_arm>  
目标分支：`feature/raspberryPi-server`

## 1. 安装 Git（若尚未安装）

```bash
sudo apt update
sudo apt install -y git
```

## 2. 配置身份（首次使用 Git 时）

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

## 3. 一键同步（推荐）

```bash
cd ~/icecream
chmod +x ./sync_to_github.sh
./sync_to_github.sh
```

脚本会：初始化仓库、添加远程、拉取远程分支（允许无关历史合并）、提交本地改动并推送。

## 4. 认证说明

- **HTTPS**：推送时 GitHub 会要求登录；建议使用 [Personal Access Token](https://github.com/settings/tokens) 代替密码。
- **SSH**：将远程改为 `git@github.com:Edsion665/icecream_arm.git`，并先在树莓派配置 SSH 公钥。

## 5. 若远程分支已有内容且你想用本地完全覆盖远程（慎用）

仅在确认要丢弃远程该分支上独有提交时使用：

```bash
cd ~/icecream
git push -u origin feature/raspberryPi-server --force
```

## 6. 忽略大文件 / 密钥

推送前请确认未包含密码、密钥或超大模型；必要时编辑 `.gitignore`。
