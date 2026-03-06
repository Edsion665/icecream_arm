## IceCream 仓库 Git 使用指南（main/develop 工作流）

本指南针对 `icecream_arm` 仓库，说明在 `main / develop / feature/*` 架构下如何创建分支、开发、提交、合并以及上线。

---

## 1. 分支角色说明

- **`main` 分支**
  - **作用**：稳定 / 上线 / 里程碑版本。
  - **要求**：代码可运行、主要功能通过自测，不在 `main` 上做日常开发。
  - **来源**：只从其他分支（主要是 `develop`）合并经过验证的改动。

- **`develop` 分支**
  - **作用**：日常开发的汇总分支。
  - 所有功能、修复最终都会先合并到 `develop`，再视情况从 `develop` 合并到 `main`。

- **功能分支 `feature/...`**
  - **作用**：针对一个功能/实验/修复的独立工作空间。
  - **命名建议**：
    - `feature/follow-box`：末端跟踪方块功能
    - `feature/circle-autotune`：圆轨迹自适应调参
    - `fix/circle-bug`：修复圆周运动相关 bug

> 记忆方式：  
> - `main` = 稳定上线版本  
> - `develop` = 开发集成版本  
> - `feature/*` = 某个具体功能的工作区

---

## 2. 基本概念快速回顾

- **`git checkout <分支>`**：切换到某个分支，在该分支上查看和修改代码。
- **`git add <文件>`**：把工作区的改动加入“下一个提交”（暂存区）。
- **`git commit -m "信息"`**：在本地生成一个版本快照，只记录在当前分支，不会自动上传到 GitHub。
- **`git push origin <分支>`**：把当前分支的新提交上传到远端（GitHub）。
- **`git pull origin <分支>`**：从远端拉取该分支的新提交并合并到当前分支。
- **`git merge <分支>`**：把“另一个分支的改动”合并进当前分支。

---

## 3. 查看仓库和分支状态

```bash
# 查看当前所在分支
git branch --show-current

# 查看本地所有分支
git branch

# 查看远端分支
git branch -r

# 查看当前仓库关联的远端地址
git remote -v

# 查看每个分支跟踪的远端分支
git branch -vv
```

---

## 4. 从远端更新 `main` / `develop`

在开始任何开发前，建议先同步远端最新代码。

```bash
# 更新 main 分支
git checkout main
git pull origin main

# 更新 develop 分支
git checkout develop
git pull origin develop
```

如遇提示需要选择 pull 策略，可使用：

```bash
git pull --no-rebase origin main
git pull --no-rebase origin develop
```

并可设置默认策略为 merge：

```bash
git config pull.rebase false
```

---

## 5. 从 `develop` 创建功能分支

所有新功能建议从 `develop` 拉分支：

```bash
cd ~/icecream_project

# 1) 确保 develop 最新
git checkout develop
git pull origin develop

# 2) 创建并切换到新功能分支（示例：跟踪方块）
git checkout -b feature/follow-box
```

从现在开始，你的所有修改、提交都会落在 `feature/follow-box` 分支上。

---

## 6. 日常开发提交流程（以功能分支为例）

假设当前分支为 `feature/follow-box`：

### 6.1 编写 / 修改代码

- 在 `icecream/code/` 目录下修改/新增脚本，例如：
  - `icecream_pose_control.py`
  - `test_jacobian_straight_line.py`
  - 新建 `icecream_follow_box.py`

### 6.2 查看当前改动

```bash
git status
```

- 红色文件：已修改但尚未被 `add`
- 绿色文件：已 `add`，正在等待 `commit`

### 6.3 把改动加入本次提交（`add`）

```bash
# 全部加入
git add .

# 或只加入部分文件
git add icecream/code/icecream_follow_box.py
git add icecream/code/test_jacobian_straight_line.py
```

### 6.4 在本地生成提交（`commit`）

```bash
git commit -m "Add follow-box tracking using PoseController"
```

建议提交信息简洁说明“做了什么 + 大概目的”。

### 6.5 推送到远端功能分支（`push`）

首次推送该分支：

```bash
git push -u origin feature/follow-box
```

之后在该分支上，只需：

```bash
git push
```

即可把新的提交推到 `origin/feature/follow-box`。

---

## 7. 把功能分支合并到 `develop`

当功能在 `feature/follow-box` 上开发完成并自测通过后：

```bash
cd ~/icecream_project

# 1) 确保 develop 和功能分支都是最新的
git checkout develop
git pull origin develop

git checkout feature/follow-box
git pull origin feature/follow-box

# 2) 切回 develop
git checkout develop

# 3) 把功能分支合并进 develop
git merge feature/follow-box
# 若有冲突，按提示修改文件 -> git add 冲突文件 -> git commit 完成合并

# 4) 推送更新后的 develop
git push origin develop
```

可选：功能已稳定且无后续开发需求时，可以删除该功能分支：

```bash
# 删除本地功能分支
git branch -d feature/follow-box

# 如无需要，也可删除远端功能分支
git push origin --delete feature/follow-box
```

---

## 8. 从 `develop` 发布到 `main`

当 `develop` 上当前一批改动已经稳定，希望“上线”到 `main` 时：

```bash
cd ~/icecream_project

# 1) 更新 develop 和 main
git checkout develop
git pull origin develop

git checkout main
git pull origin main

# 2) 在 main 上合并 develop 的改动
git merge develop
# 若有冲突，解决后：git add 冲突文件 -> git commit

# 3) 把更新后的 main 推到远端
git push origin main
```

执行完后：

- GitHub 上的 `main` 就包含了当前 `develop` 上的最新功能；
- 可以把此时的 `main` 视作一个新的“发布版本”。

---

## 9. 多人协作与冲突处理

### 9.1 多人同时改同一分支

常见情况：多人同时在 `develop` 上开发。

流程示例（你是 B，同事 A 已先 push）：

```bash
# 你本地已经有了一些提交，准备 push
git push origin develop
# 若看到 non-fast-forward 错误，说明远端有你没有的提交

# 正确做法：
git pull origin develop      # 把 A 的提交拉下来并合并
# 如果提示有冲突：手动编辑相关文件解决冲突 -> git add 冲突文件 -> git commit

git push origin develop      # 再推自己的提交
```

**不要在共享分支（main / develop）上随意使用 `git push --force`**，  
否则可能覆盖他人已推送的提交。

### 9.2 常见 push 错误：non-fast-forward / fetch first

```text
! [rejected]        main -> main (non-fast-forward)
Updates were rejected because the remote contains work that you do not have locally.
```

含义：远端比你本地“更新”，需要先 `pull` 合并再 `push`：

```bash
git checkout main
git pull origin main      # 合并远端提交
# 解决冲突（如有） -> git add 冲突文件 -> git commit
git push origin main
```

---

## 10. 推荐的日常命令组合（模板）

### 10.1 在 `develop` 上直接开发（小改动）

```bash
cd ~/icecream_project

# 开始前同步 develop
git checkout develop
git pull origin develop

# 修改代码...

git add .
git commit -m "描述本次修改"
git push origin develop
```

### 10.2 新功能：从 `develop` 拉功能分支 → 开发 → 合并 → 发布

```bash
cd ~/icecream_project

# 1) 从 develop 拉功能分支
git checkout develop
git pull origin develop
git checkout -b feature/some-feature

# 2) 在 feature 分支上开发 + 多次提交
git add .
git commit -m "实现第一部分"
git add .
git commit -m "补充测试和调参"
git push -u origin feature/some-feature

# 3) 功能完成后合并回 develop
git checkout develop
git pull origin develop
git merge feature/some-feature
git push origin develop

# 4) 需要发布到 main 时
git checkout main
git pull origin main
git merge develop
git push origin main
```

---

## 11. 使用 SSH+443 永久免密码推送 GitHub

部分网络环境会屏蔽 `github.com:22` 端口，但允许通过 `ssh.github.com:443` 访问 SSH。  
推荐为 **所有 GitHub 仓库** 配置一次 SSH，之后基本不再需要输入账号密码。

### 11.1 测试 SSH 连接（走 443 端口）

```bash
ssh -T -p 443 git@ssh.github.com
```

- 如果看到类似输出：
  - `Hi Edsion665! You've successfully authenticated, but GitHub does not provide shell access.`  
  说明 SSH key 已配置成功，可以通过 443 端口访问 GitHub。

### 11.2 在 `~/.ssh/config` 中做一次性全局配置（推荐）

在当前用户下创建/编辑 SSH 配置文件：

```bash
nano ~/.ssh/config
```

增加如下内容（如已有 `Host github.com` 段落，可按此修改）：

```text
Host github.com
    HostName ssh.github.com
    Port 443
    User git
    IdentityFile ~/.ssh/id_ed25519   # 或你的私钥路径，如 ~/.ssh/id_rsa
```

含义：

- 以后访问 `git@github.com:...` 时，实际会连到 `ssh.github.com:443`；
- 不再依赖被屏蔽的 22 端口；
- 所有仓库共用这套配置，只要远端地址是 `git@github.com:用户名/仓库.git` 即可。

### 11.3 新仓库如何添加远端（之后免密码）

```bash
cd /path/to/your_repo

# 使用标准 SSH 形式添加远端（注意替换为你自己的用户名和仓库名）
git remote add origin git@github.com:Edsion665/icecream_arm.git

# 首次推送
git push -u origin main
```

在 `~/.ssh/config` 配置生效后，这里会自动走 443 端口，且使用 SSH key 认证，无需再输入用户名和密码。

### 11.4 已有仓库如何切换到 SSH+443（以本仓库为例）

如果某个仓库之前是 HTTPS（每次都要输入账号和密码），可以改成 SSH：

```bash
cd ~/icecream_project

# 看看当前远端地址
git remote -v

# 将 origin 改为 SSH 形式
git remote set-url origin git@github.com:Edsion665/icecream_arm.git

# 再次确认
git remote -v

# 之后推送将通过 SSH，无需用户名/密码
git push origin main
```

如果出于特殊原因不想改 `~/.ssh/config`，也可以直接使用带 443 端口的 URL：

```bash
git remote set-url origin "ssh://git@ssh.github.com:443/Edsion665/icecream_arm.git"
```

但更推荐使用 11.2 的全局配置方式，方便所有仓库统一管理。

---

## 12. 分支新建 / 删除速查

### 12.1 新建远端分支（本地 + 远端同时创建）

以从 `main` 创建 `feature/reinforcement_learning` 为例：

```bash
cd ~/icecream_project

# 1) 确保 main 是最新的
git checkout main
git pull origin main

# 2) 创建并切换到本地新分支
git checkout -b feature/reinforcement_learning

# 3) 修改代码 -> git add / git commit 之后，首次推送到远端
git push -u origin feature/reinforcement_learning
```

之后在该分支上继续开发时，只需：

```bash
git push        # 因为已经有 -u 绑定，会默认推到 origin/feature/reinforcement_learning
```

### 12.2 删除远端分支

以删除 `origin/feature/reinforcement_learning` 为例：

```bash
cd ~/icecream_project

# 删除远端分支
git push origin --delete feature/reinforcement_learning
# 或等价写法：git push origin :feature/reinforcement_learning
```

如不再需要本地该分支，也可以删除本地分支：

```bash
# 已经合并到其他分支时的安全删除
git branch -d feature/reinforcement_learning

# 若 Git 提示“尚未合并”，但你确认可以强制删除
git branch -D feature/reinforcement_learning
```

> 实际推荐做法：  
> - 功能完成并成功合入 `develop`/`main` 后，再删除本地和远端功能分支；  
> - 合并前不要随意删除，避免丢失尚未集成的工作。

---

以上流程即可作为 `icecream_arm` 仓库的标准 Git 使用规范，在多人协作和长期维护中尽量遵守此约定，可以显著减少合并冲突和分支混乱的问题。
