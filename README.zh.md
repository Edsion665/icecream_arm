# icecreamPi V2（中文说明）

`icecreamPi` 是机械臂控制运行时的独立版本（不依赖 `arm_control` 的 Python 模块）。
它负责接收上位机 UDP 指令、计算控制量（含重力补偿）、通过串口下发 STM32，并通过 WebSocket 对外广播状态。

## 当前项目结构

项目采用分层结构：

- `app`：运行时编排层（启动、停机、主循环、依赖装配）
- `domain`：领域规则层（控制映射、命令语义、接口边界）
- `infra`：基础设施层（串口编解码、UDP 包解析、状态展示）

核心目录如下：

- `main.py`：兼容入口，转调 `app/runtime.py`
- `app/runtime.py`：生命周期编排与主控制循环
- `controller.py`：控制决策（UDP 新鲜度判断、hold 回退、扭矩链路）
- `calculator.py`：重力补偿求解（内置 Pinocchio 逻辑）
- `serial.py`：串口线程管理（收发、重连、调试快照）
- `listener.py`：UDP 监听与状态刷新
- `server.py`：WebSocket 状态广播
- `state_store.py`：线程安全共享状态寄存器
- `config.py`：环境变量配置
- `domain/mapping.py`：坐标轴/符号统一映射
- `domain/ports.py`：`StateReadPort` / `StateWritePort` / `MotorCommandSink` 等轻量接口
- `domain/command_handler.py`：WS 命令语义处理
- `infra/serial/codec.py`：MIT 帧编解码纯函数
- `infra/udp/packet.py`：UDP 包格式解析
- `infra/state/presenter.py`：状态 payload 组装
- `robotarm/icecream_arm_model`：内置 URDF 模型资源（重力补偿使用）
- `scripts/start.sh`：项目内独立启动脚本
- `requirements.txt`：运行时 Python 依赖清单
- `calibration/collect.py`：重力数据采集脚本（支持增量写入）
- `calibration/gravity.json`：采集输出数据

## 运行流程（高层）

1. `main.py` 启动 `app/runtime.py`
2. 启动串口线程（`serial.py`）和 UDP 监听线程（`listener.py`）
3. 控制循环调用 `controller.py` 生成 MIT 命令
4. `calculator.py` 在开启重力前馈时计算扭矩
5. `serial.py` 将 MIT 命令下发到 STM32
6. `server.py` 周期推送状态到 WebSocket 客户端

## 关键数据流

- 上位机 -> 树莓派：UDP（目标角/速度）
- 树莓派 -> STM32：MIT 二进制命令（串口）
- STM32 -> 树莓派：MIT uplink / FB 文本反馈（串口）
- 树莓派 -> 客户端：WebSocket 状态广播

协议文档见：`docs/PC_RPI_UDP_PROTOCOL.md`
- 其中 `state.data.feedback` 新增 `link5_hmat` 字段（`link0 -> link5` 的 4x4 齐次矩阵；不可用时为 `null`）。
- `link5_hmat` 使用混合角度来源：前4轴来自反馈角，joint5 来自 `udp.p_rel_deg[4]`。

## 运行方式

依赖清单见 `requirements.txt`。**推荐安装 [uv](https://github.com/astral-sh/uv)**：`scripts/start.sh` 在检测到 `uv` 时，会用  
`uv run --no-project --with-requirements icecreamPi/requirements.txt python -m icecreamPi.main`  
在仓库根目录启动（由 uv 解析并缓存依赖，**不要求**仓库内存在 `.venv`）。未安装 `uv` 时回退为系统 `python3`（需自行保证已安装依赖）。

**首次或更新依赖后**（任选其一）：

```bash
# 推荐：用 uv 按 requirements 同步（可在仓库根或 icecreamPi 目录执行）
cd ~/icecream
uv pip install -r icecreamPi/requirements.txt
# 或仅验证一次导入
uv run --no-project --with-requirements icecreamPi/requirements.txt python -c "import serial, pin; print('ok')"
```

从仓库根目录启动（与 `scripts/start.sh` 中 `cd` 一致）：

```bash
cd ~/icecream
bash icecreamPi/scripts/start.sh
```

**无 uv 时**：请自行 `pip install -r icecreamPi/requirements.txt`（系统或用户 site），再执行上述 `bash`。

启动行为说明：
- 在串口/UDP 循环启动前，运行时会阻塞预热 Pinocchio 重力模型。
- 启动日志会出现：
  - `gravity model warmup start (pinocchio)`
  - `gravity model warmup done in ...s`
- 这会增加一次性启动耗时，但可避免首次重力计算时再触发加载延迟。

Pinocchio 依赖提示：
- 需安装 `pin`（INRIA Pinocchio），不要安装同名但不兼容的 `pinocchio` 包。

## 配置说明（环境变量）

主要配置在 `config.py`，常用项包括：

- UDP：`ARM_CONTROL_RPI_UDP`、`ARM_CONTROL_RPI_UDP_PORT`
- 控制频率：`ARM_CONTROL_TAU_HZ`
- 重力补偿开关：`ARM_CONTROL_GRAVITY_FF`（1 开 / 0 关）
- 重力增益：`ARM_CONTROL_TAU_GAIN`
- WS：`ARM_CONTROL_WS_HOST`、`ARM_CONTROL_WS_PORT`

## 校准与重力数据采集

- 轨迹点文件：`calibration/root.txt`
- 采集脚本：`calibration/collect.py`
- 默认输出：`calibration/gravity.json`
- 采集脚本已支持“每个点位采完即增量写入”，中断后可保留已完成数据。

## 开发约束（README 同步）

每次改代码请遵循：

1. 先读 `README.md` / `README.zh.md`
2. 修改代码
3. 同步更新 README（如果模块职责、结构或行为有变化）
4. 提交前执行检查脚本

```bash
cd ~/icecream/icecreamPi
bash scripts/guard_readme_sync.sh
```

该脚本会在“Python 改动但 README 未更新”时阻断。
