# arm_control_bridge

[English](./README.md) | [中文](./README_zh.md)

`arm_control_bridge` 是 4 轴主臂 + 抓手通道的控制桥接模块：

- 上层 -> Bridge：TCP/HTTP 命令
- Bridge -> 树莓派：UDP 二进制帧
- 可选 Isaac Sim 运行模式（`--sim`）

## 架构概览

上层发送命令（`pose/joints/claw`）-> `listener.py` 做命令归一化 ->
`calculator.py` 生成关节目标 -> `PiController.py` 打包并下发 V2.1 UDP 帧 ->
Pi 侧执行电机/舵机控制。仿真模式下，同一命令流也驱动 Isaac Sim。

## 接口说明

### 1) TCP（JSON 行）

- 默认监听：`0.0.0.0:9888`
- 每行一条 JSON（`\n` 分隔）
- 主要参数：`--listen`、`--port`

### 2) HTTP（JSON）

- `--web-port > 0` 时启用
- `run_control.py` 默认：`127.0.0.1:8765`
- `start_pc_control.sh` 脚本默认：`0.0.0.0:8877`
- 路由：
  - `POST /api/pose`
  - `POST /api/pose_delta`
  - `POST /api/joints`
  - `POST /api/joints_delta`
  - `POST /api/claw`

### 3) 下发到 Pi 的 UDP

- 默认目标端口：`9870`（`--rpi-port`）
- 协议：固定 V2.1 `108B`（`=Id + d*12`）
- 默认控制频率：`25Hz`
- 已移除 UDP v1 路径。

详细字段请看：`doc/head2bridge.md` 与 `doc/bridge2pi.md`。

## 资源加载策略（仅 configuration）

- 默认 URDF 仅从 `arm_control_bridge/configuration/` 加载：
  - `ice_cream_v8.SLDASM.urdf`（优先）
  - `ice_cream_SINGLE.SLDASM.urdf`
- 默认 USD（`--sim`）仅从 `arm_control_bridge/configuration/` 加载：
  - `ice_cream_v8_arm.usd`
  - `ice_cream_single_arm.usd`
  - `ice_cream_arm.usd`
- 仍支持显式 `--urdf`、`--sim-usd` 覆盖路径。
- 若默认文件缺失且未显式覆盖，启动会快速失败并给出明确报错。

## 命令格式（高频摘要）

- `pose`：必填 `x, y, z`（米）
- `pose_delta`：必填 `dx, dy, dz`（米）
- `joints`：必填 `axes_rel_deg`（长度 4 数组）
- `joints_delta`：必填 `deltas_rel_deg`（长度 4 数组）
- `claw`：必填 `wrist_deg` + (`grip_state` 或 `open_close`)
- `stop` / `ping`：TCP 命令路径支持

最小 TCP 示例：

```json
{"cmd":"joints","axes_rel_deg":[0,10,-90,-70]}
{"cmd":"claw","wrist_deg":20,"open_close":"close"}
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
```

## 文件与文件夹职责

### 核心运行入口

- `run_control.py`：主入口、参数解析、sim/nosim 主循环编排
- `start_pc_control.sh`：一键启动脚本（sim 或 nosim）

### 命令入口与协议 I/O

- `listener.py`：TCP/HTTP 服务与命令归一化
- `PiController.py`：UDP 帧打包与下发

### 运动学与控制

- `calculator.py`：IK/FK、命令应用、控制帧生成
- `shower.py`：仿真侧状态显示/可视化辅助

### 配置与资源

- `config.py`：默认端口、频率、限幅、标定加载
- `configuration/`：运行时资源（例如默认 URDF）
- `web/`：测试网页（`index.html`）

### 文档

- `doc/head2bridge.md`：上层 -> bridge API 规范
- `doc/bridge2pi.md`：bridge -> Pi UDP 规范
- `doc/pi2camera.md`：相机链路相关协议说明

## 快速启动

### 一键脚本

```bash
# 仿真 + 下发
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# 无仿真开环下发
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100
```

## 最小联调

```bash
python3 - <<'PY'
import socket, json
s = socket.create_connection(("127.0.0.1", 9888))
for cmd in [
    {"cmd":"pose","x":0.35,"y":0.2,"z":0.25},
    {"cmd":"joints","axes_rel_deg":[0,0,5,0]},
    {"cmd":"claw","wrist_deg":15,"grip":0.5},
]:
    s.sendall((json.dumps(cmd) + "\n").encode("utf-8"))
s.close()
print("done")
PY
```
