# arm_control_bridge

[English](./README.md) | [中文](./README_zh.md)

`arm_control_bridge` 是 4 轴主臂 + 抓手通道的控制桥接模块：

- 上层策略 → Bridge：TCP / HTTP 命令
- Bridge → 树莓派：UDP 二进制帧（v3，140B，25Hz）
- Bridge ← 树莓派：WebSocket 状态广播（pi2camera v1，用于到位判定）
- 可选 Isaac Sim 运行模式（`--sim`）

## 架构概览

```
上层策略
  │  TCP JSON行 / HTTP POST
  ▼
io/listener.py  ───────────────────────────────────────────────────
  CommandNormalizer 归一化命令                                      │
  register_pending(tcp_conn / http_slot)  ← 预注册回传上下文        │
  │                                                                 │
  ▼                                                                 │
run_control.py 控制循环（25Hz）                                     │
  runtime/reach_tracker  ← 到位追踪 + 异步回传队列                   │
  engine.apply_command() ← 更新 IK/关节目标                         │
  engine.step()          ← 生成 JointFrame                          │
  tracker.feed(reached, result)                                     │
  │                                                                 │
  ├─ io/pi_controller.py → UDP 140B → 树莓派                        │
  │                                                                 │
  └─ reply_q (daemon 线程)                                          │
       conn.sendall / slot.event.set  ← 异步回传，不阻塞控制循环    │
                                                                    │
io/pi_feedback.py (daemon 线程)                                       │
  WebSocket 订阅 ws://rpi_ip:8765                                   │
  解析 feedback.fb_arm_rad / mit_arm_rad                            │
  get_fb_arm_rad() → 供 is_reached() 使用实机关节角 ───────────────┘
```

**到位判定数据源优先级**：有树莓派 WebSocket 回传时用实机关节角；无回传或仿真模式时用 `state.q_cmd`（仿真模式用 `arm.get_joint_positions()`）。

## 接口说明

### 1) TCP（JSON 行）

- 默认监听：`0.0.0.0:9888`
- 每行一条 JSON（`\n` 分隔）
- 到位后 bridge 在**同一连接**写回一行 JSON（见 §到位回传）

### 2) HTTP（JSON）

- **无仿真**：`CONFIG.web_test_port > 0`（默认 `8877`）时启用，绑定 `CONFIG.web_test_host`（默认 `0.0.0.0`）。
- **仿真**：由 `SIM_CONFIG.sim_web_port` / `SIM_CONFIG.sim_web_host` 控制。
- 启动时日志会打印**可在浏览器打开的 URL**（绑定 `0.0.0.0` 时用本机局域网 IP 提示，因浏览器无法直接访问 `http://0.0.0.0:8877/`）。
- 命令**阻塞直到到位**再返回（超时见 `CONFIG.reached_timeout_s`，超时返回 408）。
- 路由：`POST /api/pose` · `/api/pose_delta` · `/api/joints` · `/api/joints_delta` · `/api/claw`

### 3) 下发到 Pi 的 UDP

- 默认目标端口：`9870`（`CONFIG.default_udp_port`，可用 `--rpi-port` 覆盖）
- 协议：固定 v3 `140B`（`=Id + d*16`，8 维位置+速度），25Hz
- 详见 `doc/bridge2pi.md`

### 4) 接收 Pi 回传（WebSocket）

- 订阅 `ws://rpi_ip:8765`（pi2camera v1 协议）
- 解析 `feedback.fb_arm_rad`（优先）或 `feedback.mit_arm_rad`
- 在指定了 `--rpi-ip`（或默认 `CONFIG.rpi_ip`）时启动，断线重连间隔见 `CONFIG.pi_feedback_reconnect_interval_s`
- 无回传时静默降级，到位判定自动切换为内部指令角

## 到位回传格式（head2bridge v2.2）

命令执行完成后，bridge 向上层回传结果。格式如下：

**joints / joints_delta 到位：**
```json
{"ok": true, "reached": true, "error_joints_deg": 1.2}
```

**pose / pose_delta 到位：**
```json
{"ok": true, "reached": true, "actual_pose": {"x": 0.349, "y": 0.201, "z": 0.248}, "error_pose_m": 0.003}
```

**claw 到位（2s 定时）：**
```json
{"ok": true, "reached": true}
```

**超时（10s）：**
```json
{"ok": false, "reached": false, "error": "timeout"}
```

到位判定规则：
- `joints`：每轴误差均 < 5°（任一轴超出即不到位）
- `pose`：link4 FK 位置误差范数 < 5mm
- `claw`：无硬件回传，接收命令后 2s 自动判定到位
- 稳定缓冲：连续 5 帧（@25Hz ≈ 200ms）全部满足阈值才触发回传，防止瞬间抖动误判

## 文件职责

### 目录与职责（按功能分包）

| 路径 | 职责 |
|------|------|
| `run_control.py` | CLI 入口、`run_loop` / `run_sim_loop` 编排 |
| `config.py` / `exceptions.py` | 全局配置与桥接层异常 |
| `calculator.py` | 对 `control/`、`kinematics/` 的聚合导出 |
| `control/`、`kinematics/` | 状态机、IK、URDF 运动学 |
| `io/` | TCP/HTTP（`listener.py`）、树莓派 UDP（`pi_controller.py`）、WS 回传（`pi_feedback.py`） |
| `sim/` | Isaac 场景引导（`bootstrap.py`）、关节可视化（`shower.py`） |
| `runtime/` | 到位追踪与回传线程（`reach_tracker.py`）、UDP 帧调试（`udp_debug.py`） |
| `PiController.py` | 兼容旧导入，转发至 `io.pi_controller` |

### 命令入口与协议 I/O（`io/`）

- `io/listener.py`：TCP/HTTP、`CommandNormalizer`、`ReplySlot`、`on_pending`
- `io/pi_controller.py`：UDP v3 帧与 `motor` / `servoMotor`
- `io/pi_feedback.py`：`PiFeedbackClient`（WebSocket）

### 运动学与控制

- `calculator.py`：对外稳定导入；实现见 `control/`、`kinematics/`
- `sim/shower.py`：仿真 `ArticulationViewer` / `FrameReceiver`

### 配置与资源

- `config.py`：以 **`CONFIG`**（控制/网络/HTTP 调试/到位）、**`IK_CONFIG`**（IK）、**`SIM_CONFIG`**（仿真 USD、仿真 HTTP、PD 等）、**`RUNTIME`**（如 `udp_strict`）单例暴露默认值；少用的参数改这里，不必再堆 CLI。
- `configuration/`：URDF、USD 资源（如 `configuration/v8/`）
- `web/`：测试网页（`index.html`）

### 文档

- `doc/head2bridge.md`：上层 → bridge API 规范（v2.2，含到位回传）
- `doc/bridge2pi.md`：bridge → Pi UDP 规范（v3）
- `doc/pi2camera.md`：Pi → 相机/仿真 WebSocket 广播规范

## 命令行（CLI）

仅保留网络相关参数，其余见 `config.py`：

- `--sim`：启动 Isaac Sim
- `--listen`：TCP 监听地址（默认 `CONFIG.listen_host`）
- `--port`：TCP JSON 端口（默认 `CONFIG.default_tcp_port`，一般为 `9888`）
- `--rpi-ip`：树莓派 IP（默认 `CONFIG.rpi_ip`；不设则不下发 UDP）
- `--rpi-port`：树莓派 UDP 端口（默认 `CONFIG.default_udp_port`，一般为 `9870`）

无头仿真、HTTP 绑定、IK/USD 路径等：改 **`SIM_CONFIG`** / **`CONFIG`** / **`IK_CONFIG`**。

## 快速启动

```bash
# 仿真 + 下发到树莓派（在仓库根目录执行，例如 icecreamarm/）
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# 无仿真开环下发
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100

# 仅本地仿真（无树莓派）；HTTP/无头等在 SIM_CONFIG
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim

# 显式指定监听与树莓派（可选）
~/isaacsim/python.sh -m arm_control_bridge.run_control --sim --listen 0.0.0.0 --port 9888 --rpi-ip 192.168.1.100
```

在仓库根目录（包含 `arm_control_bridge/` 子目录，与 Python 包名一致）下执行上述命令。

## 与树莓派对接方案

### 下行（Bridge → Pi）

Bridge 以 `CONFIG.control_hz` 向 Pi 发送 UDP 140B 帧；`--rpi-ip` 或默认 `CONFIG.rpi_ip` 指定 Pi 地址。HTTP 测试页另支持 `POST /api/stepper`、`POST /api/conveyor`。

### 上行（Pi → Bridge，用于到位判定）

Pi 运行 `icecreamPi` 服务，通过 WebSocket 广播状态（`ws://pi_ip:8765`，约 20Hz）。Bridge 启动时若指定了 `--rpi-ip`，会自动启动 `PiFeedbackClient` 订阅该广播。

Pi 广播消息格式（`pi2camera v1`）：
```json
{
  "type": "state",
  "data": {
    "feedback": {
      "fb_arm_rad": [q1, q2, q3, q4],
      "mit_arm_rad": [q1, q2, q3, q4]
    }
  }
}
```

Bridge 优先使用 `fb_arm_rad`，回退 `mit_arm_rad`，两者均无时降级为内部指令角。

### 联调验证

```bash
# 发送 joints 命令，等待到位回传
curl -s -X POST http://127.0.0.1:8877/api/joints \
  -H "Content-Type: application/json" \
  -d '{"axes_rel_deg":[0,90,-180,-20]}'
# 返回：{"ok":true,"reached":true,"error_joints_deg":0.8}

# 发送 pose 命令
curl -s -X POST http://127.0.0.1:8877/api/pose \
  -H "Content-Type: application/json" \
  -d '{"x":-0.05,"y":0.05,"z":0.28}'
# 返回：{"ok":true,"reached":true,"actual_pose":{"x":-0.05,"y":0.05,"z":0.28},"error_pose_m":0.002}

# TCP 路径（到位后同一连接收到回传行）
python3 -c "
import socket, json, time
s = socket.create_connection(('127.0.0.1', 9888))
s.sendall((json.dumps({'cmd':'joints','axes_rel_deg':[0,90,-180,-20]}) + '\n').encode())
s.settimeout(15)
print('reply:', s.recv(4096).decode())
s.close()
"
```

## 命令格式摘要

| 命令 | 必填字段 | 说明 |
|---|---|---|
| `pose` | `x, y, z`（米） | link4 笛卡尔目标，阻塞到位 |
| `pose_delta` | `dx, dy, dz`（米） | 笛卡尔增量，阻塞到位 |
| `joints` | `axes_rel_deg`（长度 4） | 关节绝对目标（相对标定零位，度），阻塞到位 |
| `joints_delta` | `deltas_rel_deg`（长度 4） | 关节增量，阻塞到位 |
| `claw` | `wrist_deg` + `grip_state`/`open_close` | 手腕角 + 夹爪状态，2s 后回传 |
| `stop` / `ping` | — | 急停记录 / 连通检查 |
