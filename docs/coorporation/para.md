# icecream_arm 运行时环境变量（export 调参）

本文档列出 **可通过 `export` 在 shell 中覆盖** 的运行参数。配置在进程启动时由 `config.py` 读取（`os.environ`），**修改后需重启** `python -m icecream_arm.main`（或 `scripts/start.sh`）才生效。

> **来源**：`config.py`（主配置）；辅助脚本另读 `ARM_CONTROL_SERIAL_PORT`、`XDG_STATE_HOME`（见文末）。

---

## 用法说明

### 临时生效（当前终端）

```bash
export ARM_CONTROL_RPI_UDP=1
export ARM_CONTROL_TAU_HZ=25
export ARM_CONTROL_GRAVITY_FF=1
cd /path/to/parent && python3 -m icecream_arm.main
```

### 与 `scripts/start.sh` 的关系

`start.sh` 会对部分变量设 **默认值**（仅当未 export 时）：

| 变量 | start.sh 默认 |
|------|----------------|
| `ARM_CONTROL_RPI_UDP` | `1` |
| `ARM_CONTROL_TAU_HZ` | `25` |
| `ARM_CONTROL_RPI_UDP_PORT` | `9870` |
| `ARM_CONTROL_COLD_HOLD_SEC` | `0` |
| `ARM_CONTROL_MIT_CMD_KP_FLOAT` | `0` |
| `ARM_CONTROL_GRAVITY_FF` | `1` |
| `ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC` | `10`（秒，有限等待） |

直接 `python -m icecream_arm.main` 而不走 `start.sh` 时，上表变量使用 `config.py` 内置默认（例如 UDP 默认 **关闭**、`INIT_FEEDBACK_WAIT` 默认 **无限阻塞**）。

### 布尔型变量

以下任一为真即视为 **开启**：`1`、`true`、`yes`、`on`（大小写不敏感）。  
空字符串或未设置 → 使用文档中的默认值。

### 数值型

- **浮点**：合法浮点字符串；非法则回退默认。
- **整数**：合法整数字符串；非法则回退默认。
- 部分项在代码中有 **上下限裁剪**（见各条「约束」）。

---

## 1. 串口（Pi ↔ STM32）

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_SERIAL_PORT` | `/dev/ttyAMA2` | 字符串 | STM32 串口设备路径。 |
| `ARM_CONTROL_SERIAL_BAUD` | `115200` | 整数 | 串口波特率。 |
| `ARM_CONTROL_SERIAL_RECONNECT_SEC` | `2.0` | 浮点（秒） | 串口断开或 IO 异常后，重开串口前的等待间隔。 |

**用法示例**

```bash
# USB 转串口调试
export ARM_CONTROL_SERIAL_PORT=/dev/ttyUSB0
export ARM_CONTROL_SERIAL_BAUD=115200
```

**作用模块**：`SerialManager`（`serial.py`）；测试脚本 `scripts/test_stepper_mit42.py`、`scripts/toggle_conveyor_mit42.py` 在未传 `--port` 时同样读取本变量。

---

## 2. PC → Pi UDP 关节流

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_RPI_UDP` | `0`（关） | 布尔 | 是否启用 UDP 监听；`1` 开启后 Pi 接收 PC 下发的 140 字节关节包。 |
| `ARM_CONTROL_RPI_UDP_PORT` | `9870` | 整数 | UDP 绑定端口（`0.0.0.0`）。 |
| `ARM_CONTROL_RPI_UDP_STALE_SEC` | `0.35` | 浮点（秒） | 距上一包超过该时间视为 **过期**；若曾收过有效包则 **锁存末帧** 继续控制（见 `controller.py`）。最小裁剪 `0.05`。 |

**用法示例**

```bash
export ARM_CONTROL_RPI_UDP=1
export ARM_CONTROL_RPI_UDP_PORT=9870
export ARM_CONTROL_RPI_UDP_STALE_SEC=0.35
```

**协议**：`docs/communication/PC_RPI_UDP_PROTOCOL.md`（或 `docs/PC_RPI_UDP_PROTOCOL.md`）。

---

## 3. WebSocket 状态服务

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_WS_HOST` | `0.0.0.0` | 字符串 | WS 监听地址。 |
| `ARM_CONTROL_WS_PORT` | `8765` | 整数 | WS 监听端口。 |
| `ARM_CONTROL_STATE_PUSH_SEC` | `0.05` | 浮点（秒） | 状态推送周期；最小 `0.01`。 |

**用法示例**

```bash
export ARM_CONTROL_WS_HOST=0.0.0.0
export ARM_CONTROL_WS_PORT=8765
```

---

## 4. 控制环与 MIT 下发

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_TAU_HZ` | `25.0` | 浮点（Hz） | 主控制环频率（MIT 指令生成周期）。约束：`1.0`～`500.0`。 |
| `ARM_CONTROL_TAU_GAIN` | `1.0` | 浮点 | 重力补偿力矩总增益（Pinocchio 关节力矩 → 电机力矩后的缩放）。 |
| `ARM_CONTROL_MIT_CMD_KP_FLOAT` | `0`（关） | 布尔 | `1` 时 MIT 帧 **kp 全 0**（浮空/弱位置环模式）；`0` 使用下方 `HOLD_KP_*`。 |
| `ARM_CONTROL_HOLD_KP_1` … `_4` | `30, 60, 50, 25` | 浮点 ×4 | 四轴 MIT 位置刚度 kp（motor1～motor4）。 |
| `ARM_CONTROL_HOLD_KD_1` … `_4` | `2, 13, 10, 2` | 浮点 ×4 | 四轴 MIT 阻尼 kd。 |
| `ARM_CONTROL_MAX_SPEED_M1` … `_M4` | `0.5` | 浮点（rad/s）×4 | 单轴最大指令角速度；用于 **StartupSafeGate** 与 **Tracking Ramp**。最小 `1e-4`。 |

**用法示例**

```bash
export ARM_CONTROL_TAU_HZ=25
export ARM_CONTROL_TAU_GAIN=1.0
export ARM_CONTROL_MAX_SPEED_M1=0.3
export ARM_CONTROL_MAX_SPEED_M2=0.3
export ARM_CONTROL_MAX_SPEED_M3=0.3
export ARM_CONTROL_MAX_SPEED_M4=0.3
```

---

## 5. 标定零点（UDP 相对角 → 电机绝对角）

| 环境变量 | 默认值（rad） | 说明 |
|----------|----------------|------|
| `ARM_CONTROL_CAL_R0` | `2.00016` | motor1 标定零位 |
| `ARM_CONTROL_CAL_R1` | `3.6532` | motor2 |
| `ARM_CONTROL_CAL_R2` | `-4.1500` | motor3 |
| `ARM_CONTROL_CAL_R3` | `-2.7514` | motor4 |

映射公式（`domain/mapping.py`）：

`p_cmd[i] = CAL_Ri + MOTOR_AXIS_SIGN[i] × rad(p_rel_deg[i])`

**用法示例**（换臂/重标定后）

```bash
export ARM_CONTROL_CAL_R0=2.00016
export ARM_CONTROL_CAL_R1=3.6532
export ARM_CONTROL_CAL_R2=-4.1500
export ARM_CONTROL_CAL_R3=-2.7514
```

---

## 6. 反馈、重力补偿与安全门

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_TAU_FF_INPUT` | `mit` | 字符串 | 重力补偿与限速 ramp 使用的关节角反馈源：`mit` = 串口 MIT 上行角；`fb` = 文本 `FB` 行反馈角。 |
| `ARM_CONTROL_GRAVITY_FF` | `1`（开） | 布尔 | 是否启用 Pinocchio 重力前馈；`0` 时 MIT 帧 `t` 恒为 0。 |
| `ARM_CONTROL_GRAVITY_BIAS_M1` | `0.0` | 浮点（Nm） | **motor1** 重力前馈固定偏置（Pinocchio 对 J1 理论为 0，实机单向负载用此项补偿）。在 `apply_gravity_motor_output` 中与 `tau_pin[0]` **相加**。 |
| `ARM_CONTROL_GRAVITY_SCALE_M2` | `1.3` | 浮点 | **motor2** Pinocchio 电机空间力矩缩放。 |
| `ARM_CONTROL_GRAVITY_SCALE_M3` | `1.2` | 浮点 | **motor3** 同上。 |
| `ARM_CONTROL_GRAVITY_SCALE_M4` | `1.2` | 浮点 | **motor4** 同上。 |
| `ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC` | 未设 → **∞** | 浮点 / 特殊 | 启动后等待 **首帧串口关节反馈**。`inf`/`none`/空/≤0：无限阻塞；正数：最长等待秒数后仍继续启动（调试用）。`start.sh` 默认 `10`。 |
| `ARM_CONTROL_SERIAL_FEEDBACK_STALE_SEC` | `1.5` | 浮点（秒） | 超过该时间无新串口反馈 → **STM32 冻结**（停发 MIT）；恢复后走 Safe Gate。最小 `0.02`。 |
| `ARM_CONTROL_STARTUP_SAFE_GATE` | `1`（开） | 布尔 | 启动/重连后 **首帧 UDP 目标** 限速逼近（`app/safe_gate.py`）。 |
| `ARM_CONTROL_STARTUP_SAFE_GATE_TOL_RAD` | `0.01` | 浮点（rad） | Safe Gate 到位容差。最小 `1e-4`。 |
| `ARM_CONTROL_STARTUP_SAFE_GATE_TIMEOUT_SEC` | `20.0` | 浮点（秒） | Safe Gate 超时后强制跳到目标角。最小 `0.5`。 |
| `ARM_CONTROL_COLD_HOLD_SEC` | `0.0` | 浮点（秒） | 已在 `config.py` 定义，**当前主程序未引用**；保留供后续或外部脚本使用。 |

**用法示例**

```bash
export ARM_CONTROL_GRAVITY_FF=1
export ARM_CONTROL_GRAVITY_BIAS_M1=0.0
export ARM_CONTROL_GRAVITY_SCALE_M2=1.3
export ARM_CONTROL_GRAVITY_SCALE_M3=1.2
export ARM_CONTROL_GRAVITY_SCALE_M4=1.2
export ARM_CONTROL_TAU_FF_INPUT=mit
export ARM_CONTROL_SERIAL_FEEDBACK_STALE_SEC=1.5
export ARM_CONTROL_STARTUP_SAFE_GATE=1
export ARM_CONTROL_STARTUP_SAFE_GATE_TOL_RAD=0.01
# 调试：最多等 5s 首帧反馈
export ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC=5
```

---

## 7. Pi → 相机 UDP 广播（可选）

| 环境变量 | 默认值 | 类型 | 说明 |
|----------|--------|------|------|
| `ARM_CONTROL_CAMERA_UDP_BROADCAST` | `0`（关） | 布尔 | 是否周期性发送 `camera_state` JSON。 |
| `ARM_CONTROL_CAMERA_UDP_HOST` | `255.255.255.255` | 字符串 | 目标 IP；广播或单播相机地址。 |
| `ARM_CONTROL_CAMERA_UDP_PORT` | `9982` | 整数 | 目标 UDP 端口。 |
| `ARM_CONTROL_CAMERA_UDP_HZ` | `20.0` | 浮点（Hz） | 发送频率；约束 `1.0`～`60.0`。 |
| `ARM_CONTROL_CAMERA_UDP_SO_BROADCAST` | `1`（开） | 布尔 | 是否设置 socket `SO_BROADCAST`；单播到固定 IP 时可设 `0`。 |

**用法示例**

```bash
export ARM_CONTROL_CAMERA_UDP_BROADCAST=1
export ARM_CONTROL_CAMERA_UDP_HOST=192.168.31.100
export ARM_CONTROL_CAMERA_UDP_PORT=9982
export ARM_CONTROL_CAMERA_UDP_HZ=20
```

详见 `docs/communication/pi2camera.md`。

---
生
## 8. 不可通过 export 修改的静态项（需改代码）

以下在 `config.py` **写死**，无环境变量；协作调参时需改仓库或提 PR：

| 名称 | 位置 | 说明 |
|------|------|------|
| `MOTOR_AXIS_SIGN` | `config.py` | 四轴电机/关节符号 `(−1,−1,−1,+1)` |
| `GRAVITY_TAU_BIAS_M1` / `GRAVITY_AXIS_SCALE_M234` | `config.py` | 可由上表 `ARM_CONTROL_GRAVITY_*` 覆盖；经 `apply_gravity_motor_output()` 写入 MIT `t` |
| `SwitchGateConfig` | `config.py` | ttyAMA4 开关闭合 → pi2head（端口/主机等） |
| 腕/夹爪/步进/传送带 UDP 语义常量 | `config.py` | 如 `GRIP_*`、`WRIST_*_DEG` |

---

## 9. 辅助脚本相关（非主控 CONFIG）

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `ARM_CONTROL_SERIAL_PORT` | 同上 | `scripts/test_stepper_mit42.py`、`scripts/toggle_conveyor_mit42.py` 未指定 `--port` 时使用。 |
| `XDG_STATE_HOME` | `~/.local/state` | 仅 `toggle_conveyor_mit42.py` 用于保存传送带状态文件路径。 |

---

## 10. 常用组合示例

### 生产启动（与 start.sh 一致）

```bash
export ARM_CONTROL_RPI_UDP=1
export ARM_CONTROL_TAU_HZ=25
export ARM_CONTROL_GRAVITY_FF=1
export ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC=10
bash scripts/start.sh
```

### 关闭重力、仅位置跟踪调试

```bash
export ARM_CONTROL_GRAVITY_FF=0
export ARM_CONTROL_TAU_FF_INPUT=mit
export ARM_CONTROL_RPI_UDP=1
python3 -m icecream_arm.main
```

### 标定采集（tra_collect 会 setdefault UDP=1）

```bash
export ARM_CONTROL_CAL_R0=...
export ARM_CONTROL_CAL_R1=...
# tra_collect.py 内 os.environ.setdefault("ARM_CONTROL_RPI_UDP", "1")
python3 -m icecream_arm.calibration.tra_collect
```

---

## 11. 环境变量速查表（按字母序）

| 环境变量 | 默认 | 单位/类型 |
|----------|------|-----------|
| `ARM_CONTROL_CAL_R0` | 2.00016 | rad |
| `ARM_CONTROL_CAL_R1` | 3.6532 | rad |
| `ARM_CONTROL_CAL_R2` | -4.1500 | rad |
| `ARM_CONTROL_CAL_R3` | -2.7514 | rad |
| `ARM_CONTROL_CAMERA_UDP_BROADCAST` | 0 | bool |
| `ARM_CONTROL_CAMERA_UDP_HOST` | 255.255.255.255 | string |
| `ARM_CONTROL_CAMERA_UDP_HZ` | 20 | Hz |
| `ARM_CONTROL_CAMERA_UDP_PORT` | 9982 | int |
| `ARM_CONTROL_CAMERA_UDP_SO_BROADCAST` | 1 | bool |
| `ARM_CONTROL_COLD_HOLD_SEC` | 0 | s（未接线） |
| `ARM_CONTROL_GRAVITY_BIAS_M1` | 0.0 | Nm |
| `ARM_CONTROL_GRAVITY_FF` | 1 | bool |
| `ARM_CONTROL_GRAVITY_SCALE_M2` | 1.3 | float |
| `ARM_CONTROL_GRAVITY_SCALE_M3` | 1.2 | float |
| `ARM_CONTROL_GRAVITY_SCALE_M4` | 1.2 | float |
| `ARM_CONTROL_HOLD_KP_1` … `_4` | 30,60,50,25 | float |
| `ARM_CONTROL_HOLD_KD_1` … `_4` | 2,13,10,2 | float |
| `ARM_CONTROL_INIT_FEEDBACK_WAIT_SEC` | ∞ | s / inf |
| `ARM_CONTROL_MAX_SPEED_M1` … `_M4` | 0.5 | rad/s |
| `ARM_CONTROL_MIT_CMD_KP_FLOAT` | 0 | bool |
| `ARM_CONTROL_RPI_UDP` | 0 | bool |
| `ARM_CONTROL_RPI_UDP_PORT` | 9870 | int |
| `ARM_CONTROL_RPI_UDP_STALE_SEC` | 0.35 | s |
| `ARM_CONTROL_SERIAL_BAUD` | 115200 | int |
| `ARM_CONTROL_SERIAL_FEEDBACK_STALE_SEC` | 1.5 | s |
| `ARM_CONTROL_SERIAL_PORT` | /dev/ttyAMA2 | string |
| `ARM_CONTROL_SERIAL_RECONNECT_SEC` | 2.0 | s |
| `ARM_CONTROL_STARTUP_SAFE_GATE` | 1 | bool |
| `ARM_CONTROL_STARTUP_SAFE_GATE_TIMEOUT_SEC` | 20.0 | s |
| `ARM_CONTROL_STARTUP_SAFE_GATE_TOL_RAD` | 0.01 | rad |
| `ARM_CONTROL_STATE_PUSH_SEC` | 0.05 | s |
| `ARM_CONTROL_TAU_FF_INPUT` | mit | mit \| fb |
| `ARM_CONTROL_TAU_GAIN` | 1.0 | float |
| `ARM_CONTROL_TAU_HZ` | 25.0 | Hz |
| `ARM_CONTROL_WS_HOST` | 0.0.0.0 | string |
| `ARM_CONTROL_WS_PORT` | 8765 | int |

---

*文档与 `config.py` 同步；若新增环境变量请同时更新本文件。*
