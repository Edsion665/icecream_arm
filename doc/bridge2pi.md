# bridge2pi API 规范（arm_control_bridge -> icecreamPi，v2.1）

- 文档状态：stable
- 适用模块：`arm_control_bridge/io/pi_controller.py`（兼容导入：`arm_control_bridge.PiController`）、`icecreamPi/listener.py`
- 对端：`arm_control_bridge` -> 树莓派（UDP）
- 相关文档：`docs/head2bridge.md`、`docs/pi2camera.md`

## 1. 范围（Scope）

本文档定义 bridge 到 Pi 的 UDP 下发协议，包括：

- UDP 传输参数
- 二进制帧结构（6 维语义）
- 字段装载与控制映射
- 兼容迁移说明（5 维 -> 6 维）

不包含：

- Pi 端控制律内部实现
- Pi 到 STM32 的串口帧（见 `docs/pi2stm.md`）

## 2. 传输与端点

- 协议：UDP（单向，无 ACK）
- 目标端口：默认 `9870`（可由 `--rpi-port` 修改）
- 发包频率：默认 `25Hz`
- 发包端：`RPiUDPStreamer`

## 3. 数据结构

### 3.1 固定帧格式（V2.1，6 维）

- Python `struct`：`=Id` + `d*12`
- 展开后：`=Idddddddddddd`
- 总长度：`108` 字节

字节布局（小端，`=` 无对齐填充）：

| 字节范围 | 字段 | 类型 | 说明 |
|---|---|---|---|
| `0..3` | `seq` | `uint32` | 帧序号 |
| `4..11` | `ts` | `float64` | 发送时间戳（秒） |
| `12..59` | `p_rel_deg[0..5]` | `float64[6]` | 6 维位置语义 |
| `60..107` | `omega_rad_s[0..5]` | `float64[6]` | 6 维速度语义 |

| 顺序 | 字段 | 类型 | 单位 | 说明 |
|---:|---|---|---|---|
| 1 | `seq` | `uint32` | - | 帧序号，每帧自增 |
| 2 | `ts` | `float64` | s | 发送时间戳 |
| 3~8 | `p_rel_deg[6]` | `float64[6]` | deg | 6 维目标角语义 |
| 9~14 | `omega_rad_s[6]` | `float64[6]` | rad/s | 6 维角速度语义 |

### 3.2 6 维语义定义

- `0..3`：4 电机关节（J1~J4）
- `4`：手腕舵机关节角（`wrist_deg`）
- `5`：夹爪状态（`grip_state`，**`0=合拢`，`1=张开`**）

### 3.3 字段装载规则

- `p_rel_deg[0:4] <- JointFrame.arm_rel_deg[0:4]`
- `omega_rad_s[0:4] <- JointFrame.arm_omega_rad_s[0:4]`
- `p_rel_deg[4] <- wrist_deg`（手腕舵机角度）
- `omega_rad_s[4] <- 0.0`（手腕当前不发送角速度，固定保留 0）
- `p_rel_deg[5] <- grip_state`（夹爪离散状态：**`0=合拢`，`1=张开`**）
- `omega_rad_s[5] <- 0.0`（夹爪无角速度语义，固定保留 0）

即：
- **4 个电机**：发送位置 + 角速度；
- **手腕舵机**：只发送角度（速度位填 `0.0`）；
- **夹爪舵机**：只发送 `0/1` 状态（速度位填 `0.0`）。

## 4. 接收与控制语义（Pi 侧）

### 4.1 四电机映射

前 4 轴映射保持不变：

- motor1 = `calibration[0] + (-1) * rad(p_rel_deg[0])`
- motor2 = `calibration[1] + (+1) * rad(p_rel_deg[1])`
- motor3 = `calibration[2] + (-1) * rad(p_rel_deg[2])`
- motor4 = `calibration[3] + (-1) * rad(p_rel_deg[3])`

速度使用相同符号关系。

### 4.2 手腕与夹爪语义

- `p_rel_deg[4]` 作为手腕关节角参与末端姿态估计（FK）。
- `p_rel_deg[5]` 表示夹爪开合状态，不参与 FK。

## 5. 错误模型与建议校验

发送端（bridge）：

- `sendto()` 异常：捕获 `OSError` 并忽略（不中断主循环）。

接收端（Pi）建议最小校验：

- 帧长必须为 `108` 字节。
- `seq` 连续性检查。
- `p_rel_deg[5]` 非 `0/1` 时可采用阈值归一（如 `<0.5 -> 0`, `>=0.5 -> 1`）。
- 超时进入 hold 策略。

## 6. 最小联调示例

```python
import struct

FMT = "=Id" + "d" * 12
assert struct.calcsize(FMT) == 108

def decode(pkt: bytes):
    seq, ts, *rest = struct.unpack(FMT, pkt)
    p_rel_deg = rest[:6]
    omega_rad_s = rest[6:12]
    return seq, ts, p_rel_deg, omega_rad_s
```

## 7. 迁移说明（旧版 -> 新版）

- 旧版（v2）为 5 维（92 字节）：`p_rel_deg[5] + omega_rad_s[5]`。
- 新版（v2.1）为 6 维（108 字节）：新增第 6 维夹爪状态语义。
- 若接收端仍按 92 字节解析，将直接丢帧；升级时需同步修改帧长与数组维度。

## 8. 交叉引用

- 上游命令协议：`docs/head2bridge.md`
- 相机链路协议：`docs/pi2camera.md`
- 串口链路协议：`docs/pi2stm.md`
