# bridge2pi API 规范（arm_control_bridge -> icecreamPi，v3）

- 文档状态：stable
- 适用模块：`arm_control_bridge/PiController.py`、`icecreamPi/listener.py`、`icecreamPi/infra/udp/packet.py`
- 对端：`arm_control_bridge` -> 树莓派（UDP）
- 相关文档：`docs/head2bridge.md`、`docs/pi2camera.md`、`docs/pi2stm.md`

## 1. 范围（Scope）

本文档定义 bridge 到 Pi 的 UDP 下发协议，包括：

- UDP 传输参数
- 二进制帧结构（**8 维**语义，与 Pi→STM32 `docs/pi2stm.md` v3 对齐的辅机字段）
- 字段装载与控制映射
- 兼容迁移说明（v2 / v2.1 → **v3**）

不包含：

- Pi 端控制律内部实现细节（四电机映射除外）
- UART 42 字节帧的 XOR / MIT 打包细节（见 `docs/pi2stm.md`）

## 2. 传输与端点

- 协议：UDP（单向，无 ACK）
- 目标端口：默认 `9870`（可由 `--rpi-port` 修改）
- 发包频率：默认 `25Hz`
- 发包端：`RPiUDPStreamer`

## 3. 数据结构

### 3.1 固定帧格式（v3，8 维）

- Python `struct`：`=Id` + `d*16`
- 展开后：`seq` + `ts` + **16** 个 `float64`（8 维位置 + 8 维速度）
- 总长度：**`140`** 字节

字节布局（小端，`=` 无对齐填充）：

| 字节范围 | 字段 | 类型 | 说明 |
|---|---|---|---|
| `0..3` | `seq` | `uint32` | 帧序号 |
| `4..11` | `ts` | `float64` | 发送时间戳（秒） |
| `12..75` | `p_rel_deg[0..7]` | `float64[8]` | 8 维位置语义 |
| `76..139` | `omega_rad_s[0..7]` | `float64[8]` | 8 维速度语义 |

### 3.2 8 维语义定义（`p_rel_deg` / `omega_rad_s`）

| 索引 | `p_rel_deg` | `omega_rad_s` |
|---:|---|---|
| `0..3` | 四电机关节（J1~J4），单位 **deg** | 对应关节角速度 **rad/s** |
| `4` | 手腕舵机关节角 `wrist_deg`（deg） | 固定 **`0.0`**（无速度语义） |
| `5` | 夹爪状态 `grip_state`（推荐 **`0=open`**, **`1=close`**） | 固定 **`0.0`** |
| `6` | **步进增量角** `stepper_deg`（deg），语义与 `docs/pi2stm.md` 下行 `stepper_deg` **一致**（`0` = 本周期不追加步进指令） | 固定 **`0.0`** |
| `7` | **传送带启停** `conveyor_run`（浮点 **`0.0≈停`** / **`1.0≈转`**；离散包可用 `0/1`） | 固定 **`0.0`** |

### 3.3 字段装载规则（bridge 发送端）

- `p_rel_deg[0:4] <- JointFrame.arm_rel_deg[0:4]`
- `omega_rad_s[0:4] <- JointFrame.arm_omega_rad_s[0:4]`
- `p_rel_deg[4] <- wrist_deg`
- `omega_rad_s[4] <- 0.0`
- `p_rel_deg[5] <- grip_state`
- `omega_rad_s[5] <- 0.0`
- `p_rel_deg[6] <- stepper_deg_cmd`（与 STM32 下行增量语义一致；无指令时发 **`0.0`**）
- `omega_rad_s[6] <- 0.0`
- `p_rel_deg[7] <- conveyor_run_cmd`（**`0.0`** / **`1.0`**）
- `omega_rad_s[7] <- 0.0`

Pi 侧将 `[6]`、`[7]` 解码并写入 **同一周期** 的 MIT **42 字节**下行帧（与四电机 + 腕/爪舵机拼接发送），见 §4.3。

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

### 4.3 步进与传送带（转发至 `docs/pi2stm.md`）

- `p_rel_deg[6]`：经 Pi **四舍五入为 int** 并限幅到 **`[-180, 180]`**，作为 `encode_mit_cmd_42(..., stepper_deg=...)` 的参数。
- `p_rel_deg[7]`：与夹爪相同阈值规则离散化为 **`0|1`**（默认 **`>= 0.5 -> 1`**），作为 `conveyor_run`。

 UART 层帧长 **42**、校验与电机打包细节见 `docs/pi2stm.md`。

## 5. 错误模型与建议校验

发送端（bridge）：

- `sendto()` 异常：捕获 `OSError` 并忽略（不中断主循环）。

接收端（Pi）建议最小校验：

- 帧长必须为 **`140`** 字节。
- `seq` 连续性检查。
- `p_rel_deg[5]`、`p_rel_deg[7]` 非理想离散值时可阈值归一（例如 `<0.5 -> 0`, `>=0.5 -> 1`）。
- 超时进入 hold / 锁存策略（沿用既有 UDP 逻辑）。

## 6. 最小联调示例

```python
import struct

VECTOR_DIM = 8
FMT = "=Id" + "d" * (VECTOR_DIM * 2)
assert struct.calcsize(FMT) == 140

def decode(pkt: bytes):
    seq, ts, *rest = struct.unpack(FMT, pkt)
    p_rel_deg = rest[:VECTOR_DIM]
    omega_rad_s = rest[VECTOR_DIM:]
    return seq, ts, p_rel_deg, omega_rad_s
```

## 7. 迁移说明（旧版 -> v3）

| 版本 | 维度 | 总长 | 变更摘要 |
|---|---:|---:|---|
| v2 | 5 | 92 | 无腕部专用维（历史） |
| v2.1 | 6 | 108 | +夹爪 `p_rel_deg[5]` |
| **v3** | **8** | **140** | +`stepper_deg`、`conveyor_run`，与 `docs/pi2stm.md` v3 下行辅段对齐 |

- 若接收端仍按 **108** 字节解析，将直接丢帧；升级时需 **同步** bridge 与 icecreamPi。
- Pi / STM32 串口须已升级到 **42** 字节 MIT v3（见 `docs/pi2stm.md` §8）。

## 8. 交叉引用

- 上游命令协议：`docs/head2bridge.md`
- 相机链路协议：`docs/pi2camera.md`
- 串口链路协议：`docs/pi2stm.md`
