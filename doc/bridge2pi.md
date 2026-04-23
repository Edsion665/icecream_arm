# bridge2pi API 规范（arm_control_bridge ie v1）

- 文档状态：stable
- 适用模块：`arm_control_bridge/PiController.py`
- 对端：`arm_control_bridge` -> 树莓派（UDP）
- 相关文档：`doc/head2bridge.md`

## 1. 范围（Scope）

本文档定义 control bridge 到 Pi 的 UDP 下发协议，包括：

- UDP 传输参数
- 二进制帧结构
- 字段语义与映射
- 兼容与演进规则

不包含：

- Pi 端控制律内部实现
- 上层到 bridge 的命令语义（见 `doc/head2bridge.md`）

## 2. 传输与端点

- 协议：UDP（单向，无 ACK）
- 目标端口：默认 `9870`（可由 `--rpi-port` 修改）
- 发包端：`RPiUDPStreamer`
- 发包频率：跟随控制循环（默认约 25Hz）

说明：`--udp-format v1|v2` 当前都发送同一 V2 固定帧。

## 3. 数据结构

### 3.1 固定帧格式

- Python `struct`：`=Idddddddddd`
- 总长度：`92` 字节

| 顺序 | 字段 | 类型 | 单位 | 约束 | 说明 |
|---:|---|---|---|---|---|
| 1 | `seq` | `uint32` | - | 每帧自增 | 帧序号 |
| 2 | `ts` | `float64` | s | `time.monotonic()` | 发送时间 |
| 3~7 | `p_rel_deg[5]` | `float64[5]` | deg | 固定 5 维 | 相对标定角 |
| 8~12 | `omega_rad_s[5]` | `float64[5]` | rad/s | 固定 5 维 | 关节角速度 |

### 3.2 字段装载规则（当前实现）

- `p_rel_deg[0:4] <- JointFrame.arm_rel_deg[0:4]`
- `omega_rad_s[0:4] <- JointFrame.arm_omega_rad_s[0:4]`
- `p_rel_deg[4] <- JointFrame.joint5_rel_deg`
- `omega_rad_s[4]` 当前通常为 `0.0`（预留）

## 4. 发送与接收语义

### 4.1 发送语义

`RpiProtocolAdapter.send_frame(frame)` 会将 `JointFrame` 适配为 5 轴数组后发包。

抓手字段说明：

- `frame.servo_deg` 只在适配器内部缓存为 `_last_servo`。
- 当前 V2 帧不含 `servo_deg` 字段。
- `servoMotor.send()` 仅刷新缓存，不改变 UDP 帧结构。

### 4.2 接收侧约定（兼容历史）

若 Pi 端沿用历史映射，前 4 轴可按如下符号关系转电机角：

- motor1 = `calibration[0] + (-1) * rad(p_rel_deg[0])`
- motor2 = `calibration[1] + (+1) * rad(p_rel_deg[1])`
- motor3 = `calibration[2] + (-1) * rad(p_rel_deg[2])`
- motor4 = `calibration[3] + (-1) * rad(p_rel_deg[3])`

速度使用相同符号关系。

## 5. 错误模型

发送端行为（`PiController.py`）：

- `sendto()` 失败：捕获 `OSError` 并忽略（不中断主循环）。
- 本协议无应用层 ACK/错误回包。

接收端建议最小校验：

- 帧长必须为 92 字节
- `seq` 连续性检查
- 超时降级策略（保持/置零）

## 6. 兼容性与版本演进

- 当前版本：V2（固定 92 字节）
- 向后兼容：保留 `--udp-format` 参数名，但底层统一 V2
- 演进建议：
  - 若需下发抓手，可扩展 V3（增加 `servo_deg` 字段）
  - 或使用独立第二路 UDP 通道下发抓手

## 7. 最小联调示例

Python 解析示例（Pi 侧）：

```python
import struct

FMT = "=Id" + "d" * 10
assert struct.calcsize(FMT) == 92

def decode(pkt: bytes):
    seq, ts, *rest = struct.unpack(FMT, pkt)
    p_rel_deg = rest[:5]
    omega_rad_s = rest[5:]
    return seq, ts, p_rel_deg, omega_rad_s
```

## 8. 交叉引用

- 上游命令协议：`doc/head2bridge.md`
- 关键代码：
  - `arm_control_bridge/PiController.py`
  - `arm_control_bridge/calculator.py`（`JointFrame` 定义）
