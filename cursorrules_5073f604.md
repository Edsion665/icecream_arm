# 机械臂控制框架 - 树莓派端开发规则

## 项目概述

这是一个自制机械臂的树莓派端控制框架。整体链路：

```
主机 (Python) <--WebSocket--> 树莓派 (Python) <--串口--> STM32 <--CAN/串口--> 电机/舵机
                                    |
                              USB 深度相机
```

树莓派负责：
- 与 STM32 串口通信（收发电机状态和控制指令）
- 为主机提供 WebSocket 接口（状态上报 + 命令接收）
- 采集深度相机画面并推送给主机

## 技术栈

- Python 3.11+，使用 asyncio 作为主事件循环
- `pyserial` / `aioserial` 处理串口通信
- `websockets` 库实现 WebSocket 服务端
- 相机 SDK 由具体硬件决定，预留接口即可
- 不使用 ROS，这是一个轻量独立框架

## 项目结构

```
arm_control/
├── main.py              # 入口，启动所有模块
├── config.py            # 全局配置（串口、网络、相机参数）
├── protocol.py          # 串口帧协议：封帧、解帧、CRC16
├── serial_manager.py    # 串口收发管理，线程中运行
├── ws_server.py         # WebSocket 服务端
├── camera_manager.py    # 相机采集与编码
└── state_store.py       # 线程安全的全局状态管理
```

## 串口通信协议

帧格式（二进制，小端序）：

```
| 帧头 (2B)   | 数据长度 (1B) | 命令ID (1B) | 数据 (0~250B) | CRC16 (2B) |
| 0xAA 0x55   |     len       |     cmd     |    payload    |    crc     |
```

- 帧头固定为 `0xAA 0x55`，用于帧同步
- len 是 payload 的字节数，不包含帧头、命令ID、CRC
- CRC16 校验范围：len + cmd + payload
- 接收端使用**有限状态机**逐字节解析，状态依次为：WAIT_HEAD1 -> WAIT_HEAD2 -> WAIT_LEN -> WAIT_CMD -> WAIT_DATA -> WAIT_CRC
- 波特率 115200，数据位 8，停止位 1，无校验
- 通信频率 10-50Hz

### 命令ID 约定（根据实际需求扩展）

- `0x01` 心跳
- `0x10` 电机状态上报（STM32 -> RPi）
- `0x11` 电机控制指令（RPi -> STM32）
- `0x20` 舵机状态上报
- `0x21` 舵机控制指令

## 网络通信

- WebSocket 服务端监听 `0.0.0.0:8765`
- 消息格式统一用 JSON
- 上行（RPi -> 主机）：状态推送、相机帧（base64 JPEG）
- 下行（主机 -> RPi）：控制指令
- 消息结构示例：

```json
// 状态推送
{"type": "state", "data": {"motors": [...], "servos": [...]}}

// 相机帧
{"type": "camera", "data": "<base64 encoded JPEG>"}

// 控制指令
{"type": "command", "cmd": "move", "data": {"joint": 1, "angle": 90}}
```

## 编码规范

### 必须遵守

- 所有异步函数用 `async def`，不要在 asyncio 循环中做阻塞调用
- 串口读写在**独立线程**中运行，通过 `asyncio.run_coroutine_threadsafe` 或 `queue.Queue` 与主循环交互
- 共享状态必须加锁（`threading.Lock` 或 `asyncio.Lock`）
- 所有二进制数据处理用 `struct` 模块，不要手动拼接字节
- CRC 计算必须有单元测试覆盖
- 异常不能静默吞掉，至少要 `logging.error` 记录

### 风格要求

- 类型注解：所有函数签名必须有 type hints
- 日志：统一用 `logging` 模块，不要用 `print`
- 配置：所有可调参数集中在 `config.py`，不要硬编码
- 命名：模块名 snake_case，类名 PascalCase，常量 UPPER_SNAKE_CASE
- 每个模块开头写 docstring 说明职责

### 禁止事项

- 不要用全局变量传递状态，必须通过 `StateStore` 实例
- 不要在协议解析中使用正则表达式，必须用状态机
- 不要把相机 SDK 的调用散落在多个模块中，统一在 `camera_manager.py`
- 不要在串口线程里直接操作 WebSocket，跨模块通信走状态池或消息队列

## 错误处理

- 串口断开后自动重连，重连间隔 2 秒，记录日志
- WebSocket 客户端断开后清理资源，不影响其他客户端
- CRC 校验失败的帧直接丢弃，计数器 +1，用于诊断
- 相机采集失败不应导致整个程序崩溃

## 测试

- 串口协议的封帧/解帧/CRC 必须有单元测试
- 可以用 `pytest` 框架
- 提供一个 `mock_serial.py` 用于在没有硬件时模拟串口数据

## 运行环境

- 树莓派 4B / 5，64 位 Raspberry Pi OS (Bookworm)
- Python 3.11+
- 串口设备通常为 `/dev/ttyAMA0` 或 `/dev/ttyUSB0`
- 用户已加入 `dialout` 组，有串口访问权限
