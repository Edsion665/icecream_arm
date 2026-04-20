# arm_control_bridge

`arm_control_bridge` 是 4 轴主臂 + 抓手通道的控制桥接模块，提供：

- 上层到 bridge 的 TCP/HTTP 命令入口
- bridge 到树莓派的 UDP 二进制下发
- 可选 Isaac Sim 仿真运行路径

## 文档索引

- 上层 -> bridge 接口规范：`doc/head2bridge.md`
- bridge -> Pi 接口规范：`doc/bridge2pi.md`

## 快速启动

在仓库根目录执行。

### 无仿真（开环 + UDP 下发）

```bash
python3 -m arm_control_bridge.run_control \
  --listen 0.0.0.0 \
  --port 9888 \
  --rpi-ip 192.168.1.100 \
  --udp-format v2
```

### Isaac Sim 仿真

```bash
~/isaac-sim/python.sh -m arm_control_bridge.run_control \
  --sim \
  --web-port 8765 \
  --web-host 127.0.0.1
```

### 一键脚本

```bash
# 仿真 + 下发
./arm_control_bridge/start_pc_control.sh sim 192.168.1.100

# 无仿真开环下发
./arm_control_bridge/start_pc_control.sh nosim 192.168.1.100
```

## 最小联调（TCP）

```bash
python3 - <<'PY'
import socket, json
s = socket.create_connection(("127.0.0.1", 9888))
for cmd in [
    {"cmd":"pose","x":0.35,"y":0.2,"z":0.25},
    {"cmd":"joints","axes_rel_deg":[0,0,5,0]},
    {"cmd":"claw","wrist_deg":15,"grip":0.5},
]:
    s.sendall((json.dumps(cmd) + "\\n").encode("utf-8"))
s.close()
print("done")
PY
```

