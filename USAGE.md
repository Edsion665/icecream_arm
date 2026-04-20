# sim_test 使用手册

本手册对应新架构目录：`icecream_arm/sim_test`（完全独立，不依赖 `sim_control` 模块）。

## 1. 目录说明

- `listener.py`：统一监听层（`frontend_listener`/`network_listener`/`claw_listener`）
- `calculator.py`：计算层（`IK_calculator` + `mover` + `CalculatorEngine`）
- `shower.py`：仿真显示层（`receiver` + `show`）
- `PiController.py`：树莓派下发层（`motor` + `servoMotor`）
- `start_pc_control.sh`：电脑端一键启动脚本
- `config.py`：控制参数 + 零点标定加载（`load_calibration_deg`）
- `run_control.py`：统一启动入口
- `head2controller_doc.md`：上层到控制层接口定义
- `run_control.py` 内含开环与仿真运行循环、HTTP 服务与网络监听编排
- `listener.py` 内含 TCP 兼容入口与 HTTP 服务（已融合 command/http）
- `PiController.py` 内含 UDP 协议打包（已融合 pi_stream）
- `web/index.html`：`sim_test` 自带前端测试页

## 2. 启动方式

在 `icecream_arm` 目录下：

### 2.1 无仿真（开环 + UDP下发）

```bash
python3 -m sim_test.run_control --listen 0.0.0.0 --port 9888 --rpi-ip 192.168.1.100 --udp-format v2
```

### 2.2 Isaac Sim 仿真

```bash
~/isaac-sim/python.sh -m sim_test.run_control --sim --web-port 8765 --web-host 127.0.0.1
```

### 2.3 一键脚本（推荐）

```bash
# 仿真 + 下发
./sim_test/start_pc_control.sh sim 192.168.1.100

# 无仿真开环下发
./sim_test/start_pc_control.sh nosim 192.168.1.100
```

## 3. 上层指挥层接口

推荐使用**绝对目标**（笛卡尔 `x,y,z` 或关节相对标定零位的目标角），测试页不再提供“偏移量”按钮。

TCP 每行一个 JSON：

```json
{"cmd":"pose","x":0.35,"y":0.20,"z":0.25}
{"cmd":"joints","axes_rel_deg":[5,-3,8,1.5]}
{"cmd":"claw","wrist_deg":20,"grip":0.7}
```

- **`pose`**：`x,y,z`（米）为基座系 **link4 原点**目标，无 x/y 平移。几何解耦 `q4` 由 `config.Q4_GEOMETRIC_OFFSET_DEG` 决定（v8 默认 **0°**；旧 SINGLE 可设 **120**）。
- **`joints`**：`axes_rel_deg` 长度 4（度），表示 **相对标定** 的**绝对**目标角；**四轴均按指令跟踪**，不强制上述 q4 解耦。
- **`pose_delta` / `joints_delta`**：仍支持，供旧脚本兼容；新上层可不使用。

从 **关节模式** 切换到 **`pose`** 时，内部会用当前关节状态做一次 link4 位置的 FK，写入 `prev_pose_xyz`，使笛卡尔目标变化更平滑。

HTTP 接口：

- `POST /api/pose`（推荐）
- `POST /api/joints`（推荐）
- `POST /api/claw`
- `POST /api/pose_delta`、`POST /api/joints_delta`（兼容）

## 4. 注意事项

- **URDF / USD**：默认 URDF 优先 `icecream_model_v8/.../ice_cream_v8.SLDASM.urdf`（否则 SINGLE）；`--sim` 下 USD 优先 `sim_code/ice_cream_v8_arm.usd`，否则 `ice_cream_single_arm.usd`、`ice_cream_arm.usd`。需与同一套模型导出的 USD 配套。
- **Isaac Sim（`--sim`）**：加载 USD 后 `reset_command()`：初始绝对角 = **`initial_position.md`（或 `config` 默认）标定角**，相对角为 0；再 `viewer.initialize` 写入仿真。物理步与 **`--ik-rate` 锁定同频**（每周期一次 `world.step`，与 `sim_code/cartesian_ik_verify` 一致）；`--physics-hz` 传入会被忽略。默认对 Articulation 设 **高 PD**（可用 `--sim-gain-scale` 缩放）。**跟踪对齐**：`‖q_actual−q_cmd‖` 超阈值时用 `set_joint_positions` **把仿真对齐到当前指令**；`--no-sim-resync` 可关闭。
- 主臂关节接口固定为 4 轴；迁移期兼容长度 5 输入，内部自动截断为前 4 轴。
- `claw` 通道目前按你的要求只用于实机路径，仿真不驱动夹爪模型。
- 当前测试页资源为 `sim_test/web/index.html`。
- 当前实现已改为 `sim_test` 自包含实现，可独立迁移部署。
- UDP 协议已按 `PC_RPI_UDP_PROTOCOL.md` 适配为固定 92B：`=Idddddddddd`（seq, ts, p_rel_deg[5], omega_rad_s[5]）。
- `--udp-format v1|v2` 参数仍保留兼容，但当前都会按上述协议发送。

## 5. 快速联调

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

