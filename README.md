
# IceCream 关节协作机械臂项目

![alt text](https://github.com/Edsion665/icecream_arm/blob/main/icecream.jpg?raw=true)

Dairy-Prince团队自研的 **5 自由度关节协作机械臂** 项目。

甜品家族一号成员冰淇淋！

本仓库当前主要包含：

- **仿真与验证代码**（基于 NVIDIA Isaac Sim）
- **机械臂模型与工程结构**（ROS 包骨架等）
- **Git 使用与协作流程文档**

后续将逐步补充：

- **底层驱动 / 嵌入式固件代码**（电机驱动、关节编码器、CAN/串口通信等）
- **上层控制策略代码**（任务空间控制、路径规划、人机交互等）

---

## 1. 项目总体结构

当前仓库的大致结构（只列出主要目录）：

```text
icecream_arm/
├─ sim_code/                 # 仿真与运动学验证相关的 Python 脚本（Isaac Sim）
│  ├─ icecream_kinematics.py       # 机械臂运动学：FK / Jacobian / 数值 IK
│  ├─ test_jacobian_straight_line.py # 雅可比直线&圆轨迹验证（Isaac 场景下）
│  ├─ urdf_to_usd.py                # 将 URDF 导入 Isaac 并导出 ice_cream_arm.usd
│  └─ ...（后续仿真/控制脚本）
│
├─ icecream_model/           # 机械臂模型与 ROS 包骨架（路径命名统一为 icecream_model）
│  ├─ CMakeLists.txt
│  ├─ package.xml
│  ├─ config/
│  │   └─ joint_names_ice_cream_0208.SLDASM.yaml
│  ├─ launch/
│  │   ├─ display.launch
│  │   └─ gazebo.launch
│  └─ urdf/                  # ★ 建议放置 ice_cream_0208.SLDASM.urdf 等 URDF 文件（待补充）
│
├─ git_tutorial/
│  └─ README.md              # 项目 Git 工作流说明（main / develop / feature 分支）
│
└─ README.md                 # 本文件：项目总览与使用指南
```

> 说明：过去的代码主要位于 `icecream/` 目录。现在已重构为：
> - 仿真代码归入 `sim_code/`
> - 模型与 ROS 包归入 `icecream_model/`  
> 老的 `icecream/` 结构不会再维护。

---

## 2. 分支策略（main / develop / feature）

本仓库采用简单的三层分支策略：

- **`main`**
  - 稳定 / 上线分支；
  - 只接受从 `develop` 合并过来的、已经测试过的改动。

- **`develop`**
  - 日常开发汇总分支；
  - 所有新功能或改动，最终会先合入 `develop`；
  - 需要相对可运行，但可以包含调试/实验代码。

- **`feature/*` 功能分支**
  - 针对某个具体功能 / 实验 / bug 修复的短期分支，例如：
    - `feature/follow-box`
    - `feature/circle-autotune`
    - `fix/jacobian-circle-drift`
  - 功能完成 → 合并回 `develop` →（必要时）再从 `develop` 合并到 `main`。

详细 Git 使用说明可参考：

```text
git_tutorial/README.md
```

---

## 3. 仿真环境与依赖（Isaac Sim）

### 3.1 依赖环境

- **操作系统**：Ubuntu 20.04 / 22.04（建议与 Isaac 官方支持版本一致）
- **Isaac Sim**：4.x（建议使用 NGC/官方安装方式）
- **Python**：使用 Isaac 自带 Python 解释器（`./python.sh` 或 `isaacsim.app` 内的 Python）

> 注意：`sim_code/` 里的脚本假设在 **Isaac Sim Python 环境** 中运行，  
> 不建议直接用系统 Python 执行（import `isaacsim` 会失败）。

### 3.2 配置 Python 路径（可选）

如果你在终端中通过 `./python.sh` 运行脚本，建议在仓库根目录下执行，例如：

```bash
cd ~/icecream_project
/path/to/isaac-sim/python.sh sim_code/test_jacobian_straight_line.py --help
```

Isaac Sim 启动本身（GUI 模式）时，部分脚本会内部创建 `SimulationApp`，不需要你手动再启。

---

## 4. 机械臂运动学模块（sim_code/icecream_kinematics.py）

`sim_code/icecream_kinematics.py` 提供了机械臂的核心运动学工具：

- **URDFKinematics**
  - 关节列表、连杆几何参数（目前带有从 URDF 精确提取的硬编码参数）
  - 正向运动学（FK）：`forward_kinematics`、`forward_kinematics_position`、`forward_kinematics_rotation`
  - 几何雅可比：解析雅可比 `jacobian_analytical` + 数值雅可比 `jacobian`
  - 可操纵性指标：`manipulability`
  - 数值 IK（阻尼最小二乘）：`inverse_kinematics`

- **URDF 导入模式**
  - `URDFKinematics(urdf_path=...)`：
    - 如果 `urdf_path` 有效 → 从指定 URDF 解析；
    - 否则 → 使用硬编码 `_HARDCODED_JOINTS` 参数（与 URDF 一致）。
  - 测试脚本 `test_jacobian_straight_line.py` 中，通过 `_find_urdf()` 自行搜索 URDF 路径（后续推荐统一迁移到 `icecream_model/urdf/`）。

---

## 5. 雅可比直线 / 圆周运动验证（sim_code/test_jacobian_straight_line.py）

该脚本用于在 Isaac Sim 中验证雅可比矩阵与任务空间控制的效果：

- **主要功能**
  1. 从 Isaac Sim 中实时读取机械臂关节角；
  2. 基于解析雅可比，将期望末端笛卡尔速度映射为关节速度（DLS 伪逆）；
  3. 让末端沿给定直线或圆周轨迹运动；
  4. 在场景中用球体标记：
     - 绿色：实际轨迹
     - 红色：理想轨迹
  5. 打印偏离轨迹的误差、位置误差、路径可操纵性（最小奇异值）等。

### 5.1 运行方式（示例）

进入仓库根目录，然后用 Isaac 的 Python 运行脚本（路径示例）：

```bash
cd ~/icecream_project

# 直线模式
/path/to/isaac-sim/python.sh sim_code/test_jacobian_straight_line.py \
    --mode line \
    --direction 0 1 0 \
    --length 0.08 \
    --speed 0.015 \
    --speed-z 0.005

# 圆周模式
/path/to/isaac-sim/python.sh sim_code/test_jacobian_straight_line.py \
    --mode circle \
    --radius 0.03 \
    --circle-speed 0.01
```

部分关键参数（完整请看脚本中的 `argparse` 定义）：

- `--mode {line,circle}`：直线或圆轨迹
- `--radius`：圆半径（m）
- `--circle-speed`：圆周切向线速度（m/s）
- `--damping`：雅可比 DLS 阻尼参数
- `--track-kp` / `--track-kp-z` / `--track-kp-ori`：任务空间 P 增益（位置 & 姿态）
- `--stabilize-*`：回 HOME 阶段的收敛步数与增益

脚本内部包含自动调参逻辑和接近奇异点时的减速/限速，以提高在不同速度、半径下的稳定性。

---

## 6. URDF → USD 导出（sim_code/urdf_to_usd.py）

`sim_code/urdf_to_usd.py` 用于在 Isaac Sim 环境中将 URDF 模型导入并导出为 USD 文件，供仿真使用：

- **主要流程**
  1. 从 `URDF_FILE` 指定的 URDF 文件中读取模型；
  2. 将 `package://.../meshes` 替换为相对路径 `../meshes/`（`prepare_urdf_with_resolved_meshes()`）；
  3. 通过 `URDFParseAndImportFile` 导入到新的 Stage；
  4. 导出为 `sim_code/ice_cream_arm.usd`。

- **使用前准备（推荐）**
  - 将 URDF 文件放到：`icecream_model/urdf/ice_cream_0208.SLDASM.urdf`
  - 修改 `URDF_DIR` / `URDF_FILE` 指向 `icecream_model/urdf/`。

- **运行示例**

  ```bash
  cd ~/icecream_project
  /path/to/isaac-sim/python.sh sim_code/urdf_to_usd.py
  ```

---

## 7. 未来计划模块（待补充）

当前仓库主要覆盖了**仿真与运动学验证**部分。后续将逐步补充以下模块：

### 7.1 底层驱动 / 嵌入式固件（计划目录示例：`firmware/`）

- **目标**
  - 面向电机驱动板 / 关节控制器的嵌入式代码（MCU 固件）；
  - 实现：
    - 电机电流/位置/速度闭环控制；
    - 编码器读取与零点校准；
    - 温度 / 电流等状态监测与保护；岛
    - 通信协议（CAN / RS485 / UART 等）。

- **预期内容**
  - `firmware/` 下的 MCU 工程（如 STM32 / ESP32 等）；
  - 通信协议说明文档；
  - 与上位机协议的一致性约定。

### 7.2 上位机驱动与通信（计划目录示例：`driver_pc/` 或 `ros2_ws/`）

- **目标**
  - 提供 PC / 工控机 侧的驱动层：
    - 与嵌入式控制板通信；
    - 封装为 ROS2 节点或独立进程；
    - 暴露“关节空间/任务空间接口”（如 `set_joint_positions`、`get_joint_states` 等）。

- **预期内容**
  - ROS2 包：`icecream_driver`（C++/Python）；
  - 命令行工具：关节点动、关节回零、自检等；
  - 与仿真接口的对齐：尽量复用 `sim_code` 中的控制接口定义。

### 7.3 高层控制策略与应用（计划目录示例：`control_strategies/`）

- **目标**
  - 针对 IceCream 机械臂的高层控制与应用场景：
    - 任务空间控制器（当前 `PoseController` 的真实机械版本）；
    - 轨迹规划：直线插补、圆弧插补、多段轨迹拼接；
    - 自动标定、相机引导、抓取任务等。

- **预期内容**
  - 可直接在真实机械臂上运行的高层控制脚本；
  - 与仿真脚本共享逻辑，以便在 Isaac Sim 中先验证再上机。

---

## 8. 贡献与协作

欢迎对本项目感兴趣的同学/开发者参与：

- **协作流程建议**
  1. Fork 仓库或在本仓库开功能分支（`feature/*`）；
  2. 在分支上开发和测试；
  3. 提交 PR / 或在团队内部通过 `develop` 分支合并；
  4. 由仓库维护者定期把稳定的 `develop` 合并到 `main`。

- **Git 工作流细节**
  - 请阅读 `git_tutorial/README.md`，其中介绍了：
    - `main / develop / feature` 架构；
    - 提交/推送/合并的命令模板；
    - 如何避免覆盖他人提交等。

---

## 9. 联系方式

- 仓库所有者：`@Edsion665`
- 如果你在使用或开发过程中遇到问题，建议在 GitHub 上开 Issue，  
  说明：
  - 使用的分支（`main` / `develop` / `feature/...`）
  - 运行的脚本和命令行参数
  - Isaac Sim 版本 / 系统环境  
  便于后续一起排查和改进。

