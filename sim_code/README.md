# Ice Cream 机械臂：URDF → USD 与关节驱动

- **URDF 文件**：`/home/phoenix/isaacsim/icecream/ice_cream_1.SLDASM/urdf/ice_cream_1.SLDASM.urdf`
- **第一套**：把 URDF 导入并保存为 USD
- **第二套**：加载 USD，用关节角度驱动机械臂

## 1. 第一套：URDF 转 USD

在 Isaac Sim 的 Python 环境中运行（或使用 `isaacsim -p`）：

```bash
cd /home/phoenix/isaacsim
./python.sh icecream/code/urdf_to_usd.py
```

会生成 `icecream/code/ice_cream_arm.usd`。脚本会把 URDF 里的 `package://` 网格路径替换为相对路径 `../meshes/`，再调用 Isaac Sim 的 URDF 导入并写入上述 USD。

## 2. 第二套：关节角度驱动

先完成第一套生成 `ice_cream_arm.usd`，再运行：

```bash
./python.sh icecream/code/drive_arm_joints.py
```

或指定 USD、测试模式：

```bash
./python.sh icecream/code/drive_arm_joints.py --usd /path/to/ice_cream_arm.usd
./python.sh icecream/code/drive_arm_joints.py --test
```

机械臂有 5 个关节（joint1～joint5），脚本会按预设的关节角度序列循环运动；可用 `controller.apply_action(ArticulationAction(joint_positions=...))` 传入自己的 5 维关节角（弧度）做控制。

## 3. 末端位姿控制与坐标系

- **运动学**：`icecream_kinematics.py` 从 URDF 解析，提供基座系下 FK、雅可比、数值 IK。
- **位姿控制**：`icecream_pose_control.py` 提供 `PoseController`，支持**位置控制**（IK 角度）与**速度控制**（雅可比伪逆），目标位姿在基座系下给出。
- **坐标系关系**：世界系 → 基座系（link0）→ 末端系（link5）= FK(q)；相机系由 link5 安装参数定义。详见 `code/COORDINATES.md`。

### 位姿到达验证（Isaac Sim）

在 Isaac Sim 中运行，使末端到达并保持给定目标位姿，带调试输出：

```bash
# 默认目标 (0.35, 0.2, 0.15) 米，位置控制
./python.sh icecream/code/icecream_pose_reach.py

# 指定目标 x y z [roll pitch yaw 度]，速度控制，每 120 步打印误差
./python.sh icecream/code/icecream_pose_reach.py --target 0.4 0.15 0.2 0 0 0 --mode velocity --debug-interval 120
```

参数：`--mode position|velocity`、`--frequency`、`--target`、`--position-only`、`--debug-interval`。详见 `COORDINATES.md`。

## 4. 本地用 RViz 查看 URDF 与关节（SINGLE 模型）

在本地用 RViz 打开 SINGLE 的 URDF 包，可查看模型、TF 树，并用 **Joint State Publisher GUI** 拖动关节检查运动与命名是否正确。

**前提**：已安装 ROS（Melodic/Noetic），工作空间已包含 `ice_cream_SINGLE.SLDASM` 包（或把项目路径加入 `ROS_PACKAGE_PATH`）。

1. 进入工作空间并 source：
   ```bash
   cd /path/to/your_ws
   source devel/setup.bash
   ```
2. 启动 display（会加载 URDF、joint_state_publisher_gui、robot_state_publisher、RViz）：
   ```bash
   roslaunch ice_cream_SINGLE.SLDASM display.launch
   ```
   若包名中的点号在 catkin 中报错，可把包目录改名为 `ice_cream_SINGLE_SLDASM` 并相应修改 `package.xml` 与 `find` 中的包名。
3. 在 RViz 中：
   - **RobotModel**：显示机械臂模型；
   - **TF**：显示各 link 坐标系，固定帧为 `link0`；
   - 弹出 **Joint State Publisher GUI** 窗口，拖动各关节滑块即可驱动关节，观察模型与 TF 变化，排查关节轴、限位与命名问题。
4. RViz 配置已放在 `ice_cream_SINGLE.SLDASM/config/display.rviz`，launch 会通过 `-d` 自动加载；若缺失该文件，去掉 launch 中的 `-d` 后可在 RViz 里手动添加 RobotModel 和 TF 显示。

## 5. 本地用 ROS2 Humble + RViz2 查看 URDF 与关节（SINGLE 模型）

本仓库提供 ROS2 包 **ice_cream_single_sldasm**，与 `ice_cream_SINGLE.SLDASM` 共用同一套 URDF/mesh/config，用 **colcon** 构建后即可用 RViz2 和 **Joint State Publisher GUI** 查看模型与 TF、拖动关节排查问题。

**前提**：本机已安装 **ROS2 Humble**，并已安装：

```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher-gui ros-humble-rviz2
```

**注意**：若终端启用了 **Anaconda/conda**（提示符带 `(base)`），colcon 会用到 conda 的 Python，可能报 `ModuleNotFoundError: No module named 'catkin_pkg'`。构建前请先退出 conda 再 source ROS2：
```bash
conda deactivate
source /opt/ros/humble/setup.bash
```
或在当前 conda 环境安装：`pip install catkin_pkg` 后再构建。

**步骤：**

1. 进入工作空间并 source ROS2（若用 conda，先执行上面的 `conda deactivate`）：
   ```bash
   source /opt/ros/humble/setup.bash
   cd /path/to/your_ws   # 将 icecream_project 或包含 ice_cream_single_sldasm 的目录作为工作空间根
   ```
2. 在工作空间根目录（即包含 `ice_cream_single_sldasm` 的目录，例如项目根 `icecream_project`）构建并 source：
   ```bash
   cd /path/to/icecream_project
   colcon build
   source install/setup.bash
   ```
   若构建报错 `unrecognized arguments: --packages-select`，说明当前 colcon 较旧，请直接使用上面的 `colcon build`（会构建工作空间内所有包；已对 ROS1 包加 COLCON_IGNORE，一般只会编本包）。
3. 启动 display（加载 URDF、joint_state_publisher_gui、robot_state_publisher、RViz2）：**必须先 source 本仓库的 install**（不要 source 其他工作空间如 dev_ws，否则会报 Package not found）：
   ```bash
   source /path/to/icecream_project/install/setup.bash
   ros2 launch ice_cream_single_sldasm display.launch.py
   ```
   若 `install/setup.bash` 不存在（例如之前构建时其他包失败导致未生成），可先 source ROS2 再 source 包内 local_setup：
   ```bash
   source /opt/ros/humble/setup.bash
   source /path/to/icecream_project/install/ice_cream_single_sldasm/share/ice_cream_single_sldasm/local_setup.bash
   ros2 launch ice_cream_single_sldasm display.launch.py
   ```
   或重新执行一次 `colcon build`（已对 ROS1 包加 COLCON_IGNORE，只会编本包），生成 `install/setup.bash` 后再用上面第一条命令。
4. 可选：指定 RViz2 配置文件：
   ```bash
   ros2 launch ice_cream_single_sldasm display.launch.py rviz_config:=/path/to/your.rviz
   ```

**说明**：launch 会将 URDF 中的 `package://ice_cream_SINGLE.SLDASM/` 替换为本包 share 路径，mesh 会正确加载。RViz2 中固定帧为 `link0`，可用 **TF** 和 **RobotModel** 查看；**Joint State Publisher GUI** 窗口内拖动滑块即可驱动关节，便于检查关节轴与命名。
