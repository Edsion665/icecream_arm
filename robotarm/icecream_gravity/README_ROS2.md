# icecream_gravity — ROS2 + RViz2

## 目录说明

| 路径 | 说明 |
|------|------|
| `ice_cream_v5.SLDASM/` | SolidWorks 原始导出（ROS1 catkin，`package.xml` 为 catkin） |
| `ice_cream_v5_description/` | **ROS2 ament** 包：已修正 URDF 中 `package://` 为 `ice_cream_v5_description`，并复制 `meshes/` |

## 仓库内目录结构（robotarm）

```
icecream_gravity/
├── ice_cream_v5.SLDASM/          # 原始导出（ROS1 catkin）
├── ice_cream_v5_description/     # ROS2 ament 包（给 RViz2 用）
└── README_ROS2.md
```

本仓库已在 `robotarm/ros2_ws/` 下放好 **符号链接** 指向 `ice_cream_v5_description`，可直接编译。

## 编译与运行（本机）

```bash
cd /home/huangjianan/robotics/robotarm/ros2_ws
source /opt/ros/humble/setup.bash

# 若使用 Anaconda 且 colcon 报错缺少 catkin_pkg：让 cmake 用系统 Python
PATH="/usr/bin:$PATH" colcon build --paths src/ice_cream_v5_description

# 先 overlay 工作空间（须在 ros2_ws 目录下；若只有 setup.sh 没有 setup.bash，用下面这一行）
source install/setup.sh

ros2 launch ice_cream_v5_description display_rviz.launch.py
```

新开终端时建议顺序：

```bash
source /opt/ros/humble/setup.bash
cd /home/huangjianan/robotics/robotarm/ros2_ws
source install/setup.sh
ros2 launch ice_cream_v5_description display_rviz.launch.py
```

也可把 `ice_cream_v5_description` 链到任意 `~/ros2_ws/src/` 后同样 `colcon build --paths src/ice_cream_v5_description`。

## RViz2

launch 已加载 `rviz/icecream_arm.rviz`：**Fixed Frame = link0**，并已添加 **RobotModel**。

若仍**只有地面网格、没有机械臂**，几乎都是 **Fixed Frame 仍是 `map`**（默认 RViz 如此，而你的 TF 树里没有 `map`）。请把 **Global Options → Fixed Frame** 改成 **`link0`**，并确认 **Displays** 里已勾选 **RobotModel**，且 **Description Topic** 为 `/robot_description`。

重新编译安装后需再 `source install/setup.sh` 才能用到新 rviz 文件：

```bash
PATH="/usr/bin:$PATH" colcon build --paths src/ice_cream_v5_description
```

## 依赖

- `ros-humble-*` / `ros-jazzy-*`：`robot_state_publisher`、`joint_state_publisher_gui`、`rviz2`（按发行版安装）。
