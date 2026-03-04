# 坐标系与末端位姿控制说明

## 坐标系定义与相对关系

本工程中涉及以下坐标系，均来自 URDF 与 Isaac Sim 场景约定：

| 坐标系 | 说明 | 与其它系关系 |
|--------|------|--------------|
| **世界系 world** | Isaac Sim 场景固定参考系，单位米 | 场景原点 |
| **基座系 base** | 机械臂底座 link0 的固连系 | `T_world_base` 由场景中 arm 的 prim 位姿给出（通常与 world 对齐或固定偏移） |
| **末端系 ee** | link5 原点，即“末端执行器”参考点 | `T_base_ee = FK(q)`，由关节角 q 决定 |
| **相机系 camera** | 安装在 link5 上的深度相机光学中心 | `T_link5_camera` 由 `icecream_camera` 的 offset + euler 定义，随 link5 运动 |

### 变换关系

- **末端在世界系下**：`T_world_ee = T_world_base * T_base_ee(q)`
- **末端在基座系下**：`T_base_ee(q)` 由 `icecream_kinematics.URDFKinematics.forward_kinematics(q)` 得到
- **相机在世界系下**：`T_world_camera = T_world_ee * T_ee_camera`（其中 `T_ee_camera` 由相机相对 link5 的安装参数给出）

位姿控制模块中的**目标位姿均在基座系 base 下**给出，这样与机械臂的 URDF 定义一致，且不依赖场景中 base 相对 world 的放置。

## 位姿控制与调试接口

### 1. 运动学模块 `icecream_kinematics.py`

- **FK**：`forward_kinematics(q)` → 4x4 `T_base_ee`
- **雅可比**：`jacobian(q)` → 6x5，末端线速度/角速度对关节速度的映射
- **IK**：`inverse_kinematics(target_position, target_orientation, ...)` → 关节角与是否成功

### 2. 位姿控制器 `icecream_pose_control.py`

- **设置目标**：`set_target_pose(position, orientation)` 或 `set_target_position(position)`（仅位置）
- **当前位姿**：`get_current_pose(q)` → 基座系下 position (3,) + rotation (3,3)
- **误差**：`get_pose_error(q)` → 位置误差、姿态误差（轴角）、范数
- **每步更新**：`update(dt, current_joint_positions)` → 目标关节角（交给 driver 并 apply）

支持两种模式：

- **position**：每步用 IK 计算目标关节角，适合“到达并保持”位姿
- **velocity**：用雅可比伪逆将位姿误差转为关节速度，积分得到目标关节角，再交给现有关节位置控制器

### 3. 验证脚本 `icecream_pose_reach.py`（Isaac Sim 内运行）

在 Isaac Sim 中加载机械臂并运行位姿控制，通过命令行设置目标与调试输出：

```bash
# 默认：位置控制，目标 (0.35, 0.2, 0.15) 米，RPY 0,0,0，每 60 步打印误差
./python.sh icecream/code/icecream_pose_reach.py

# 指定目标位姿（x y z roll pitch yaw，后三个为度）
./python.sh icecream/code/icecream_pose_reach.py --target 0.4 0.15 0.2 0 0 0

# 仅位置（3 个数），不约束姿态
./python.sh icecream/code/icecream_pose_reach.py --target 0.35 0.2 0.15

# 速度控制，控制频率 30 Hz，每 120 步打印
./python.sh icecream/code/icecream_pose_reach.py --mode velocity --frequency 30 --debug-interval 120

# 仅位置控制（IK 只解位置）
./python.sh icecream/code/icecream_pose_reach.py --position-only --target 0.3 0.2 0.25
```

参数摘要：

- `--mode position|velocity`：角度控制（IK）或速度控制（雅可比）
- `--frequency`：控制律更新频率 Hz
- `--target x y z [roll pitch yaw]`：基座系下目标，单位米与度
- `--position-only`：只控位置不约束姿态
- `--debug-interval N`：每 N 步打印一次位姿误差（0 关闭）

## 使用流程

1. 已由 `urdf_to_usd.py` 生成 `ice_cream_arm.usd`
2. 在 Isaac Sim 中运行 `icecream_pose_reach.py`，按需传入 `--target`、`--mode`、`--frequency`、`--debug-interval`
3. 末端将朝目标位姿运动并在到达后保持；调试输出中可查看当前末端位置与目标、位置误差与姿态误差
