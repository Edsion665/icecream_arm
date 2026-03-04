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
