# SolidWorks → URDF → Pinocchio 重力补偿完整教程

## 第一部分：SolidWorks 准备工作

### Step 1：安装 sw_urdf_exporter 插件

1. 前往 [github.com/ros/solidworks_urdf_exporter/releases](https://github.com/ros/solidworks_urdf_exporter/releases) 下载最新 `.exe` 安装包
2. 关闭 SolidWorks，以**管理员身份**运行安装包
3. 重新打开 SolidWorks，菜单栏会出现 **Tools → Export as URDF**

> 支持 SW 2018 SP5 ~ 2021，更新版本可能需要手动注册 DLL

---

### Step 2：为每个零件设置正确材料（最关键）

这一步决定质量和惯量是否准确。

**对每个零件（.SLDPRT）操作：**

1. 在 FeatureManager 树中，右键点击零件名称顶部的材料图标（显示为 `<未指定>` 或当前材料名）
2. 选择 **Edit Material**
3. 在材料库中选择对应材料：
   - 铝合金结构件 → `Aluminum Alloys → 6061 Alloy`（密度 2700 kg/m³）
   - 钢制零件 → `Steel → Alloy Steel`（密度 7700 kg/m³）
   - 电机外壳（铝） → 同上
4. 点击 **Apply → Close**

**验证质量是否合理：**

- 菜单 **Evaluate → Mass Properties**
- 检查 `Mass` 数值是否和实物称重接近（误差 < 10% 可接受）
- 记录 `Center of mass` 坐标（后面验证用）

---

### Step 3：为每个关节定义坐标系

sw_urdf_exporter 需要你在 SW 中预先建好坐标系，插件才能知道关节轴在哪里。

**对每个关节操作：**

1. 打开装配体（.SLDASM），选中对应的子装配体或零件
2. 菜单 **Insert → Reference Geometry → Coordinate System**
3. 设置原点：选择关节旋转中心点（通常是轴孔圆心）
4. 设置 Z 轴：选择关节旋转轴方向（**Z 轴必须是旋转轴**，这是 URDF 约定）
5. 命名为有意义的名字，如 `joint1_frame`、`joint2_frame`

**5-DOF 机械臂需要建 6 个坐标系：**
- `base_frame`（世界坐标系/底座）
- `joint1_frame` ~ `joint5_frame`（每个关节处）

---

### Step 4：为每个关节定义旋转轴参考线（可选但推荐）

1. **Insert → Reference Geometry → Axis**
2. 选择关节孔的圆柱面，SW 自动提取中心轴
3. 命名为 `joint1_axis` 等

---

### Step 5：使用插件导出 URDF

1. 打开装配体，菜单 **Tools → Export as URDF**
2. 插件会弹出树形配置界面：

```
base_link
└── link1  [joint: joint1, type: revolute]
    └── link2  [joint: joint2, type: revolute]
        └── link3  [joint: joint3, type: revolute]
            └── link4  [joint: joint4, type: revolute]
                └── link5  [joint: joint5, type: revolute]
```

3. 对每个 joint 配置：
   - **Joint Type**: `revolute`（有限转动）或 `continuous`（无限转动）
   - **Axis**: 选择你在 Step 4 建的参考轴，或手动输入 `0 0 1`（Z轴）
   - **Coordinate System**: 选择对应的 `jointN_frame`
   - **Limits**: 填写角度限位（弧度），如 `-1.57 to 1.57`

4. 点击 **Export**，选择输出目录
5. 生成文件结构：
```
robot_description/
├── urdf/robot.urdf
├── meshes/
│   ├── link1.STL
│   ├── link2.STL
│   └── ...
└── launch/display.launch
```

---

## 第二部分：手动补充电机质量

如果 SW 模型中电机是简化几何体（没有真实质量），需要手动修改 URDF：

```xml
<link name="link2">
  <inertial>
    <origin xyz="0.05 0.0 0.02" rpy="0 0 0"/>
    <mass value="0.85"/>  <!-- SW质量 + 电机质量（从电机规格书获取） -->
    <inertia ixx="0.002" ixy="0.0" ixz="0.0"
             iyy="0.002" iyz="0.0" izz="0.001"/>
  </inertial>
</link>
```

电机质量从规格书（datasheet）的 `Weight` 字段获取，单位换算为 kg。

**注意：转子惯量（reflected inertia）对静态重力补偿无影响，暂时不需要添加。**

---

## 第三部分：用 Pinocchio 验证参数正确性

```python
import pinocchio as pin
import numpy as np
import json

# 加载 URDF
model, _, _ = pin.buildModelsFromUrdf("robot.urdf")
data = model.createData()

# 从 captures.jsonl 读取一个静止姿态
with open("captures.jsonl") as f:
    sample = json.loads(f.readline())

# 提取关节角度（deg×100 → rad）
q = np.array([
    sample["avg"]["motors"]["0"]["position"] / 100 * np.pi / 180,
    sample["avg"]["motors"]["1"]["position"] / 100 * np.pi / 180,
    sample["avg"]["motors"]["2"]["position"] / 100 * np.pi / 180,
    sample["avg"]["motors"]["3"]["position"] / 100 * np.pi / 180,
])

# 计算理论重力矩
pin.computeGeneralizedGravity(model, data, q)
tau_theory = data.g  # 单位 Nm

# 读取实测力矩（字段名是 angle 但实际是 Nm）
tau_measured = np.array([
    sample["avg"]["joints"]["0"]["angle"],
    sample["avg"]["joints"]["1"]["angle"],
    sample["avg"]["joints"]["2"]["angle"],
    sample["avg"]["joints"]["3"]["angle"],
])

print("理论重力矩:", tau_theory)
print("实测力矩:  ", tau_measured)
print("误差 (Nm): ", np.abs(tau_theory - tau_measured))
```

**判断标准：**

| 误差范围 | 结论 | 下一步 |
|----------|------|--------|
| < 0.3 Nm | 参数可用 | 进入 STM32 实现阶段 |
| 0.3 ~ 0.8 Nm | 参数偏差 | 检查材料设置或补充电机质量 |
| > 0.8 Nm | 参数错误 | 检查关节坐标系方向定义 |

---

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 导出的质量明显偏小 | 材料未设置，用了默认密度 | 重新设置材料后重新导出 |
| 力矩方向反了 | Z 轴方向定义反了 | 坐标系 Z 轴取反，或 URDF 里 `<axis xyz="0 0 -1">` |
| 惯量数值异常大 | 单位是 mm 而非 m | SW 导出时确认单位，插件会自动换算 |
| pinocchio 加载报错 | mesh 路径不对 | 用绝对路径或 `package://` 路径 |
