## Ice Cream 机械臂代码说明与 API 文档

本目录包含一个简单的 5 自由度 “Ice Cream” 机械臂示例工程，从 **URDF → USD 转换**、到 **关节控制**、再到 **末端深度相机测量物体世界坐标**。

- **URDF 文件**：`~/isaacsim/icecream/ice_cream_1.SLDASM/urdf/ice_cream_1.SLDASM.urdf`
- **核心脚本**：
  - `urdf_to_usd.py`：URDF 转 USD
  - `drive_arm_joints.py`：按预设关节轨迹驱动机械臂
  - `icecream_move.py`：键盘控制各个关节角度
  - `icecream_camera.py`：机械臂 + link5 深度相机 + 键盘控制 + 物体世界坐标测量
  - `icecream_driver.py`：机械臂驱动与 link5 深度相机封装（主要 API）

---

## 1. URDF → USD：`urdf_to_usd.py`

在 Isaac Sim 的 Python 环境中运行（或使用 `isaacsim -p`）：

```bash
cd ~/isaacsim
./python.sh icecream/code/urdf_to_usd.py
```

- 会生成：`icecream/code/ice_cream_arm.usd`
- 脚本会把 URDF 中的 `package://` 网格路径替换为相对路径 `../meshes/`，然后调用 Isaac Sim 的 URDF 导入，把机械臂写入 USD。

---

## 2. 关节角度驱动：`drive_arm_joints.py`

在生成 `ice_cream_arm.usd` 之后，可以用预设轨迹驱动机械臂：

```bash
cd ~/isaacsim
./python.sh icecream/code/drive_arm_joints.py
```

常用参数：

- `--usd PATH`：指定机械臂 USD 文件，默认使用 `ice_cream_arm.usd`
- `--test`：测试模式，只跑一小段时间后退出

脚本内部会：

- 把机械臂 USD 引用到 `/World/IceCreamArm`
- 使用 `ArticulationAction(joint_positions=...)` 在若干个 5 维关节角 waypoints 之间循环切换，实现简单的示例运动

---

## 3. 键盘控制关节：`icecream_move.py`

`icecream_move.py` 使用 `IceCreamArmDriver` 封装关节控制，并绑定键盘：

```bash
cd ~/isaacsim
./python.sh icecream/code/icecream_move.py
```

键位说明：

- `W / S`：切换当前选中的关节（W 下一个，S 上一个）
- `A / D`：减小 / 增大当前关节角度

可选参数：

- `--usd PATH`：机械臂 USD 路径（默认使用 `ice_cream_arm.usd`）
- `--step_deg FLOAT`：每次 A/D 调节的角度步长（度），默认 5°

---

## 4. 深度相机与物体世界坐标测量：`icecream_camera.py`

`icecream_camera.py` 在 `icecream_move.py` 的基础上，增加了：

- 在 `link5` 上安装一个深度相机（`Link5DepthCamera`）
- 创建一个场景中的测试目标立方体 `/World/TargetCube`
- 按下键盘 `C` 键时：
  1. 取立方体几何中心的世界坐标（真值）
  2. 用相机把该点投影到图像平面，得到像素坐标 (u, v)
  3. 从深度图 `distance_to_camera` 中读取该像素的深度 d
  4. 调用相机 API 把 (u, v, d) 反投影回世界坐标，得到估计世界坐标
  5. 在日志中对比真值 / 估计值

运行命令示例：

```bash
cd ~/isaacsim
./python.sh icecream/code/icecream_camera.py
```

常用参数：

- `--usd PATH`：机械臂 USD 文件路径
- `--step_deg FLOAT`：W/S 选中关节后 A/D 调节的角度步长（度）
- `--link5-path STR`：link5 的 prim 路径（默认 `/World/IceCreamArm/link5`）
- `--resolution W H`：相机分辨率（宽 高），默认 `640 480`
- `--frequency FLOAT`：相机采样频率（Hz），默认 `20.0`

键盘说明（与 `icecream_move.py` 类似，并新增 C 键）：

- `W / S`：选关节
- `A / D`：减小 / 增大当前关节角度
- `C`：使用 link5 深度相机测量目标立方体中心点的世界坐标，并在日志中输出结果

---

## 5. 主要 Python API 接口

### 5.1 `IceCreamArmDriver`（在 `icecream_driver.py`）

封装了 5 关节机械臂的常用控制接口。

**构造函数：**

```python
IceCreamArmDriver(
    world: World,
    usd_path: Optional[str] = None,
    prim_path: str = "/World/IceCreamArm",
    name: str = "ice_cream_arm",
    kp: Optional[Union[float, List[float], np.ndarray]] = None,
    kd: Optional[Union[float, List[float], np.ndarray]] = None,
)
```

- **world**：Isaac Sim 的 `World` 实例
- **usd_path**：机械臂 USD 路径，`None` 时默认使用本目录下的 `ice_cream_arm.usd`
- **prim_path**：机械臂在场景中的 prim 路径
- **kp / kd**：关节位置控制的刚度 / 阻尼，可以是标量或长度为 5 的数组

**常用方法：**

- `set_joint_positions(positions)`  
  设置全部 5 个关节目标角度（弧度，array-like 长度 5，自动裁剪到关节限位）。

- `set_joint_position(joint_index, value_rad)`  
  设置单个关节目标角度（弧度），`joint_index` ∈ [0, 4]。

- `set_joint_positions_deg(positions_deg)` / `set_joint_position_deg(joint_index, value_deg)`  
  与上面类似，但参数是“度”。

- `get_joint_positions()` / `get_joint_positions_deg()`  
  读取当前关节实际角度（弧度 / 度）。

- `get_target_positions()`  
  读取当前内部保存的目标关节角（弧度）。

- `set_joint_velocities(velocities)` / `set_joint_velocity(joint_index, value_rad_per_s)`  
  设置全部 / 单个关节目标角速度（弧度/秒）。

- `set_joint_velocities_deg(velocities_deg_per_s)` / `set_joint_velocity_deg(joint_index, value_deg_per_s)`  
  速度版本的“度”为单位接口。

- `get_joint_velocities()` / `get_joint_velocities_deg()`  
  读取当前关节角速度。

- `get_target_velocities()`  
  返回当前目标速度数组，或 `None`（尚未设置）。

- `clear_velocity_targets()`  
  清除速度目标，使 `apply()` 只发位置控制。

- `home()`  
  把目标关节角全部设为 0，并清除速度目标。

- `apply()`  
  将当前目标关节角（及速度，如已设置）打包为 `ArticulationAction`，调用底层控制器。  
  **使用方式**：在仿真循环中，每一步 `world.step()` 前或后调用一次。

属性：

- `articulation`：底层 `SingleArticulation` 对象
- `joint_names`：关节名称列表（长度 5）

---

### 5.2 `Link5DepthCamera`（在 `icecream_driver.py`）

封装了安装在 link5 上方的深度相机。

**构造函数：**

```python
Link5DepthCamera(
    link5_prim_path: str,
    name: str = "Link5DepthCamera",
    resolution: Tuple[int, int] = (640, 480),
    frequency: float = 20.0,
    offset_above_link5: Tuple[float, float, float] = (0.0, 0.0, 0.1),
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    euler_deg: Optional[Tuple[float, float, float]] = None,
)
```

- **link5_prim_path**：机械臂 link5 的 prim 路径，例如 `/World/IceCreamArm/link5`
- **name**：相机 prim 的名称，将挂载到 `link5_prim_path/name`
- **resolution**：相机分辨率 (宽, 高)
- **frequency**：相机刷新频率（Hz）
- **offset_above_link5**：相机在 link5 局部坐标系下的平移偏移
- **roll_deg / pitch_deg / yaw_deg / euler_deg**：欧拉角（度），定义相机在 link5 局部坐标系下的朝向

**主要方法：**

- `initialize()`  
  初始化相机、设置裁剪平面、焦距等。

- `add_distance_to_camera_to_frame()`  
  在输出帧中添加 `distance_to_camera` 通道（深度图），供后续测量使用。

- `get_world_pose()`  
  返回相机在世界坐标系下的姿态 `(position, orientation_quat_wxyz)`。

- `get_current_frame()`  
  返回当前帧字典，例如 `{"rgb": ..., "distance_to_camera": ...}`。

属性：

- `prim_path`：相机 prim 路径
- `camera`：底层的 `isaacsim.sensors.camera.Camera` 对象  
  可进一步调用：
  - `get_image_coords_from_world_points(points_world)`
  - `get_world_points_from_image_coords(image_coords, depths)`

---

### 5.3 `DepthCameraObjectMeasurer`（在 `icecream_driver.py`）

这是针对本示例封装的一个小工具类，用于：

- **API 1**：世界坐标 → 图像像素坐标  
- **API 2**：图像像素坐标 + 深度 → 世界坐标  

**构造函数：**

```python
DepthCameraObjectMeasurer(link5_camera: Link5DepthCamera)
```

- **link5_camera**：上面创建的 `Link5DepthCamera` 实例

**核心 API：**

- `world_to_pixel(world_point, image_shape=None) -> dict`  
  - 功能：**世界坐标 → 图像坐标**  
  - 入参：
    - `world_point`：形状 `(3,)` 的世界坐标
    - `image_shape`：可选 `(H, W)`，若提供则会返回像素是否在范围内  
  - 返回：
    ```python
    {
        "uv": (u, v),        # 浮点像素坐标
        "pixel": (u_i, v_i), # 四舍五入后的整数像素坐标
        "in_bounds": bool,   # 若给了 image_shape，则表示是否在范围内
    }
    ```
  - 你可以只使用这个函数来从世界坐标获取像素坐标，然后用自己的方法去处理深度图。

- `pixel_depth_to_world(u, v, depth) -> np.ndarray`  
  - 功能：**图像像素坐标 + 深度 → 世界坐标**  
  - 入参：
    - `u, v`：像素坐标（可以是 float 或 int）
    - `depth`：该像素对应的深度值（米），通常来自 `distance_to_camera[v, u]`  
  - 返回：
    - 形状 `(3,)` 的世界坐标向量  
  - 用途：当你有其他方法算出像素坐标 (u, v) 和深度值时，直接调用该函数即可得到世界坐标，无需再自己处理相机内外参。

---

以上即为本目录中主要脚本与类的功能说明和 API 概览。你可以基于这些接口继续扩展更复杂的控制逻辑（轨迹规划、抓取、视觉伺服等）。
