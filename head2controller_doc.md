# head2controller 接口文档（sim_test v1）

本文档定义上层指挥层到 `sim_test` 的输入协议，适用于 4 轴主臂 + claw 独立通道。

## 命令模型

- 主臂（4 轴）— **推荐绝对目标**：
  - **`pose`**：`x,y,z`（米）为基座系下 **link4 原点** 目标，无额外平移；IK 与 `inverse_kinematics_link4_geometric_decouple` 一致。**仅 POSE 模式**下几何解耦 `q4 = rad(config.Q4_GEOMETRIC_OFFSET_DEG) - (q2+q3)`（v8 默认偏移 0°；旧 SINGLE 曾为 120°）。
  - **`joints`**：`axes_rel_deg`（长度 4，单位度）。各元为 **相对标定零位** 的**绝对**目标角；**四轴独立**，不强制 q4 解耦。
- 主臂 — **兼容旧用法**（不推荐新前端依赖）：
  - `pose_delta`：`dx,dy,dz`（米）
  - `joints_delta`：`deltas_rel_deg`（长度 4，单位度）
- claw（仅实机）：
  - `claw`：`wrist_deg`、`grip` 或 `servo_deg=[wrist,grip]`

## 模式与初态

- 上电并完成 `reset_command()` 后，默认处于 **关节模式**，关节初值为一组固定「零位」姿态；之后由 `pose` / `joints` 切换控制模式。
- 从 **关节模式** 收到 **`pose`** 时，控制器会将 `prev_pose_xyz` 设为当前 **link4** 的 FK 位置，减轻模式切换时的目标跳变。

## 传输

- TCP JSON 行协议：默认 `:9888`，每行一个 JSON 命令。
- HTTP API（测试页）：
  - `POST /api/pose`
  - `POST /api/joints`
  - `POST /api/claw`
  - `POST /api/pose_delta`、`POST /api/joints_delta`（兼容）

## 兼容

- 迁移期可接收长度 5 的 `axes_rel_deg` / `deltas_rel_deg`，内部只取前 4 轴。
