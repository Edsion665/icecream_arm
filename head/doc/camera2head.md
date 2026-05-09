# camera2head API 规范（相机 / 感知 -> icecream head，v1.1）

- 文档状态：stable
- 适用场景：相机或视觉节点向 **head** 上报一帧（或一次）检测结果：**物体语义分类**（可抓物体 / 落点目标 / 盖子）、**三维位置**、**手腕偏转角**，供 `listener` 缓存与 `planner` / `run` 状态机使用
- 对端：**head** 进程内嵌的 ingestion 服务（TCP/HTTP 监听）；相机侧为客户端
- 线格式：与 [`head2bridge.md`](head2bridge.md) 相同——TCP JSON 行 + HTTP JSON、`Content-Type: application/json`、响应 `ok` / `error`
- **v1.1**：新增 `role`、`wrist_yaw_deg`、可选 `track_id`；补充 **head 侧多帧融合缓存** 约定及与 **bridge 到位** 的协同说明

## 1. 范围（Scope）

- 定义**上行**感知载荷（相机 → head）：单帧 `objects[]` 列表。
- **不包含**：head 内部 `observe1` / `observe2` 等状态名实现（由 `src/run.py` 写死）；不包含向 bridge 下发的 `pose`/`claw` 细节（见 `head2bridge.md`）。
- **head 侧**：`src/listener.py` 在收到每帧后，按 §7 将观测合并进**跟踪数组**（非相机必传字段，为实现约定；融合实现在 `src/tracker.py`）。

## 2. 传输与端点

**默认示例端口**（与 `head2bridge` 文档中的 bridge 端口区分；head ingestion 可单独配置）：

### 2.1 TCP（JSON 行）

- 默认监听：`0.0.0.0:9799`（示例；实现可合并到 head 单一端口，以代码为准）
- 编码：UTF-8
- 每行一条 JSON（`\n` 分隔），语义与 §4 HTTP 请求体相同

### 2.2 HTTP（JSON）

- 默认服务：`127.0.0.1:8776`（示例）
- `Content-Type: application/json`

## 3. 接口总览

| 命令 `cmd`（`type` 等价） | 说明 |
|---|---|
| `detection` | 上报一帧检测结果（别名：`objects`、`perception`） |

HTTP 路由：

- `POST /api/detection` → 等价 `cmd=detection`
- `POST /api/objects` → 与 `/api/detection` 等价（别名路由）

## 4. 请求体：`detection`

### 4.1 通用规则

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `cmd` | string | 否 | 与 `type` 二选一；值：`detection`（或别名） |
| `type` | string | 否 | 与 `cmd` 二选一 |

- 命令名大小写不敏感。
- 使用 HTTP 路由 `POST /api/detection` 时，body 可省略 `cmd`/`type`，实现应视为 `cmd=detection`；若同时提供路由与 `cmd`，以 `cmd` 为准且须与 `detection` 一致。

### 4.2 载荷字段

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `frame` | string | 是 | 位置与角度解释坐标系，见 §4.3 |
| `objects` | array | 是 | 本帧检测到的条目列表；允许空数组表示「当前帧无检测」 |
| `frame_id` | integer | 否 | 单调帧号，便于 head 排序 |
| `ts` | string 或 number | 否 | ISO8601 字符串或 Unix 时间戳（秒），便于去重 |

### 4.3 坐标系 `frame` 枚举

| 取值 | 含义 |
|---|---|
| `robot_base` | `position` 为机器人基座标系下的三维位置（单位 m）；`wrist_yaw_deg` 为**绕机器人约定竖轴的偏航角**（单位 deg），与 [`head2bridge.md`](head2bridge.md) 中 `claw.wrist_deg` 的旋转语义一致、可直接映射 |
| `camera_optical` | `position` 为相机光学坐标系（OpenCV 常用：x 右、y 下、z 前），单位 m；`wrist_yaw_deg` 为**在相机前平面内**估计的偏转角（deg），head 应在融合或下发前变换到 `robot_base` 或与 `wrist_deg` 约定一致 |

未列出的取值：实现应返回 `invalid_value: frame 不支持`。

### 4.4 语义分类 `role`（必选）

每条 `objects[]` 元素**必须**带 `role`，表示该检测在任务中的语义，便于 `src/run.py` 在 observe 阶段区分「是否已看到目标 / 物体 / 盖子」：

| 取值 | 含义 |
|---|---|
| `object` | 可抓取物体（蛋筒、杯体等） |
| `target` | 落点 / 工位目标位置（出料口下方、杯位等） |
| `lid` | 盖子（开合盖工位相关） |

大小写敏感（小写）。

### 4.5 `objects[]` 每项字段

| 字段 | 类型 | 必选 | 说明 |
|---|---|---:|---|
| `role` | string | 是 | 见 §4.4 |
| `class_id` | string 或 integer | 是 | 稳定类别标识（SKU / 模型类别 id） |
| `label` | string | 是 | 人类可读名称（如 `wafer_cone`） |
| `position` | object | 是 | 必含 `x`、`y`、`z`（number，单位 **m**），解释见当前请求的 `frame` |
| `wrist_yaw_deg` | number | 是 | **偏转角度**（单位 **deg**），供后续经 `speaker` 映射为 `claw` 的 `wrist_deg`（或与 `joints` 组合）；表示接近/抓取时手腕绕约定轴的朝向 |
| `track_id` | string 或 integer | 否 | 若视觉侧可稳定关联多帧同一物理实例，应填写；head 融合时**优先**按 `track_id` 匹配，否则按 §7.2 规则 |
| `confidence` | number | 否 | 0~1，检测置信度 |
| `bbox_2d` | object | 否 | 归一化框 `x0`,`y0`,`x1`,`y1`，均为 **0~1**（相对图像宽高） |

`position` 示例：`{"x": 0.42, "y": -0.05, "z": 0.18}`

`bbox_2d` 示例：`{"x0": 0.2, "y0": 0.3, "x1": 0.55, "y1": 0.7}`

## 5. 响应

成功：

```json
{"ok": true}
```

失败：

```json
{"ok": false, "error": "错误描述"}
```

## 6. 错误模型

| 场景 | HTTP | 错误字符串示例 |
|---|---:|---|
| JSON 解析失败 | 400 | `invalid json` |
| 非 detection 且无路由隐含 | 400 | `missing_field: 缺少字段 cmd 或 type` |
| 缺少 `frame` | 400 | `missing_field: detection 需要 frame` |
| `frame` 非法 | 400 | `invalid_value: frame 不支持` |
| 缺少 `objects` 或非数组 | 400 | `missing_field: detection 需要 objects 数组` |
| 缺少 `role` 或取值非法 | 400 | `invalid_value: role 必须为 object|target|lid` |
| 缺少 `class_id` / `label` / `position` / `wrist_yaw_deg` | 400 | `missing_field: objects[i] 缺少必选字段` |
| `position` 缺少 x/y/z | 400 | `missing_field: position 需要 x, y, z` |

说明：TCP 路径错误可记录当前行并继续服务。

## 7. Head 侧缓存与多帧融合（实现约定，非单帧 JSON 字段）

以下供 `src/listener.py` 与 `src/run.py` 对齐；**相机仍按帧发送 §4 载荷**，不必上传整段跟踪数组。

### 7.1 跟踪数组 `tracks`

- `listener`（ingestion）与 `tracker` 线程在内存中维护 `tracks: array`（实现上为 `SceneSlots`），每一项表示当前估计的一条物理/语义轨迹，建议字段：`track_id`（若上游未给则由 head 生成）、`role`、`class_id`、`label`、`position`、`wrist_yaw_deg`、`last_frame_id`、`last_ts`、`confidence`（取最近帧或最大置信度策略由实现选定）。
- **物体与目标的绝对位姿会随时间变化**：数组表达「当前最佳估计」，而非历史全量。

### 7.2 更新规则（相邻帧合并）

收到新帧 `F_k` 后，对其中每个检测项 `d`：

1. **匹配**：若 `d.track_id` 存在且与某 `tracks[i].track_id` 相同，则匹配到 `tracks[i]`。
2. 否则在 `tracks` 中查找与 `d` **相同 `role` 且 `class_id` 相同** 的项；若多条，取与 `d.position` **欧氏距离最小** 且距离小于 `merge_pos_eps_m` 的一条（推荐默认 **0.005 m**）。
3. 若仍无匹配项，**追加**新轨迹到 `tracks`。
4. 若匹配成功，且 `||p_k - p_{k-1}||_2 <= merge_pos_eps_m` 且（可选）`|yaw_k - yaw_{k-1}| <= merge_yaw_eps_deg`（推荐默认 **3 deg**），则视为**同一稳定观测**：**原地更新**该元素的 `position`、`wrist_yaw_deg`、`confidence`、`last_frame_id`、`last_ts` 等。
5. 若位置跳变超过阈值，可实现为**新轨迹**或**覆盖**（由 `planner` 需求决定；文档推荐：大跳变时新 `track_id` 或新条目，避免错误合并）。

### 7.3 与 bridge「到位」的协同

- `observe1` / `observe2` 等为 **head 内部状态名**，可写死；向机械臂下发指令须走 `head2bridge`（如 `pose` / `joints` / `claw`）。
- **到位**：以 bridge 返回的**成功信号**为准（HTTP `ok: true`；对笛卡尔建议同时要求 `reached == true` 且存在 `actual_pose`，与 `head2bridge` v2.2 语义一致）。相机帧仅用于判断「当前跟踪数组中是否已含有 `role==target` / `role==object` 等的有效估计」，二者由 `src/run.py` 组合判定。

## 8. curl 联调示例

假定 head ingestion HTTP 为 `127.0.0.1:8776`。

**多条目（目标 + 可抓物体 + 盖子）、`robot_base`：**

```bash
curl -sS -X POST http://127.0.0.1:8776/api/detection \
  -H 'Content-Type: application/json' \
  -d '{
    "cmd": "detection",
    "frame_id": 1204,
    "ts": "2026-05-03T12:00:00Z",
    "frame": "robot_base",
    "objects": [
      {
        "role": "target",
        "track_id": "slot_a",
        "class_id": "dispense_slot",
        "label": "cone_target",
        "confidence": 0.92,
        "position": {"x": 0.45, "y": -0.02, "z": 0.12},
        "wrist_yaw_deg": 15.0,
        "bbox_2d": {"x0": 0.22, "y0": 0.31, "x1": 0.48, "y1": 0.72}
      },
      {
        "role": "object",
        "track_id": "cone_7",
        "class_id": 1,
        "label": "wafer_cone",
        "confidence": 0.94,
        "position": {"x": 0.40, "y": 0.01, "z": 0.11},
        "wrist_yaw_deg": -10.5
      },
      {
        "role": "lid",
        "class_id": "lid_stock",
        "label": "lid",
        "confidence": 0.68,
        "position": {"x": 0.52, "y": 0.08, "z": 0.15},
        "wrist_yaw_deg": 0.0
      }
    ]
  }'
```

**单物体最短字段：**

```bash
curl -sS -X POST http://127.0.0.1:8776/api/detection \
  -H 'Content-Type: application/json' \
  -d '{"frame":"camera_optical","objects":[{"role":"object","class_id":0,"label":"cone","position":{"x":0,"y":0,"z":0.55},"wrist_yaw_deg":12.3}]}'
```

**空帧（无检测）：**

```bash
curl -sS -X POST http://127.0.0.1:8776/api/detection \
  -H 'Content-Type: application/json' \
  -d '{"cmd":"detection","frame":"robot_base","objects":[]}'
```

## 9. TCP 单行示例

```json
{"cmd":"detection","frame":"robot_base","frame_id":99,"objects":[{"role":"object","track_id":1,"class_id":1,"label":"wafer_cone","confidence":0.9,"position":{"x":0.4,"y":0,"z":0.1},"wrist_yaw_deg":-5.0}]}
```

## 10. 交叉引用

- 机械臂控制（head → bridge）：[`head2bridge.md`](head2bridge.md)（`claw.wrist_deg` 与本文 `wrist_yaw_deg` 映射关系由 `src/speaker.py` 实现）

## 11. 版本迁移（v1.0 → v1.1）

- v1.0 仅要求 `class_id` / `label` / `position`。
- v1.1 **必选** `role`、`wrist_yaw_deg`；推荐上游提供 `track_id` 以利于 §7 融合。
- 旧客户端须升级；否则 head 校验应返回 `missing_field` / `invalid_value`。
