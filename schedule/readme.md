````markdown
# ROS 2 日程管理器（schedule_manager）

一个基于 **ROS 2 Topic** 的轻量日程管理节点：通过发布/订阅 `std_msgs/String`（承载 JSON）实现“查询/添加/修改/删除/列出/获取时间戳”等操作，适合 UI 或大模型作为调用方。

---

## 目录结构

- `schedule_manager.py`  
  ROS 2 节点实现，节点名 `schedule_manager`。负责：
  - 加载/保存 `schedule_data.json`
  - 日程 CRUD
  - 冲突检测（时间段重叠 → 标记 `status=conflict` 并返回冲突列表）
  - 定时提醒（按 `reminder_minutes` 触发一次性提醒）
  - 提供 Topic-RPC 接口（JSON over Topic）

- `schedule_data.json`  
  日程持久化数据文件（JSON）。节点启动时加载，修改后保存。

- `demo_schedule_rpc.sh`  
  Demo 脚本：在第三个终端用 `ros2 topic pub` 依次测试所有 RPC 接口（`now/list/add/get/update/delete`，并测试一次冲突）。

- `readme.md`  
  本说明文档。

---

## 数据结构（schedule）

每个日程项是一个 JSON 对象，核心字段（满足“事件名称、事件id、开始时间、结束时间、创建时间、事件描述、是否需要提醒、提醒时间、提醒内容、备注”）：

- `id`：事件 id（字符串，唯一）
- `title`：事件名称
- `description`：事件描述
- `start_time`：开始时间（ISO8601 字符串，建议带时区，例如 `2025-12-15T14:00:00+08:00`）
- `end_time`：结束时间（ISO8601）
- `created_at`：创建时间（ISO8601）
- `needs_reminder`：是否需要提醒（bool）
- `reminder_minutes`：提醒时间点（数组，单位分钟，表示“开始前 N 分钟提醒”，例如 `[30, 10]`）
- `reminder_message`：提醒内容（字符串）
- `notes`：备注（字符串）

附加字段（实现内部使用/扩展）：
- `status`：`pending/completed/cancelled/conflict` 等
- `reminder_state.fired_minutes`：已触发过的提醒分钟数（用于去重）
- `updated_at`：更新事件时间（可选）

---

## ROS 2 调用方式（JSON over Topic）

节点使用两个 Topic 组成“请求/响应”模式：

- 请求：`/schedule_manager/request`  （`std_msgs/msg/String`，内容为 JSON）
- 响应：`/schedule_manager/response`（`std_msgs/msg/String`，内容为 JSON）

### 请求格式

```json
{
  "request_id": "可选，用于对应请求/响应",
  "op": "now|list|get|add|delete|update",
  "args": { ... }
}
```

### 响应格式

```json
{
  "request_id": "...",
  "ok": true,
  "op": "...",
  "data": { ... }
}
```

失败时：

```json
{
  "request_id": "...",
  "ok": false,
  "op": "...",
  "error": { "code": "...", "message": "..." }
}
```

---

## 使用方法（推荐“三终端”）

### 终端 1：启动节点

先 source ROS 2（以 Jazzy 为例）：

````bash
source setup.bash
python3 schedule_manager.py
````

> 数据文件默认路径：`/home/schuyler/robot-class/schedule/schedule_data.json`  
> 若文件不存在，节点会创建一个空表并保存。

---

### 终端 2：监听响应

````bash
source setup.bash
ros2 topic echo /schedule_manager/response
````

---

### 终端 3：发起请求（示例）

同样 source 后再调用：

````bash
source setup.bash
````

#### 1) 获取当前时间戳

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"1\",\"op\":\"now\",\"args\":{}}'}"
````

#### 2) 列出所有日程

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"2\",\"op\":\"list\",\"args\":{}}'}"
````

#### 3) 添加日程（若与现有日程时间段重叠，会返回 `conflicts`，并将新日程 `status=conflict`）

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"3\",\"op\":\"add\",\"args\":{\"schedule\":{\"id\":\"2025-12-15-meeting\",\"title\":\"团队会议\",\"description\":\"项目同步\",\"start_time\":\"2025-12-15T14:00:00+08:00\",\"end_time\":\"2025-12-15T15:00:00+08:00\",\"needs_reminder\":true,\"reminder_minutes\":[30,10],\"reminder_message\":\"即将开始团队会议\",\"notes\":\"3楼A区\"}}}'}"
````

#### 4) 获取单条日程

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"4\",\"op\":\"get\",\"args\":{\"id\":\"2025-12-15-meeting\"}}'}"
````

#### 5) 修改日程（patch）

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"5\",\"op\":\"update\",\"args\":{\"id\":\"2025-12-15-meeting\",\"patch\":{\"notes\":\"改到2楼B区\",\"needs_reminder\":false}}}'}"
````

#### 6) 删除日程

````bash
ros2 topic pub --once /schedule_manager/request std_msgs/msg/String \
"{data: '{\"request_id\":\"6\",\"op\":\"delete\",\"args\":{\"id\":\"2025-12-15-meeting\"}}'}"
````

---

## 测试方法（使用自带 demo 脚本）

`demo_schedule_rpc.sh` 会顺序测试所有接口：
- `now`
- `list`
- `add`（正常）
- `get`
- `update`
- `get`（验证更新）
- `add`（制造冲突）
- `list`（验证 conflict）
- `delete` x2
- `list`（最终）

运行：

````bash
chmod +x demo_schedule_rpc.sh
bash demo_schedule_rpc.sh
````

同时保持终端 2 在 `echo /schedule_manager/response`，观察每步返回 JSON。

