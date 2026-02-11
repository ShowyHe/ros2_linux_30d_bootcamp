# W1D6 — ROS2 CLI 证据链：topic QoS / param / service / action（Humble + TB3 + Nav2）

## 0. 范围与目标
本文记录 W1D6 学习与实操结论，面向“可复现/可核验”的工程证据链。覆盖：
- `ros2 topic info -v`：发布者/订阅者计数与 QoS 端点证据
- `ros2 param get/set/dump`：参数“前→改→后”的可核验证据
- `ros2 service list/type/call`：服务调用（一次请求一次响应）的可核验证据
- `ros2 action list -t/info/send_goal`：动作任务（goal/feedback/result）的可核验证据

---

## 1. 统一排查口径（存在 → 关系 → 运行态）
### 1.1 存在性（对象是否出现）
    ros2 node list
    ros2 topic list
    ros2 service list
    ros2 action list -t

### 1.2 关系（谁连着谁）
- 节点侧（订阅/发布、服务、动作）：
    ros2 node info <node>
- Topic 侧（端点、计数、QoS）：
    ros2 topic info -v <topic>
- Service 侧（协议模板）：
    ros2 service type <service>
- Action 侧（server/client 归属）：
    ros2 action info <action>

### 1.3 运行态（是否在跑）
- Topic 数据采样/频率（用于“有发布者但数据不来”的确认）：
    ros2 topic echo --once <topic>
    ros2 topic hz <topic>
- Service 调用（一次请求一次响应）：
    ros2 service call ...
- Action 任务（goal→feedback→result）：
    ros2 action send_goal ... --feedback

---

## 2. type 的定义（协议模板）
type 是“通信协议模板”，规定字段名、层级结构与类型；不按模板写会导致解析失败或业务失败。

- topic type：`pkg/msg/MsgName`
- service type：`pkg/srv/SrvName`（Request / Response）
- action type：`pkg/action/ActionName`（Goal / Result / Feedback）

查看模板（以定义为准）：
    ros2 interface show <type>

---

## 3. QoS 口径（QoS 属于 endpoint，端点之间按规则匹配）
### 3.1 endpoint QoS：每个端点一份，并按 3.3 规则相互匹配
QoS 不属于“topic 自身一份固定值”；QoS 属于 **endpoint（端点）**：
- Publisher endpoint：发布端点（谁在发）
- Subscription endpoint：订阅端点（谁在收）

`ros2 topic info -v <topic>` 展示的是该 topic 上所有 pub/sub endpoint 及各自 QoS。  
这些 endpoint 之间需要按 3.3 规则做兼容匹配；不兼容会出现“看见发布者但收不到/显示空”等问题。

### 3.3 QoS 兼容规则（最小必备）
**Reliability**
- Pub=RELIABLE，Sub=BEST_EFFORT ✅
- Pub=BEST_EFFORT，Sub=RELIABLE ❌

**Durability**
- Pub=TRANSIENT_LOCAL，Sub=VOLATILE ✅
- Pub=VOLATILE，Sub=TRANSIENT_LOCAL ❌

### 3.4 `/clock` 的 QoS 表述（严格写法）
`/clock` 是 topic 名字，本身没有“唯一 QoS”。  
但 `/clock` 上的 **publisher endpoint** 与 **subscriber endpoints** 各自有 QoS，并按 3.3 规则匹配。

示例证据（来自 `ros2 topic info -v /clock`）：
- 发布者：gazebo（Publisher count=1）
  - QoS：BEST_EFFORT + VOLATILE
- 订阅者：多个 Nav2/Gazebo/RViz 节点（Subscription count>0）
  - QoS：同为 BEST_EFFORT + VOLATILE
- 结论：端点 QoS 兼容，链路成立。

---

## 4. Node name / Node namespace 与“全名”的定义
在 `ros2 topic info -v` 输出中常见两项：
- **Node name**：节点短名（name）
- **Node namespace**：节点命名空间（namespace，用于分组/隔离）

ROS2 的节点**全名**由两者拼接得到：
- 全名 = `<namespace>/<name>`

原因：
- 支持同名节点在不同 namespace 下共存（避免命名冲突）
- 支持系统分层（例如 costmap 组件在各自 namespace 下组织）

示例（从 topic -v 得到）：
- Node name: `global_costmap`
- Node namespace: `/global_costmap`
- 节点全名：`/global_costmap/global_costmap`

因此查询节点信息应使用全名：
    ros2 node info /global_costmap/global_costmap

---

## 6. Service：一次请求一次响应（不需要 `--once` 这种采样思路）
### 6.1 模型定义
Service 是同步请求-响应模型：
- client 发送 request
- server 返回 response
一次调用结束；因此不需要像 topic/action 那样用 `--once` 来“采样消息流”。

### 6.2 Humble CLI 子命令范围（无 `ros2 service info`）
Humble 版本 `ros2 service` 子命令为：
- `list / type / call / find`
无 `info` 子命令，出现 invalid choice 属正常行为。

### 6.3 `type` 与 `call` 的含义
- `ros2 service type <service>`：给出服务的协议模板（`pkg/srv/Name`）
- `ros2 service call <service> <type> "<request>"`：按模板构造 request 并发送，等待 response

#### 例 A：Empty 服务（空 request / 空 response）
命令：
    ros2 service call /reset_world std_srvs/srv/Empty "{}"

解释：
- type：`std_srvs/srv/Empty`
- request：`{}` 合法（无字段）
- response：`Empty_Response()`（无字段）
- 证据点：调用成功返回 response，链路成立。

接口定义验证：
    ros2 interface show std_srvs/srv/Empty
输出仅 `---`，表示 request/response 两侧均无字段。

#### 例 B：SpawnEntity（业务字段缺失导致失败）
命令（示例）：
    ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity {}

现象：
- request 中关键字段（如 `xml`）为空字符串
- response：`success=False`，并提示 XML 解析失败

结论：
- 语法层面可调用，但业务字段不满足要求导致失败；失败本身也是证据。

### 6.4 如何定位“哪个节点提供该 service”
`ros2 service call` 不显示 provider。可从节点侧反查（扫描各节点的 Service Servers）：
    for n in $(ros2 node list); do ros2 node info $n | grep -q "/reset_world" && echo "provider: $n"; done

---

## 7. Action：任务式通信（goal / feedback / result）
### 7.1 模型定义
Action 面向“持续执行的任务”：
- client 发送 goal
- server 可持续发送 feedback
- server 最终返回 result

### 7.2 `send_goal` 的定义
`send_goal` 的作用是：按 action type 解析 goal 数据并发送给 action server；可选打印 feedback；等待 result。

### 7.3 `NavigateToPose` 示例：命令与 goal 字段来源（把 `{pose: ...}` 写死）
完整命令（含落盘）：
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" --feedback 2>&1 | tee -a logs/w1_d6_action_test.log

逐段含义：
- `ros2 action send_goal`：向 action server 发送“任务目标（Goal）”
- `/navigate_to_pose`：action 名称（接口名）
- `nav2_msgs/action/NavigateToPose`：action type（协议模板）
- `"{pose: {...}}"`：Goal 数据（YAML），字段结构必须符合模板
- `--feedback`：打印执行过程反馈
- `2>&1 | tee -a ...`：合并 stdout/stderr 并追加写入日志

Goal 字段从哪里来：以接口定义为准（不可猜）：
    ros2 interface show nav2_msgs/action/NavigateToPose

其中 Goal 段定义包含 `pose`（类型为 `geometry_msgs/msg/PoseStamped`）。因此 `{pose: ...}` 是给 Goal.pose 赋值。  
该 `pose` 内部结构（header/pose/position/orientation）由 `PoseStamped/Pose` 等 msg 定义决定。

---

## 8. Param：`dump` 报错原因（命令语义问题，不是系统问题）
现象：
- 执行 `ros2 param dump <node> <param_name>` 报 `unrecognized arguments: <param_name>`

原因：
- `ros2 param dump` 语义是“导出节点参数集合”，不支持附带单个参数名作为位置参数。

正确用法：
    ros2 param dump <node> 2>&1 | tee -a logs/w1_d6_action_test.log

若只需某个参数值，应使用：
    ros2 param get <node> <param_name>

---

## 9. 日志落盘：`2>&1 | tee -a` 的含义（最小必须会）
用于把“正常输出 + 错误输出”一起落盘，避免只记录成功信息。

- `2>&1`：将 stderr（2）重定向到 stdout（1）当前指向处
- `| tee -a file`：同时输出到终端并追加写入 file

标准落盘模板：
    <command> 2>&1 | tee -a logs/w1_d6_action_test.log
