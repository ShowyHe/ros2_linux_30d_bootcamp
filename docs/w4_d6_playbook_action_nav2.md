# w4_d6 Playbook：Action / Nav2 排障手册

## 1. 适用场景
本页用于处理以下类型的问题：

- 发了 goal，但机器人看起来没动
- `goal_sender` 执行后没有预期行为
- 不知道 action server 是否真的存在
- 不知道是 goal 没发出去，还是发出去了但执行没完成
- 出现 timeout，不确定该查 action 层还是其他层

本页的目标不是讲 action 理论，而是给出一个固定排障顺序：如何确认 action server 是否存在、如何确认 goal 是否真正被接受、以及 timeout 时该把问题归到哪一层。

## 2. 先看什么：action server 是否存在

### 2.1 查看系统中有哪些 action
命令：
    ros2 action list

作用：
- 列出当前系统中可见的 action 名称

判断：
- 若 `/navigate_to_pose` 根本不在列表中，先不要讨论 goal 点是否合理，问题首先在 action server 层

### 2.2 查看 `/navigate_to_pose` 的详细信息
命令：
    ros2 action info /navigate_to_pose

作用：
- 查看该 action 的类型，以及 server / client 情况

判断：
- 若 action 名存在，说明系统至少暴露了导航 action 接口
- 若 action 名不存在，应回到 bringup / lifecycle / healthcheck 层排查

## 3. 如何确认 goal 是否真正发出并被接受

### 3.1 使用 `goal_sender` 发送正常目标点
命令：
    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=1.8 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=60.0 \
      -p server_timeout_sec:=3.0 \
      -p action_name:=/navigate_to_pose

正常观察点：
- `reason=goal_sent`
- `reason=goal_accepted`
- 最后出现：
  - `reason=result_received`
  或
  - `reason=result_timeout`

判断：
- 若看到了 `goal_sent`，说明客户端侧发送动作已执行
- 若看到了 `goal_accepted`，说明 action server 已接受该 goal
- 若后续 timeout，则问题不再是“发没发出去”，而是“执行阶段未在时限内完成”

## 4. 如何构造并确认“server 不存在”的失败样例

为了验证工具是否能识别 action server 层异常，可故意指定一个不存在的 action 名：

命令：
    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=1.8 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=10.0 \
      -p server_timeout_sec:=1.0 \
      -p action_name:=/ghost_action

异常现象：
- 返回 `server_not_ready`
- 进程退出码非 0

结论：
- 这类问题说明 action server 本身不可用
- 应优先回到：
  - Nav2 bringup
  - lifecycle
  - healthcheck
  层排查，而不是先改目标点

## 5. CLI 原生命令证据

除了自研 `goal_sender`，还可以用 ROS2 CLI 直接发 goal：

命令：
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 1.8, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
    --feedback

作用：
- 用原生命令确认 action 接口可用
- 用 `--feedback` 观察执行过程中的反馈

判断：
- 若 CLI 方式也无法工作，应优先怀疑系统 action 层本身，而不是单纯怀疑自研工具实现

## 6. 推荐排障顺序

### 第一步：确认 action 名是否存在
命令：
    ros2 action list
    ros2 action info /navigate_to_pose

目的：
- 先判断 action server 在不在

### 第二步：确认 server 是否 ready
命令：
    ros2 run nav2_toolkit goal_sender --ros-args ... -p action_name:=/navigate_to_pose

目的：
- 看是否出现 `server_not_ready`

### 第三步：确认 goal 是否被接受
观察：
- `goal_sent`
- `goal_accepted`

目的：
- 区分“客户端发没发出去”和“服务端有没有接收”

### 第四步：看最终结果
观察：
- `result_received`
- `result_timeout`

目的：
- 若 timeout，说明问题已经进入执行阶段，不应再停留在 action server 存在性层

## 7. 如何读结果

### 情况 A：`server_not_ready`
判断：
- action server 不可用

处理：
- 先查 bringup / lifecycle / healthcheck

### 情况 B：`goal_sent` 有，但没有 `goal_accepted`
判断：
- 重点怀疑发送链路或服务端处理异常

处理：
- 先查 action server 状态与接口一致性

### 情况 C：`goal_accepted` 后 `result_timeout`
判断：
- action 链路通了，但执行阶段没在规定时间内完成

处理：
- 转到 TF / map / controller / localization / goal 合理性层排查

### 情况 D：`result_received`
判断：
- action 整条链闭合
- 接下来再根据结果是 `SUCCEEDED / ABORTED / CANCELED` 做后续分析

## 8. 结论
Action 排障的关键，不在于“发了一次 goal 就算会了”，而在于能按固定顺序给证据：

1. action server 在不在
2. server 是否 ready
3. goal 是否发出
4. goal 是否被接受
5. 执行是否完成

只有先把 action 层切清楚，后面再查 TF、QoS、map、controller 才不会乱。