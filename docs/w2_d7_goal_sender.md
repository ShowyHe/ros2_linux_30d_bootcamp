# w2_d7 Goal Sender MVP（不依赖 RViz 发 Nav2 Goal）

## 1. 今日目标
完成 `nav2_goal_sender`（Action Client），通过 `/navigate_to_pose` 发送导航目标，并按统一 status line 契约输出证据；随后进行 Map1 回归测试并落 CSV。
今日输出：
### 代码与工具（Python 源码）
- `ros2_ws/src/nav2_toolkit/nav2_toolkit/goal_sender.py`（本日新增，Nav2 Action Client MVP）

### 文档
- `docs/w2_d7_goal_sender.md`

### 日志与结果
- `logs/w2_d7_goal_sender.log`
- `results/w2_d7_map1_5runs.csv`

## 2. 完成内容
- 已新增 `goal_sender`（`nav2_toolkit` 内）
- 已通过 `/navigate_to_pose` 成功发送 goal
- 已拿到最终结果（`result_received`）
- 已验证结果状态码 `4 = SUCCEEDED`
- 已完成多次回归（日志显示 1 次冒烟 + 10 次正式回归候选）

## 3. Action 链路（证据化说明）
`goal_sender` 的执行链路如下：

1. `wait_for_server(timeout_sec=...)`
   - 等待 `/navigate_to_pose` 的 action server 就绪
   - 若超时，输出 `FAIL server_not_ready`

2. `send_goal_async(goal_msg)` + `spin_until_future_complete(...)`
   - 异步发送 goal，请求 server 接单
   - 获取 `goal_handle`
   - 若 `goal_handle.accepted == False`，输出 `FAIL goal_rejected`

3. `goal_handle.get_result_async()` + `spin_until_future_complete(..., timeout_sec=...)`
   - 等待导航任务最终结果
   - 若超时，输出 `FAIL result_timeout`
   - 若收到结果，输出 `result_status` 与 `result_status_name`

## 4. 本次验证到的关键结论
本次日志中多次出现：

    reason=result_received ... result_status=4 result_status_name=SUCCEEDED

说明：
- Action server 接单成功
- Nav2 执行完成并返回成功结果
- 当前 `goal_sender` MVP 已满足 Week2 Day7 的“可发 goal + 可收结果”要求

## 5. 使用命令（最小可运行）

### 5.1 build（在 ros2_ws）
    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    colcon build --packages-select nav2_toolkit --symlink-install
    source install/setup.bash

### 5.2 单次发送 goal（在训练根目录）
    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash
    cd ~/ros2_linux_30d_bootcamp

    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=1.8 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=120.0

### 5.3 反向目标（回到起点，保证每次起点终点一致）
    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=-2.0 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=120.0

## 6. 参数说明（当前 MVP）
- `frame_id`：目标位姿所属坐标系（默认 `map`）
- `x` / `y` / `yaw`：目标位置与朝向
- `timeout_sec`：等待最终结果超时（导航执行阶段）
- `server_timeout_sec`：等待 action server 就绪超时
- `action_name`：action 接口名（默认 `/navigate_to_pose`）

## 7. 输出契约（status line）样例

### 7.1 goal 已发送
    ts=2026-02-24T14:17:17+08:00 tool=goal_sender level=OK status=OK reason=goal_sent action=/navigate_to_pose frame=map x=1.800 y=0.000 yaw=0.000

### 7.2 server 已接单
    ts=2026-02-24T14:17:17+08:00 tool=goal_sender level=OK status=OK reason=goal_accepted action=/navigate_to_pose

### 7.3 收到最终结果（成功）
    ts=2026-02-24T14:17:51+08:00 tool=goal_sender level=OK status=OK reason=result_received action=/navigate_to_pose result_status=4 result_status_name=SUCCEEDED time_sec=34.746

## 8. 回归记录（本次日志摘要）
本次日志包含：
- 1 次冒烟测试（x=0.0）
- 10 次往返回归候选（x=1.8 与 x=-2.0 交替）
