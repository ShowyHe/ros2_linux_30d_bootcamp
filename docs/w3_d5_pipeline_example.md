# w3_d5 live Nav2 全链路日志示例（healthcheck + topic_probe + tf_guard + goal_sender）

## 1. 今日目标

本日任务不是 bag 回放，也不是参数优化，而是将前面已实现的工具真正接入 live Nav2 流程，并将整条链路的输出统一落盘为一份可复盘、可排障、可追溯的总日志。目标包括：

- 在 live Gazebo + live Nav2 环境下运行系统
- 启动并记录 `healthcheck`
- 启动并记录 `topic_probe`
- 启动并记录 `tf_guard`
- 使用 `goal_sender` 触发一次真实导航动作
- 将以上输出统一写入：

    logs/w3_d5_full_pipeline.log

本日的核心不是“每个工具都单独能跑”，而是验证这些工具能否在同一条 live pipeline 中协同工作，并形成一份工程化证据日志。

---

## 2. 本次 live pipeline 的组成

本次 live pipeline 由以下部分构成：

- Gazebo 仿真环境
- Nav2 导航系统
- `w3_d2_nav2_healthcheck.sh`
- `nav2_toolkit/topic_probe`
- `nav2_toolkit/tf_guard`
- `nav2_toolkit/goal_sender`

其中，各组件的职责如下：

### 2.1 healthcheck
用于验证系统是否已进入基本健康状态，包括：

- 关键节点是否存在
- 关键 topic 是否存在发布者
- `map_server` 是否处于 `active` 状态

### 2.2 topic_probe
用于持续观察 `/odom` 的频率与消息新鲜度，判断机器人运动状态数据是否持续可用。

### 2.3 tf_guard
用于持续检查关键 TF 链是否健康。本次固定检查：

    map -> odom

### 2.4 goal_sender
用于发送一次真实 goal，触发 live Nav2 执行动作，使系统进入完整导航流程。

---

## 3. 复现步骤

## 3.1 准备主日志文件

在训练仓库根目录执行：

    cd ~/ros2_linux_30d_bootcamp
    mkdir -p logs
    : > logs/w3_d5_full_pipeline.log

该步骤用于清空旧日志，保证本日总日志仅包含当天运行结果。

## 3.2 启动 live 系统

### Terminal A：Gazebo

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Terminal B：Nav2

    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

## 3.3 运行 healthcheck

### Terminal C

    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ./scripts/w3_d2_nav2_healthcheck.sh 2>&1 | tee -a logs/w3_d5_full_pipeline.log
    echo "hc_script_exit_code=${PIPESTATUS[0]} tee_exit_code=${PIPESTATUS[1]}" | tee -a logs/w3_d5_full_pipeline.log

## 3.4 运行 topic_probe

### Terminal D

    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ros2 run nav2_toolkit topic_probe --ros-args -p topic_odom:=/odom -p print_rate:=1.0 -p min_rate_threshold:=10.0 2>&1 | tee -a logs/w3_d5_full_pipeline.log

## 3.5 运行 tf_guard

### Terminal E

    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ros2 run nav2_toolkit tf_guard --ros-args -p parent_frame:=map -p child_frame:=odom -p timeout_sec:=0.2 2>&1 | tee -a logs/w3_d5_full_pipeline.log

## 3.6 运行 goal_sender

### Terminal F

    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ros2 run nav2_toolkit goal_sender --ros-args -p frame_id:=map -p x:=1.8 -p y:=0.0 -p yaw:=0.0 2>&1 | tee -a logs/w3_d5_full_pipeline.log

在 goal_sender 完成后，topic_probe 与 tf_guard 继续运行数秒，以记录动作前后状态变化。

---

## 4. 主日志文件

本日所有关键工具输出统一落盘到：

    logs/w3_d5_full_pipeline.log

该日志的作用不是保存单一工具输出，而是保存一次完整 live Nav2 过程中的多工具联合证据，包括：

- 系统基础健康状态
- TF 链健康状态
- `/odom` 频率与新鲜度
- goal 发送、接受与结果

因此，这份日志可作为后续排障、复盘、演示和文档撰写的主证据源。

---

## 5. 实际结果

## 5.1 healthcheck 结果

日志中可见：

    reason=summary result=PASS fail_count=0

同时，前置检查项也全部通过，包括：

- `/bt_navigator` 存在
- `/planner_server` 存在
- `/controller_server` 存在
- `/map_server` 存在
- `/tf` 发布者正常
- `/tf_static` 发布者正常
- `/map` 发布者正常
- `/map_server state=active`

这说明本次 pipeline 并非带故障启动，而是在基础链路健康的前提下运行。

## 5.2 topic_probe 结果

日志中持续出现：

    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=28.998 ...
    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=30.006 ...
    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=29.001 ...

说明：

- `/odom` 持续在流
- 频率稳定在约 29~30Hz
- 未出现 `low_rate`
- 未出现 `stale_timeout`
- 未出现 `stale_no_msg`

此外，输出中的 `lin_x`、`ang_z` 在动作阶段存在明显变化，随后逐渐回落到接近静止值，说明机器人在 goal 执行阶段确实发生了运动，并在完成后进入近静止状态。

## 5.3 tf_guard 结果

日志中连续出现：

    tool=tf_guard level=OK status=OK reason=tf_ok parent=map child=odom timeout_sec=0.200

说明本次导航过程中关键 TF 链：

    map -> odom

始终健康，未出现 timeout 或缺失现象。

这意味着本次导航链路中，TF 不是问题来源。

## 5.4 goal_sender 结果

日志中可见：

    tool=goal_sender level=OK status=OK reason=goal_sent action=/navigate_to_pose frame=map x=1.800 y=0.000 yaw=0.000
    tool=goal_sender level=OK status=OK reason=goal_accepted action=/navigate_to_pose
    tool=goal_sender level=OK status=OK reason=result_received action=/navigate_to_pose result_status=4 result_status_name=SUCCEEDED time_sec=26.363

说明：

- goal 已成功发出
- goal 已被 action server 接受
- 最终成功完成
- 本次耗时约 `26.363s`

这与 `topic_probe` 中动作阶段的速度变化以及 `tf_guard` 的持续 `tf_ok` 相互印证，形成完整闭环。

---

## 6. 异常排查时的证据优先级

本日的一个核心产出，不是“所有工具都跑了一遍”，而是明确了异常排查时的证据阅读顺序。建议按以下优先级处理：

## 6.1 第一优先：healthcheck

首先查看：

    tool=healthcheck ... reason=summary result=...

若 `healthcheck` 未通过，则优先处理系统基础问题，例如：

- 关键节点未启动
- 关键 topic 无发布者
- `map_server` 未进入 active

在此阶段，不应急于分析 TF 或 action 结果。

## 6.2 第二优先：tf_guard

若 healthcheck 已通过，但导航表现异常，应优先检查：

    tool=tf_guard ...

重点看：

- 是否持续 `tf_ok`
- 是否出现 timeout
- 是否出现 transform 缺失

若 `map -> odom` 不健康，则后续导航动作往往不具备可靠解释价值。

## 6.3 第三优先：topic_probe

在 TF 正常的前提下，再查看：

    tool=topic_probe ...

重点看：

- 是否持续 `rate_ok`
- 是否出现 `low_rate`
- 是否出现 `stale_timeout`
- 是否出现 `stale_no_msg`

该步骤用于判断 `/odom` 是否持续可用，以及运动状态数据是否正常。

## 6.4 第四优先：goal_sender

当前三类基础证据都正常后，再查看：

    tool=goal_sender ...

重点看：

- goal 是否 sent
- goal 是否 accepted
- 是否拿到 result
- result 是 `SUCCEEDED`、`ABORTED` 还是其他状态

这种顺序可以避免将“基础链路故障”误判为“goal 没发出去”或“action 异常”。

---

## 7. 结论

本日已完成 live Nav2 全链路日志的最小工程化闭环，具体表现为：

- live Gazebo 与 live Nav2 环境正常启动
- `healthcheck` 通过，系统基础状态健康
- `tf_guard` 连续输出 `tf_ok`，关键 TF 链健康
- `topic_probe` 连续输出 `rate_ok`，`/odom` 稳定在约 29~30Hz
- `goal_sender` 完成一次成功导航，耗时约 `26.363s`
- 所有关键输出统一落盘到：

    logs/w3_d5_full_pipeline.log

这说明当前工具链已经不再是分散的单点工具，而是形成了一条可记录、可复盘、可排障的 live Nav2 pipeline。后续进入 w3_d6 时，可直接在该基础上推进回归脚本与 CSV 结果落盘，而不需要重新定义观测与排障口径。