# w4_d2 Demo 3min

## 1. 演示目标
本次 3 分钟演示的目标，不是展示复杂地图或极限场景，而是用最短路径证明我已经具备以下能力：

- 能确认 Nav2 系统是否处于可运行状态
- 能用工具给出链路健康证据
- 能通过 action client 发送目标点并读取结果
- 能在失败时根据证据判断先排查哪一层

## 2. 演示前提
### Terminal A：Gazebo
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Terminal B：Nav2
    source /opt/ros/humble/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

## 3. Terminal C 演示剧本

### 3.1 加载环境
    cd ~/ros2_linux_30d_bootcamp
    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash

### 3.2 健康检查
    ./scripts/w3_d2_nav2_healthcheck.sh

说明：
- 先确认关键节点、话题和系统运行前提是否满足
- 避免在系统未就绪时直接发 goal

### 3.3 TF 证据
    ros2 run nav2_toolkit tf_guard --ros-args \
      -p parent_frame:=map \
      -p child_frame:=base_link \
      -p timeout_sec:=0.2

说明：
- 用于确认地图坐标系到机器人本体坐标系的链路是否可查询
- 启动初期若出现一次瞬态 FAIL，应观察后续是否恢复为连续 OK

### 3.4 发送目标点
    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=1.8 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=60.0 \
      -p server_timeout_sec:=3.0 \
      -p action_name:=/navigate_to_pose

说明：
- 重点观察 `goal_sent`、`goal_accepted`、`result_received / result_timeout`
- 目标点采用 Map1 已验证过的稳定点位

## 4. 口播重点
- healthcheck 证明系统具备执行条件
- tf_guard 证明关键 TF 链可用
- goal_sender 证明 action 链路打通
- 若失败，先看 healthcheck，再看 TF，再看 action 返回结果，而不是盲猜

## 5. 演示结论
这套 3 分钟演示的价值不在于“现场一定成功一次导航”，而在于能够用统一工具链和统一证据口径，快速说明系统状态、链路健康和动作执行结果。