# w4_d1 Tools Polish

## 1. 目标
本日目标是将 `nav2_toolkit` 收口为一个可交付、可复现、可展示的小工具包，使新终端在完成环境加载后，能够直接运行 `topic_probe`、`tf_guard`、`goal_sender` 中任意一个工具。

## 2. 包内工具清单
当前 `nav2_toolkit` 对外暴露的可执行入口为：

- `topic_probe`
- `tf_guard`
- `goal_sender`

对应 `setup.py` 中的 `console_scripts` 已确认如下：

- `topic_probe = nav2_toolkit.topic_probe:main`
- `tf_guard = nav2_toolkit.tf_guard:main`
- `goal_sender = nav2_toolkit.goal_sender:main`

## 3. 统一使用前提

### 3.1 环境加载
    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash

### 3.2 系统前提
- Gazebo 已启动
- Nav2 已启动
- `/odom` 可用
- TF 链路已建立
- `/navigate_to_pose` action server 可用

## 4. 工具与参数口径

### 4.1 topic_probe
功能：订阅里程计话题，输出频率、速度与断流状态。

参数：
- `topic_odom`
- `print_rate`
- `min_rate_threshold`
- `stale_timeout_sec`

示例命令：
    ros2 run nav2_toolkit topic_probe --ros-args \
      -p topic_odom:=/odom \
      -p print_rate:=1.0 \
      -p min_rate_threshold:=10.0 \
      -p stale_timeout_sec:=1.0

本次 smoke test 结果：能够连续输出 `tool=topic_probe` 状态行，频率约 29~30 Hz，状态为 `rate_ok`。

### 4.2 tf_guard
功能：检查指定 TF 链路是否可用。

参数：
- `parent_frame`
- `child_frame`
- `timeout_sec`

示例命令：
    ros2 run nav2_toolkit tf_guard --ros-args \
      -p parent_frame:=map \
      -p child_frame:=base_link \
      -p timeout_sec:=0.2

本次 smoke test 结果：启动初期出现一次瞬态 `tf_missing`，随后连续输出 `tf_ok`。说明工具可正常工作，判读时应关注连续状态而非单条启动瞬态。

### 4.3 goal_sender
功能：向 Nav2 `navigate_to_pose` action 发送目标点。

参数：
- `frame_id`
- `x`
- `y`
- `yaw`
- `timeout_sec`
- `server_timeout_sec`
- `action_name`

示例命令：
    ros2 run nav2_toolkit goal_sender --ros-args \
      -p frame_id:=map \
      -p x:=1.8 \
      -p y:=0.0 \
      -p yaw:=0.0 \
      -p timeout_sec:=120.0 \
      -p server_timeout_sec:=3.0 \
      -p action_name:=/navigate_to_pose

本次 smoke test 结果：成功建立 action 链路，出现 `goal_sent` 与 `goal_accepted`，本轮最终为 `result_timeout`。这说明工具本身工作正常，但该目标点在本次 live 场景下未在 120 s 内完成导航。

## 5. 输出口径
三个工具均采用状态行输出，核心字段包括：

- `ts`
- `tool`
- `level`
- `status`
- `reason`

其中：
- `topic_probe` 还会输出 `topic / rate_hz / threshold / stale_sec`
- `tf_guard` 还会输出 `parent / child / timeout_sec`
- `goal_sender` 还会输出 `action / frame / x / y / yaw / time_sec`

## 6. 本日结论
Week4 Day1 的重点不是增加新工具，而是将已有 `nav2_toolkit` 收口到“别人拿到后能理解、能复制、能运行”的程度。

本日已确认：
- 对外工具名稳定为 `topic_probe / tf_guard / goal_sender`
- 参数口径基本明确
- 三个工具均完成了 live 环境 smoke test
- 源码树与 install 版本不一致问题已被定位，需以源码修复和重新 build 作为后续收口标准

这一步的价值高于继续增加新脚本，因为它直接关系到仓库的可复现性、对外展示质量和面试中的可信度。