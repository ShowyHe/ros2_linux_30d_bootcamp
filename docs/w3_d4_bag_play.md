# w3_d4 bag 回放工程化（固定脚本 + 固定口径 + 固定证据链）

## 1. 今日目标

本日任务不是重新录制 bag，也不是调试导航参数，而是将 bag 回放流程固化为可复现、可验证、可交接的工程化步骤。目标包括：

- 编写统一的回放脚本 `scripts/w3_d4_bag_play.sh`
- 固定 replay 输入样本与 topic 回放口径
- 明确 `--clock` 与 `/clock` 的区别及本次使用原则
- 通过 CLI 与自研工具验证 replay 数据确实可被消费，而不仅仅是“能播放”
- 将验证过程与结果统一落盘到日志

本次回放使用的输入样本为：

    bags/w3_d3_map1_run02

该样本来自 w3_d3 修复后的最终录制结果，已通过 `ros2 bag info` 验证可读。

---

## 2. 输入样本与回放范围

## 2.1 输入样本

本次 replay 固定使用：

    bags/w3_d3_map1_run02

该 bag 在 w3_d3 中已验证包含以下 9 个核心 topic：

- `/clock`
- `/tf`
- `/tf_static`
- `/scan`
- `/map`
- `/odom`
- `/amcl_pose`
- `/cmd_vel`
- `/navigate_to_pose/_action/status`

## 2.2 回放范围

本次回放不采用“播放 bag 内全部 topic”的做法，而是仅回放与导航链路验证直接相关的核心 topic。这样做的原因如下：

- 维持 replay 口径稳定，避免不同 run 之间 topic 集合漂移
- 降低无关 topic 对排查与日志阅读的干扰
- 为后续工具验证、bag 回放排障和自动化脚本复用提供统一输入合同

---

## 3. `/clock` 与 `--clock` 的口径说明

## 3.1 `/clock` 是 topic

`/clock` 是 bag 中实际记录的一条 topic，其消息类型为：

    rosgraph_msgs/msg/Clock

在本次输入样本中，`/clock` 已被录入，因此 replay 时可以直接播放 bag 内已有的时钟消息。

## 3.2 `--clock` 是 ros2 bag play 的选项

`--clock` 不是 topic，而是 `ros2 bag play` 的命令行选项。其作用是由 player 额外生成并发布 `/clock`。
之前在ros2_nav2_protfolio里面，加了--clock，导致出现了两个时间，时间出现问题

## 3.3 本次 replay 的使用原则

由于 `bags/w3_d3_map1_run02` 已经包含 `/clock`，因此本次回放**不额外使用 `--clock`**。原因是：

- 若 bag 自身回放 `/clock`，同时又额外启用 `--clock`，则可能出现多个 `/clock` 发布者
- 多发布者会破坏时钟链路的证据口径，增加后续排查复杂度
- 当前阶段目标是验证“bag 内已有时钟能否被稳定回放”，而不是人为生成第二份时钟

因此，本次口径为：

- bag 内包含 `/clock`
- replay 时不加 `--clock`
- 目标是将 `/clock` 的 publisher count 保持为单一发布者

---

## 4. 回放脚本设计

本日编写脚本：

    scripts/w3_d4_bag_play.sh

脚本设计目标如下：

- 输入一个 bag 目录
- 可选支持 `--loop`
- 固定 topic 白名单
- 不使用 `--clock`
- 将 replay 命令封装为统一入口，避免手工临时敲命令导致口径漂移

脚本的核心使用方式为：

    ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02
    ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02 --loop

---

## 5. 复现步骤

## 5.1 检查输入 bag 可读

在训练仓库根目录执行：

    ros2 bag info bags/w3_d3_map1_run02 | tee -a logs/w3_d4_bag_replay.log

该步骤用于确认输入样本存在、可读、topic 集合完整。

## 5.2 启动 replay

执行：

    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02 2>&1 | tee -a logs/w3_d4_bag_replay.log

该步骤用于启动固定口径的 bag 回放。

## 5.3 验证 `/clock` 发布者口径

在 replay 运行期间执行：

    ros2 topic info -v /clock | sed -n '1,25p' | tee -a logs/w3_d4_bag_replay.log

本次实际输出显示：

    Type: rosgraph_msgs/msg/Clock
    Publisher count: 1
    Node name: rosbag2_player

这说明本次 replay 中 `/clock` 由 `rosbag2_player` 单一发布，符合“不加 `--clock`、不产生双发布者污染”的预期。

## 5.4 验证 `/clock` 实际有消息

执行：

    ros2 topic echo /clock --once | tee -a logs/w3_d4_bag_replay.log

本次实际获得如下消息：

    clock:
      sec: 2978
      nanosec: 200000000
    ---

这说明 `/clock` 不仅存在，而且确实在 replay 中有消息流出。

说明：本次曾尝试使用 `timeout 2 ros2 topic echo /clock | sed ...` 作为辅助检查，但由于 `timeout` 会主动终止前台命令，日志中出现 `Terminated`。该现象不应解释为 `/clock` 无消息，而应视为命令组合方式不适合作为主证据。因此，本文将 `ros2 topic echo /clock --once` 作为 `/clock` 消息链路的正式验证命令。

## 5.5 验证工具可消费 replay 数据

在 replay 运行期间执行：

    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    ros2 run nav2_toolkit topic_probe --ros-args -p topic_odom:=/odom -p print_rate:=1.0 -p min_rate_threshold:=10.0 2>&1 | tee -a logs/w3_d4_bag_replay.log

本次实际输出中，连续出现：

    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=28.997 ...
    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=29.995 ...
    tool=topic_probe level=OK status=OK reason=rate_ok ... topic=/odom rate_hz=29.004 ...

说明 replay 输出的 `/odom` 能被自研工具 `topic_probe` 正常消费，且频率稳定在约 29~30Hz。

---

## 6. 实际结果与现象分析

## 6.1 `nav2_toolkit` 初始不可见的原因

本次验证中，首先仅执行了：

    source /opt/ros/humble/setup.bash

随后运行：

    ros2 pkg executables nav2_toolkit

结果显示：

    Package 'nav2_toolkit' not found

该现象是正常的。原因在于：

- `/opt/ros/humble/setup.bash` 仅加载系统 ROS2 环境
- `nav2_toolkit` 属于用户自建工作区包
- 若未额外执行工作区的：

        source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash

  则系统不会将该包加入当前 shell 的可见环境中

在正确 source 工作区后，执行：

    ros2 pkg executables nav2_toolkit

可见：

    nav2_toolkit goal_sender
    nav2_toolkit tf_guard
    nav2_toolkit topic_probe

说明该包安装链路正常。

## 6.2 将 `topic_probe` 指向 `/clock` 的失败原因

本次曾执行：

    ros2 run nav2_toolkit topic_probe --ros-args -p topic_odom:=/clock

结果出现：

- QoS incompatible
- `No messages will be received from it`
- 连续 `stale_no_msg`

这并不表示 replay 链路本身失败，而是因为 `topic_probe` 的设计目标是对 odom 风格数据进行频率与速度字段观察。将其直接指向 `/clock` 属于工具用途错位，同时也触发了 QoS 不匹配问题。

因此，本日正式验证中不再使用 `topic_probe` 检查 `/clock`，而是采用：

- `ros2 topic info -v /clock`
- `ros2 topic echo /clock --once`

作为 `/clock` 验证手段；  
将 `topic_probe` 仅用于验证 `/odom` 的 replay 可消费性。

---

## 7. 结果结论

本日已完成 bag 回放工程化最小闭环，具体表现如下：

- 固定了 replay 输入样本：`bags/w3_d3_map1_run02`
- 编写了统一回放脚本：`scripts/w3_d4_bag_play.sh`
- 明确了 `/clock` 与 `--clock` 的区别，并确定本次回放不使用 `--clock`
- 通过 `ros2 topic info -v /clock` 验证本次 replay 中 `/clock` 的发布者为 `rosbag2_player`，且 `Publisher count = 1`
- 通过 `ros2 topic echo /clock --once` 验证 replay 时钟消息确实可读
- 通过 `topic_probe` 对 `/odom` 的连续 `rate_ok` 输出，验证 replay 数据可作为后续工具链输入，而不仅仅是“能播放”

这说明 w3_d4 的目标已经达到：bag replay 已从“临时会放”提升为“具备固定脚本、固定口径、固定证据链的工程化流程”。

---

## 8. 本日交付物

- `scripts/w3_d4_bag_play.sh`
- `docs/w3_d4_bag_play.md`
- `logs/w3_d4_bag_replay.log`
- `docs/w3_d4_daily_checklist.md`

后续进入 w3_d5 时，可直接基于本日 replay 口径继续做“工具接入 Nav2 流程并落盘全链路日志”，而无需重新定义 bag 回放输入合同。