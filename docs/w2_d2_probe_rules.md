# Week2 Day2（w2_d2）— topic_probe 预警升级（LOW_RATE / STALE / NO_MSG）

## 目标
在 w2_d1 的 topic_probe（订阅 /odom，输出 rate_hz / linear.x / angular.z）基础上，增加“异常检测 + 预警输出”，并能用可复现实验制造告警，形成可核验的证据链。

本日只交付一件事：**让 topic_probe 的参数改变能够产生可观察的行为变化**，并在日志中留下明确的 WARN 证据。

---

## 今日新增能力

### 1) LOW_RATE（频率过低）
- 判据：统计窗口内 `rate_hz < min_rate_threshold`
- 典型用途：判断发布端频率异常 / 丢帧 / 节点卡顿

### 2) STALE（断流/卡顿：超过阈值未收到新消息）
- 判据：`now - last_msg_time > stale_timeout_sec`（前提是曾经收到过消息）
- 典型用途：发布源短暂停顿、网络/仿真卡顿、回放时钟异常等

### 3) NO_MSG（从启动到现在一次都没收到）
- 判据：`last_msg_time is None`（启动后从未进入订阅回调）
- 典型原因：
  - topic 名写错（例如 /odom__nope）
  - topic 类型不匹配（例如把 /scan 当 odom 订阅；/scan 通常是 LaserScan，不是 Odometry）
  - 发布源未启动

---

## 关键参数
- topic_odom（string）：默认 /odom
- print_rate（float, Hz）：默认 1.0（计时器打印频率）
- min_rate_threshold（float, Hz）：默认 10.0（触发 LOW_RATE 的阈值）
- stale_timeout_sec（float, sec）：默认 1.0（触发 STALE 的超时阈值）

---

## 执行步骤

### Step 0 — 环境与工作区
说明：
- 先 source ROS2 系统环境，拿到 ros2 / rclpy 等运行环境；
- 再 source 工作区 overlay（install/setup.bash），让系统识别工作区里新编译的包与可执行入口。

命令：
    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    colcon build --symlink-install
    source install/setup.bash

---

## 故障注入实验（必须至少制造 1 次 FAIL；建议三类都跑齐）

### Case A — LOW_RATE（阈值拉高）
目的：人为把阈值设到高于实际 /odom 频率（TB3 仿真常见约 30Hz），触发 LOW_RATE。

命令（同时落盘日志）：
    mkdir -p ~/ros2_linux_30d_bootcamp/logs
    ros2 run nav2_toolkit topic_probe --ros-args -p min_rate_threshold:=40.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

日志路径：
    ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

---

### Case B — STALE（超时设得过小）
目的：把 stale_timeout_sec 设到小于消息周期（30Hz 的周期约 0.033s），让系统频繁判为 STALE。

命令：
    ros2 run nav2_toolkit topic_probe --ros-args -p stale_timeout_sec:=0.01 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

日志路径：
    ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

解释要点（面试口径）：
- STALE 是按“距离上次消息的间隔”判定；
- 30Hz 的间隔约 0.033s，设置 0.01s 属于“故意严苛”，因此会频繁告警。

---

### Case C — NO_MSG（切到错误 topic 或类型不匹配）
目的：让订阅回调永远不触发，从而持续 NO_MSG。

两种方式任选其一：

方式 1：topic 名写错
    ros2 run nav2_toolkit topic_probe --ros-args -p topic_odom:=/odom__nope -p stale_timeout_sec:=1.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

方式 2：切到 /scan（常见类型为 LaserScan，与 Odometry 类型不匹配，回调不会触发）
    ros2 run nav2_toolkit topic_probe --ros-args -p topic_odom:=/scan -p stale_timeout_sec:=1.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

日志路径：
    ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log

---

## 证据提取
从日志中抽取三类告警行：

    grep -nE "LOW_RATE:|STALE:|NO_MSG:" ~/ros2_linux_30d_bootcamp/logs/w2_d2_topic_warning.log | head -n 50

---

## 完成判据（DoD）
- 至少制造 1 次 FAIL，并在日志中出现明确的 WARN 行（LOW_RATE / STALE / NO_MSG 任一）
- 能解释：
  - 三类告警各自判据是什么
  - 为什么参数改变会导致行为改变
  - 为什么 /scan 会 NO_MSG

---

## 产出物
- 代码：
  - ros2_ws/src/nav2_toolkit/nav2_toolkit/topic_probe.py（新增 min_rate_threshold / stale_timeout_sec 并生效）
- 日志证据：
  - logs/w2_d2_topic_warning.log
- 相关前置材料（已在仓库中）：
  - docs/w2_d1_topic_probe_readme.md
  - logs/w2_d1_topic_probe.log

---