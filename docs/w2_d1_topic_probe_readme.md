# Week2 Day1 — nav2_toolkit + topic_probe MVP（w2_d1）

## 目标
- 创建 1 个 ROS2 Python 工具包 `nav2_toolkit`。
- 实现第 1 个入口 `topic_probe`：订阅 `/odom` 并输出证据字段（频率、linear.x、angular.z）。
- 能用 CLI 验证：节点在跑、topic 有数据、参数可读可改。

## 环境与约定
- ROS2：Humble
- 工作区：`~/ros2_linux_30d_bootcamp/ros2_ws`
- 包名：`nav2_toolkit`
- 可执行名：`topic_probe`
- 节点名：`w2_d1_topic_probe`
- 默认订阅：`/odom`
- 参数：
  - `topic_odom`：订阅的里程计 topic（默认 `/odom`）
  - `print_rate`：输出频率（Hz，默认 `1.0`）

## 交付物
- 代码：`ros2_ws/src/nav2_toolkit/`
- 文档：`docs/w2_d1_topic_probe_readme.md`（本文件）
- 日志：`logs/w2_d1_topic_probe.log`
- 清单：`docs/w2_d1_daily_checklist.md`

---

## Step 0 — 预检：确认 /odom 存在且在发布
加载 ROS2 环境：
    source /opt/ros/humble/setup.bash

确认 ros2 命令存在：
    which ros2

确认 /odom 有数据：
    ros2 topic echo /odom --once

---

## Step 1 — 创建 ROS2 Python 包 nav2_toolkit
进入工作区 src：
    cd ~/ros2_linux_30d_bootcamp/ros2_ws/src

创建包（ament_python + 依赖）：
    ros2 pkg create nav2_toolkit --build-type ament_python --dependencies rclpy nav_msgs

检查包结构：
    ls -la nav2_toolkit
    ls -la nav2_toolkit/nav2_toolkit

---

## Step 2 — 新增 topic_probe.py
创建文件（注意：用编辑器创建“文件”，不要用 mkdir 创建成目录）：
    nano nav2_toolkit/nav2_toolkit/topic_probe.py

实现要点：
- Subscription：`nav_msgs/msg/Odometry`
- 每次收到消息更新：
  - `last_lin_x` = `msg.twist.twist.linear.x`
  - `last_ang_z` = `msg.twist.twist.angular.z`
- Timer 每 `1/print_rate` 秒输出一次：
  - `rate_hz`（用计数 / elapsed 估算）
  - `lin_x`
  - `ang_z`

---

## Step 3 — 注册 console_scripts（setup.py）
编辑 `nav2_toolkit/setup.py`，确保 `entry_points` 中包含：
    'console_scripts': [
        'topic_probe = nav2_toolkit.topic_probe:main',
    ],

快速检查（示例）：
    cd ~/ros2_linux_30d_bootcamp/ros2_ws/src/nav2_toolkit
    grep -n "console_scripts" setup.py
    grep -n "topic_probe" setup.py

---

## Step 4 — build + source
回到工作区根目录：
    cd ~/ros2_linux_30d_bootcamp/ros2_ws

构建：
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install

加载工作区 overlay：
    source install/setup.bash

验证可执行入口已生成：
    ros2 pkg executables nav2_toolkit

期望输出包含：
- `nav2_toolkit topic_probe`

---

## Step 5 — 运行与日志落盘
准备日志目录：
    mkdir -p ~/ros2_linux_30d_bootcamp/logs
    rm -f ~/ros2_linux_30d_bootcamp/logs/w2_d1_topic_probe.log

运行并落盘：
    ros2 run nav2_toolkit topic_probe 2>&1 | tee ~/ros2_linux_30d_bootcamp/logs/w2_d1_topic_probe.log

期望现象：
- 按 `print_rate` 的频率持续输出
- 输出字段包含：`rate_hz`、`lin_x`、`ang_z`

---

## Step 6 — CLI 证据链验证
新开终端（同样要 source）：
    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash

1) 节点是否存在：
    ros2 node list | grep w2_d1

2) 参数是否存在：
    ros2 param list /w2_d1_topic_probe
    ros2 param get /w2_d1_topic_probe topic_odom
    ros2 param get /w2_d1_topic_probe print_rate

3) 修改参数（验证可控性）：
    ros2 param set /w2_d1_topic_probe print_rate 2.0
    ros2 param get /w2_d1_topic_probe print_rate

---

## 证据样例（建议粘贴到这里）
- `ros2 pkg executables nav2_toolkit` 的输出（包含 topic_probe）
- `ros2 node list | grep w2_d1` 的输出（包含 /w2_d1_topic_probe）
- 日志中连续 10 行输出（rate_hz/lin_x/ang_z）

---

## 常见坑（本日真实口径）
1) ros2 pkg 子命令写错
- 错：`ros2 pkg executable ...`
- 对：`ros2 pkg executables nav2_toolkit`

2) source 路径写错（少 `/` 或扩展名错）
- 错：`source opt/ros/humble/setup.bash`
- 错：`source /opt/ros/humble/setup.py`
- 对：`source /opt/ros/humble/setup.bash`

3) 把 .py “文件”建成目录
- 错：`mkdir -p nav2_toolkit/nav2_toolkit/topic_probe.py`
- 现象：rm 提示 Is a directory
- 对：用编辑器/ touch 创建文件
    nano nav2_toolkit/nav2_toolkit/topic_probe.py
    touch nav2_toolkit/nav2_toolkit/topic_probe.py

4) 初次 build 的 AMENT_PREFIX_PATH 警告
- 现象：提示 install/<pkg> 路径不存在
- 原因：环境里残留了旧路径，首次 build 前尚未生成
- 处理：完成一次成功 build 后再 source install/setup.bash 即可
