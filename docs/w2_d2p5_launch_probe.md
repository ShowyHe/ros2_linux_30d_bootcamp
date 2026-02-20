# Week2 Day2.5（w2_d2p5）Launch / 参数注入 / namespace 证据链

日期：2026-02-20

## 目标

用 Python launch 启动 `nav2_toolkit/topic_probe`，并完成三件事的“可核验”证据链：

- 参数注入：`topic_odom / print_rate / min_rate_threshold / stale_timeout_sec` 能被 launch 覆写，并被节点读回
- namespace 生效：节点全名体现 namespace（例如 `/r1/w2_d1_topic_probe`）
- 行为变化可观测：通过调整 `min_rate_threshold` 或 `topic_odom / stale_timeout_sec` 触发 `LOW_RATE / NO_MSG / STALE` 等告警

## 交付物

- `ros2_ws/src/nav2_toolkit/launch/probes.launch.py`
- `ros2_ws/src/nav2_toolkit/setup.py`（安装 launch 文件到 share）
- `logs/w2_d2p5_launch_probe.log`
- 证据命令输出：`ros2 node list` / `ros2 param get` / `ros2 node info`

## 实现步骤

### 1. 创建 launch 文件

路径：

    ros2_ws/src/nav2_toolkit/launch/probes.launch.py

核心要点：

- 使用 Launch 参数：`ns / topic_odom / print_rate / min_rate_threshold / stale_timeout_sec`
- Node 启动 `nav2_toolkit` 的 `topic_probe`
- namespace 绑定到 `ns`
- parameters 中将 LaunchConfiguration 注入到节点参数

（示例结构，实际以仓库内容为准）

    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    def generate_launch_description():
        ns = LaunchConfiguration("ns")
        topic_odom = LaunchConfiguration("topic_odom")
        print_rate = LaunchConfiguration("print_rate")
        min_rate_threshold = LaunchConfiguration("min_rate_threshold")
        stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")

        return LaunchDescription([
            DeclareLaunchArgument("ns", default_value="r1"),
            DeclareLaunchArgument("topic_odom", default_value="/odom"),
            DeclareLaunchArgument("print_rate", default_value="1.0"),
            DeclareLaunchArgument("min_rate_threshold", default_value="10.0"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="1.0"),

            Node(
                package="nav2_toolkit",
                executable="topic_probe",
                name="w2_d1_topic_probe",
                namespace=ns,
                output="screen",
                parameters=[{
                    "topic_odom": topic_odom,
                    "print_rate": print_rate,
                    "min_rate_threshold": min_rate_threshold,
                    "stale_timeout_sec": stale_timeout_sec,
                }],
            ),
        ])

### 2. 让 ros2 launch 找得到 launch 文件（setup.py）

在 `setup.py -> data_files` 增加安装项：

    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

意义：把 `launch/*.launch.py` 安装到

    install/nav2_toolkit/share/nav2_toolkit/launch/

否则命令：

    ros2 launch nav2_toolkit probes.launch.py

会找不到 launch 文件。

### 3. build + source（两次 source 证据链）

    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    source /opt/ros/humble/setup.bash
    rm -rf build install log
    colcon build --symlink-install
    source install/setup.bash

- source /opt/ros/humble/setup.bash：加载 ROS2 系统环境（ros2 命令、系统包）
- source install/setup.bash：将本工作区 install 前缀叠加进环境，使 ROS2 能发现本地包/可执行/launch 文件

### 4. 运行

落盘日志：

    mkdir -p ~/ros2_linux_30d_bootcamp/logs

启动：

    ros2 launch nav2_toolkit probes.launch.py \
      ns:=r1 topic_odom:=/odom print_rate:=1.0 \
      min_rate_threshold:=10.0 stale_timeout_sec:=1.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

常见误用：

- 用 `ros2 run nav2_toolkit probes.launch.py` 会报 “No executable found”
  - 原因：launch 文件不是 console_scripts 可执行入口

## 验证与证据链

### 1) namespace 生效（node 全名）

    ros2 node list

### 2) 参数注入成功（读回参数）

    ros2 param get /r1/w2_d1_topic_probe topic_odom
    ros2 param get /r1/w2_d1_topic_probe print_rate
    ros2 param get /r1/w2_d1_topic_probe min_rate_threshold
    ros2 param get /r1/w2_d1_topic_probe stale_timeout_sec

### 3) 订阅/发布/服务可见（node info）

    ros2 node info /r1/w2_d1_topic_probe

## 失败注入（证明行为变化）

### Case A：LOW_RATE（阈值设高）

说明：/odom 约 30Hz，设置 min_rate_threshold 大于 30 会触发告警。

注意：如果 declare 为 DOUBLE，传参必须写 40.0（不要写 40）。

    ros2 launch nav2_toolkit probes.launch.py \
      ns:=r1 topic_odom:=/odom print_rate:=1.0 \
      min_rate_threshold:=40.0 stale_timeout_sec:=1.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

- 结果详见： ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

### Case B：NO_MSG（把订阅话题指向不存在）

    ros2 launch nav2_toolkit probes.launch.py \
      ns:=r1 topic_odom:=/scan print_rate:=1.0 \
      min_rate_threshold:=10.0 stale_timeout_sec:=1.0 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

- 结果详见： ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

### Case C：STALE（超时阈值极小，制造抖动/漏包也会触发）

    ros2 launch nav2_toolkit probes.launch.py \
      ns:=r1 topic_odom:=/odom print_rate:=1.0 \
      min_rate_threshold:=10.0 stale_timeout_sec:=0.03 \
      2>&1 | tee -a ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

- 结果详见： ~/ros2_linux_30d_bootcamp/logs/w2_d2p5_launch_probe.log

## 踩坑记录（本次遇到的真实问题）

### 1) 参数类型不匹配（INTEGER vs DOUBLE）

现象：launch 启动时进程直接退出，报：

- Trying to set parameter 'min_rate_threshold' to '40' of type 'INTEGER', expecting type 'DOUBLE'

原因：

- `declare_parameter("min_rate_threshold", 10.0)` 将参数类型固定为 DOUBLE
- launch 传 `40` 会被当成 INTEGER

修复：

- 传参写 `40.0`（或把 declare 默认写成 int，但不建议在这个场景改）

### 2) 命令用错（run vs launch）

现象：`ros2 run nav2_toolkit probes.launch.py ...` 报 “No executable found”

原因：

- launch 文件不是 console_scripts 的可执行入口

修复：

- 使用 `ros2 launch nav2_toolkit probes.launch.py ...`

## 结论（一句话落锤）

本日完成了 launch 的“可核验证据链”：launch 参数可注入、namespace 可生效、参数可读回、且能通过参数变化触发可观测的告警行为（LOW_RATE / NO_MSG / STALE）。