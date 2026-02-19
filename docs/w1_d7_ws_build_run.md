# Week1 Day7 — colcon 工作区 + 最小 rclpy 包（w1_d7_min_pkg）

## 目的
- 在一个标准 ROS2 工作区里，从零创建并跑通一个最小 `ament_python` 包。
- 证明你能完成“创建包 → 写节点 → build → source → ros2 run → CLI 验证”的闭环。
- 本日最小节点：`HeartbeatNode` 周期发布字符串到 `/w1_d7/heartbeat`。

## 环境
- ROS2：Humble
- 工作区：`~/ros2_linux_30d_bootcamp/ros2_ws`
- 包名：`w1_d7_min_pkg`
- 节点名：`w1_d7_heartbeat`
- topic：`/w1_d7/heartbeat`
- 参数：`rate_hz`（发布频率，默认 1.0）

## 交付物
- 包代码：`ros2_ws/src/w1_d7_min_pkg/`
- 文档：`docs/w1_d7_ws_build_run.md`（本文件）

---

## Step 0 — 启动工作区
1) 加载 ROS2 环境（注意是 `.bash`，并且路径以 `/` 开头）  
    source /opt/ros/humble/setup.bash

2) 进入工作区  
    cd ~/ros2_linux_30d_bootcamp/ros2_ws

---

## Step 1 — 创建 ament_python 包
在 `src/` 下创建包：  
    cd ~/ros2_linux_30d_bootcamp/ros2_ws/src
    ros2 pkg create w1_d7_min_pkg --build-type ament_python --dependencies rclpy std_msgs

检查结构（应包含 `setup.py`、`w1_d7_min_pkg/` 目录等）：  
    ls -la w1_d7_min_pkg
    ls -la w1_d7_min_pkg/w1_d7_min_pkg

---

## Step 2 — 编写节点（heartbeat_node.py）
文件路径：  
- `ros2_ws/src/w1_d7_min_pkg/w1_d7_min_pkg/heartbeat_node.py`

代码（示例实现，按需微调即可）：  
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class HeartbeatNode(Node):
        def __init__(self):
            super().__init__('w1_d7_heartbeat')

            self.declare_parameter('rate_hz', 1.0)
            rate_hz = float(self.get_parameter('rate_hz').value)
            period = 1.0 / max(rate_hz, 0.1)

            self.pub = self.create_publisher(String, '/w1_d7/heartbeat', 10)
            self.timer = self.create_timer(period, self._on_timer)
            self.count = 0

            self.get_logger().info(f"started: rate_hz={rate_hz}")

        def _on_timer(self):
            msg = String()
            msg.data = f"time count={self.count}"
            self.pub.publish(msg)
            self.get_logger().info(msg.data)
            self.count += 1

    def main():
        rclpy.init()
        node = HeartbeatNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if __name__ == "__main__":
        main()

---

## Step 3 — 注册 console_scripts（setup.py）
编辑 `ros2_ws/src/w1_d7_min_pkg/setup.py`，确保 `entry_points` 中包含一条可执行入口：  
    'console_scripts': [
        'heartbeat_node = w1_d7_min_pkg.heartbeat_node:main',
    ],

快速检查（看到上述行即可）：  
    cd ~/ros2_linux_30d_bootcamp/ros2_ws/src/w1_d7_min_pkg
    grep -n "console_scripts" setup.py
    grep -n "heartbeat_node" setup.py

---

## Step 4 — 构建与加载（colcon + source）
回到工作区根目录 build：  
    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install

加载工作区环境：  
    source install/setup.bash

验证可执行程序已出现：  
    ros2 pkg executables w1_d7_min_pkg

期望输出包含：  
- `w1_d7_min_pkg heartbeat_node`

---

## Step 5 — 运行与验证（证据链）
1) 运行节点（默认 1Hz）：  
    ros2 run w1_d7_min_pkg heartbeat_node

2) 新开终端做证据检查（同样要 source）：  
    source /opt/ros/humble/setup.bash
    source ~/ros2_linux_30d_bootcamp/ros2_ws/install/setup.bash

检查节点是否存在：  
    ros2 node list | grep w1_d7

检查 topic 是否在发：  
    ros2 topic echo /w1_d7/heartbeat --once

检查参数是否存在：  
    ros2 param list /w1_d7_heartbeat | grep rate_hz
    ros2 param get /w1_d7_heartbeat rate_hz

3) 运行节点并覆盖参数（例：2Hz）：  
    ros2 run w1_d7_min_pkg heartbeat_node --ros-args -p rate_hz:=2.0

---

## 常见坑（本日真实踩坑口径）
1) source 路径写错
- 错：`source opt/ros/humble/setup.bash`（少了开头 `/`）
- 错：`source /opt/ros/humble/setup.py`（扩展名错）
- 对：`source /opt/ros/humble/setup.bash`

2) ros2 pkg 子命令写错
- 错：`ros2 pkg executable ...`
- 对：`ros2 pkg executables <package_name>`

3) 用 mkdir 创建 .py 文件（把文件建成目录）
- 错：`mkdir -p xxx.py`
- 对：用编辑器或 touch 创建文件
    nano xxx.py
    touch xxx.py

4) Ctrl+C 退出报 double shutdown（rclpy）
- 修复：finally 中 shutdown 前加判断
    if rclpy.ok():
        rclpy.shutdown()

---

## 本日闭卷要点（最小可复现口径）
- 不看笔记能敲出：
  - `ros2 pkg create ... --build-type ament_python ...`
  - `colcon build --symlink-install`
  - `source /opt/ros/humble/setup.bash` + `source install/setup.bash`
  - `ros2 pkg executables <pkg>`、`ros2 run <pkg> <exec>`
  - `ros2 node list`、`ros2 topic echo --once`、`ros2 param get`
- 能解释：
  - 为什么要 source 两次（ROS2 环境 + 工作区 overlay）
    - 第一次 source /opt/ros/humble/setup.bash：加载 ROS2 基础环境（把 ros2 CLI、rclpy、各类系统包的路径加入环境变量，如 PATH / PYTHONPATH / AMENT_PREFIX_PATH），让终端具备运行 ROS2 工具链与系统包的能力。
    - 第二次 source <ws>/install/setup.bash：加载 工作区 overlay（把 colcon build 生成的包加入同一套环境变量），让 ros2 run / ros2 pkg 能找到编译出来的包和入口程序。
  - 为什么 console_scripts 决定 `ros2 run` 能不能找到可执行
    - 因为这里写了如何通过可执行名字，来找到我的rclpy包，当console_scripts写了之后，在ros2 run的时候，就可以直接通过可执行名字打开我的rclpy包
