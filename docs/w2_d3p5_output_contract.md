## 今天干了什么
我把 `topic_probe` 的输出从“人能看懂的散装日志”，改成“脚本也能稳定解析的一行状态（key=value）”。  
目的：后面 Week3 一键出报告、Week4 CI 才有稳定输入。

---

## 输出契约（Status Line Contract v1）
约束：**一行=一个状态**，字段用空格分隔，全部是 `key=value`。

### 必选字段（每行都必须出现）
- ts：时间戳（ISO8601）
- tool：工具名（今天是 topic_probe）
- level：OK / WARN / FAIL
- status：OK / WARN / FAIL（和 level 一致）
- reason：原因（不带空格，用下划线）

### 可选字段（按需出现，但建议键名稳定）
- topic：订阅 topic（如 /odom）
- rate_hz：估算频率
- threshold：阈值（min_rate_threshold）
- stale_sec：断流阈值（stale_timeout_sec）
- stale_age_sec：距离上次消息的秒数（只在 STALE 时出现）
- lin_x / ang_z：最后一次消息的速度字段（可选）

---

## 解析规则
日志里会混入 ROS logger 行（例如 `[INFO] ... started: ...`），它不是 `ts=...` 格式。  
后续汇总脚本只解析**以 `ts=` 开头的行**，其他行忽略。

---

## 今天的真实样例（直接从日志里摘）
OK（rate_ok）：
    ts=2026-02-22T10:54:35+08:00 tool=topic_probe level=OK status=OK reason=rate_ok lin_x=2.897e-06 ang_z=-6.115e-08 topic=/odom rate_hz=30.002 threshold=10.000 stale_sec=1.000

WARN（low_rate，阈值拉到 40，odom 实际 29~30Hz，所以必 WARN）：
    ts=2026-02-22T10:59:56+08:00 tool=topic_probe level=WARN status=WARN reason=low_rate lin_x=3.363e-06 ang_z=-5.997e-08 topic=/odom rate_hz=30.007 threshold=40.000 stale_sec=1.000

FAIL（stale_timeout，今天出现过一次断流/延迟）：
    ts=2026-02-22T10:54:40+08:00 tool=topic_probe level=FAIL status=FAIL reason=stale_timeout stale_age_sec=1.396 topic=/odom rate_hz=8.312 threshold=10.000 stale_sec=1.000

---

## 今天踩到的坑（
### 1) 参数类型严格：40 和 40.0 不一样
我用 `min_rate_threshold` 覆写时，如果写成整数 40，会触发类型异常：
    rclpy.exceptions.InvalidParameterTypeException: Trying to set parameter 'min_rate_threshold' to '40' of type 'INTEGER', expecting type 'DOUBLE'

修复方式：传 double
    -p min_rate_threshold:=40.0
或在 launch/参数文件里写 40.0。

### 2) Package 'nav2_toolkit' not found
出现这个说明环境没 overlay 到当前终端（没 source 或没 build）。
标准处理：
    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    colcon build --packages-select nav2_toolkit --symlink-install
    source install/setup.bash