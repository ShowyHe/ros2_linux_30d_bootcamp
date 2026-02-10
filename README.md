# ros2_linux_30d_bootcamp

30 天全职强训仓库（统一命名：wX_dY_）。目标是把 Linux/ROS2 的“会用”变成“可复现、可交付、可面试复述”的证据链。

---

## 全程通用规则

### 规则A：每天 1 个“闭卷环节”
- 禁止：直接复制旧 GitHub 库里整段命令/整段脚本。
- 允许：查 `--help` / `man` / `ros2 <cmd> -h` / 官方文档片段；
- 但当天必须有一步是自己从零敲出来并解释。

### 规则B：命令解释 + 日更落盘
- 命令：每天至少跑 10 条命令，并在当天 md 里写清楚：
  - 这条命令用来回答什么问题？
  - 证据是什么（关键输出行/落盘文件路径）？

---

## 目录结构

- `docs/`：每天与每周总结文档（含 daily_checklist）
- `scripts/`：脚本化工具（bash 为主）
- `tools/`：ROS2 工具节点（rclpy/rclcpp）
- `results/`：CSV/统计结果等结构化数据
- `bags/`：rosbag2 目录（真实可复现的 run 结构）
- `configs/`：参数/配置文件（如需要）
- `logs/`：所有命令/脚本输出落盘（强烈建议保留）

---

## 命名规范

- 每日文件统一使用：`wXdY_` 前缀  
  示例：
  - `docs/w1_d3_daily_checklist.md`
  - `scripts/w2_d14_run5.sh`
  - `logs/w3_d16_healthcheck.log`
  - `results/w3_d21_map1_5runs.csv`

---

## 每天固定产出（最低交付）

每一天至少完成：
- 当天计划列出的脚本/文档/日志/CSV 产出


---

## 学习路线（高层概览）

- Week1：bash 脚本化 Linux 生存技能 + ROS2 CLI 证据链
- Week2：低成本引入 rclpy 工具节点 + 1 天 rclcpp 最小闭环
- Week3：lifecycle/bag/healthcheck/回归脚本化（真实 run 结构）
- Week4：交付化（工具包/排障手册/自动汇总/闭卷 demo/模拟面试）

---
