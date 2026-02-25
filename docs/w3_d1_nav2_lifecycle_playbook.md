# w3_d1 Daily Checklist（Week3 Day1 / Global Day15）

## 今日目标

把 lifecycle 从“概念”变成“可执行剧本”：  
能用 `ros2 lifecycle get/set` 对 Nav2 关键节点做状态查询与状态迁移验证，并记录异常案例与证据链。

---

## 今日产出文件

- `docs/w3_d1_nav2_lifecycle_playbook.md`
- `docs/w3_d1_daily_checklist.md`
- `logs/w3_d1_lifecycle.log`

---

## 闭卷环节（规则A）

### 闭卷内容
不看旧文档，独立完成以下动作并落盘：
1. 查询 10 个 Nav2 lifecycle 节点状态
2. 对 `/planner_server` 执行 `deactivate`
3. 高频连续 `get` 抓取状态变化
4. 记录异常现象并形成文字结论

### 闭卷结果
- [x] 完成
- 关键证据：已抓到 `/planner_server` 从 `inactive [2]` 恢复到 `active [3]` 的过程

---

## 今日命令清单（计入“10 条命令”）

说明：每条命令都要回答“在验证什么”，并说明证据在哪。

### 1）查询 `/behavior_server` 状态
命令：

    ros2 lifecycle get /behavior_server 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- `/behavior_server` 当前是否处于 `active`？

证据：
- `logs/w3_d1_lifecycle.log` 中出现 `active [3]`

---

### 2）查询 `/bt_navigator` 状态
命令：

    ros2 lifecycle get /bt_navigator 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 行为树导航器是否激活？

证据：
- 日志中出现 `active [3]`

---

### 3）查询 `/controller_server` 状态
命令：

    ros2 lifecycle get /controller_server 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 控制器服务是否激活？

证据：
- 日志中出现 `active [3]`

---

### 4）查询 `/global_costmap/global_costmap` 状态
命令：

    ros2 lifecycle get /global_costmap/global_costmap 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 全局代价地图节点是否激活？

证据：
- 日志中出现 `active [3]`

---

### 5）查询 `/local_costmap/local_costmap` 状态
命令：

    ros2 lifecycle get /local_costmap/local_costmap 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 局部代价地图节点是否激活？

证据：
- 日志中出现 `active [3]`

---

### 6）查询 `/map_server` 状态
命令：

    ros2 lifecycle get /map_server 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 地图服务节点是否激活？

证据：
- 日志中出现 `active [3]`

---

### 7）查询 `/planner_server` 状态（基线）
命令：

    ros2 lifecycle get /planner_server 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 规划器节点操作前是否处于健康态？

证据：
- 日志中出现 `active [3]`

---

### 8）错误示范：拼错 transition（FAIL 案例）
命令：

    ros2 lifecycle set /planner_server deactive 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- CLI 对非法 transition 名会返回什么提示？是否能辅助纠错？

证据：
- 日志中出现 `Unknown transition requested`
- 日志中列出可用项 `deactivate [4]`、`shutdown [7]`

---

### 9）对 `/planner_server` 执行合法 `deactivate`
命令：

    ros2 lifecycle set /planner_server deactivate 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- `deactivate` 请求是否被成功执行？

证据：
- 日志中出现 `Transitioning successful`

---

### 10）高频连续采样 `/planner_server` 状态（抓现行）
命令：

    for i in 1 2 3 4 5; do
      ros2 lifecycle get /planner_server 2>&1 | tee -a logs/w3_d1_lifecycle.log
      sleep 0.2
    done

回答的问题：
- `deactivate` 后状态是否持续保持？是否会被外部流程恢复？

证据：
- 日志中出现状态序列：
  - `inactive [2]`
  - `inactive [2]`
  - `active [3]`
  - `active [3]`
  - `active [3]`

---

## 额外命令

### A）对照实验：`/waypoint_follower`
命令：

    ros2 lifecycle set /waypoint_follower deactivate 2>&1 | tee -a logs/w3_d1_lifecycle.log
    for i in 1 2 3 4 5; do
      ros2 lifecycle get /waypoint_follower 2>&1 | tee -a logs/w3_d1_lifecycle.log
      sleep 0.2
    done

回答的问题：
- 其他 lifecycle 节点是否也会出现“短暂状态变化后被恢复”的现象？

证据：
- 日志中出现状态序列：
  - `inactive [2]`
  - `unconfigured [1]`
  - `inactive [2]`
  - `inactive [2]`
  - `active [3]`

结论（工作假设）：
- 节点状态变化存在自动管理行为影响，不是单节点孤立现象

---

### B）管理器证据
命令：

    ros2 node list | grep lifecycle_manager 2>&1 | tee -a logs/w3_d1_lifecycle.log
    ros2 service list | grep manage_nodes 2>&1 | tee -a logs/w3_d1_lifecycle.log

回答的问题：
- 系统里是否存在 lifecycle manager 节点与管理服务？

证据：
- /lifecycle_manager_localization
    /lifecycle_manager_navigation
    /lifecycle_manager_localization/manage_nodes
    /lifecycle_manager_navigation/manage_nodes

---

## 今日关键结论

`ros2 lifecycle set` 返回成功，只表示状态迁移请求执行成功；节点是否持续停留在该状态，还需要结合后续 `get`（最好连续采样）和外部管理行为一起判断。

---

## Git 微闭环（规则C，从 Day7 起生效）

> w3_d1 非回归日，今天不强制 `git push`，但建议至少完成到 `commit`。

### 建议收尾命令（写入实际执行结果）
    git status -sb
    git diff
    git add -A
    git commit -m "w3_d1: add lifecycle playbook and evidence logs"

执行情况：
- [ ] `git status -sb`
- [ ] `git diff`
- [ ] `git add -A`
- [ ] `git commit -m "..."`
- [ ] （非必须）`git push`

