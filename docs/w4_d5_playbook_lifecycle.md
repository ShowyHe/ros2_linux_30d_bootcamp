# w4_d5 Playbook：Lifecycle 排障手册

## 1. 适用场景
本页用于处理以下类型的问题：

- Nav2 已经 launch，但系统行为像“没完全起来”
- 节点存在，但功能没有真正 ready
- `goal_sender` 发 goal 前后怀疑导航节点状态不对
- 想确认问题到底是“节点没启动”还是“节点没进入 active”

本页的目标不是解释 lifecycle 全部理论，而是给出一个可执行的排障顺序：如何确认 lifecycle 节点是否存在、如何确认它是否进入 active、以及遇到异常时应优先查看哪一层证据。

## 2. 先看什么：有哪些 lifecycle 节点

命令：
    ros2 lifecycle nodes

作用：
- 列出当前系统中支持 lifecycle 的节点

判断：
- 如果预期中的 Nav2 关键节点根本不在列表里，先不要讨论 active / inactive，先回到系统启动层确认该节点是否真正启动

## 3. 如何确认节点是否真正 ready

### 3.1 查询单个节点状态
命令示例：
    ros2 lifecycle get /map_server
    ros2 lifecycle get /planner_server
    ros2 lifecycle get /controller_server
    ros2 lifecycle get /bt_navigator

正常现象：
- 返回 `active [3]`（或等价的 active 状态提示）

结论：
- 节点处于 active，说明其已进入工作状态
- 只有进程存在但未 active，不能视为系统真正 ready

## 4. 生命周期排障的三种基本情况

### 情况 A：节点不存在
现象：
- `ros2 lifecycle get /node_name` 报错
- 或返回 node not found / node not available

结论：
- 问题不在 lifecycle 状态切换，而在节点根本未启动
- 应先回到 launch / bringup / healthcheck 层排查

### 情况 B：节点存在，但不在 active
现象：
- `ros2 lifecycle get /node_name` 返回：
  - `inactive`
  - `unconfigured`
  - `configuring`
  - `activating`
  - 或其他非 active 状态

结论：
- 说明进程可能活着，但功能未真正 ready
- 此时不应将系统视为可用
- 应进一步检查：
  - 启动顺序
  - 配置参数
  - 依赖节点是否正常
  - 上游资源是否就绪

### 情况 C：节点 active，但系统仍异常
现象：
- `ros2 lifecycle get /node_name` 为 active
- 但导航仍有问题

结论：
- 说明 lifecycle 至少不是首要矛盾
- 下一步应转到：
  - TF
  - QoS
  - map
  - action
  - goal 口径
  等其他层继续排查

## 5. 推荐排障顺序

### 第一步：先跑整体健康检查
命令：
    ./scripts/w3_d2_nav2_healthcheck.sh

目的：
- 先判断系统整体是否具备运行前提
- 避免在系统整体未就绪时，把问题过早归因到单个 lifecycle 节点

### 第二步：列出 lifecycle 节点
命令：
    ros2 lifecycle nodes

目的：
- 先确认关键节点是否真的存在

### 第三步：查询关键节点状态
命令：
    ros2 lifecycle get /map_server
    ros2 lifecycle get /planner_server
    ros2 lifecycle get /controller_server
    ros2 lifecycle get /bt_navigator

目的：
- 确认关键节点是否已经 active

### 第四步：根据状态决定下一层排查方向
- 节点不存在：回到 bringup / launch
- 节点存在但不 active：查 lifecycle / 配置 / 启动依赖
- 节点 active：转去 TF / QoS / action / map 层

## 6. 本页使用的证据来源
本页证据来自以下两类输出：

- `ros2 lifecycle nodes`
- `ros2 lifecycle get <node_name>`
- 辅助对照：`./scripts/w3_d2_nav2_healthcheck.sh`

这些证据组合起来的作用，是把“进程存在”与“节点真正 ready”区分开来。

## 7. 结论
Lifecycle 排障的关键，不是背状态名，而是先分层判断：

1. 节点在不在
2. 节点是不是 active
3. 如果已经 active，问题是不是已经转移到其他层

对 Nav2 这类系统来说，“launch 成功”不等于“系统可用”，只有关键 lifecycle 节点进入 active，才更接近真正 ready 的状态。