# w4_d3 Playbook：TF 排障手册

## 1. 适用场景
本页用于处理以下类型的问题：

- Nav2 已启动，但机器人不动
- 地图、定位或路径规划看起来异常
- `goal_sender` 已发出 goal，但系统行为不符合预期
- 怀疑 TF 链路不完整或 frame 不存在

本页的目标不是解释 TF 的全部理论，而是给出一个可执行的排障顺序：如何快速确认 TF 健康、如何快速确认 TF 缺失，以及遇到异常时优先查看哪些证据。

## 2. 先看什么：判断 TF 是否健康

### 2.1 用 `tf_guard` 检查健康链路
命令：
    ros2 run nav2_toolkit tf_guard --ros-args \
      -p parent_frame:=map \
      -p child_frame:=base_link \
      -p timeout_sec:=0.2

正常现象：
- 可能在启动初期出现一次瞬态 `FAIL`
- 随后应连续输出：
  - `tool=tf_guard`
  - `status=OK`
  - `reason=tf_ok`
  - `parent=map`
  - `child=base_link`

结论：
- 若短暂 FAIL 后持续恢复为 OK，可视为系统启动瞬态
- 若持续输出 `tf_ok`，说明 `map -> base_link` 链路当前可查询

### 2.2 用 `/tf` 话题确认 TF 数据在流动
命令：
    ros2 topic echo /tf --once

正常现象：
- 能拿到一条 `/tf` 消息
- 说明 TF 数据确实在发布

结论：
- `/tf` 有数据，不代表所有链路都完整
- 但若 `/tf` 连一条都拿不到，说明应先怀疑 TF 发布本身是否正常

## 3. 如何制造并确认“不健康链路”

为了验证工具是否能识别异常，可故意查询一个不存在的 frame：

命令：
    ros2 run nav2_toolkit tf_guard --ros-args \
      -p parent_frame:=map \
      -p child_frame:=ghost_frame \
      -p timeout_sec:=0.2

异常现象：
- 持续输出：
  - `status=FAIL`
  - `reason=tf_missing`

结论：
- 这说明 `tf_guard` 不只是“能跑”，而是能够明确区分健康链路与缺失链路
- 在真实排障中，若查询真实目标链路时出现持续 `tf_missing`，则应优先检查 frame 名称、TF 发布者以及系统初始化状态

## 4. 推荐排障顺序

### 第一步：确认系统前提是否满足
先执行：
    ./scripts/w3_d2_nav2_healthcheck.sh

目的：
- 避免在系统整体未就绪时，直接把问题归因到 TF

### 第二步：检查健康链路
执行：
    ros2 run nav2_toolkit tf_guard --ros-args \
      -p parent_frame:=map \
      -p child_frame:=base_link \
      -p timeout_sec:=0.2

目的：
- 先看核心导航链路是否能查到

### 第三步：检查 `/tf` 是否有数据
执行：
    ros2 topic echo /tf --once

目的：
- 区分“TF 整体没发布”与“某条链路缺失”

### 第四步：检查是否是 frame 名称错误
若 `tf_guard` 持续 FAIL，应优先确认：
- parent_frame 写得是否正确
- child_frame 写得是否正确
- 是否误加了 `/`
- 是否引用了不存在的 frame

## 5. 如何读结果

### 情况 A：启动初期先 FAIL，随后 OK
判断：
- 多数属于启动瞬态
- 重点看后续是否能稳定恢复为 `tf_ok`

处理：
- 不要看到第一条 FAIL 就直接判系统坏
- 观察连续输出

### 情况 B：持续 `tf_missing`
判断：
- 说明查询的链路不可用
- 可能是 frame 名称错误，也可能是发布链路本身缺失

处理：
- 先确认 frame 名称
- 再确认 TF 发布者与系统前提

### 情况 C：`/tf` 没有数据
判断：
- 应先怀疑 TF 发布本身，而不是某一条具体链路

处理：
- 回到 healthcheck 和系统 bringup 检查

## 6. 本页使用的证据来源
本页证据来自以下两类输出：

- `nav2_toolkit` 的 `tf_guard` 状态行输出
- ROS2 CLI 的 `/tf` 原生命令输出

这两类证据组合使用的目的，是避免只依赖单一工具做判断。

## 7. 结论
TF 排障不应靠“看起来像不对”来判断，而应按顺序给证据：

1. 系统是否整体就绪
2. 核心链路是否可查询
3. `/tf` 是否在发布
4. frame 名称是否正确