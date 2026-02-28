# w4_d7 Playbook：bag replay 排障手册

## 1. 适用场景
本页用于处理以下类型的问题：

- 想复现之前录过的导航现场
- bag 能播放，但 RViz / 工具看起来没反应
- 不确定 replay 时 `/clock`、`/tf`、`/tf_static`、`/map` 是否正常
- 想判断问题停留在 replay 层，还是已经转移到 QoS / TF / lifecycle / action 层

本页的目标不是解释 rosbag 的全部理论，而是给出一个固定的 replay 排障顺序：先确认 bag 内容，再确认 `/clock`，再确认 `/tf` / `/tf_static` / `/map`，最后再决定把问题归到哪一层。

## 2. 本次验证对象
验证 bag：
    bags/w3_d3_map1_run01

`ros2 bag info` 结果表明：
- Duration: 226.353325346 s
- Messages: 22086
- `/clock`: Count = 2188
- `/tf`: Count = 11774
- `/tf_static`: Count = 1
- `/map`: Count = 1
- `/odom`: Count = 6434
- `/scan`: Count = 1094

结论：
- 该 bag 包含 replay 排障所需的核心 topic
- `/map` 与 `/tf_static` 均为低频 topic，其中 `/map` 仅 1 条，验证时需要注意 CLI 订阅窗口与时序问题

## 3. 如何启动 replay
命令：
    ros2 bag play -l bags/w3_d3_map1_run01

作用：
- 播放 bag
- `-l` 表示循环播放，便于多次验证 topic 可见性

说明：
- 本次验证中，直接 replay 后即可取到 `/clock`、`/tf`、`/tf_static`、`/map`
- 说明该 bag 的 replay 基础链路正常

## 4. 基础验证方法

### 4.1 验证 `/clock`
命令：
    ros2 topic echo /clock --once

本次现象：
- 成功取到一条 `/clock` 消息

结论：
- replay 时间基准正常

### 4.2 验证 `/tf`
命令：
    ros2 topic echo /tf --once

本次现象：
- 成功取到一条 `/tf` 消息

结论：
- 动态 TF 在 replay 中可见

### 4.3 验证 `/tf_static`
命令：
    ros2 topic echo /tf_static --once

本次现象：
- 成功取到静态 TF 数据

结论：
- 静态 TF 在 replay 中可见

### 4.4 验证 `/map`
命令：
    ros2 topic echo /map

本次现象：
- 成功取到 `/map` 消息，消息头 `frame_id: map`
- 地图元信息可见，包括 resolution、width、height、origin 等字段

结论：
- replay 中 `/map` 可见
- bag replay 的地图链路成立

## 5. 关于 `/map` 短窗口验证失败的解释
曾使用以下命令验证 `/map`：
    timeout 3 ros2 topic echo /map --qos-durability transient_local --once

现象：
- 命令被 `timeout` 终止
- 退出码为 143

正确解释：
- 该结果只能说明“在 3 秒窗口内，当前 CLI 订阅者未取到 `/map` 消息”
- 不能据此得出“bag 中没有 `/map`”或“replay 不发布 `/map`”的结论

结合后续验证：
- `ros2 bag info` 已证实 `/map` 存在，Count = 1
- 直接执行 `ros2 topic echo /map` 已成功取到地图消息

因此更合理的解释是：
- 对单次或低频 topic，`timeout + --once` 的短窗口验证不稳定
- 若要验证这类 topic，不能只依赖一次很短的 CLI 取样结果

## 6. 推荐排障顺序
1. 先看 bag 内容
   - `ros2 bag info <bag_dir>`
2. 再确认 replay 基础 topic
   - `/clock`
   - `/tf`
   - `/tf_static`
   - `/map`
3. 若 replay 基础链路成立，再决定是否转去下一层：
   - QoS / map / RViz
   - TF
   - lifecycle
   - action

## 7. 如何读结果

### 情况 A：`/clock` 不通
判断：
- 优先怀疑 replay 命令、播放状态或 bag 本身

### 情况 B：`/clock` 正常，`/tf` 不通
判断：
- 优先进入 TF 层排障

### 情况 C：`/clock` 正常，`/tf_static` / `/map` 不稳定
判断：
- 先区分是 bag 中未录到，还是 CLI 短窗口验证不稳定
- 不要直接把责任归到 replay 层

### 情况 D：`/clock`、`/tf`、`/tf_static`、`/map` 都正常
判断：
- replay 基础链路成立
- 后续问题更可能在 QoS / TF / lifecycle / action 层

## 8. 结论
bag replay 排障的关键，不是“会不会播放一条 bag”，而是能否按固定顺序确认：

1. bag 中录了什么
2. `/clock` 是否正常
3. `/tf`、`/tf_static`、`/map` 是否可见
4. 问题到底停留在 replay 层，还是已经转移到其他层

基于 `w3_d3_map1_run01` 的实际验证结果，可以确认该 bag 的 replay 基础链路正常。
