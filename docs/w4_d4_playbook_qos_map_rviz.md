# w4_d4 Playbook：QoS / map / RViz 排障手册

## 1. 适用场景
本页用于处理以下类型的问题：

- RViz 中看不到 `/map`
- 系统已经启动，但晚启动订阅者拿不到地图
- 回放或晚启动场景下，订阅者拿不到 `/tf_static`
- 怀疑不是 Nav2 算法问题，而是 QoS 口径不匹配

本页的目标不是解释 QoS 的全部理论，而是给出一个可执行的排障顺序：如何先看 topic 的真实 QoS，如何构造健康 / 失败样例，以及遇到问题时如何判断“是 QoS 口径错了，还是系统根本没发布”。

## 2. 先看什么：抓 topic 的真实 QoS

### 2.1 查看 `/map`
命令：
    ros2 topic info -v /map

重点关注：
- Publisher count
- Reliability
- Durability

判断：
- 如果 `/map` 没有 publisher，先不要谈 QoS，先回到系统启动层排查
- 如果 `/map` 已有 publisher，再根据输出判断订阅者应采用什么 QoS 口径

### 2.2 查看 `/tf_static`
命令：
    ros2 topic info -v /tf_static

重点关注：
- Publisher count
- Reliability
- Durability

判断：
- `/tf_static` 常见问题不是“没人发”，而是“晚启动订阅者没有按正确 durability 订阅”

## 3. 如何确认“看不到 map”是否由 durability 引起

### 3.1 故意用 volatile 订阅 `/map`
命令：
    timeout 3 ros2 topic echo /map --qos-durability volatile --once

现象：
- 在系统已运行一段时间后再启动订阅者，可能无法立即拿到 map

结论：
- 这说明对 `/map` 这种依赖历史消息的 topic，仅用 volatile 可能不足以获取已经发布过的地图数据

### 3.2 改用 transient_local 订阅 `/map`
命令：
    timeout 3 ros2 topic echo /map --qos-durability transient_local --once

现象：
- 更容易直接拿到一条 map 消息

结论：
- 若 transient_local 能拿到而 volatile 拿不到，则“看不到 map”更可能是订阅端 durability 口径不对，而不是 map_server 没工作

## 4. 如何确认“订阅不到 /tf_static”是否由 durability 引起

### 4.1 故意用 volatile 订阅 `/tf_static`
命令：
    timeout 3 ros2 topic echo /tf_static --qos-durability volatile --once

现象：
- 晚启动时可能收不到消息

结论：
- `/tf_static` 通常依赖 transient_local 保存静态变换，volatile 晚启动订阅者可能拿不到历史已发布的 static TF

### 4.2 改用 transient_local 订阅 `/tf_static`
命令：
    timeout 3 ros2 topic echo /tf_static --qos-durability transient_local --once

现象：
- 更容易直接拿到 `/tf_static` 消息

结论：
- 若 transient_local 可以、volatile 不行，则问题在订阅者 QoS，而不是 TF 发布链路本身一定坏了

## 5. RViz 中“看不到 map”的最小检查顺序

### 第一步：先看 `/map` 是否真的有 publisher
命令：
    ros2 topic info -v /map

如果没有 publisher：
- 先回到系统启动层检查 map_server / Nav2 bringup

### 第二步：确认 RViz Map Display 订阅的是 `/map`
避免话题名写错。

### 第三步：检查 Map Display 的 Durability
若 RViz 晚启动，且 `/map` 为 transient_local 发布：
- Map Display 的 Durability 应设为 `Transient Local`

若设置为 `Volatile`：
- 可能出现看不到 map 的现象

## 6. 推荐判断规则

### 情况 A：`/map` 有 publisher，但晚启动订阅拿不到
优先怀疑：
- 订阅端 durability 不对

### 情况 B：`/tf_static` 有 publisher，但晚启动订阅拿不到
优先怀疑：
- 订阅端没有按 transient_local 订阅

### 情况 C：`/map` 或 `/tf_static` 根本没有 publisher
结论：
- 先回到系统 bringup / lifecycle / 启动流程排查
- 此时先别把问题归因到 QoS

## 7. 本页使用的证据来源
本页证据来自以下几类输出：

- `ros2 topic info -v /map`
- `ros2 topic info -v /tf_static`
- `ros2 topic echo /map --qos-durability ... --once`
- `ros2 topic echo /tf_static --qos-durability ... --once`

这些命令组合起来的作用，是区分三类问题：

1. 根本没有发布者
2. 发布正常，但订阅 QoS 不匹配
3. 晚启动订阅者没有拿到历史消息

## 8. 结论
QoS / map / RViz 排障的关键，不在于背诵术语，而在于先确认 topic 的真实发布 QoS，再用可控的晚启动订阅样例判断问题是否来自 durability 配置。

对于 `/map` 和 `/tf_static` 这类依赖历史消息的 topic，若晚启动订阅者没有按 `transient_local` 口径订阅，就可能出现“看不到 map”或“接不到 tf_static”的现象。此时优先修订订阅端 QoS，而不是直接怀疑 Nav2 算法本体。