# w4_d7 Nav2 Troubleshooting Playbook

## 1. 文档目的
本页用于汇总 Week4 已完成的排障文档，形成统一入口。目标不是重复展开细节，而是先帮助读者判断问题属于哪一层，再跳转到对应页面处理。

## 2. 分层目录

### 2.1 TF 层
适用问题：
- 机器人不动
- frame 缺失
- transform 不存在或链路异常

对应文档：
- `docs/w4_d3_playbook_tf.md`

### 2.2 QoS / map / RViz 层
适用问题：
- RViz 看不到 `/map`
- 晚启动订阅者拿不到 `/tf_static`
- 怀疑 QoS 口径不匹配

对应文档：
- `docs/w4_d4_playbook_qos_map_rviz.md`

### 2.3 Lifecycle 层
适用问题：
- Nav2 已启动，但系统不像 fully ready
- 节点存在，但能力不完整
- 怀疑关键节点未进入 active

对应文档：
- `docs/w4_d5_playbook_lifecycle.md`

### 2.4 Action 层
适用问题：
- 发了 goal 但没动
- 不知道 goal 是否真正发出 / 被接受
- 出现 timeout

对应文档：
- `docs/w4_d6_playbook_action_nav2.md`

### 2.5 Bag replay 层
适用问题：
- 想复现之前现场
- bag 播了但系统 / RViz 没反应
- 不确定 replay 是否正常
- 需要确认 `/clock`、`/tf`、`/tf_static`、`/map`

对应文档：
- `docs/w4_d7_playbook_bag.md`

## 3. 推荐总排障顺序
1. 先分层，不要乱猜
2. 先查该层最直接证据
3. 不要跨层乱跳

示例：
- replay 不通，就不要先改 goal
- lifecycle 没 active，就不要先怪 action timeout
- QoS 不对，就不要先怀疑 controller

## 4. 本周手册的价值
Week4 的几页手册并不追求覆盖 Nav2 所有问题，而是先把最常见、最容易混淆的几层问题切开。只要能先判断“问题属于哪一层”，后续排障效率就会高很多。

## 5. 结论
排障最怕的不是问题复杂，而是层级混乱。本页将 TF、QoS、lifecycle、action、bag replay 五页文档汇总为统一入口，目的是让每次导航异常都先分层、再找证据、最后决定修复方向。
