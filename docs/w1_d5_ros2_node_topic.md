## 今日目标
- 建立 ROS2 通信“概念闭环”：node、DDS、topic/service/action 的关系
- 掌握 node/topic 的证据链方法：能判断“节点活没活、topic 有没有发布、有没有数据、频率是否正常”
- 理解并能口头解释 topic 的两类关键 QoS：reliability / durability（以及基本匹配关系）
- 对 /tf、/scan、/map 三个关键 topic 能各跑一遍并说清楚输出含义

## 今日操作命令一览（按“证据链顺序”）
- ros2 node list：列出当前 ROS 图里的所有节点（判断“节点是否存在/活着”）
- ros2 node info <node>：查看节点的 Publishers / Subscriptions / Services / Actions（判断“节点在干活：输出/输入/接口”）
- ros2 topic list：列出所有 topic（判断“这个管道是否存在”）
- ros2 topic info -v <topic>：查看 Publisher/Subscriber 数量 + QoS（判断“有没有发布者 & QoS 是否匹配”）
- ros2 topic echo <topic> --once：只抓一条消息就退出（判断“topic 里是否有数据”，避免刷屏）
- ros2 topic hz <topic>：统计发布频率（判断“频率是否正常/是否稳定”）

## 概念总结（今天的主要内容）
- node：ROS2 里真正执行逻辑的实体（发布/订阅、提供 service、发起/处理 action）
- DDS：ROS2 底层通信中间件（负责发现、匹配、传输）
- node 与 node 的三种通信方式：
  - topic：发布/订阅（数据流，传感器/状态最常见）
  - service：请求/响应（一次性问答）
  - action：目标/反馈/结果（长任务，Nav2 导航典型）

一句话落锤：
- node 是“谁在说话”，DDS 是“怎么把话送到对方”，topic/service/action 是“说话的协议类型”。

## Topic 的关键 QoS（今天重点）
### reliability（可靠性）
- reliable：要求可靠传输（尽量不丢）
- best_effort：尽力而为（允许丢包，常见于高频传感器）

### durability（持久性）
- volatile：只接收“从现在开始”的数据（不补历史）
- transient_local：允许接收发布者缓存的“最后一份”（典型：/map 后来者也能立刻拿到）

### QoS 匹配（你今天的口径）
- 发布端与订阅端需要能匹配；常见可配组合：
  - ①①（reliable ↔ reliable）
  - ①②（reliable ↔ best_effort）
  - ②②（best_effort ↔ best_effort）
- durability 同理：transient_local 与 volatile 的差异会直接影响“后来订阅者是否马上能收到”（/map 最典型）

## node 证据链（判断“节点活没活”）
- 第一证据：ros2 node list
  - 能看到关键节点名 → 节点“活着”
- 第二证据：ros2 node info <node>
  - Publishers：它在发布什么（输出）
  - Subscriptions：它在订阅什么（输入）
  - Services / Actions：它对外提供什么接口（Nav2 常见 action）

## topic 证据链（判断“有没有发布/有没有数据”）
### 1) topic 是否存在
- ros2 topic list
  - list 里有 topic 名：只是说明“管道存在”
  - 还需要 info -v 才能确认是否有人往里写

### 2) 有没有发布者 + QoS 是否匹配（硬证据）
- ros2 topic info -v /tf
- ros2 topic info -v /scan
- ros2 topic info -v /map
你今天抓的关键信息：
- Publisher count ≥ 1：证明“有人在发布”
- QoS（reliability/durability）：解释“为什么我可能 echo 收不到”（/map 重点看 durability）

### 3) 有没有数据（避免刷屏，用 --once）
- ros2 topic echo /scan --once
- ros2 topic echo /tf --once
解释：
- 不加 --once 会变成持续信息流，刷屏且不利于留证据
- --once 是最小证据：收到 1 条就退出 = topic 真在流

### 4) 发布频率是否正常（用 hz）
- ros2 topic hz /scan
- ros2 topic hz /tf
解释：
- hz 用来统计频率与抖动，跑几秒后 Ctrl+C 退出是正常操作

## Mini-Check（闭卷自检）
- 我能闭卷讲清楚：node、DDS、topic/service/action 的关系
- 我能闭卷按顺序说出“证据链”：node list → node info → topic list → topic info -v → echo --once → hz
- 我能闭卷解释两个 QoS 关键字的含义与作用：
  - reliability：reliable vs best_effort
  - durability：transient_local vs volatile
- 我能解释为什么 echo 要用 --once：拿到“有数据”的最小证据，避免刷屏
- 我能解释 hz 的用途：验证发布频率是否正常/是否稳定
