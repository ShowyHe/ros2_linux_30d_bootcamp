## 今日目标
- 把 ROS2 CLI 的 **param / service / action** 用成“证据链工具”，不是瞎猜
- 我能闭卷回答两件事：
  - **参数是否生效**：改到了哪个 node、值有没有被接收、有没有快照
  - **action 状态怎么看**：server 在不在、goal 有没有被接收、执行证据是什么（成功/失败都算，只要有证据）

## 今日操作命令一览（我今天用到的核心）
- param
  - ros2 param list <node>
  - ros2 param get <node> <param>
  - ros2 param set <node> <param> <value>
  - ros2 param dump <node>
- service
  - ros2 service list
  - ros2 service type <service>
  - ros2 service call <service> <type> "<request>"
- action
  - ros2 action list -t
  - ros2 action info <action>
  - ros2 action send_goal <action> <type> "<goal>" --feedback

## 我今天学到的逻辑（按证据链）
### 1) Param：参数是否生效，怎么证明
我不靠“感觉”，只认这条证据链：

- 先读旧值（get old）
- 再 set
- 再读新值（get new）
- 最后 dump 一份快照（可复现/可对照）

我今天的最小闭环标准：
- 至少一次 get/set/get/dump
- 如果 set 失败也算完成，但必须把失败原因写清楚（这是证据，不是“没跑出来”）

我今天实际做法（示例命令长这样，具体 node/param 以我当时 node list 为准）：
    ros2 param get <node> <param>
    ros2 param set <node> <param> <value>
    ros2 param get <node> <param>
    ros2 param dump <node>

我今天确认的点：
- set 成功 ≠ 100% 影响行为，但至少证明 **node 接收了新值**
- dump 的意义：把当下配置锁成“证据快照”，后面做对照实验/复现不靠记忆

### 2) Service：同步问答，怎么证明我真的 call 成功了
service 我只认三连：

- list：系统里确实有这个 service（存在性）
- type：我知道它的 srv 类型（格式正确）
- call：我发请求拿到了 response（功能证据）

我今天的最小闭环标准：
- 至少一次 type + call
- response 必须真实看到（不然就是没调用成功）

我今天实际做法（典型就用 lifecycle 的 get_state 这类最稳）：
    ros2 service type <service>
    ros2 service call <service> <type> "{}"

我今天确认的点：
- 能拿到 response，就说明：service 在、类型对、请求能到达并返回

### 3) Action：状态怎么看，怎么证明我真的发过 goal 且系统有反应
action 这块，我不瞎发 goal。我按顺序来：

- 先 action list -t：确认 action 名称 + 类型（避免 type 写错）
- 再 action info：确认 **server 存在**（否则 send_goal 必失败）
- 最后 send_goal --feedback：拿执行证据（反馈/结果/失败原因都算）

我今天的最小闭环标准：
- info 里能看到 server
- send_goal 有“执行证据”（成功/失败都可以，但必须有内容：被接收/反馈/结果/失败原因）

我今天实际做法：
    ros2 action list -t
    ros2 action info /navigate_to_pose
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "<goal>" --feedback

我今天确认的点：
- action 的“状态怎么看”不是靠 RViz，是靠：
  - info 里 server 的存在性
  - send_goal 的反馈/结果/失败原因（这就是工程证据）

## 今日交付（我自己验收口径）
只看我落盘的日志：logs/w1_d6_action_test.log

里面必须同时有三段证据（我今天都做了）：
- Param 段：至少一次 get/set/get/dump（或 set 的失败原因）
- Service 段：至少一次 type + call，且有 response
- Action 段：info 有 server + send_goal 有执行证据（成功/失败均可）
