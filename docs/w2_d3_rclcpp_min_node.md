# w2_d3_rclcpp_min_node.md（Week2 Day3 / Global Day10）

## 今日目标
- 我能从零创建一个 ament_cmake 的 C++ 包（cpp_min_node）
- 我能用 ROS2 CLI 给证据链：节点在跑、topic 在发、订阅在收
- 创建cpp_min_node的ros2包，并在建立了两个cpp源码文件src/listener.cpp，src/talker.cpp
- 我能证明参数不是摆设：参数可读回，并且确实影响运行行为

---

## 完成判据
- ros2 node list 能看到 /cpp_talker /cpp_listener
- /cpp_chatter 存在且 pub/sub 数量正确
- ros2 topic hz 能测到稳定频率（当前约 4.664Hz）
- ros2 param get 能读回 publish_rate=5.0、prefix=FAST（参数注入成功）

---

## 环境与前置
工作区：
    ~/ros2_linux_30d_bootcamp/ros2_ws

命令：
    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp/ros2_ws
    source install/setup.bash

---

## 证据链

### 1) 节点存在
命令：
    ros2 node list

我看到（关键行）：
    /cpp_listener
    /cpp_talker

---

### 2) 发布/订阅关系清楚
listener：
    ros2 node info /cpp_listener

关键输出：
    Subscribers:
      /cpp_chatter: std_msgs/msg/String

talker：
    ros2 node info /cpp_talker

关键输出：
    Publishers:
      /cpp_chatter: std_msgs/msg/String

---

### 3) topic 存在 + endpoint/QoS
topic 列表里确实有：
    ros2 topic list

关键输出：
    /cpp_chatter

topic 基本信息：
    ros2 topic info /cpp_chatter

输出：
    Type: std_msgs/msg/String
    Publisher count: 1
    Subscription count: 1

详细信息（含节点名、namespace、QoS）：
    ros2 topic info -v /cpp_chatter

关键输出（摘重点）：
    Publisher count: 1
    Node name: cpp_talker
    Node namespace: /
    QoS profile:
      Reliability: RELIABLE
      History (Depth): KEEP_LAST (10)
      Durability: VOLATILE

    Subscription count: 1
    Node name: cpp_listener
    Node namespace: /
    QoS profile:
      Reliability: RELIABLE
      History (Depth): KEEP_LAST (10)
      Durability: VOLATILE

---

### 4) 发布频率测出来了
命令：
    ros2 topic hz /cpp_chatter

我看到：
    average rate: 4.664
    min: 0.214s max: 0.215s ...

结论：
- 当前发布频率稳定在 ~4.664Hz（接近 5Hz）
- 这属于正常误差范围（系统调度/计时/取整等都会影响），重点是“可测、稳定、接近目标”

---

### 5) 参数注入成功，并能读回
命令：
    ros2 param get /cpp_talker publish_rate
输出：
    Double value is: 5.0

    ros2 param get /cpp_talker prefix
输出：
    String value is: FAST

结论：
- 参数确实注入进 node 里了，不是我嘴上说说

---

## 我今天踩的坑（实事求是）
1) 我敲了 ROS1 的命令：
    ros topic list
系统报：
    Command 'ros' not found
原因：
- 这是 ROS2 环境，正确命令是 ros2

2) 我乱加了 topic info 的参数：
    ros2 topic info -t /cpp_chatter
报错：
    unrecognized arguments: -t
原因：
- ROS2 的 ros2 topic info 没有 -t
正确用法：
    ros2 topic info -v /cpp_chatter
    ros2 topic type /cpp_chatter

---

## 结论
今天的完整证据链：
- node list：节点存在
- node info：发布/订阅关系清楚
- topic info -v：endpoint + QoS 清楚
- topic hz：频率硬证据
- param get：参数注入硬证据