# docs/w2_d4_ros2_qos_for_nav2.md（Week2 Day4 / Global Day11）

## 今日目标
- 用 CLI 把 QoS 读出来并落盘
- 解释：为什么订阅端 QoS 不匹配会导致“看起来像没数据”
- 同一个 topic（/tf_static），只改 QoS 就能从“收不到”变成“收到”

---

## 证据源文件（落盘）
- logs/w2_d4_qos_dump.txt

生成方式：
    source /opt/ros/humble/setup.bash
    cd ~/ros2_linux_30d_bootcamp
    mkdir -p logs
    : > logs/w2_d4_qos_dump.txt

    echo "=== /tf_static ===" | tee -a logs/w2_d4_qos_dump.txt
    ros2 topic info -v /tf_static 2>&1 | tee -a logs/w2_d4_qos_dump.txt

---

## 我今天用的判断口径
### 规则 1：先看 Publisher count
- 如果 `ros2 topic info -v <topic>` 里 `Publisher count = 0`，我怎么改 QoS 都收不到，因为根本没人发。

### 规则 2：订阅端 QoS 必须“整套匹配”
- 只改 durability 不一定够。
- reliability（RELIABLE / BEST_EFFORT）不匹配时，订阅端会一直收不到。
- 所以我订阅 /tf_static 时，至少显式指定：
  - durability
  - reliability
  - history + depth

---

## 硬证据：/tf_static 的 QoS 匹配后可以订阅到结果
### 1) 先读 QoS
命令：
    ros2 topic info -v /tf_static

目的：
- 看 Publisher count 是否 >= 1
- 抄出发布端 QoS（Reliability / Durability / Depth）

---

### 2) 订阅 /tf_static
命令（我今天用的关键参数：reliability + durability + history/depth）：
    timeout 2 ros2 topic echo /tf_static \
      --qos-durability transient_local \
      --qos-reliability reliable \
      --qos-history keep_last \
      --qos-depth 1 \
      --once
    echo "exit_code=$?"

我看到的现象：
- 能输出 transforms（说明订阅成功）
- exit_code=0（说明 2 秒内收到了数据）

说明：
- /tf_static 属于“静态数据”，晚订阅也要收到，所以 durability 通常要 transient_local
- reliability 必须与发布端一致，否则可能永远收不到
- history/depth 与发布端对齐，避免不必要的不匹配

---

## 今天踩坑
### 1) 只写 durability 不够，reliability 不匹配会导致收不到
结论：
- QoS 要整套匹配，尤其 reliability

### 2) history/depth 也会影响是否能看到结果
结论：
- 我显式指定 history=keep_last + depth=1 后能稳定看到输出

---

## 最终结论
- QoS 不是“玄学”，我能用 `topic info -v` 把发布端 QoS 抄出来
- 订阅端按发布端 QoS 匹配（reliability + durability + history/depth）就能把“收不到”变成“收到”
- 遇到“RViz 看不到 /tf_static 或 /map”，第一反应不是重启，而是先做 QoS 证据链