# docs/w2_d6_tf_guard_config.md（Week2 Day6 / Global Day13）

## 今日目标
- 我把 tf_guard 从“链路写死”升级为“参数可配置”
- 我能用 CLI 覆写 parent/child，检查任意 TF 链路
- 我能用证据链证明参数生效：同一个可执行，换参数得到 OK/FAIL
- 输出仍按 status line 契约：一行一状态（key=value），字段不丢

---

## 参数定义（v1）
- parent_frame（string）：默认 map
- child_frame（string）：默认 odom
- timeout_sec（double）：默认 0.2

---

## 运行方式（3 组用例）
    cd ~/ros2_linux_30d_bootcamp
    mkdir -p logs
    : > logs/w2_d6_tf_guard_cases.log

### 用例A：map -> odom（OK）
    ros2 run nav2_toolkit tf_guard --ros-args -p parent_frame:=map -p child_frame:=odom \
      2>&1 | tee -a logs/w2_d6_tf_guard_cases.log

证据（摘自： logs/w2_d6_tf_guard_cases.log）：
    ts=2026-02-23T18:55:28+08:00 tool=tf_guard level=OK status=OK reason=tf_ok parent=map child=odom timeout_sec=0.200

---

### 用例B：odom -> base_link（OK）
    ros2 run nav2_toolkit tf_guard --ros-args -p parent_frame:=odom -p child_frame:=base_link \
      2>&1 | tee -a logs/w2_d6_tf_guard_cases.log

证据（摘今天真实输出样例）：
    ts=2026-02-23T18:55:56+08:00 tool=tf_guard level=OK status=OK reason=tf_ok parent=odom child=base_link timeout_sec=0.200

---

### 用例C：base_link -> no_sucn_frame（FAIL）
    ros2 run nav2_toolkit tf_guard --ros-args -p parent_frame:=base_link -p child_frame:=no_sucn_frame \
      2>&1 | tee -a logs/w2_d6_tf_guard_cases.log

证据（摘今天真实输出样例）：
    ts=2026-02-23T18:57:42+08:00 tool=tf_guard level=FAIL status=FAIL reason=tf_missing parent=base_link child=no_sucn_frame ok=False timeout_sec=0.200

---

## 今日踩坑
1) 代码残留旧字段导致崩溃：
- 报错：AttributeError: 'TfGuard' object has no attribute 'parent1'
- 原因：_on_timer 里仍引用 parent1/child1（旧版字段）
- 修复：全文件统一只使用 parent_frame/child_frame

2) frame 名带 `/` 会导致 TF 查不到：
- /map->/odom FAIL，但 map->odom OK
- 修复：参数读取后对 parent/child 做 lstrip("/") 归一化，避免手滑造成“同一链路忽然 FAIL”的假象

---

## 结论
- 我用同一个 tf_guard，通过参数覆写实现了任意 TF 链路检查
- OK/FAIL 都有稳定证据行，输出字段齐全，可被脚本解析