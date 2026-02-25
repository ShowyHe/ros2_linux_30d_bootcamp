# w3_d2 Nav2 Healthcheck

## 1. 今日目标

实现一个最小可用的 `nav2_healthcheck.sh`，用于一键判断 Nav2 系统是否“基本健康”，并满足以下要求：

- 检查关键节点是否存在
- 检查关键 topic 是否至少有 1 个 publisher
- 检查 `/map_server` 的 lifecycle 状态是否为 `active [3]`
- 失败时返回非 0（便于后续接入回归脚本 / CI）

---

## 2. 今日产出文件

### 新增 / 更新文件
- `scripts/w3_d2_nav2_healthcheck.sh`（核心脚本）
- `logs/w3_d2_healthcheck_run.log`（健康场景 + 故障场景日志）
- `docs/w3_d2_nav2_healthcheck.md`（本文档）

---

## 3. 检查范围

### 3.1 关键节点存在性（node exists）
检查以下节点是否出现在 `ros2 node list` 中（精确匹配）：

- `/bt_navigator`
- `/planner_server`
- `/controller_server`
- `/map_server`

### 3.2 关键 topic 发布者数量（publisher count）
检查以下 topic 的 `Publisher count` 是否 `>= 1`：

- `/tf`
- `/tf_static`
- `/map`

### 3.3 Lifecycle 状态检查
- `/map_server` 必须是 `active [3]`

---

## 4. 返回码约定（exit code contract）

脚本返回码定义如下：

- `0`：PASS（全部检查通过）
- `1`：FAIL（至少一项检查失败）
- `2`：环境错误（例如 `ros2` 命令不存在）

---

## 5. 输出格式

脚本输出统一为单行状态记录（key=value），便于日志检索和后续脚本解析。

### 必选字段（本脚本当前版本）
- `ts`
- `tool`
- `level`
- `status`
- `reason`

### 常见附加字段
- `node`
- `topic`
- `publisher_count`
- `min_pub`
- `rc`
- `state`
- `result`
- `fail_count`

---

## 6. 核心实现思路（为什么这样做）

### 6.1 先检查 `ros2` 命令是否存在
在任何 ROS2 CLI 调用之前先做：

    command -v ros2

原因：
- 如果当前终端没 `source /opt/ros/humble/setup.bash`，后续所有 `ros2 xxx` 都会失败
- 先单独报 `cmd_not_found`，能把“环境没配好”和“Nav2 不健康”区分开

### 6.2 节点检查使用“精确匹配”
通过 `ros2 node list` 获取节点列表，再用 `grep -Fxq` 精确匹配目标节点名。

原因：
- `-F`：按普通字符串匹配（不是正则）
- `-x`：整行匹配，避免误匹配相似名字
- `-q`：静默模式，仅通过返回码判断是否命中

### 6.3 topic 健康先看 publisher count（最低成本）
通过 `ros2 topic info <topic>` 获取 `Publisher count`，判断是否 `>= 1`。

原因：
- MVP 目标是快速确认“有没有源在发”
- 不引入 topic 内容解析，先把基础健康检查跑通

### 6.4 lifecycle 检查只做“当前快照”
通过 `ros2 lifecycle get /map_server` 检查是否为 `active [3]`。

原因：
- 本脚本是 healthcheck，不应该主动改变系统状态
- 只做快照检查，避免脚本干扰系统运行（非侵入）

---

## 7. 实测证据（健康场景）

以下为健康场景日志片段（Nav2 正常运行时）：

    ts=2026-02-25T17:05:12+0800 tool=healthcheck level=OK status=OK reason=start script=w3_d2_nav2_healthcheck
    ts=2026-02-25T17:05:12+0800 tool=healthcheck level=OK status=OK reason=cmd_found cmd=ros2
    ts=2026-02-25T17:05:12+0800 tool=healthcheck level=OK status=OK reason=node_exists node=/bt_navigator
    ts=2026-02-25T17:05:12+0800 tool=healthcheck level=OK status=OK reason=node_exists node=/planner_server
    ts=2026-02-25T17:05:12+0800 tool=healthcheck level=OK status=OK reason=node_exists node=/controller_server
    ts=2026-02-25T17:05:13+0800 tool=healthcheck level=OK status=OK reason=node_exists node=/map_server
    ts=2026-02-25T17:05:13+0800 tool=healthcheck level=OK status=OK reason=topic_publishers_ok topic=/tf publisher_count=3 min_pub=1
    ts=2026-02-25T17:05:13+0800 tool=healthcheck level=OK status=OK reason=topic_publishers_ok topic=/tf_static publisher_count=1 min_pub=1
    ts=2026-02-25T17:05:13+0800 tool=healthcheck level=OK status=OK reason=topic_publishers_ok topic=/map publisher_count=1 min_pub=1
    ts=2026-02-25T17:05:14+0800 tool=healthcheck level=OK status=OK reason=lifecycle_active node=/map_server state=active
    ts=2026-02-25T17:05:14+0800 tool=healthcheck level=OK status=OK reason=summary result=PASS fail_count=0

结论：
- 节点存在、topic 发布者正常、`/map_server` 为 active
- 脚本 summary 为 `PASS`

---

## 8. 实测证据（故障场景）

以下为故障场景日志片段（未启动 Nav2 / Nav2 已关闭时）：

    ts=2026-02-25T17:14:47+0800 tool=healthcheck level=OK status=OK reason=start script=w3_d2_nav2_healthcheck
    ts=2026-02-25T17:14:47+0800 tool=healthcheck level=OK status=OK reason=cmd_found cmd=ros2
    ts=2026-02-25T17:14:47+0800 tool=healthcheck level=FAIL status=FAIL reason=node_missing node=/bt_navigator
    ts=2026-02-25T17:14:48+0800 tool=healthcheck level=FAIL status=FAIL reason=node_missing node=/planner_server
    ts=2026-02-25T17:14:48+0800 tool=healthcheck level=FAIL status=FAIL reason=node_missing node=/controller_server
    ts=2026-02-25T17:14:48+0800 tool=healthcheck level=FAIL status=FAIL reason=node_missing node=/map_server
    ts=2026-02-25T17:14:48+0800 tool=healthcheck level=FAIL status=FAIL reason=topic_info_error topic=/tf rc=1
    ts=2026-02-25T17:14:48+0800 tool=healthcheck level=FAIL status=FAIL reason=topic_info_error topic=/tf_static rc=1
    ts=2026-02-25T17:14:49+0800 tool=healthcheck level=FAIL status=FAIL reason=topic_info_error topic=/map rc=1
    ts=2026-02-25T17:14:49+0800 tool=healthcheck level=FAIL status=FAIL reason=lifecycle_get_error node=/map_server rc=1
    ts=2026-02-25T17:14:49+0800 tool=healthcheck level=FAIL status=FAIL reason=summary result=FAIL fail_count=8

结论：
- 故障场景能稳定触发 FAIL
- 失败原因可读（节点缺失 / topic 查询失败 / lifecycle 查询失败）
- `fail_count` 统计正确（4 + 3 + 1 = 8）

---

## 9. 今日踩坑与修正

### 9.1 管道后直接 `echo $?` 会拿到 `tee` 的返回码，不是脚本返回码
错误写法（会误判为 0）：

    ./scripts/w3_d2_nav2_healthcheck.sh 2>&1 | tee -a logs/w3_d2_healthcheck_run.log
    echo "exit_code=$?"

原因：
- 管道最后一个命令是 `tee`
- `$?` 默认取上一条命令返回码（即 `tee`）

因此即使 healthcheck 脚本已经 FAIL，`tee` 写日志成功时仍可能显示 `exit_code=0`

### 9.2 正确获取 healthcheck 脚本返回码：使用 `PIPESTATUS`
推荐写法（标准口径）：

    ./scripts/w3_d2_nav2_healthcheck.sh 2>&1 | tee -a logs/w3_d2_healthcheck_run.log
    echo "script_exit_code=${PIPESTATUS[0]} tee_exit_code=${PIPESTATUS[1]}"

说明：
- `PIPESTATUS[0]`：healthcheck 脚本返回码
- `PIPESTATUS[1]`：tee 返回码

### 9.3 `2>1` 和 `2>&1` 不是一回事
- `2>&1`：stderr 合并到 stdout（正确）
- `2>1`：stderr 重定向到名为 `1` 的文件

---

## 10. 标准运行命令（后续固定口径）

### 10.1 健康检查 + 落盘 + 正确拿返回码
    ./scripts/w3_d2_nav2_healthcheck.sh 2>&1 | tee -a logs/w3_d2_healthcheck_run.log
    rc=${PIPESTATUS[0]}
    echo "script_exit_code=$rc"

---

## 11. 当前版本限制（MVP 的边界）

当前版本是最小可用版，已满足 w3_d2 目标，但仍有以下边界：

- 节点清单写死在脚本内（尚未参数化）
- topic 检查只看 `Publisher count`，未检查 QoS 兼容性
- lifecycle 只检查 `/map_server`
- 未检查 action server（如 `/navigate_to_pose`）健康状态
- 未检查 `/amcl_pose`、`/odom`、`/scan` 等业务 topic

这些扩展项可在后续版本中逐步加入，但不影响当前版本完成度。

---

## 12. 本日结论（一句话）

本日完成了可落地、可落盘、可判返回码的 `nav2_healthcheck.sh` 最小版本，并已验证健康场景与故障场景；同时识别并修正了 shell 管道返回码误判问题（`$?` vs `PIPESTATUS`），为后续回归脚本与自动化报告脚本打下基础。