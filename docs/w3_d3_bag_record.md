# w3_d3 bag 录制工程化

## 1. 今日目标

本日任务不是调导航参数，也不是评估成功率，而是将 bag 录制流程工程化，形成可复现、可验证、可交接的证据采集链路。目标包括：

- 固定 topic 白名单，避免无控制地录制全部 topic
- 固定输出目录结构，统一 bag 命名口径
- 录制后使用 `ros2 bag info` 做最小验证
- 将白名单选择依据、复现命令和验证方式写入文档

本次交付对应脚本为：

    scripts/w3_d3_bag_record.sh

本次样本 bag 目录为：

    bags/w3_d3_map1_run01/
    bags/w3_d3_map1_run02/

其中，`run01` 录制本身成功，但脚本尾部存在 `PIPESTATUS` 取值缺陷；`run02` 为修复后再次验证通过的样本，作为本日最终验证结果。

---

## 2. 白名单 topic 设计

本日固定录制以下 topic：

- `/clock`
- `/tf`
- `/tf_static`
- `/scan`
- `/map`
- `/odom`
- `/amcl_pose`
- `/cmd_vel`
- `/navigate_to_pose/_action/status`

### 2.1 白名单选择原则

本次不采用“全量录制所有 topic”的方式，原因如下：

- 全量录制会引入大量与当前分析无关的数据，增加存储和后处理负担
- topic 集合不固定，会导致不同 run 之间采集口径漂移，不利于后续回放、归因和自动化脚本复用
- 当前阶段的目标是验证导航链路中的核心观测、状态、控制与 action 状态，因此应只保留必要证据源

### 2.2 各 topic 的作用

#### `/clock`
用于回放时提供仿真时间基准。后续 bag 回放与 `use_sim_time:=True` 链路需要该 topic 参与。

#### `/tf`
记录动态 TF 变换，是导航运行过程中机器人位姿关系变化的重要证据源。

#### `/tf_static`
记录静态 TF 变换。许多显示与坐标链校验依赖静态 TF，缺失后容易在回放或可视化阶段出现异常。

#### `/scan`
记录激光雷达观测，是导航输入证据之一，可用于后续分析感知输入是否正常。

#### `/map`
记录地图数据，是全局导航的基础输入。回放或 QoS 排查时常需要验证其存在与发布情况。

#### `/odom`
记录里程计输出，可用于分析机器人运动状态与底盘反馈。

#### `/amcl_pose`
记录定位估计结果，用于判断定位链路是否工作及其更新频率。

#### `/cmd_vel`
记录控制输出，反映导航系统是否实际下发速度命令，是“系统是否真的在驱动机器人”的直接证据。

#### `/navigate_to_pose/_action/status`
记录导航 action 状态，用于保留任务执行过程中的状态变化证据，如 goal 是否被接受、是否完成、是否失败等。

---

## 3. 脚本设计与目录口径

本日实现脚本：

    scripts/w3_d3_bag_record.sh

脚本主要设计点如下：

- 输入参数为 bag 名称，例如：

        ./scripts/w3_d3_bag_record.sh w3_d3_map1_run02

- 输出目录统一放到：

        ~/ros2_linux_30d_bootcamp/bags/

- 最终目录结构示例：

        bags/w3_d3_map1_run02/

- 录制前可选执行：

        scripts/w3_d2_nav2_healthcheck.sh

- 若输出目录已存在，则脚本显式失败，避免覆盖旧证据
- 使用 `--include-hidden-topics`，确保 `_action/status` 等隐藏 topic 可被录制
- 录制结束后自动执行：

        ros2 bag info <bag_dir>

  对 bag 做最小验证

---

## 4. 复现步骤

## 4.1 启动导航系统

先按既有口径启动 Gazebo 与 Nav2，确保导航系统处于正常运行状态。

## 4.2 录制前健康检查

在训练仓库根目录执行：

    ./scripts/w3_d2_nav2_healthcheck.sh 2>&1 | tee -a logs/w3_d3_precheck.log
    echo "script_exit_code=${PIPESTATUS[0]} tee_exit_code=${PIPESTATUS[1]}"

本次实际结果为：

- `script_exit_code=0`
- `map_server` 状态为 `active`
- `/tf`、`/tf_static`、`/map` 发布者检查通过

说明录制前系统健康状态满足要求。

## 4.3 启动录制

在训练仓库根目录执行：

    ./scripts/w3_d3_bag_record.sh w3_d3_map1_run02

脚本启动后进入持续录制状态，等待手动 `Ctrl+C` 停止。

## 4.4 录制期间制造有效数据

录制期间应至少执行一次导航动作，避免得到“仅有静态环境、缺少行为证据”的 bag。  
本次做法是在录制进行中发送一次导航 goal，以产生：

- `/cmd_vel`
- `/odom`
- `/amcl_pose`
- `/navigate_to_pose/_action/status`

等 topic 的有效数据。

## 4.5 停止录制并验证

停止录制后执行以下命令检查产物：

    ls -lah bags/w3_d3_map1_run02
    find bags/w3_d3_map1_run02 -maxdepth 2 -type f | sort
    ros2 bag info bags/w3_d3_map1_run02 | tee -a logs/w3_d3_bag_record_info.log

---

## 5. 实际验证结果

## 5.1 bag 目录验证

`bags/w3_d3_map1_run02/` 已成功生成，目录内包含：

- `metadata.yaml`
- `w3_d3_map1_run02_0.db3`

满足“bag 目录存在且含元信息与数据库文件”的最小要求。

## 5.2 ros2 bag info 验证

对 `run02` 执行 `ros2 bag info`，结果如下：

- Files: `w3_d3_map1_run02_0.db3`
- Bag size: `3.0 MiB`
- Duration: `49.455921478s`
- Messages: `5512`

Topic 统计如下：

- `/cmd_vel`：743
- `/amcl_pose`：45
- `/map`：1
- `/navigate_to_pose/_action/status`：3
- `/clock`：481
- `/tf`：2585
- `/odom`：1413
- `/scan`：240
- `/tf_static`：1

说明：

- 9 个白名单 topic 均已录入
- `_action/status` 成功被录到，说明 `--include-hidden-topics` 生效
- bag 可被 `ros2 bag info` 正常解析，满足本日 DoD

---

## 6. 录制过程中的现象与问题

## 6.1 预检查阶段 `/navigate_to_pose/_action/status` 缺失告警

脚本在录制前 topic 预检查阶段输出过如下告警：

- `topic_missing_before_record topic=/navigate_to_pose/_action/status`

该现象并不表示录制失败，而是反映录制前时刻该 topic 尚未被发现。随后 recorder 在 discovery 阶段成功订阅该 topic，最终 `ros2 bag info` 也验证了该 topic 已录到 3 条消息。

因此，该告警应解释为：

- 录制前时序差异导致的 WARN
- 不属于最终录制失败

## 6.2 run01 暴露的脚本问题

`run01` 录制完成后，脚本尾部报错：

- `PIPESTATUS[1]: unbound variable`

问题原因是：

- 脚本在 `set -u` 下直接分两步读取 `PIPESTATUS[0]` 与 `PIPESTATUS[1]`
- Bash 在后续命令执行后会刷新 `PIPESTATUS`
- 导致访问不存在的数组元素时触发 `unbound variable`

修复方法为：

- 在 pipeline 执行后，立即用数组快照保存 `PIPESTATUS[@]`
- 再从快照数组中分别读取两个返回码

修复后重新录制 `run02`，脚本尾部验证通过，问题消失。

---

## 7. 结论

本日完成了 bag 录制的最小工程化闭环，具体表现为：

- 固定了 topic 白名单
- 固定了输出目录结构
- 实现了录制脚本 `w3_d3_bag_record.sh`
- 通过 `ros2 bag info` 完成了录制结果验证
- 对录制过程中的时序告警与脚本尾部错误完成了证据化定位与修复

这意味着后续 Week3 Day4 的 bag 回放工程化，已经具备稳定输入样本与统一采集口径，不再依赖临时手工录制。

---

## 8. 本日交付物

- `scripts/w3_d3_bag_record.sh`
- `docs/w3_d3_bag_record.md`
- `bags/w3_d3_map1_run01/`
- `bags/w3_d3_map1_run02/`
- `logs/w3_d3_precheck.log`
- `logs/w3_d3_bag_record_info.log`

如需只保留“修复后最终样本”，可在后续整理阶段将 `run02` 作为主样本引用。