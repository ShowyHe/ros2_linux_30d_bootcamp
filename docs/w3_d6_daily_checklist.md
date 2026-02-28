# w3_d6 Daily Checklist

## 今日目标
完成最小回归脚本 `scripts/w3_d6_run_regression.sh`，在 live Gazebo + live Nav2 环境下自动运行 3 次 goal，并生成稳定的 CSV 结果文件与对应 run 日志。

## 输入口径
- 地图：Map1
- 系统：live Gazebo + live Nav2
- 固定 goal：
  - frame_id=map
  - x=1.8
  - y=0.0
  - yaw=0.0

## 今日实现内容
- 新增 `scripts/w3_d6_run_regression.sh`
- 脚本支持参数 `N`，默认运行 3 次
- 每次 run 调用一次 `nav2_toolkit` 的 goal sender
- 每次 run 输出单独写入 `logs/w3_d6_runs/`
- 脚本从 run 日志中解析 `result`、`time_sec`
- 汇总生成 `results/w3_d6_regression_dryrun.csv`

## 今日产出
- `scripts/w3_d6_run_regression.sh`
- `results/w3_d6_regression_dryrun.csv`
- `docs/w3_d6_daily_checklist.md`

## 代码
```bash
#!/usr/bin/env bash
set -euo pipefail

# w3_d6_run_regression.sh
# 用法：
#   ./scripts/w3_d6_run_regression.sh
#   ./scripts/w3_d6_run_regression.sh 3
#
# 说明：
# - 默认跑 3 次
# - 每次调用 nav2_toolkit 的 goal sender
# - 每次 run 输出单独写日志
# - 最终汇总到 results/w3_d6_regression_dryrun.csv

N="${1:-3}"

BOOTCAMP_ROOT="${HOME}/ros2_linux_30d_bootcamp"
RESULTS_DIR="${BOOTCAMP_ROOT}/results"
RUN_LOG_DIR="${BOOTCAMP_ROOT}/logs/w3_d6_runs"
CSV_PATH="${RESULTS_DIR}/w3_d6_regression_dryrun.csv"

mkdir -p "${RESULTS_DIR}" "${RUN_LOG_DIR}"

# 固定目标点（今天先用最稳的一组）
FRAME_ID="map"
GOAL_X="1.8"
GOAL_Y="0.0"
GOAL_YAW="0.0"

source /opt/ros/humble/setup.bash
source "${BOOTCAMP_ROOT}/ros2_ws/install/setup.bash"

# 每次重新生成今天这份 dryrun CSV
echo "run_id,goal_id,result,time_sec,notes,recovery,recovery_types,recovery_hits,log_path" > "${CSV_PATH}"

for run_id in $(seq 1 "${N}"); do
  ts="$(date +%Y%m%d_%H%M%S)"
  run_log="${RUN_LOG_DIR}/run_${run_id}_${ts}.log"

  echo "[INFO] run_id=${run_id} log=${run_log}"

  set +e
  ros2 run nav2_toolkit nav2_goal_sender --ros-args \
    -p frame_id:="${FRAME_ID}" \
    -p x:="${GOAL_X}" \
    -p y:="${GOAL_Y}" \
    -p yaw:="${GOAL_YAW}" 2>&1 | tee "${run_log}"
  pipe_rc=("${PIPESTATUS[@]}")
  set -e

  goal_sender_rc="${pipe_rc[0]}"

  goal_id=""
  result="UNKNOWN"
  time_sec=""
  notes="goal_sender_rc=${goal_sender_rc};frame=${FRAME_ID};x=${GOAL_X};y=${GOAL_Y};yaw=${GOAL_YAW}"

  # 解析 result
  if grep -q "result_status_name=SUCCEEDED" "${run_log}"; then
    result="SUCCEEDED"
  elif grep -q "result_status_name=ABORTED" "${run_log}"; then
    result="ABORTED"
  elif grep -q "result_status_name=CANCELED" "${run_log}"; then
    result="CANCELED"
  elif grep -q "goal_rejected" "${run_log}"; then
    result="REJECTED"
  elif grep -q "server_not_available" "${run_log}"; then
    result="SERVER_UNAVAILABLE"
  fi

  # 解析 goal_id（如果日志里有）
  if grep -q "goal_id=" "${run_log}"; then
    goal_id="$(grep 'goal_id=' "${run_log}" | tail -n 1 | sed -E 's/.*goal_id=([^ ]+).*/\1/')"
  fi

  # 解析 time_sec（如果日志里有）
  if grep -q "time_sec=" "${run_log}"; then
    time_sec="$(grep 'time_sec=' "${run_log}" | tail -n 1 | sed -E 's/.*time_sec=([0-9.]+).*/\1/')"
  fi

  echo "${run_id},${goal_id},${result},${time_sec},${notes},,,${run_log}" >> "${CSV_PATH}"

  sleep 2
done

echo "[INFO] done csv=${CSV_PATH}"
```

## 完成判据（DoD）
- `./scripts/w3_d6_run_regression.sh 3` 能正常执行完成
- `results/w3_d6_regression_dryrun.csv` 包含 1 行 header + 3 行数据
- `logs/w3_d6_runs/` 下存在 3 份 run 日志
- CSV 中的 `log_path` 字段可追溯到对应 run 日志

## 本次运行结果
根据终端输出与 run 日志，本次 dryrun 共执行 3 次：

- run1：`SUCCEEDED`，`time_sec=28.750`
- run2：`SUCCEEDED`，`time_sec=0.063`
- run3：`SUCCEEDED`，`time_sec=0.064`

每次 run 均出现完整的 action 证据链：
- `goal_sent`
- `goal_accepted`
- `result_received`
- `result_status_name=SUCCEEDED`

说明：
- `run1` 为有效导航样本
- `run2` 与 `run3` 由于沿用同一目标点、且机器人在 `run1` 后已到达目标附近，因此出现近零耗时成功
- 因此，`run2` 与 `run3` 主要作为“自动化流程正确”的证据，不作为独立导航样本用于性能比较

## 今日关键工程结论
今天的重点不是导航性能评估，而是将 live Nav2 操作固化为一个“可重复执行、可自动落盘、可追溯日志”的最小回归流程。

从执行结果看，`w3_d6_run_regression.sh` 已满足最小可用要求：
- 能自动重复执行 N 次
- 能为每次 run 生成独立日志
- 能生成稳定 CSV
- 能建立 CSV 与日志之间的追溯关系

本次 dryrun 还暴露出一个实验口径问题：若不在每次 run 前重置机器人状态，且连续发送相同 goal，则后续 run 可能出现“近零耗时成功”。该问题不影响今日脚本雏形的完成判定，但后续若要形成有统计意义的回归结果，需要增加“重置起点”或“交替目标点”的机制。