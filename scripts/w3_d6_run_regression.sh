#!/usr/bin/env bash
set -euo pipefail

# w3_d6_run_regression.sh
# 用法：
#   ./scripts/w3_d6_run_regression.sh
#   ./scripts/w3_d6_run_regression.sh 3
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

set +u
source /opt/ros/humble/setup.bash
source "${BOOTCAMP_ROOT}/ros2_ws/install/setup.bash"
set -u

# 每次重新生成今天这份 dryrun CSV
echo "run_id,goal_id,result,time_sec,notes,recovery,recovery_types,recovery_hits,log_path" > "${CSV_PATH}"

for run_id in $(seq 1 "${N}"); do
  ts="$(date +%Y%m%d_%H%M%S)"
  run_log="${RUN_LOG_DIR}/run_${run_id}_${ts}.log"

  echo "[INFO] run_id=${run_id} log=${run_log}"

  set +e
  ros2 run nav2_toolkit goal_sender --ros-args \
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