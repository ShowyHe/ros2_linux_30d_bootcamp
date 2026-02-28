#!/usr/bin/env bash
set -euo pipefail

# w3_d6_run_regression.sh
# 用法示例：
#   ./scripts/w3_d6_run_regression.sh 3
# 可选环境变量：
#   OUT_CSV=results/w3_d7_map1_5runs.csv
#   RUN_TAG=w3_d7_map1_runs
#   FRAME_ID=map
#   GOAL_SEQUENCE="1.8,0.0,0.0;0.0,0.0,0.0;1.8,0.0,0.0"
#   GOAL_X=1.8
#   GOAL_Y=0.0
#   GOAL_YAW=0.0
#   SLEEP_BETWEEN_RUNS=2
# 说明：
# - 默认跑 3 次
# - 每次调用 nav2_toolkit 的 goal_sender
# - 每次 run 单独写日志
# - 默认输出到 results/w3_d6_regression_dryrun.csv
# - 支持 GOAL_SEQUENCE，为每一轮指定不同 goal，避免“同点连发伪成功”

N="${1:-3}"

BOOTCAMP_ROOT="${HOME}/ros2_linux_30d_bootcamp"
RESULTS_DIR="${BOOTCAMP_ROOT}/results"
LOGS_DIR="${BOOTCAMP_ROOT}/logs"

OUT_CSV="${OUT_CSV:-${RESULTS_DIR}/w3_d6_regression_dryrun.csv}"
FRAME_ID="${FRAME_ID:-map}"
RUN_TAG="${RUN_TAG:-w3_d6_runs}"
RUN_LOG_DIR="${LOGS_DIR}/${RUN_TAG}"
GOAL_SEQUENCE="${GOAL_SEQUENCE:-}"

DEFAULT_GOAL_X="${GOAL_X:-1.8}"
DEFAULT_GOAL_Y="${GOAL_Y:-0.0}"
DEFAULT_GOAL_YAW="${GOAL_YAW:-0.0}"

SLEEP_BETWEEN_RUNS="${SLEEP_BETWEEN_RUNS:-2}"

mkdir -p "${RESULTS_DIR}" "${RUN_LOG_DIR}" "$(dirname "${OUT_CSV}")"

# 环境
set +u
source /opt/ros/humble/setup.bash

WS_SETUP="${BOOTCAMP_ROOT}/ros2_ws/install/setup.bash"
if [ ! -f "${WS_SETUP}" ]; then
  echo "[ERROR] workspace setup not found: ${WS_SETUP}" >&2
  exit 1
fi
source "${WS_SETUP}"
set -u

# 如果传了 GOAL_SEQUENCE，则按分号切成数组
# 例如：
# GOAL_SEQUENCE="1.8,0.0,0.0;0.0,0.0,0.0;1.8,0.0,0.0"
if [ -n "${GOAL_SEQUENCE}" ]; then
  IFS=';' read -r -a GOALS <<< "${GOAL_SEQUENCE}"
  #把GOAL_SEQUENCE里面的值，按；分割成元素，然后每个存到GOALS数组里面
  if [ "${#GOALS[@]}" -lt "${N}" ]; then
    echo "[ERROR] GOAL_SEQUENCE has fewer goals than N=${N}" >&2
    echo "[ERROR] goal_count=${#GOALS[@]} N=${N}" >&2
    exit 1
  fi
fi

# 每次运行都重新生成这份 CSV，避免旧数据残留
echo "run_id,goal_id,result,time_sec,notes,recovery,recovery_types,recovery_hits,log_path" > "${OUT_CSV}"

for run_id in $(seq 1 "${N}"); do
  # 当前轮的目标点，完成后进行循环
  if [ -n "${GOAL_SEQUENCE}" ]; then
    goal_triplet="${GOALS[$((run_id - 1))]}"
    #从goals数组里面取值，然后赋值到goal_triplet
    IFS=',' read -r GOAL_X GOAL_Y GOAL_YAW <<< "${goal_triplet}"
  else
    GOAL_X="${DEFAULT_GOAL_X}"
    GOAL_Y="${DEFAULT_GOAL_Y}"
    GOAL_YAW="${DEFAULT_GOAL_YAW}"
  fi

  ts="$(date +%Y%m%d_%H%M%S)"
  run_log="${RUN_LOG_DIR}/run_${run_id}_${ts}.log"

  echo "[INFO] run_id=${run_id} log=${run_log}"
  echo "[INFO] frame_id=${FRAME_ID} x=${GOAL_X} y=${GOAL_Y} yaw=${GOAL_YAW}" | tee -a "${run_log}"

  set +e
  ros2 run nav2_toolkit goal_sender --ros-args \
    -p frame_id:="${FRAME_ID}" \
    -p x:="${GOAL_X}" \
    -p y:="${GOAL_Y}" \
    -p yaw:="${GOAL_YAW}" 2>&1 | tee -a "${run_log}"
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
  elif grep -q "timeout" "${run_log}"; then
    result="TIMEOUT"
  fi

  # 解析 goal_id（如果日志里有）
  if grep -q "goal_id=" "${run_log}"; then
    goal_id="$(grep 'goal_id=' "${run_log}" | tail -n 1 | sed -E 's/.*goal_id=([^ ]+).*/\1/')"
  fi

  # 解析 time_sec（如果日志里有）
  if grep -q "time_sec=" "${run_log}"; then
    time_sec="$(grep 'time_sec=' "${run_log}" | tail -n 1 | sed -E 's/.*time_sec=([0-9.]+).*/\1/')"
  fi

  echo "${run_id},${goal_id},${result},${time_sec},${notes},,,${run_log}" >> "${OUT_CSV}"

  # 最后一轮不 sleep
  if [ "${run_id}" -lt "${N}" ]; then
    sleep "${SLEEP_BETWEEN_RUNS}"
  fi
done

echo "[INFO] done csv=${OUT_CSV}"

#这段脚本先定义运行次数、输出路径、日志目录和默认 goal 参数，然后加载 ROS2 与 workspace 环境；若传入 GOAL_SEQUENCE，
#则先按分号拆成多组目标，再在每一轮循环中取出对应目标并拆成 x/y/yaw，调用 nav2_toolkit 的 goal_sender 发送 goal。
#每轮运行的终端输出会同时写入独立日志文件。脚本随后从该日志中解析 result、goal_id、time_sec 等字段，
#并连同本轮 goal 参数、命令退出码和日志路径一起写入 CSV，形成可追溯的最小回归结果表。