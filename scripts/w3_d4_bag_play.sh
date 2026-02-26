#!/usr/bin/env bash
set -euo pipefail

# 用法：
#   ./scripts/w3_d4_bag_play.sh <bag_dir> [--loop]
#
# 示例：
#   ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02
#   ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02 --loop

usage() {
  cat <<'EOF'
Usage:
  ./scripts/w3_d4_bag_play.sh <bag_dir> [--loop]

Examples:
  ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02
  ./scripts/w3_d4_bag_play.sh bags/w3_d3_map1_run02 --loop
EOF
}

if [[ $# -lt 1 ]]; then
  echo "[ERROR] missing bag_dir"
  usage
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found"
  exit 127
fi

BAG_DIR="$1"
LOOP_FLAG="${2:-}"

if [[ ! -d "${BAG_DIR}" ]]; then
  echo "[ERROR] bag_dir not found: ${BAG_DIR}"
  exit 3
fi

TOPICS=(
  /clock
  /tf
  /tf_static
  /scan
  /map
  /odom
  /amcl_pose
  /cmd_vel
  /navigate_to_pose/_action/status
)

CMD=(ros2 bag play "${BAG_DIR}")

if [[ "${LOOP_FLAG}" == "--loop" ]]; then
  CMD+=(--loop)
fi

CMD+=(--topics "${TOPICS[@]}")

echo "[INFO] bag_dir=${BAG_DIR}"
echo "[INFO] loop_flag=${LOOP_FLAG:-none}"
echo "[INFO] replay_topics=${TOPICS[*]}"
echo "[INFO] policy=bag_contains_/clock_so_do_not_use_--clock"

exec "${CMD[@]}"