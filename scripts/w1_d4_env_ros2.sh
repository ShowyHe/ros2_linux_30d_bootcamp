#!/usr/bin/env bash
set -euo pipefail

WS_PATH="${1:-}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
	echo "[ERR] /opt/ros/humble/setup.bash not foung"
	return 1 2>/dev/null || exit 1
fi

source /opt/ros/humble/setup.bash

if [[ -n "$WS_PATH" ]]; then
	if [[ ! -f "$WS_PATH/install/setup.bash" ]]; then
		echo "[ERR] overlay not found: $WS_PATH/install/setup.bash"
		return 1 2>/dev/null || exit 1
	fi
	source "$WS_PATH/install/setup.bash"
fi

echo "[OK] ROS_DISTRO=${ROS_DISTRO:-<empty>}"
echo "[OK] which ros2: $(which ros2 2>/dev/null || echo '<not found>')"
echo "[OK] AMENT_PREFIX_PATH(head);"
echo "${AMENT_PREFIX_PATH:-}" |tr ':' '\n' | head -n 5
echo "[OK] PATH(head):"
echo "${PATH:-}" | tr ':' '\n' | head -n 5
