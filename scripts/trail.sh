#!/usr/bin/env bash
set -euo pipefail

mkdir -p $HOME/ros2_linux_30d_bootcamp/logs
TM="$(date +%{Y%m%d_%H%M%S})"
LOG="$HOME/ros2_linux_30d_bootcamp/logs/trail_${TM}.log"
PTR="nav2_container|gzclient|gzserver|amcl|bt_navigator|map_server|controller_server|planner_server"

echo "[INFO] before cleaning" | tee -a $LOG
ps aux | grep -E $PTR | grep -v grep | tee -a $LOG || true

echo "[INFO] cleaning" | tee -a $LOG
pkill -2 $PTR | tee -a $LOG || true

echo "[INFO] cleaning" | tee -a $LOG
pkill -15 $PTR | tee -a $LOG || true

echo "[INFO] after cleaning" | tee -a $LOG
ps aux | grep -E $PTR | grep -v grep | tee -a $LOG || true

echo "[INFO] cleaning done" | tee -a $LOG
