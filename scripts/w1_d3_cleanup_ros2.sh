#!/usr/bin/env bash
set -euo pipefail
mkdir -p $HOME/ros2_linux_30d_bootcamp/logs

TM="$(date +%Y%m%d_%H%M%S)"
LOG="$HOME/ros2_linux_30d_bootcamp/logs/trail_${TM}.log"
PAT="component_container_isolated|nav2_container|gzserver|gzclient|bt_navigator|map_server|amcl|controller_server|planner_server"

echo "[INFO] before cleaning" | tee -a "$LOG" 
ps aux | grep -E "$PAT" | grep -v grep | tee -a "$LOG" || true

echo "[INFO] cleaning" | tee -a "$LOG" 
pkill -2 -f "$PAT" | tee -a "$LOG" || true

sleep 2

echo "[INFO] cleaning" | tee -a "$LOG"
pkill -15 -f "$PAT" | tee -a "$LOG" || true

sleep 2

echo "[INFO] after cleaning" | tee -a "$LOG"
ps aux | grep -E "$PAT" | grep -v grep | tee -a "$LOG" || true

echo "[INFO] cleaning done, log=$LOG" |tee -a "$LOG"
