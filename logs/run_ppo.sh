#!/bin/bash
# ================================================================
# run_ppo.sh
# 설명: ppo_demo.launch.py 실행 + 전체 출력을 타임스탬프 로그 파일로 저장.
# 사용법:
#   ~/idle_ws/logs/run_ppo.sh [launch 인수...]
#   ~/idle_ws/logs/run_ppo.sh armed:=true image_device:=4 whisper_model_size:=small
# ================================================================
set -eo pipefail

LOG_DIR=~/idle_ws/logs
mkdir -p "$LOG_DIR"

TS=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$LOG_DIR/ppo_${TS}.log"

echo "▶ 로그 저장: $LOG_FILE"
ln -sf "$LOG_FILE" "$LOG_DIR/latest.log"

source /opt/ros/humble/setup.bash
source ~/idle_ws/install/setup.bash

ros2 launch demo_supervisor ppo_demo.launch.py \
    "plan_diag_csv_path:=$LOG_DIR/ppo_${TS}_plan.csv" \
    "$@" 2>&1 | tee "$LOG_FILE"

echo ""
echo "▶ 로그 완료: $LOG_FILE"
