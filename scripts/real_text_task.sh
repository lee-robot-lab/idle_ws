#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ "$#" -lt 1 ]; then
  echo "usage: $0 '파란 블록을 바구니에 넣어줘'" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text "$*" \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --device "${MODEL_DEVICE:-cuda}" \
  --publish
