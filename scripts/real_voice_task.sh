#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

PYTHONPATH=src/ml:${PYTHONPATH:-} python3 src/ml/stage4/vision_task_orchestrator.py \
  --voice \
  --parser "${PARSER:-qwen}" \
  --qwen-compact \
  --capture-camera \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --infer-scene-from-snapshot \
  --scene-source model \
  --device "${MODEL_DEVICE:-cuda}" \
  --qwen-max-new-tokens "${QWEN_MAX_NEW_TOKENS:-1024}" \
  --publish
