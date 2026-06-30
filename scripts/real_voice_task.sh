#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# STT로 텍스트 받아서 orchestrator에 전달
# PYTHONPATH=src/ml python3 src/ml/stt/record_and_transcribe.py 같은 STT 모듈이
# 텍스트를 stdout으로 출력하면 아래처럼 연결한다.
# 현재는 터미널 입력으로 대체 (STT 모듈 연결 전 테스트용).

echo "음성 명령을 입력하세요 (텍스트로 대체):" >&2
read -r TASK_TEXT

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text "${TASK_TEXT}" \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --device "${MODEL_DEVICE:-cuda}" \
  --publish
