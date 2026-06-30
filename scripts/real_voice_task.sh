#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# STT로 음성 녹음 → Whisper 변환 → Qwen/rule 파서 → /pickplace/command
# stt.py --voice 모드는 스페이스바로 녹음 시작/중단, 결과를 stdout에 JSON 출력
TRANSCRIBED=$(python3 src/stt/stt.py --voice --parser "${PARSER:-rule}" 2>/dev/null | python3 -c "
import json, sys
plan = json.loads(sys.stdin.read())
if plan.get('success') and plan.get('steps'):
    print(plan['steps'][0].get('object','') + ' ' + plan['steps'][0].get('target',''))
")

if [ -z "${TRANSCRIBED}" ]; then
  echo "STT 파싱 실패" >&2
  exit 1
fi

PYTHONPATH=src/ml python3 src/ml/stage4/vision_task_orchestrator.py \
  --text "${TRANSCRIBED}" \
  --parser "${PARSER:-rule}" \
  --camera-device "${CAMERA_DEVICE:-1}" \
  --camera-width "${CAMERA_WIDTH:-1280}" \
  --camera-height "${CAMERA_HEIGHT:-720}" \
  --device "${MODEL_DEVICE:-cuda}" \
  --publish
