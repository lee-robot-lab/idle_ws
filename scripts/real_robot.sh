#!/usr/bin/env bash
set -euo pipefail

WS="/home/parkshinyoung/idle_real_ws"
PRESETS="${TASK_PRESETS_YAML_PATH:-$WS/param/tuned/task_presets.yaml}"
CAMERA_DEVICE="${CAMERA_DEVICE:-2}"
CAMERA_WIDTH="${CAMERA_WIDTH:-1280}"
CAMERA_HEIGHT="${CAMERA_HEIGHT:-720}"
CAMERA_PIXFMT="${CAMERA_PIXFMT:-MJPG}"
LOG_DIR="${REAL_ROBOT_LOG_DIR:-/tmp/idle_real_robot_logs}"

usage() {
  cat >&2 <<EOF
usage:
  $0 build
  $0 can
  $0 monitor
  $0 bridge
  $0 control
  $0 up
  $0 text '제일 오른쪽에 있는 블록을 바구니에 넣어줘.'
  $0 voice
  $0 home

env:
  CAMERA_DEVICE=$CAMERA_DEVICE
  CAMERA_WIDTH=$CAMERA_WIDTH
  CAMERA_HEIGHT=$CAMERA_HEIGHT
  REAL_ROBOT_LOG_DIR=$LOG_DIR
EOF
}

source_ros() {
  cd "$WS"
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f install/setup.bash ]; then
    source install/setup.bash
  fi
  set -u
}

build_ws() {
  cd "$WS"
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  colcon build --symlink-install --packages-select msgs sim can_interface idle_common phy idle_launch
}

setup_can() {
  sudo ip link set can0 down || true
  sudo ip link set can0 type can bitrate 1000000
  sudo ip link set can0 up
  ip link show can0
}

setup_camera() {
  v4l2-ctl --device "/dev/video${CAMERA_DEVICE}" \
    --set-fmt-video="width=${CAMERA_WIDTH},height=${CAMERA_HEIGHT},pixelformat=${CAMERA_PIXFMT}"
}

run_monitor() {
  cd "$WS/motor"
  python3 monitor.py --can_id 1 2 3 4 5 6 7
}

run_bridge() {
  source_ros
  ros2 run can_interface can_bridge_node
}

run_control() {
  source_ros
  ros2 launch idle_launch pick_place_control.launch.py \
    task_presets_yaml_path:="$PRESETS"
}

run_up() {
  source_ros
  mkdir -p "$LOG_DIR"

  local control_log="$LOG_DIR/pick_place_control.log"
  local monitor_log="$LOG_DIR/motor_monitor.log"

  echo "[real_robot] logs: $LOG_DIR"
  echo "[real_robot] starting pick_place_control.launch.py"
  ros2 launch idle_launch pick_place_control.launch.py \
    task_presets_yaml_path:="$PRESETS" 2>&1 | tee "$control_log" &
  local control_pid=$!

  sleep 1

  echo "[real_robot] starting motor monitor"
  run_monitor 2>&1 | tee "$monitor_log" &
  local monitor_pid=$!

  cleanup() {
    echo
    echo "[real_robot] stopping..."
    kill "$monitor_pid" "$control_pid" 2>/dev/null || true
    wait "$monitor_pid" "$control_pid" 2>/dev/null || true
  }
  trap cleanup INT TERM EXIT

  echo "[real_robot] ready. CAN and can_bridge are separate commands. Press Ctrl-C to stop."
  wait "$control_pid" "$monitor_pid"
}

run_text() {
  if [ "$#" -lt 1 ]; then
    echo "usage: $0 text '작업 문장'" >&2
    exit 2
  fi
  source_ros
  setup_camera
  CAMERA_DEVICE="$CAMERA_DEVICE" CAMERA_WIDTH="$CAMERA_WIDTH" CAMERA_HEIGHT="$CAMERA_HEIGHT" \
    "$WS/scripts/real_text_task.sh" "$*"
}

run_voice() {
  source_ros
  setup_camera
  CAMERA_DEVICE="$CAMERA_DEVICE" CAMERA_WIDTH="$CAMERA_WIDTH" CAMERA_HEIGHT="$CAMERA_HEIGHT" \
    "$WS/scripts/real_voice_task.sh"
}

run_home() {
  source_ros
  ros2 service call /go_home std_srvs/srv/Trigger {}
}

cmd="${1:-}"
if [ -z "$cmd" ]; then
  usage
  exit 2
fi
shift

case "$cmd" in
  build)
    build_ws
    ;;
  can)
    setup_can
    ;;
  monitor)
    run_monitor
    ;;
  bridge)
    run_bridge
    ;;
  control)
    run_control
    ;;
  up)
    run_up
    ;;
  text)
    run_text "$@"
    ;;
  voice)
    run_voice
    ;;
  home)
    run_home
    ;;
  *)
    usage
    exit 2
    ;;
esac
