#!/bin/bash

TIMEOUT=${TIMEOUT:-5}
MONITORED_NODES=("dart_aim_node" "hik_camera")
LAUNCH_FILE=${LAUNCH_FILE:-"yq_dart_aim start.launch.py"}

USER_NAME="${SUDO_USER:-$(whoami)}"
HOME_DIR=$(eval echo "~$USER_NAME")
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${WORKSPACE_DIR:-}" && -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  WORKING_DIR="$WORKSPACE_DIR"
elif [[ -f "$SCRIPT_DIR/../../install/setup.bash" ]]; then
  WORKING_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
elif [[ -f "$HOME_DIR/dart_ws_on/install/setup.bash" ]]; then
  WORKING_DIR="$HOME_DIR/dart_ws_on"
else
  echo "[watchdog] Cannot locate ROS2 workspace (missing install/setup.bash)."
  exit 1
fi

OUTPUT_FILE="${OUTPUT_FILE:-$WORKING_DIR/screen.output}"
BAG_DIR=${BAG_DIR:-"$WORKING_DIR/bag_records/debug_record"}
LOG_FILE="/tmp/rm_watchdog.log"

log() {
  echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

rmw="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_IMPLEMENTATION="$rmw"
export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

source /opt/ros/humble/setup.bash || { log "ERROR: failed to source humble setup"; exit 1; }
source "$WORKING_DIR/install/setup.bash" || { log "ERROR: failed to source workspace setup"; exit 1; }

bringup_bag() {
  mkdir -p "$BAG_DIR" 2>/dev/null || true
  chown "$USER_NAME":"$USER_NAME" "$BAG_DIR" 2>/dev/null || true
  local bag_name="$BAG_DIR/$(date +%Y%m%d_%H%M%S)_$$"
  nohup ros2 bag record /debug_record/mask_compressed /debug_record/image_compressed \
    --storage mcap \
    -o "$bag_name" \
    >> /tmp/ros2_bag_record.log 2>&1 &
  log "bag record started: $bag_name (pid $!)"
}

# web_tuner 是否启用（设置 WEB_TUNER=true 启用）
WEB_TUNER=${WEB_TUNER:-false}
WEB_TUNER_PORT=${WEB_TUNER_PORT:-8080}

function bringup_web_tuner() {
  if [[ "$WEB_TUNER" == "true" ]]; then
    local script="$WORKING_DIR/install/yq_dart_aim/share/yq_dart_aim/web_tuner/server.py"
    if [[ -f "$script" ]]; then
      nohup python3 "$script" --port "$WEB_TUNER_PORT" >> /tmp/web_tuner.log 2>&1 &
      log "web_tuner started on port $WEB_TUNER_PORT (pid $!)"
    else
      log "WARN: web_tuner script not found: $script"
    fi
  fi
}

function is_web_tuner_running() {
  pgrep -f "web_tuner/server.py" > /dev/null 2>&1
}

function bringup() {
  source /opt/ros/humble/setup.bash || true
  source "$WORKING_DIR/install/setup.bash" || true
  nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &
  # bringup_bag
  bringup_web_tuner
  sleep 1
  chown -R "$USER_NAME":"$USER_NAME" "$BAG_DIR" 2>/dev/null || true
}

function stop_stack() {
  log "stopping stack"
  pkill -f "ros2 bag record" 2>/dev/null || true
  pkill -f "web_tuner/server.py" 2>/dev/null || true
  pkill -f "ros2 launch $LAUNCH_FILE" 2>/dev/null || true
  for node in "${MONITORED_NODES[@]}"; do
    pkill -f "$node" 2>/dev/null || true
  done
}

function restart() {
  log "restarting full stack"
  stop_stack
  ros2 daemon stop || true
  ros2 daemon start || true
  bringup
  sleep "$TIMEOUT"
}

function is_node_running() {
  local node_name="$1"
  ros2 node list 2>/dev/null | grep -Eq "^/${node_name}$|^${node_name}$"
}

is_bag_running() {
  pgrep -f "ros2 bag record" > /dev/null 2>&1
}

trap 'log "watchdog exiting (signal received)"; stop_stack; exit 0' SIGTERM SIGINT

log "watchdog starting"
bringup
sleep "$TIMEOUT"
sleep "$TIMEOUT"

while true; do
  for node in "${MONITORED_NODES[@]}"; do
    echo "- Check $node"
    if is_node_running "$node"; then
      echo "    $node is running"
    else
      log "WARN: $node is missing, restarting all nodes"
      restart
      break
    fi
  done

  # if is_bag_running; then
  #   echo "- bag record is running"
  # else
  #   log "WARN: bag record not running, restarting it"
  #   bringup_bag
  # fi

  # 检查 web_tuner
  if [[ "$WEB_TUNER" == "true" ]]; then
    if is_web_tuner_running; then
      echo "- web_tuner is running"
    else
      log "WARN: web_tuner not running, restarting it"
      bringup_web_tuner
    fi
  fi

  chown -R "$USER_NAME":"$USER_NAME" "$BAG_DIR" 2>/dev/null || true
  sleep "$TIMEOUT"
done

