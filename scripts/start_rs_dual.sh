#!/usr/bin/env bash

set -euo pipefail

if [[ "${REBOTARM_RS_HARDWARE_CONFIRM:-}" != "I_UNDERSTAND_RS_WILL_MOVE" ]]; then
  echo "Dual launch includes the real RS arm and is blocked by default." >&2
  echo "Set REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE to continue." >&2
  exit 2
fi

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

REBOTARM_SIM_LOCK=/tmp/rebotarm_rs_sim.lock
exec 9>"${REBOTARM_SIM_LOCK}"
if ! flock -n 9; then
  echo "An RS simulation or dual launch is already running." >&2
  exit 1
fi

if ! ip link show can0 >/dev/null 2>&1; then
  echo "SocketCAN interface can0 does not exist." >&2
  exit 1
fi

REBOTARM_CHILD_PIDS=()

cleanup() {
  if ((${#REBOTARM_CHILD_PIDS[@]})); then
    # Phase 1: graceful SIGTERM to process groups
    for pid in "${REBOTARM_CHILD_PIDS[@]}"; do
      kill -TERM -- "-${pid}" 2>/dev/null || true
    done
    # Allow time for graceful shutdown
    sleep 2
    # Phase 2: force SIGKILL -- process groups first, then session survivors
    for pid in "${REBOTARM_CHILD_PIDS[@]}"; do
      kill -KILL -- "-${pid}" 2>/dev/null || true
    done
    # Catch nodes that created their own process groups within the session
    for pid in "${REBOTARM_CHILD_PIDS[@]}"; do
      local survivors
      survivors=$(ps -eo pid,sid --no-headers | awk -v sid="$pid" '$2==sid{print $1}' | tr '\n' ' ')
      if [[ -n "${survivors// /}" ]]; then
        kill -KILL ${survivors} 2>/dev/null || true
      fi
    done
    wait "${REBOTARM_CHILD_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch rebotarm_bringup bringup.launch.py \
  model:=rs channel:=can0 arm_namespace:=rebot_arm_rs \
  ee_frame_id:=gripper_end use_rviz:="${REBOTARM_USE_RVIZ:-false}" 9>&- &
REBOTARM_CHILD_PIDS+=("$!")

setsid ros2 launch rebotarm_bringup fake_rs_bringup.launch.py \
  arm_namespace:=rebot_arm_rs_sim use_rviz:=false 9>&- &
REBOTARM_CHILD_PIDS+=("$!")

if command -v ss >/dev/null && ss -ltn | grep -qE '[:.]9090[[:space:]]'; then
  echo "Reusing the rosbridge server already listening on port 9090."
else
  setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:=9090 address:=0.0.0.0 9>&- &
  REBOTARM_CHILD_PIDS+=("$!")
fi

echo "RS hardware namespace: /rebot_arm_rs"
echo "RS simulation namespace: /rebot_arm_rs_sim"
echo "rosbridge: ws://0.0.0.0:9090"
wait
