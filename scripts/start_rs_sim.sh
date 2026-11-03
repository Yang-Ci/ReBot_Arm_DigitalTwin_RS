#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

REBOTARM_SIM_LOCK=/tmp/rebotarm_rs_sim.lock

# --force / -f: kill stale RS processes and remove the lock before starting
if [[ "${1:-}" == "--force" || "${1:-}" == "-f" ]]; then
  echo "Force-cleaning stale RS simulation processes..." >&2
  fuser -k "${REBOTARM_SIM_LOCK}" 2>/dev/null || true
  # Match the complete simulation namespace: /rebot_arm_rs is real hardware.
  pkill -f 'arm_namespace:=rebot_arm_rs_sim([[:space:]]|$)' 2>/dev/null || true
  sleep 1
  rm -f "${REBOTARM_SIM_LOCK}"
fi

exec 9>"${REBOTARM_SIM_LOCK}"
if ! flock -n 9; then
  echo "RS simulation is already running (lock: ${REBOTARM_SIM_LOCK})." >&2
  echo "Use --force to kill stale processes and restart." >&2
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx '/fake_rebotarm_rs_driver'; then
  echo "RS simulation nodes are already present; refusing to start duplicates." >&2
  echo "Stop the existing start_rs_sim.sh process first (or use --force)." >&2
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

setsid ros2 launch rebotarm_bringup fake_rs_bringup.launch.py \
  arm_namespace:=rebot_arm_rs_sim \
  max_joint_speed:="${REBOTARM_SIM_MAX_JOINT_SPEED:-0.0}" \
  use_rviz:="${REBOTARM_USE_RVIZ:-false}" 9>&- &
REBOTARM_CHILD_PIDS+=("$!")

setsid ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebot_arm_rs_sim \
  simulation_mode:="${REBOTARM_MUJOCO_MODE:-physics}" \
  wrist_camera_model:="${REBOTARM_WRIST_CAMERA_MODEL:-d405}" \
  enable_camera_selector:="${REBOTARM_CAMERA_SELECTOR:-${REBOTARM_MUJOCO_VIEWER:-true}}" \
  use_viewer:="${REBOTARM_MUJOCO_VIEWER:-true}" 9>&- &
REBOTARM_CHILD_PIDS+=("$!")

if [[ "${REBOTARM_START_AGENT:-true}" == "true" ]]; then
  if command -v ss >/dev/null && ss -ltn | grep -qE '[:.]8081[[:space:]]'; then
    echo "Reusing the MCP agent already listening on port 8081."
  else
    setsid ros2 launch rebotarm_agent rebotarm_mcp.launch.py \
      arm_namespace:=rebot_arm_rs_sim \
      motion_mode:=allow \
      host:=127.0.0.1 \
      port:=8081 9>&- &
    REBOTARM_CHILD_PIDS+=("$!")
  fi
fi

if command -v ss >/dev/null && ss -ltn | grep -qE '[:.]9090[[:space:]]'; then
  echo "Reusing the rosbridge server already listening on port 9090."
else
  setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    port:=9090 address:=0.0.0.0 9>&- &
  REBOTARM_CHILD_PIDS+=("$!")
fi

echo "RS simulation: /rebot_arm_rs_sim"
echo "Fake driver joint speed limit: ${REBOTARM_SIM_MAX_JOINT_SPEED:-0.0} rad/s (0 = disabled)"
echo "MuJoCo mode: ${REBOTARM_MUJOCO_MODE:-physics}"
echo "MuJoCo viewer: ${REBOTARM_MUJOCO_VIEWER:-true}"
echo "Wrist camera: ${REBOTARM_WRIST_CAMERA_MODEL:-d405} (press C in MuJoCo to switch)"
echo "MuJoCo state: /rebot_arm_rs_sim/mujoco/joint_states"
echo "Object states: /rebot_arm_rs_sim/mujoco/object_states"
echo "Color detections: /rebot_arm_rs_sim/vision/color_blocks/detections"
if [[ "${REBOTARM_START_AGENT:-true}" == "true" ]]; then
  echo "MCP grasp agent: http://127.0.0.1:8081/mcp"
fi
echo "rosbridge: ws://0.0.0.0:9090"
wait
