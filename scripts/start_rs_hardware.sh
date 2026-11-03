#!/usr/bin/env bash

set -euo pipefail

REBOTARM_HARDWARE_LOCK="${XDG_RUNTIME_DIR:-/tmp}/rebotarm_rs_hardware.lock"
REBOTARM_REPLACE_TIMEOUT="${REBOTARM_REPLACE_TIMEOUT:-35}"

declare -A REBOTARM_OLD_DDS_FILES=()

remember_dds_files() {
  local pid fd target
  for pid in "$@"; do
    for fd in "/proc/${pid}/fd/"*; do
      [[ -e "${fd}" ]] || continue
      target="$(readlink "${fd}" 2>/dev/null || true)"
      target="${target% (deleted)}"
      if [[ "${target}" == /dev/shm/fastrtps_port* ]]; then
        REBOTARM_OLD_DDS_FILES["${target}"]=1
      fi
    done
  done
}

show_processes() {
  local pid
  for pid in "$@"; do
    ps -o pid=,ppid=,stat=,args= -p "${pid}" >&2 || true
  done
}

signal_processes() {
  local signal="$1"
  shift
  local pid
  for pid in "$@"; do
    kill "-${signal}" "${pid}" 2>/dev/null || true
  done
}

wait_for_processes() {
  local timeout="$1"
  shift
  local deadline=$((SECONDS + timeout)) pid alive
  while ((SECONDS < deadline)); do
    alive=0
    for pid in "$@"; do
      if kill -0 "${pid}" 2>/dev/null; then
        alive=1
        break
      fi
    done
    ((alive == 0)) && return 0
    sleep 0.2
  done
  return 1
}

stop_processes() {
  local reason="$1"
  shift
  (($#)) || return 0

  echo "Replacing ${reason}:" >&2
  show_processes "$@"
  remember_dds_files "$@"

  # A job stopped with Ctrl+Z must be continued before it can handle SIGINT.
  signal_processes CONT "$@"
  signal_processes INT "$@"
  if wait_for_processes "${REBOTARM_REPLACE_TIMEOUT}" "$@"; then
    return 0
  fi

  echo "Graceful shutdown timed out; sending SIGTERM." >&2
  signal_processes TERM "$@"
  if wait_for_processes 5 "$@"; then
    return 0
  fi

  echo "Controller is still stuck; sending SIGKILL." >&2
  signal_processes KILL "$@"
  wait_for_processes 3 "$@"
}

quarantine_old_dds_files() {
  local path quarantine="" moved=0
  for path in "${!REBOTARM_OLD_DDS_FILES[@]}"; do
    [[ -e "${path}" ]] || continue
    if fuser "${path}" >/dev/null 2>&1; then
      continue
    fi
    if [[ -z "${quarantine}" ]]; then
      quarantine="$(mktemp -d /tmp/rebot-fastrtps-quarantine.XXXXXX)"
    fi
    mv -n -- "${path}" "${quarantine}/"
    moved=1
  done
  if ((moved)); then
    echo "Moved stale Fast DDS files to ${quarantine}" >&2
  fi
}

clean_stale_dds_files() {
  if command -v fastdds >/dev/null 2>&1; then
    # Fast DDS identifies zombie resources internally and leaves live
    # participants untouched.
    fastdds shm clean >&2 || true
  else
    quarantine_old_dds_files
  fi
}

if [[ "${REBOTARM_RS_HARDWARE_CONFIRM:-}" != "I_UNDERSTAND_RS_WILL_MOVE" ]]; then
  echo "Hardware launch blocked." >&2
  echo "Confirm the work area is clear and a physical emergency stop is available." >&2
  echo "Then set REBOTARM_RS_HARDWARE_CONFIRM=I_UNDERSTAND_RS_WILL_MOVE." >&2
  exit 2
fi

exec 9>"${REBOTARM_HARDWARE_LOCK}"
if ! flock -n 9; then
  mapfile -t REBOTARM_LOCK_HOLDERS < <(
    fuser "${REBOTARM_HARDWARE_LOCK}" 2>/dev/null \
      | awk -v self="$$" \
          '{for (i = 1; i <= NF; i++) if ($i != self) print $i}'
  )
  if ((${#REBOTARM_LOCK_HOLDERS[@]} == 0)); then
    echo "Hardware lock is busy but its owner cannot be identified." >&2
    exit 1
  fi
  stop_processes "the previous RS hardware launch" "${REBOTARM_LOCK_HOLDERS[@]}"
  if ! flock -w 5 9; then
    echo "Previous launch did not release ${REBOTARM_HARDWARE_LOCK}." >&2
    exit 1
  fi
  clean_stale_dds_files
fi

# A controller started outside this script will not own the lock. Detect it
# before opening SocketCAN so duplicate launches cannot compete for motors or
# Fast DDS shared-memory ports.
REBOTARM_EXISTING_CONTROLLERS=()
for cmdline_path in /proc/[0-9]*/cmdline; do
  [[ -r "${cmdline_path}" ]] || continue
  cmdline="$(tr '\0' ' ' < "${cmdline_path}" 2>/dev/null || true)"
  if [[ "${cmdline}" == *"/rebotarmcontroller/reBotArmController"* ]]; then
    REBOTARM_EXISTING_CONTROLLERS+=("${cmdline_path#/proc/}")
  fi
done
if ((${#REBOTARM_EXISTING_CONTROLLERS[@]})); then
  REBOTARM_CONTROLLER_PIDS=()
  for pid_entry in "${REBOTARM_EXISTING_CONTROLLERS[@]}"; do
    pid="${pid_entry%/cmdline}"
    REBOTARM_CONTROLLER_PIDS+=("${pid}")
  done
  stop_processes \
    "a reBotArm controller started outside this script" \
    "${REBOTARM_CONTROLLER_PIDS[@]}"
  clean_stale_dds_files
fi
unset REBOTARM_EXISTING_CONTROLLERS REBOTARM_CONTROLLER_PIDS

# Fast DDS may leave its port files behind after a suspended controller is
# force-stopped. Its cleanup command removes zombie resources only.
clean_stale_dds_files

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

if ! ip link show can0 >/dev/null 2>&1; then
  echo "SocketCAN interface can0 does not exist." >&2
  exit 1
fi

ip -details link show can0
exec ros2 launch rebotarm_bringup bringup.launch.py \
  model:=rs \
  channel:=can0 \
  arm_namespace:=rebot_arm_rs \
  ee_frame_id:=gripper_end \
  joint_state_rate:="${REBOTARM_JOINT_STATE_RATE:-60.0}" \
  hardware_feedback_poll_rate:="${REBOTARM_HARDWARE_FEEDBACK_POLL_RATE:-20.0}" \
  use_rviz:="${REBOTARM_USE_RVIZ:-false}"
