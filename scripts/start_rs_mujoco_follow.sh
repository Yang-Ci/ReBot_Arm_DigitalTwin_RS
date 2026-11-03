#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

REBOTARM_MIRROR_NAMESPACE="${REBOTARM_MIRROR_NAMESPACE:-rebot_arm_rs}"

if ! ros2 topic info "/${REBOTARM_MIRROR_NAMESPACE}/joint_states" >/dev/null 2>&1; then
  echo "Warning: /${REBOTARM_MIRROR_NAMESPACE}/joint_states is not visible yet." >&2
  echo "Start the real RS controller first; MuJoCo will begin following once it publishes." >&2
fi

# The MuJoCo wrapper only subscribes to the real JointState topic and never
# sends torque commands or opens SocketCAN, so no hardware-confirm flag is
# required for this display-only follower.
exec ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:="${REBOTARM_MIRROR_NAMESPACE}" \
  simulation_mode:=kinematic \
  use_viewer:=true \
  enable_task_tools:=false
