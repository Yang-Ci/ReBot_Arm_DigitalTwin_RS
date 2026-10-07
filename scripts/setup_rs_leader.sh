#!/usr/bin/env bash
set -euo pipefail
REBOTARM_TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${REBOTARM_TASK_ROOT}/scripts/rs_env.sh"
python -m pip install -r "${REBOTARM_TASK_ROOT}/requirements-rs-leader.txt"
cd "${REBOTARM_TASK_ROOT}/rebotarm_ros2_RS"
colcon build --symlink-install --packages-select rebotarm_msgs rebotarmcontroller
echo 'Leader interfaces built. Restart RS/Fake driver and rosbridge; reload the web console.'
