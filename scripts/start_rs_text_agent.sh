#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"

export REBOTARM_NAMESPACE="${REBOTARM_NAMESPACE:-rebot_arm_rs_sim}"
export REBOTARM_MCP_URL="${REBOTARM_MCP_URL:-http://127.0.0.1:8081/mcp}"

REBOTARM_AGENT_HOST="${REBOTARM_AGENT_HTTP_HOST:-0.0.0.0}"
REBOTARM_AGENT_PORT="${REBOTARM_AGENT_HTTP_PORT:-8082}"

echo "RS text agent: ${REBOTARM_MCP_URL}"
echo "HTTP dashboard: http://${REBOTARM_AGENT_HOST}:${REBOTARM_AGENT_PORT}"

exec python3 -m rebotarm_agent.rebotarm_text_agent \
  --http-server \
  --http-host "${REBOTARM_AGENT_HOST}" \
  --http-port "${REBOTARM_AGENT_PORT}" \
  --yes \
  "$@"
