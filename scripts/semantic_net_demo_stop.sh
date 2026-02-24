#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/demo_common.sh"
VIS_PORT="${1:-}"
PIDS_FILE="${ROOT_DIR}/.semantic_net_demo_pids"
PORT_FILE="${ROOT_DIR}/.semantic_net_demo_port"
TAG="semantic_net_demo_stop"

if [[ ! -f "${PIDS_FILE}" ]]; then
  kill_pattern "swarm_semantic_net_node"
  log_info "${TAG}" "no pid file, cleaned by process name"
  exit 0
fi

read -r MANAGER_PID TOPO_PID SEM_PID VIS_PID < "${PIDS_FILE}" || true

kill_pid_graceful "${MANAGER_PID:-}"
kill_pid_graceful "${TOPO_PID:-}"
kill_pid_graceful "${SEM_PID:-}"
kill_pid_graceful "${VIS_PID:-}"

kill_pattern "swarm_semantic_net_node"
kill_pattern "swarm_topology_analyzer_node"
kill_pattern "swarm_uav_manager_node"
kill_pattern "ros2_visualization_server.py"
if [[ -f "${PORT_FILE}" ]]; then
  kill_port_listener "$(cat "${PORT_FILE}" 2>/dev/null || true)"
fi
if [[ -n "${VIS_PORT:-}" ]]; then
  kill_port_listener "${VIS_PORT}"
fi

rm -f "${PIDS_FILE}"
rm -f "${PORT_FILE}"
log_info "${TAG}" "stopped"
