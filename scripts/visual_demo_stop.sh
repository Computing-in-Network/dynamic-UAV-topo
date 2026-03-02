#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.visual_demo_pids"
PORT_FILE="${ROOT_DIR}/.visual_demo_port"
TAG="visual_demo_stop"
source "${ROOT_DIR}/scripts/demo_common.sh"

if [[ ! -f "${PIDS_FILE}" ]]; then
  kill_pattern "ros2_visualization_server.py"
  kill_pattern "swarm_topology_analyzer_node"
  kill_pattern "swarm_uav_manager_node"
  kill_pattern "px4_sitl_default/bin/px4"
  if [[ -f "${PORT_FILE}" ]]; then
    kill_port_listener "$(cat "${PORT_FILE}" 2>/dev/null || true)"
  fi
  rm -f "${PORT_FILE}"
  log_info "${TAG}" "no pid file, cleaned by process name"
  exit 0
fi

read -r MANAGER_PID TOPO_PID SERVER_PID < "${PIDS_FILE}" || true

kill_pid_graceful "${MANAGER_PID:-}"
kill_pid_graceful "${TOPO_PID:-}"
kill_pid_graceful "${SERVER_PID:-}"

kill_pattern "px4_sitl_default/bin/px4"
if [[ -f "${PORT_FILE}" ]]; then
  kill_port_listener "$(cat "${PORT_FILE}" 2>/dev/null || true)"
fi

rm -f "${PIDS_FILE}"
rm -f "${PORT_FILE}"
log_info "${TAG}" "stopped"
