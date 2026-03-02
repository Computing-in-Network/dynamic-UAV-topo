#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"
PORT_FILE="${ROOT_DIR}/.fire_demo_port"
TAG="fire_mission_demo_stop"
source "${ROOT_DIR}/scripts/demo_common.sh"
VIS_PORT="${1:-}"

if [[ -z "${VIS_PORT}" && -f "${PORT_FILE}" ]]; then
  VIS_PORT="$(cat "${PORT_FILE}" 2>/dev/null || true)"
fi

if [[ -f "${PIDS_FILE}" ]]; then
  read -r MANAGER_PID TOPO_PID FIRE_PID PLANNER_PID VIS_PID < "${PIDS_FILE}" || true
  for p in "${MANAGER_PID:-}" "${TOPO_PID:-}" "${FIRE_PID:-}" "${PLANNER_PID:-}" "${VIS_PID:-}"; do
    if [[ -n "${p}" ]]; then
      kill_pid_graceful "${p}"
    fi
  done
  rm -f "${PIDS_FILE}"
fi

kill_pattern "fire_adapter_demo.py"
kill_pattern "fire_adapter_fds.py"
kill_pattern "mission_planner.py"
kill_pattern "swarm_topology_analyzer_node"
kill_pattern "swarm_uav_manager_node"
kill_pattern "ros2_visualization_server.py"
kill_port_listener "${VIS_PORT}"
rm -f "${PORT_FILE}"
log_info "${TAG}" "stopped"
