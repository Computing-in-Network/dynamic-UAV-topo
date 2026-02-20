#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.visual_demo_pids"

if [[ ! -f "${PIDS_FILE}" ]]; then
  pkill -f "ros2_visualization_server.py" 2>/dev/null || true
  pkill -f "swarm_topology_analyzer_node" 2>/dev/null || true
  pkill -f "swarm_uav_manager_node" 2>/dev/null || true
  pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
  echo "[visual_demo_stop] no pid file, cleaned by process name"
  exit 0
fi

read -r MANAGER_PID TOPO_PID SERVER_PID < "${PIDS_FILE}" || true

if [[ -n "${MANAGER_PID:-}" ]]; then
  kill "${MANAGER_PID}" 2>/dev/null || true
fi
if [[ -n "${TOPO_PID:-}" ]]; then
  kill "${TOPO_PID}" 2>/dev/null || true
fi
if [[ -n "${SERVER_PID:-}" ]]; then
  kill "${SERVER_PID}" 2>/dev/null || true
fi

pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true

rm -f "${PIDS_FILE}"
echo "[visual_demo_stop] stopped"
