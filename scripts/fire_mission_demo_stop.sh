#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"

kill_pattern() {
  local pattern="$1"
  pkill -f "${pattern}" >/dev/null 2>&1 || true
  sleep 0.2
  pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
}

if [[ -f "${PIDS_FILE}" ]]; then
  read -r MANAGER_PID TOPO_PID FIRE_PID PLANNER_PID VIS_PID < "${PIDS_FILE}" || true
  for p in "${MANAGER_PID:-}" "${TOPO_PID:-}" "${FIRE_PID:-}" "${PLANNER_PID:-}" "${VIS_PID:-}"; do
    if [[ -n "${p}" ]]; then
      kill "${p}" >/dev/null 2>&1 || true
    fi
  done
  rm -f "${PIDS_FILE}"
fi

kill_pattern "fire_adapter_demo.py"
kill_pattern "mission_planner.py"
kill_pattern "swarm_topology_analyzer_node"
kill_pattern "swarm_uav_manager_node"
kill_pattern "ros2_visualization_server.py"
echo "[fire_mission_demo_stop] stopped"
