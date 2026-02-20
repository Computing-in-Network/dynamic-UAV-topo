#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"

if [[ -f "${PIDS_FILE}" ]]; then
  read -r MANAGER_PID TOPO_PID FIRE_PID PLANNER_PID VIS_PID < "${PIDS_FILE}" || true
  for p in "${MANAGER_PID:-}" "${TOPO_PID:-}" "${FIRE_PID:-}" "${PLANNER_PID:-}" "${VIS_PID:-}"; do
    if [[ -n "${p}" ]]; then
      kill "${p}" >/dev/null 2>&1 || true
    fi
  done
  rm -f "${PIDS_FILE}"
fi

pkill -f "fire_adapter_demo.py" >/dev/null 2>&1 || true
pkill -f "mission_planner.py" >/dev/null 2>&1 || true
pkill -f "swarm_topology_analyzer_node" >/dev/null 2>&1 || true
pkill -f "swarm_uav_manager_node" >/dev/null 2>&1 || true
pkill -f "ros2_visualization_server.py" >/dev/null 2>&1 || true
echo "[fire_mission_demo_stop] stopped"
