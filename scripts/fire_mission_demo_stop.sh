#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"
PORT_FILE="${ROOT_DIR}/.fire_demo_port"
VIS_PORT="${1:-}"

kill_pattern() {
  local pattern="$1"
  pkill -f "${pattern}" >/dev/null 2>&1 || true
  sleep 0.2
  pkill -9 -f "${pattern}" >/dev/null 2>&1 || true
}

kill_pid_graceful() {
  local pid="$1"
  kill "${pid}" >/dev/null 2>&1 || true
  for _ in $(seq 1 10); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return
    fi
    sleep 0.1
  done
  kill -9 "${pid}" >/dev/null 2>&1 || true
}

kill_port_listener() {
  local port="$1"
  if [[ -z "${port}" ]]; then
    return
  fi
  local pids=""

  if command -v lsof >/dev/null 2>&1; then
    pids="$(lsof -tiTCP:${port} -sTCP:LISTEN 2>/dev/null | sort -u || true)"
  fi

  if [[ -z "${pids}" ]] && command -v fuser >/dev/null 2>&1; then
    pids="$(fuser -n tcp "${port}" 2>/dev/null | tr ' ' '\n' | sed '/^$/d' | sort -u || true)"
  fi

  if [[ -z "${pids}" ]]; then
    pids="$(ss -ltnp 2>/dev/null | sed -n "s/.*:${port}[[:space:]].*pid=\([0-9]\+\).*/\1/p" | sort -u || true)"
  fi

  if [[ -n "${pids}" ]]; then
    for p in ${pids}; do
      kill_pid_graceful "${p}"
    done
  fi
}

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
echo "[fire_mission_demo_stop] stopped"
