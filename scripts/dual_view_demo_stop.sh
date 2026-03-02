#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="dual_view_demo_stop"
PIDS_FILE="${ROOT_DIR}/.dual_view_demo_pids"
source "${ROOT_DIR}/scripts/demo_common.sh"

if [[ -f "${PIDS_FILE}" ]]; then
  read -r GAZEBO_PID FOCUS_PID _ _ _ < "${PIDS_FILE}" || true
  kill_pid_graceful "${GAZEBO_PID:-}"
  kill_pid_graceful "${FOCUS_PID:-}"
  rm -f "${PIDS_FILE}"
fi

kill_pattern "dual_view_focus_mock_server.py"
kill_pattern "gz sim"
kill_pattern "gazebo --verbose"
kill_pattern "ign gazebo"
kill_pattern "px4_sitl_default/bin/px4"

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
log_info "${TAG}" "stopped"
