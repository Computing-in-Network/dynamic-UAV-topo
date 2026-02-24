#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="dual_view_demo_start"
PIDS_FILE="${ROOT_DIR}/.dual_view_demo_pids"
source "${ROOT_DIR}/scripts/demo_common.sh"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-4}"
BASE_PORT="${3:-8899}"
GAZEBO_MODE="${4:-mock}"
GAZEBO_CMD="${GAZEBO_CMD:-}"

"${ROOT_DIR}/scripts/dual_view_demo_stop.sh" >/dev/null 2>&1 || true

"${ROOT_DIR}/scripts/visual_demo_start.sh" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${BASE_PORT}"
VIS_PORT="$(cat "${ROOT_DIR}/.visual_demo_port" 2>/dev/null || echo "${BASE_PORT}")"

GAZEBO_PID=""
if [[ "${GAZEBO_MODE}" == "gazebo" ]]; then
  if [[ -z "${GAZEBO_CMD}" ]]; then
    log_error "${TAG}" "E_GAZEBO_CMD" "GAZEBO_MODE=gazebo 但未提供 GAZEBO_CMD"
    "${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
    exit 21
  fi
  bash -lc "${GAZEBO_CMD}" > /tmp/dual_view_gazebo.log 2>&1 &
  GAZEBO_PID=$!
else
  log_warn "${TAG}" "GAZEBO_MODE=${GAZEBO_MODE}，仅启动前端全景视图（特写窗口占位）"
fi

echo "${GAZEBO_PID}" > "${PIDS_FILE}"
log_info "${TAG}" "started visual_port=${VIS_PORT} gazebo_mode=${GAZEBO_MODE} gazebo_pid=${GAZEBO_PID:-none}"
log_info "${TAG}" "全景视图: http://127.0.0.1:${VIS_PORT}/cesium"
