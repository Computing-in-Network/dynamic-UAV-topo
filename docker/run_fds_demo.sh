#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/workspace/dynamic-uav-topo}"
INSTANCE_COUNT="${INSTANCE_COUNT:-6}"
TOTAL_CORES="${TOTAL_CORES:-6}"
VIS_PORT="${VIS_PORT:-8914}"
PLANNER_MODE="${PLANNER_MODE:-coverage}"
INPUT_CSV="${INPUT_CSV:-${REPO_DIR}/data/fds_cases/min_fire/fds_min_fire_devc.csv}"
MAPPING_JSON="${MAPPING_JSON:-${REPO_DIR}/data/fds_cases/min_fire/fds_min_fire_devc_mapping.json}"
PUBLISH_HZ="${PUBLISH_HZ:-4.0}"
TOPO_ENABLE_PROFILE="${TOPO_ENABLE_PROFILE:-true}"
TOPO_PROFILE_EVERY_N="${TOPO_PROFILE_EVERY_N:-20}"
TOPO_PROFILE_TARGET_MS="${TOPO_PROFILE_TARGET_MS:-20.0}"

if [[ ! -d "${REPO_DIR}" ]]; then
  echo "[docker-runner] repo not found: ${REPO_DIR}"
  exit 2
fi

cd "${REPO_DIR}"

cleanup() {
  echo "[docker-runner] stopping mission demo on port ${VIS_PORT}..."
  ./scripts/fire_mission_demo_stop.sh "${VIS_PORT}" >/tmp/docker_stop.log 2>&1 || true
  exit 0
}
trap cleanup SIGINT SIGTERM

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${REPO_DIR}/ros2_ws/install/setup.bash" ]]; then
  source "${REPO_DIR}/ros2_ws/install/setup.bash"
fi
set -u

echo "[docker-runner] starting fire mission fds demo..."
./scripts/fire_mission_fds_start.sh \
  "${INSTANCE_COUNT}" \
  "${TOTAL_CORES}" \
  "${VIS_PORT}" \
  "${PLANNER_MODE}" \
  "${INPUT_CSV}" \
  "${MAPPING_JSON}" \
  "${PUBLISH_HZ}" \
  "${TOPO_ENABLE_PROFILE}" \
  "${TOPO_PROFILE_EVERY_N}" \
  "${TOPO_PROFILE_TARGET_MS}"

echo "[docker-runner] running. visit: http://127.0.0.1:${VIS_PORT}/cesium"
while true; do
  sleep 3600
done
