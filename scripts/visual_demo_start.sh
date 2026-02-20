#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.visual_demo_pids"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-$(nproc)}"
VIS_PORT="${3:-8899}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

if [[ ! -x "${ROOT_DIR}/third_party/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]]; then
  echo "[visual_demo_start] PX4 SITL not built, building..."
  (cd "${ROOT_DIR}/third_party/PX4-Autopilot" && CCACHE_DISABLE=1 make px4_sitl -j"$(nproc)")
fi

PX4_CMD="cd ${ROOT_DIR}/third_party/PX4-Autopilot && PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i {index} -d ./build/px4_sitl_default/etc"

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:=5 \
  -p healthcheck_hz:=1 \
  -p demo_motion:=true \
  -p command_template:="${PX4_CMD}" \
  > /tmp/swarm_manager.log 2>&1 &
MANAGER_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_server.log 2>&1 &
SERVER_PID=$!

echo "${MANAGER_PID} ${SERVER_PID}" > "${PIDS_FILE}"
echo "[visual_demo_start] manager_pid=${MANAGER_PID} server_pid=${SERVER_PID}"
echo "[visual_demo_start] 打开浏览器: http://127.0.0.1:${VIS_PORT}"
