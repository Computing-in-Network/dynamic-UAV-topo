#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.visual_demo_pids"
PORT_FILE="${ROOT_DIR}/.visual_demo_port"
TAG="visual_demo_start"
source "${ROOT_DIR}/scripts/demo_common.sh"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-$(nproc)}"
BASE_PORT="${3:-8899}"
DRY_RUN="${DEMO_DRY_RUN:-0}"

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

VIS_PORT="$(pick_available_port "${BASE_PORT}" 50)"
if [[ "${VIS_PORT}" == "-1" ]]; then
  log_error "${TAG}" "E_PORT_FULL" "从端口 ${BASE_PORT} 起未找到可用端口"
  exit 4
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  log_info "${TAG}" "DRY_RUN=1，跳过 ROS/PX4 启动"
  log_info "${TAG}" "selected_port=${VIS_PORT}"
  exit 0
fi

set +u
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  set -u
  log_error "${TAG}" "E_ROS_ENV" "缺少 /opt/ros/humble/setup.bash"
  exit 5
fi
source /opt/ros/humble/setup.bash || {
  set -u
  log_error "${TAG}" "E_ROS_SOURCE" "加载 ROS 环境失败"
  exit 6
}
source "${ROOT_DIR}/ros2_ws/install/setup.bash" || {
  set -u
  log_error "${TAG}" "E_WS_SOURCE" "加载工作区环境失败，请先构建 ros2_ws"
  exit 7
}
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  log_error "${TAG}" "E_ROS2_CMD" "未找到 ros2 命令"
  exit 8
fi

if [[ ! -x "${ROOT_DIR}/third_party/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]]; then
  log_warn "${TAG}" "PX4 SITL 未构建，开始自动构建"
  (cd "${ROOT_DIR}/third_party/PX4-Autopilot" && CCACHE_DISABLE=1 make px4_sitl -j"$(nproc)")
fi

PX4_CMD="cd ${ROOT_DIR}/third_party/PX4-Autopilot && PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i {index} -d ./build/px4_sitl_default/etc"

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:=5 \
  -p healthcheck_hz:=1 \
  -p demo_motion:=true \
  -p output_topic:=/swarm/state_raw \
  -p command_template:="${PX4_CMD}" \
  > /tmp/swarm_manager.log 2>&1 &
MANAGER_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:=/swarm/state_raw \
  -p output_topic:=/swarm/state \
  > /tmp/swarm_topology.log 2>&1 &
TOPO_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_server.log 2>&1 &
SERVER_PID=$!

echo "${MANAGER_PID} ${TOPO_PID} ${SERVER_PID}" > "${PIDS_FILE}"
echo "${VIS_PORT}" > "${PORT_FILE}"
log_info "${TAG}" "manager_pid=${MANAGER_PID} topo_pid=${TOPO_PID} server_pid=${SERVER_PID}"
if [[ "${VIS_PORT}" != "${BASE_PORT}" ]]; then
  log_warn "${TAG}" "端口 ${BASE_PORT} 被占用，已自动回退到 ${VIS_PORT}"
fi
log_info "${TAG}" "打开浏览器: http://127.0.0.1:${VIS_PORT}"
