#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/demo_common.sh"
INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-4}"
BASE_PORT="${3:-8899}"
TAG="semantic_net_demo_start"
DRY_RUN="${DEMO_DRY_RUN:-0}"

PACKET_DROP_RATE="${4:-0.20}"
DROP_MODE="${5:-weight}"
THROTTLE_MODE="${6:-weight}"
DELAY_BASE_MS="${7:-0}"
DELAY_SCALE_MS="${8:-0}"
SEED="${9:-0}"

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

VIS_PORT="$(pick_available_port "${BASE_PORT}" 50)"
if [[ "${VIS_PORT}" == "-1" ]]; then
  log_error "${TAG}" "E_PORT_FULL" "从端口 ${BASE_PORT} 起未找到可用端口"
  exit 4
fi

if [[ "${DRY_RUN}" == "1" ]]; then
  log_info "${TAG}" "DRY_RUN=1，跳过 ROS/PX4 启动"
  log_info "${TAG}" "open http://127.0.0.1:${VIS_PORT}"
  log_info "${TAG}" "started_semantic_demo=(dry-run)"
  exit 0
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  log_error "${TAG}" "E_ROS_ENV" "缺少 /opt/ros/humble/setup.bash"
  exit 5
fi
source /opt/ros/humble/setup.bash || {
  log_error "${TAG}" "E_ROS_SOURCE" "加载 ROS 环境失败"
  exit 6
}
source "${ROOT_DIR}/ros2_ws/install/setup.bash" || {
  log_error "${TAG}" "E_WS_SOURCE" "加载工作区环境失败，请先构建 ros2_ws"
  exit 7
}
if ! command -v ros2 >/dev/null 2>&1; then
  log_error "${TAG}" "E_ROS2_CMD" "未找到 ros2 命令"
  exit 8
fi

if [[ "${TOTAL_CORES}" -lt 1 ]]; then
  TOTAL_CORES=1
fi

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:=20 \
  -p output_topic:=/swarm/state_raw \
  -p command_template:="sleep 1000" \
  > /tmp/swarm_manager_semantic.log 2>&1 &
MANAGER_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:=/swarm/state_raw \
  -p output_topic:=/swarm/state \
  > /tmp/swarm_topology_semantic.log 2>&1 &
TOPO_PID=$!

ros2 run swarm_topology_analyzer swarm_semantic_net_node --ros-args \
  -p input_topic:=/swarm/state \
  -p output_topic:=/swarm/state_semantic \
  -p packet_drop_rate:="${PACKET_DROP_RATE}" \
  -p drop_mode:="${DROP_MODE}" \
  -p throttle_mode:="${THROTTLE_MODE}" \
  -p delay_base_ms:="${DELAY_BASE_MS}" \
  -p delay_weight_scale_ms:="${DELAY_SCALE_MS}" \
  -p seed:="${SEED}" \
  > /tmp/swarm_semantic_net.log 2>&1 &
SEM_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" SWARM_STATE_TOPIC="/swarm/state_semantic" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_semantic.log 2>&1 &
VIS_PID=$!

echo "${MANAGER_PID} ${TOPO_PID} ${SEM_PID} ${VIS_PID}" > "${ROOT_DIR}/.semantic_net_demo_pids"
echo "${VIS_PORT}" > "${ROOT_DIR}/.semantic_net_demo_port"
log_info "${TAG}" "started manager_pid=${MANAGER_PID} topo_pid=${TOPO_PID} semantic_pid=${SEM_PID} vis_pid=${VIS_PID}"
log_info "${TAG}" "open http://127.0.0.1:${VIS_PORT}"
log_info "${TAG}" "semantic params packet_drop_rate=${PACKET_DROP_RATE} drop_mode=${DROP_MODE} throttle_mode=${THROTTLE_MODE} delay_base_ms=${DELAY_BASE_MS} delay_scale_ms=${DELAY_SCALE_MS} seed=${SEED}"
