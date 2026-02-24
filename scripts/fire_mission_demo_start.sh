#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"
PORT_FILE="${ROOT_DIR}/.fire_demo_port"
TAG="fire_mission_demo_start"
source "${ROOT_DIR}/scripts/demo_common.sh"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-4}"
BASE_PORT="${3:-8899}"
FIRE_SOURCE_MODE="${4:-demo}"
FDS_INPUT_PATH="${5:-${ROOT_DIR}/docs/examples/fds_hotspots_sample.csv}"
FDS_INPUT_FORMAT="${6:-csv}"
DRY_RUN="${DEMO_DRY_RUN:-0}"

if [[ "${FIRE_SOURCE_MODE}" != "demo" && "${FIRE_SOURCE_MODE}" != "fds" ]]; then
  log_error "${TAG}" "E_USAGE" "不支持的 FIRE_SOURCE_MODE=${FIRE_SOURCE_MODE}，可选: demo|fds"
  exit 2
fi

if [[ "${FIRE_SOURCE_MODE}" == "fds" && ! -f "${FDS_INPUT_PATH}" ]]; then
  log_error "${TAG}" "E_INPUT" "FDS 输入文件不存在: ${FDS_INPUT_PATH}"
  exit 3
fi

VIS_PORT="$(pick_available_port "${BASE_PORT}" 50)"
if [[ "${VIS_PORT}" == "-1" ]]; then
  log_error "${TAG}" "E_PORT_FULL" "从端口 ${BASE_PORT} 起未找到可用端口"
  exit 4
fi

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" "${VIS_PORT}" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

if [[ "${DRY_RUN}" == "1" ]]; then
  log_info "${TAG}" "DRY_RUN=1，跳过 ROS 进程启动"
  log_info "${TAG}" "selected_port=${VIS_PORT} fire_source=${FIRE_SOURCE_MODE}"
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

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:=5 \
  -p healthcheck_hz:=1 \
  -p output_topic:=/swarm/state_raw \
  -p mission_follow_enabled:=true \
  -p mission_topic:=/swarm/mission_targets \
  -p command_template:="sleep 1000" \
  > /tmp/swarm_manager_fire_demo.log 2>&1 &
MANAGER_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:=/swarm/state_raw \
  -p output_topic:=/swarm/state \
  > /tmp/swarm_topology_fire_demo.log 2>&1 &
TOPO_PID=$!

if [[ "${FIRE_SOURCE_MODE}" == "demo" ]]; then
  python3 "${ROOT_DIR}/scripts/fire_adapter_demo.py" \
    --ros-args -p output_topic:=/env/fire_state -p publish_hz:=1.0 \
    > /tmp/fire_adapter_demo.log 2>&1 &
  FIRE_PID=$!
elif [[ "${FIRE_SOURCE_MODE}" == "fds" ]]; then
  python3 "${ROOT_DIR}/scripts/fire_adapter_fds.py" \
    --ros-args \
    -p output_topic:=/env/fire_state \
    -p publish_hz:=2.0 \
    -p input_path:="${FDS_INPUT_PATH}" \
    -p input_format:="${FDS_INPUT_FORMAT}" \
    -p time_mode:=source_offset \
    > /tmp/fire_adapter_fds.log 2>&1 &
  FIRE_PID=$!
fi

python3 "${ROOT_DIR}/scripts/mission_planner.py" \
  --ros-args -p fire_topic:=/env/fire_state -p swarm_topic:=/swarm/state -p plan_topic:=/swarm/mission_targets \
  > /tmp/mission_planner.log 2>&1 &
PLANNER_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_server_fire_demo.log 2>&1 &
VIS_PID=$!

echo "${MANAGER_PID} ${TOPO_PID} ${FIRE_PID} ${PLANNER_PID} ${VIS_PID}" > "${PIDS_FILE}"
echo "${VIS_PORT}" > "${PORT_FILE}"
log_info "${TAG}" "manager=${MANAGER_PID} topo=${TOPO_PID} fire=${FIRE_PID} planner=${PLANNER_PID} vis=${VIS_PID} fire_source=${FIRE_SOURCE_MODE}"
if [[ "${VIS_PORT}" != "${BASE_PORT}" ]]; then
  log_warn "${TAG}" "端口 ${BASE_PORT} 被占用，已自动回退到 ${VIS_PORT}"
fi
log_info "${TAG}" "可视化: http://127.0.0.1:${VIS_PORT}/cesium"
