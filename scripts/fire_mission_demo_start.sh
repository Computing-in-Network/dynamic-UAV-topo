#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-4}"
VIS_PORT="${3:-8899}"

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
sleep 0.3

if ! python3 - <<PY
import socket,sys
port=${VIS_PORT}
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.bind(("127.0.0.1", port))
except OSError:
    sys.exit(1)
finally:
    s.close()
PY
then
  local_next_port="$((VIS_PORT + 1))"
  echo "[fire_mission_demo_start] 端口 ${VIS_PORT} 已被占用，请换一个端口重试（例如 ${local_next_port}）"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

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

python3 "${ROOT_DIR}/scripts/fire_adapter_demo.py" \
  --ros-args -p output_topic:=/env/fire_state -p publish_hz:=1.0 \
  > /tmp/fire_adapter_demo.log 2>&1 &
FIRE_PID=$!

python3 "${ROOT_DIR}/scripts/mission_planner.py" \
  --ros-args -p fire_topic:=/env/fire_state -p swarm_topic:=/swarm/state -p plan_topic:=/swarm/mission_targets \
  > /tmp/mission_planner.log 2>&1 &
PLANNER_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_server_fire_demo.log 2>&1 &
VIS_PID=$!

echo "${MANAGER_PID} ${TOPO_PID} ${FIRE_PID} ${PLANNER_PID} ${VIS_PID}" > "${PIDS_FILE}"
echo "[fire_mission_demo_start] manager=${MANAGER_PID} topo=${TOPO_PID} fire=${FIRE_PID} planner=${PLANNER_PID} vis=${VIS_PID}"
echo "[fire_mission_demo_start] 可视化: http://127.0.0.1:${VIS_PORT}/cesium"
