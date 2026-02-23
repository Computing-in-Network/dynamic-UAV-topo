#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS_FILE="${ROOT_DIR}/.fire_demo_pids"
PORT_FILE="${ROOT_DIR}/.fire_demo_port"

INSTANCE_COUNT="${1:-6}"
TOTAL_CORES="${2:-6}"
VIS_PORT="${3:-8913}"
PLANNER_MODE="${4:-coverage}"
INPUT_CSV="${5:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc.csv}"
MAPPING_JSON="${6:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc_mapping.json}"
PUBLISH_HZ="${7:-2.0}"

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" "${VIS_PORT}" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

if ! python3 - <<PY
import socket,sys,time
port=${VIS_PORT}
deadline=time.time()+4.0
while time.time()<deadline:
    s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", port))
        sys.exit(0)
    except OSError:
        time.sleep(0.2)
    finally:
        s.close()
sys.exit(1)
PY
then
  echo "[fire_mission_fds_start] 端口 ${VIS_PORT} 已被占用，请换一个端口（例如 $((VIS_PORT+1))）"
  exit 1
fi

if [[ ! -f "${INPUT_CSV}" ]]; then
  echo "[fire_mission_fds_start] 输入 CSV 不存在: ${INPUT_CSV}"
  echo "[fire_mission_fds_start] 先执行: ./scripts/fds_run_min_case.sh"
  exit 2
fi
if [[ ! -f "${MAPPING_JSON}" ]]; then
  echo "[fire_mission_fds_start] 映射文件不存在: ${MAPPING_JSON}"
  exit 3
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

python3 "${ROOT_DIR}/scripts/fire_adapter_fds.py" --ros-args \
  -p input_path:="${INPUT_CSV}" \
  -p input_format:=fds_devc_csv \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p output_topic:=/env/fire_state \
  -p mode:=offline \
  -p fds_devc_mapping:="${MAPPING_JSON}" \
  > /tmp/fire_adapter_fds_real.log 2>&1 &
FIRE_PID=$!

python3 "${ROOT_DIR}/scripts/mission_planner.py" \
  --ros-args -p fire_topic:=/env/fire_state -p swarm_topic:=/swarm/state -p plan_topic:=/swarm/mission_targets \
  -p planner_mode:="${PLANNER_MODE}" -p coverage_revisit_sec:=12.0 \
  > /tmp/mission_planner.log 2>&1 &
PLANNER_PID=$!

python3 "${ROOT_DIR}/scripts/mission_status_tracker.py" \
  --ros-args -p plan_topic:=/swarm/mission_targets -p swarm_topic:=/swarm/state -p status_topic:=/swarm/mission_status \
  > /tmp/mission_status_tracker.log 2>&1 &
MISSION_STATUS_PID=$!

SWARM_VIS_PORT="${VIS_PORT}" LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/local/lib:${LD_LIBRARY_PATH:-}" \
  python3 "${ROOT_DIR}/scripts/ros2_visualization_server.py" > /tmp/swarm_visual_server_fire_demo.log 2>&1 &
VIS_PID=$!

echo "${MANAGER_PID} ${TOPO_PID} ${FIRE_PID} ${PLANNER_PID} ${MISSION_STATUS_PID} ${VIS_PID}" > "${PIDS_FILE}"
echo "${VIS_PORT}" > "${PORT_FILE}"
echo "[fire_mission_fds_start] manager=${MANAGER_PID} topo=${TOPO_PID} fire_fds=${FIRE_PID} planner=${PLANNER_PID} mission_status=${MISSION_STATUS_PID} vis=${VIS_PID}"
echo "[fire_mission_fds_start] planner_mode=${PLANNER_MODE} input=${INPUT_CSV}"
echo "[fire_mission_fds_start] 可视化: http://127.0.0.1:${VIS_PORT}/cesium"
