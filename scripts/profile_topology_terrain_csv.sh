#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE_COUNT="${1:-20}"
TOTAL_CORES="${2:-20}"
PUBLISH_HZ="${3:-100}"
DURATION_SECONDS="${4:-8}"
TARGET_MS_RAW="${5:-20}"
TERRAIN_CSV="${6:-${ROOT_DIR}/docs/examples/terrain_occlusion_sample.csv}"
TARGET_MS="$(python3 - <<PY
print(float("${TARGET_MS_RAW}"))
PY
)"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

cleanup() {
  kill "${TOPO_PID:-}" "${MANAGER_PID:-}" >/dev/null 2>&1 || true
  "${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p healthcheck_hz:=1 \
  -p demo_motion:=true \
  -p output_topic:=/swarm/state_raw \
  -p command_template:="sleep 1000" \
  > /tmp/swarm_manager_profile_terrain.log 2>&1 &
MANAGER_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:=/swarm/state_raw \
  -p output_topic:=/swarm/state \
  -p occlusion_mode:=terrain_csv \
  -p occlusion_terrain_csv:="${TERRAIN_CSV}" \
  -p occlusion_terrain_clearance_m:=20.0 \
  -p occlusion_terrain_samples:=24 \
  -p enable_profile_log:=true \
  -p profile_log_every_n:="${PUBLISH_HZ}" \
  -p profile_target_ms:="${TARGET_MS}" \
  > /tmp/swarm_topology_profile_terrain.log 2>&1 &
TOPO_PID=$!

sleep "${DURATION_SECONDS}"

profile_line="$(grep 'profile frames=' /tmp/swarm_topology_profile_terrain.log | tail -n 1 || true)"
if [[ -z "${profile_line}" ]]; then
  echo "[profile_topology_terrain_csv] FAIL: 未采集到 profile 日志"
  echo "--- topology log ---"
  cat /tmp/swarm_topology_profile_terrain.log || true
  exit 1
fi

echo "[profile_topology_terrain_csv] ${profile_line}"
if [[ "${profile_line}" == *"status=PASS"* ]]; then
  echo "[profile_topology_terrain_csv] PASS: terrain_csv 模式拓扑单帧耗时满足目标(<${TARGET_MS}ms)"
else
  echo "[profile_topology_terrain_csv] WARN: terrain_csv 模式拓扑单帧耗时未达到目标(<${TARGET_MS}ms)"
  exit 2
fi
