#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/third_party/PX4-Autopilot"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-$(nproc)}"
PUBLISH_HZ="${3:-5}"
HEALTHCHECK_HZ="${4:-1}"
OUTPUT_TOPIC="${5:-/swarm/state_raw}"

if [[ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
  echo "[ros2_run_swarm_manager] PX4 SITL binary not found, building..."
  (cd "${PX4_DIR}" && CCACHE_DISABLE=1 make px4_sitl -j"$(nproc)")
fi

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

PX4_CMD="cd ${PX4_DIR} && PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i {index} -d ./build/px4_sitl_default/etc"

exec ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p healthcheck_hz:="${HEALTHCHECK_HZ}" \
  -p output_topic:="${OUTPUT_TOPIC}" \
  -p command_template:="${PX4_CMD}"
