#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/third_party/PX4-Autopilot"
BUILD_DIR="${ROOT_DIR}/build"

INSTANCE_COUNT="${1:-4}"
TOTAL_CORES="${2:-$(nproc)}"
RUNTIME_SECONDS="${3:--1}"

if [[ ! -x "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]]; then
  echo "[run_px4_swarm] PX4 SITL binary not found, building..."
  (cd "${PX4_DIR}" && CCACHE_DISABLE=1 make px4_sitl -j"$(nproc)")
fi

cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" >/dev/null
cmake --build "${BUILD_DIR}" -j >/dev/null

# Placeholder variables consumed by NodeUavManager:
# {id} {index} {mavlink_udp_port} {mavlink_tcp_port} {sim_udp_port}
PX4_CMD="cd ${PX4_DIR} && PX4_SIM_MODEL=none ./build/px4_sitl_default/bin/px4 -i {index} -d ./build/px4_sitl_default/etc"

exec "${BUILD_DIR}/node_uav_manager" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${PX4_CMD}" "${RUNTIME_SECONDS}"
