#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

log() { echo "[p1_acceptance] $*"; }
fail() { log "FAIL: $*"; exit 1; }
retry() {
  local attempts="$1"
  shift
  local n=1
  while true; do
    if "$@"; then
      return 0
    fi
    if [[ "${n}" -ge "${attempts}" ]]; then
      return 1
    fi
    n=$((n + 1))
    log "retry ${n}/${attempts}: $*"
    sleep 1
  done
}

set +u
source /opt/ros/humble/setup.bash
if [[ -f "ros2_ws/install/setup.bash" ]]; then
  source "ros2_ws/install/setup.bash"
fi
set -u

log "start"

log "task planning regression"
if ! python3 tests/test_mission_planner.py; then
  fail "任务规划稳态回归未通过"
fi

log "terrain occlusion acceptance"
if ! bash ./scripts/test_occlusion_terrain_csv.sh; then
  fail "terrain_csv 真实遮挡验收未通过"
fi

log "terrain occlusion fallback"
if ! bash ./scripts/test_occlusion_terrain_fallback.sh; then
  fail "terrain_csv 缺失回退验收未通过"
fi

log "terrain occlusion invalid-input"
if ! bash ./scripts/test_occlusion_terrain_invalid.sh; then
  fail "terrain_csv 非法输入验收未通过"
fi

log "terrain occlusion compare"
if ! bash ./scripts/compare_occlusion_modes.sh; then
  fail "遮挡模式对比验收未通过"
fi

log "stress metrics"
if ! bash ./scripts/stress_100hz.sh 20 20 100 6 20.0; then
  fail "100Hz 压测指标验收未通过"
fi

log "fire mission fds acceptance"
if ! retry 2 bash ./scripts/fire_mission_demo_check.sh 8899 4 4; then
  fail "Fire Mission FDS 链路验收未通过"
fi

log "PASS: P1 acceptance baseline"
