#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

log() { echo "[release_smoke] $*"; }
fail() { log "FAIL: $*"; exit 1; }
usage() {
  cat <<'USAGE'
用法：scripts/release_smoke.sh [--ros|--no-ros]

参数：
  --ros      强制执行可选 ROS2/C++ 构建检查（需要有 ROS2 与 colcon）
  --no-ros   跳过 ROS2/C++ 检查（默认自动按环境检测）
  --help     打印帮助
USAGE
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  usage
  exit 0
fi

ROS_MODE="auto"
if [[ "${1:-}" == "--ros" ]]; then
  ROS_MODE="force"
elif [[ "${1:-}" == "--no-ros" ]]; then
  ROS_MODE="skip"
elif [[ -n "${1:-}" ]]; then
  fail "未知参数: ${1} (支持 --ros/--no-ros)"
fi

echo "[release_smoke] start"

if ! python3 - <<'PY'
import importlib.util
import sys
if importlib.util.find_spec("numpy") is None:
    print("missing_numpy")
    sys.exit(1)
PY
then
  fail "缺少 numpy 依赖，请先执行: pip install numpy"
fi

for py in scripts/*.py tests/*.py; do
  log "compile ${py}"
  python3 -m py_compile "${py}"
done

for f in tests/test_*.py; do
  log "run ${f}"
  python3 "${f}"
done

log "run ricci cuda poc (no-cuda mode)"
if ! python3 - <<'PY'
import json
import subprocess
import sys

cmd = [
    "python3",
    "scripts/ricci_cuda_poc.py",
    "--disable-cuda",
    "--nodes", "12",
    "--density", "0.2",
    "--seed", "2026",
    "--repeat", "2",
]
res = subprocess.run(cmd, check=False, capture_output=True, text=True)
if res.returncode != 0:
    print("ricci_cuda_poc_exit_nonzero")
    sys.exit(2)

try:
    obj = json.loads(res.stdout.strip())
except json.JSONDecodeError:
    print("ricci_cuda_poc_output_invalid")
    sys.exit(3)

backend = obj.get("backend", {}).get("cuda", {})
if not isinstance(backend, dict) or "available" not in backend:
    print("ricci_cuda_poc_no_backend")
    sys.exit(4)
PY
then
  fail "ricci_cuda_poc 回归检查未通过"
fi

run_ros_smoke=false
if [[ "${ROS_MODE}" == "force" ]]; then
  run_ros_smoke=true
elif [[ "${ROS_MODE}" == "auto" ]]; then
  if command -v colcon >/dev/null 2>&1 && [[ -f /opt/ros/humble/setup.bash ]]; then
    run_ros_smoke=true
  else
    log "skip ros smoke: 未检测到 ROS2/colcon"
  fi
fi

if [[ "${run_ros_smoke}" == true ]]; then
  log "run ros smoke checks"
  if ! bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    if [[ -f "ros2_ws/install/setup.bash" ]]; then
      source ros2_ws/install/setup.bash
    fi
    set -u
    colcon build \
      --base-paths ros2_ws \
      --build-base /tmp/release_smoke_build \
      --install-base /tmp/release_smoke_install \
      --packages-select swarm_interfaces swarm_uav_manager swarm_topology_analyzer \
      --merge-install
  '; then
    fail "ROS2/C++ 构建 smoke 未通过"
  fi

  log "run fire mission fds acceptance"
  if ! bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    if [[ -f "ros2_ws/install/setup.bash" ]]; then
      source ros2_ws/install/setup.bash
    fi
    set -u
    ./scripts/fire_mission_demo_check.sh 8899 4 4
  '; then
    fail "Fire Mission FDS 默认链路验收未通过"
  fi
fi

log "PASS: release smoke"
