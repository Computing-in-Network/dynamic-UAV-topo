#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="/tmp/full_validation"
SUMMARY_FILE="${LOG_DIR}/summary.txt"
mkdir -p "${LOG_DIR}"
: > "${SUMMARY_FILE}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

export FI_PROVIDER="${FI_PROVIDER:-tcp}"
export I_MPI_FABRICS="${I_MPI_FABRICS:-shm:tcp}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"

pick_free_port() {
  local base="${1:-8913}"
  python3 - <<PY
import socket
base=int("${base}")
for p in range(base, base+80):
    s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1",p))
        print(p)
        break
    except OSError:
        pass
    finally:
        s.close()
else:
    print(base)
PY
}

run_step() {
  local name="$1"
  shift
  local log_file="${LOG_DIR}/${name}.log"
  echo "[run_full_validation] START ${name}" | tee -a "${SUMMARY_FILE}"
  if "$@" >"${log_file}" 2>&1; then
    echo "[run_full_validation] PASS ${name}" | tee -a "${SUMMARY_FILE}"
  else
    local ec=$?
    echo "[run_full_validation] FAIL ${name} exit=${ec}" | tee -a "${SUMMARY_FILE}"
  fi
}

run_step ros2_build ./scripts/ros2_build.sh
run_step planner_eval ./scripts/eval_planner_coverage.py --ticks 120 --seed 42 --uavs 8 --hotspots 10
run_step fire_mission_check ./scripts/fire_mission_demo_check.sh 8913 6 6
run_step fds_min_case ./scripts/fds_run_min_case.sh
PORT_STRESS="$(pick_free_port 8921)"
PORT_RECOVERY="$(pick_free_port 8931)"
run_step fds_stress ./scripts/stress_fds_pipeline.sh 20 20 "${PORT_STRESS}" 20 20.0 \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  4.0 coverage 0
run_step fds_recovery ./scripts/test_fire_adapter_recovery.sh 6 6 "${PORT_RECOVERY}" coverage \
  data/fds_cases/min_fire/fds_min_fire_devc.csv \
  data/fds_cases/min_fire/fds_min_fire_devc_mapping.json \
  2.0

echo "[run_full_validation] ---- SUMMARY ----"
cat "${SUMMARY_FILE}"
echo "[run_full_validation] log_dir=${LOG_DIR}"
