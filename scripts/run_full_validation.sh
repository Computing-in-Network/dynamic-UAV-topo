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
run_step fds_stress ./scripts/stress_fds_pipeline.sh 20 20 8913 20 20.0
run_step fds_recovery ./scripts/test_fire_adapter_recovery.sh 6 6 8913 coverage

echo "[run_full_validation] ---- SUMMARY ----"
cat "${SUMMARY_FILE}"
echo "[run_full_validation] log_dir=${LOG_DIR}"
