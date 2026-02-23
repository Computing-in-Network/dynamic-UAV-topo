#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CASE_DIR="${1:-${ROOT_DIR}/data/fds_cases/min_fire}"
CASE_FILE="${2:-fds_min_fire.fds}"
CHID="${3:-fds_min_fire}"

if [[ ! -d "${CASE_DIR}" ]]; then
  echo "[fds_run_min_case] case dir not found: ${CASE_DIR}"
  exit 1
fi

if [[ ! -f "${CASE_DIR}/${CASE_FILE}" ]]; then
  echo "[fds_run_min_case] case file not found: ${CASE_DIR}/${CASE_FILE}"
  exit 1
fi

set +u
if [[ -f /home/zyren/FDS/FDS6/bin/FDS6VARS.sh ]]; then
  source /home/zyren/FDS/FDS6/bin/FDS6VARS.sh
fi
set -u

if ! command -v fds >/dev/null 2>&1; then
  echo "[fds_run_min_case] fds command not found. Please source FDS6VARS.sh first."
  exit 1
fi

# Improve compatibility on some hosts where OFI defaults are not usable.
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
export FI_PROVIDER="${FI_PROVIDER:-tcp}"
export I_MPI_FABRICS="${I_MPI_FABRICS:-shm:tcp}"

pushd "${CASE_DIR}" >/dev/null
rm -f "${CHID}.out" "${CHID}_devc.csv" "${CHID}.smv" || true

echo "[fds_run_min_case] running case=${CASE_FILE}"
echo "[fds_run_min_case] log: /tmp/${CHID}.log"

if ! fds "${CASE_FILE}" >"/tmp/${CHID}.log" 2>&1; then
  echo "[fds_run_min_case] fds run failed, see /tmp/${CHID}.log"
  popd >/dev/null
  exit 2
fi

if [[ ! -f "${CHID}_devc.csv" ]]; then
  echo "[fds_run_min_case] missing output ${CHID}_devc.csv, see /tmp/${CHID}.log"
  popd >/dev/null
  exit 3
fi

rows=$(wc -l < "${CHID}_devc.csv" | awk '{print $1}')
echo "[fds_run_min_case] done: ${CASE_DIR}/${CHID}_devc.csv rows=${rows}"
popd >/dev/null
