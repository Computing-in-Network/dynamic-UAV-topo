#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rm -f "${ROOT_DIR}/.visual_demo_port" "${ROOT_DIR}/.visual_demo_pids" "${ROOT_DIR}/.fire_demo_port" "${ROOT_DIR}/.fire_demo_pids"
source "${ROOT_DIR}/scripts/demo_common.sh"

occupy_port() {
  local port="$1"
  local ready_file="/tmp/test_demo_port_${port}.ready"
  rm -f "${ready_file}"
  python3 - <<PY &
import socket, time, pathlib
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("127.0.0.1", int("${port}")))
s.listen(1)
pathlib.Path("${ready_file}").write_text("ok", encoding="utf-8")
time.sleep(8)
s.close()
PY
  for _ in $(seq 1 50); do
    if [[ -f "${ready_file}" ]]; then
      break
    fi
    sleep 0.05
  done
  if [[ ! -f "${ready_file}" ]]; then
    echo "ASSERT FAILED: occupy_port not ready for ${port}"
    exit 1
  fi
  if python3 - <<PY
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    s.bind(("127.0.0.1", int("${port}")))
    sys.exit(1)
except OSError:
    sys.exit(0)
finally:
    s.close()
PY
  then
    :
  else
    echo "ASSERT FAILED: port ${port} not occupied as expected"
    exit 1
  fi
  OCCUPY_PID=$!
}

assert_contains() {
  local txt="$1"
  local token="$2"
  if [[ "${txt}" != *"${token}"* ]]; then
    echo "ASSERT FAILED: expect contains '${token}'"
    echo "actual: ${txt}"
    exit 1
  fi
}

BASE_PORT="$(python3 - <<'PY'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("127.0.0.1", 0))
port = s.getsockname()[1]
s.close()
print(port)
PY
)"
OCCUPY_PID=""
occupy_port "${BASE_PORT}"
LOCK_PID="${OCCUPY_PID}"
trap 'kill "${LOCK_PID}" >/dev/null 2>&1 || true' EXIT

FALLBACK_PORT="$(pick_available_port "${BASE_PORT}" 8)"
if [[ "${FALLBACK_PORT}" != "$((BASE_PORT + 1))" ]]; then
  echo "ASSERT FAILED: expect fallback port=$((BASE_PORT + 1)), actual=${FALLBACK_PORT}"
  exit 1
fi

OUT1="$(DEMO_DRY_RUN=1 "${ROOT_DIR}/scripts/visual_demo_start.sh" 2 2 "${BASE_PORT}" 2>&1 || true)"
assert_contains "${OUT1}" "[visual_demo_start][INFO] DRY_RUN=1"
assert_contains "${OUT1}" "selected_port="

OUT2="$(DEMO_DRY_RUN=1 "${ROOT_DIR}/scripts/fire_mission_demo_start.sh" 2 2 "${BASE_PORT}" demo 2>&1 || true)"
assert_contains "${OUT2}" "[fire_mission_demo_start][INFO] DRY_RUN=1"
assert_contains "${OUT2}" "selected_port="

echo "test_demo_scripts: PASS"
