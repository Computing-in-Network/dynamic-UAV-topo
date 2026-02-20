#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-8899}"
INSTANCE_COUNT="${2:-2}"
TOTAL_CORES="${3:-2}"

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/visual_demo_start.sh" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${PORT}" >/tmp/visual_demo_check_start.log 2>&1

cleanup() {
  "${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ok=0
for _ in $(seq 1 20); do
  if python3 - <<PY
import json,sys,urllib.request
url = "http://127.0.0.1:${PORT}/api/swarm_state"
proxy_handler = urllib.request.ProxyHandler({})
opener = urllib.request.build_opener(proxy_handler)
try:
    data = opener.open(url, timeout=1.5).read().decode("utf-8")
    obj = json.loads(data)
    if isinstance(obj, dict) and "uavs" in obj and len(obj["uavs"]) >= 1:
        print(f"uav_count={len(obj['uavs'])}")
        sys.exit(0)
except Exception:
    pass
sys.exit(1)
PY
  then
    ok=1
    break
  fi
  sleep 1
done

if [[ "${ok}" -eq 1 ]]; then
  echo "[visual_demo_check] PASS: 可视化 API 已返回 UAV 数据"
else
  echo "[visual_demo_check] FAIL: 可视化 API 未在超时内返回有效 UAV 数据"
  echo "--- start log ---"
  cat /tmp/visual_demo_check_start.log || true
  echo "--- manager log ---"
  cat /tmp/swarm_manager.log || true
  echo "--- visual server log ---"
  cat /tmp/swarm_visual_server.log || true
  exit 1
fi
