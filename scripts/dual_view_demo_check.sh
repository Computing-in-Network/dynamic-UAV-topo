#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/demo_common.sh"
TAG="dual_view_demo_check"

BASE_PORT="${1:-8899}"
INSTANCE_COUNT="${2:-2}"
TOTAL_CORES="${3:-2}"
GAZEBO_MODE="${4:-mock}"
MAX_DIST_M="${5:-30.0}"
MAX_DT_MS="${6:-2500}"

"${ROOT_DIR}/scripts/dual_view_demo_stop.sh" >/dev/null 2>&1 || true
if ! "${ROOT_DIR}/scripts/dual_view_demo_start.sh" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${BASE_PORT}" "${GAZEBO_MODE}" >/tmp/dual_view_start.log 2>&1; then
  log_error "${TAG}" "E_START_FAIL" "启动 dual view 失败"
  echo "--- start log ---"
  cat /tmp/dual_view_start.log || true
  exit 1
fi

cleanup() {
  "${ROOT_DIR}/scripts/dual_view_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [[ ! -f "${ROOT_DIR}/.dual_view_demo_pids" ]]; then
  log_error "${TAG}" "E_NO_PID_FILE" "缺少 .dual_view_demo_pids"
  exit 2
fi

read -r _ _ VIS_PORT FOCUS_PORT _ < "${ROOT_DIR}/.dual_view_demo_pids" || true
if [[ -z "${VIS_PORT}" || -z "${FOCUS_PORT}" ]]; then
  log_error "${TAG}" "E_PID_PARSE" "无法解析 dual view 端口信息"
  exit 3
fi

python3 - <<PY
import json
import time
import urllib.request
import sys
from pathlib import Path

ROOT = Path("${ROOT_DIR}")
sys.path.insert(0, str(ROOT / "scripts"))
from dual_view_sync import evaluate_sync  # noqa: E402

vis_url = "http://127.0.0.1:${VIS_PORT}/api/swarm_state"
focus_url = "http://127.0.0.1:${FOCUS_PORT}/api/gazebo_focus_state"
max_dist_m = float("${MAX_DIST_M}")
max_dt_ms = int("${MAX_DT_MS}")

proxy_handler = urllib.request.ProxyHandler({})
opener = urllib.request.build_opener(proxy_handler)

def fetch_json(url: str):
    raw = opener.open(url, timeout=1.5).read().decode("utf-8")
    return json.loads(raw)

def pick_uav(state, uid):
    for u in state.get("uavs", []):
        if u.get("id") == uid:
            return u
    return None

pairs = []
target_uid = None
deadline = time.time() + 15.0
while time.time() < deadline and len(pairs) < 8:
    try:
        vis = fetch_json(vis_url)
        focus = fetch_json(focus_url)
        if not isinstance(vis, dict) or not isinstance(focus, dict):
            time.sleep(0.3)
            continue
        if not focus.get("ok"):
            time.sleep(0.3)
            continue
        fu = focus.get("uav", {})
        if target_uid is None:
            target_uid = fu.get("id")
        if not target_uid:
            time.sleep(0.3)
            continue
        vu = pick_uav(vis, target_uid)
        if vu is None:
            time.sleep(0.3)
            continue
        pairs.append((
            int(vis.get("timestamp", 0)),
            [float(vu["position"][0]), float(vu["position"][1]), float(vu["position"][2])],
            int(focus.get("timestamp", 0)),
            [float(fu["position"][0]), float(fu["position"][1]), float(fu["position"][2])],
        ))
    except Exception:
        pass
    time.sleep(0.35)

result = evaluate_sync(pairs, max_dist_m=max_dist_m, max_time_diff_ms=max_dt_ms)
print(f"samples={result['samples']} max_dist_m={result['max_dist_m']:.3f} max_dt_ms={result['max_dt_ms']} uid={target_uid}")
if not result["ok"]:
    print(f"[dual_view_demo_check][ERROR][E_SYNC] reason={result['reason']} thresholds(dist={max_dist_m},dt={max_dt_ms})")
    raise SystemExit(1)
print(f"[dual_view_demo_check][INFO] PASS: 双视角同步检查通过 (vis_port=${VIS_PORT}, focus_port=${FOCUS_PORT})")
PY
