#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE_COUNT="${1:-20}"
TOTAL_CORES="${2:-20}"
VIS_PORT="${3:-8913}"
DURATION_SECONDS="${4:-20}"
PROFILE_TARGET_MS="${5:-20.0}"
INPUT_CSV="${6:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc.csv}"
MAPPING_JSON="${7:-${ROOT_DIR}/data/fds_cases/min_fire/fds_min_fire_devc_mapping.json}"
FIRE_PUBLISH_HZ="${8:-4.0}"
PLANNER_MODE="${9:-coverage}"
AUTO_RUN_FDS="${10:-1}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

if [[ "${AUTO_RUN_FDS}" == "1" ]]; then
  echo "[stress_fds_pipeline] 先生成最小 FDS devc.csv"
  if ! "${ROOT_DIR}/scripts/fds_run_min_case.sh" >/tmp/fds_run_min_case_stress.log 2>&1; then
    echo "[stress_fds_pipeline] FAIL: FDS 生成 CSV 失败，见 /tmp/fds_run_min_case_stress.log"
    exit 1
  fi
fi

if [[ ! -f "${INPUT_CSV}" ]]; then
  echo "[stress_fds_pipeline] FAIL: 输入 CSV 不存在: ${INPUT_CSV}"
  exit 1
fi
if [[ ! -f "${MAPPING_JSON}" ]]; then
  echo "[stress_fds_pipeline] FAIL: 映射文件不存在: ${MAPPING_JSON}"
  exit 1
fi

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" "${VIS_PORT}" >/dev/null 2>&1 || true

cleanup() {
  "${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" "${VIS_PORT}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

"${ROOT_DIR}/scripts/fire_mission_fds_start.sh" \
  "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${VIS_PORT}" "${PLANNER_MODE}" \
  "${INPUT_CSV}" "${MAPPING_JSON}" "${FIRE_PUBLISH_HZ}" \
  >/tmp/fds_pipeline_start.log 2>&1

sleep 1

metrics_json="$(python3 - <<PY
import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, MissionPlan, MissionStatus, SwarmState

PORT = int("${VIS_PORT}")
DURATION = float("${DURATION_SECONDS}")
url = f"http://127.0.0.1:{PORT}/api/swarm_state"

proxy_handler = urllib.request.ProxyHandler({})
opener = urllib.request.build_opener(proxy_handler)

class Probe(Node):
    def __init__(self):
        super().__init__("fds_stress_probe")
        self.fire_first_seen = {}
        self.plan_latency_ms = []
        self.exec_latency_ms = []
        self.done_latency_ms = []
        self.done_seen = set()
        self.exec_seen = set()
        self.state_count = 0
        self.create_subscription(FireState, "/env/fire_state", self.on_fire, 20)
        self.create_subscription(MissionPlan, "/swarm/mission_targets", self.on_plan, 20)
        self.create_subscription(MissionStatus, "/swarm/mission_status", self.on_status, 20)
        self.create_subscription(SwarmState, "/swarm/state", self.on_state, 20)

    def now_ms(self):
        return time.time() * 1000.0

    def on_state(self, _msg):
        self.state_count += 1

    def on_fire(self, msg: FireState):
        now = self.now_ms()
        for hs in msg.hotspots:
            hid = str(hs.id)
            if hid not in self.fire_first_seen:
                self.fire_first_seen[hid] = now

    def _target_ref_from_reason(self, reason: str) -> str:
        txt = str(reason or "")
        if txt.startswith("track:"):
            parts = txt.split(":")
            if len(parts) >= 2 and parts[1]:
                return parts[1]
        return ""

    def on_plan(self, msg: MissionPlan):
        now = self.now_ms()
        for t in msg.targets:
            ref = self._target_ref_from_reason(str(t.reason))
            if not ref:
                continue
            tkey = (t.uav_id, ref)
            if ref in self.fire_first_seen and tkey not in self.exec_seen:
                self.plan_latency_ms.append(now - self.fire_first_seen[ref])
                self.exec_seen.add(tkey)

    def on_status(self, msg: MissionStatus):
        now = self.now_ms()
        for it in msg.items:
            ref = str(it.target_ref)
            if not ref or ref not in self.fire_first_seen:
                continue
            key = (it.uav_id, ref)
            state = int(it.state)
            if state == 2 and key not in self.exec_seen:
                self.exec_latency_ms.append(now - self.fire_first_seen[ref])
                self.exec_seen.add(key)
            if state == 3 and key not in self.done_seen:
                self.done_latency_ms.append(now - self.fire_first_seen[ref])
                self.done_seen.add(key)


def stat(vals, q):
    if not vals:
        return None
    xs = sorted(vals)
    idx = min(len(xs)-1, int(q * (len(xs)-1)))
    return xs[idx]

rclpy.init()
node = Probe()
end = time.time() + DURATION
api_rtt = []
api_ok = 0

while time.time() < end:
    t0 = time.time()
    try:
        data = opener.open(url, timeout=1.5).read()
        if data:
            api_ok += 1
        api_rtt.append((time.time() - t0) * 1000.0)
    except Exception:
        pass
    rclpy.spin_once(node, timeout_sec=0.2)

out = {
    "api_ok": api_ok,
    "api_rtt_avg_ms": (sum(api_rtt)/len(api_rtt)) if api_rtt else None,
    "api_rtt_p95_ms": stat(api_rtt, 0.95),
    "state_count": node.state_count,
    "state_hz": node.state_count / DURATION,
    "plan_latency_count": len(node.plan_latency_ms),
    "plan_latency_p50_ms": stat(node.plan_latency_ms, 0.50),
    "plan_latency_p95_ms": stat(node.plan_latency_ms, 0.95),
    "exec_latency_count": len(node.exec_latency_ms),
    "exec_latency_p50_ms": stat(node.exec_latency_ms, 0.50),
    "exec_latency_p95_ms": stat(node.exec_latency_ms, 0.95),
    "done_latency_count": len(node.done_latency_ms),
    "done_latency_p50_ms": stat(node.done_latency_ms, 0.50),
    "done_latency_p95_ms": stat(node.done_latency_ms, 0.95),
}
print(json.dumps(out, ensure_ascii=False))
node.destroy_node()
rclpy.shutdown()
PY
)"

echo "${metrics_json}" > /tmp/fds_stress_metrics.json

profile_line="$(grep 'profile frames=' /tmp/swarm_topology_fire_demo.log | tail -n 1 || true)"
if [[ -z "${profile_line}" ]]; then
  echo "[stress_fds_pipeline] FAIL: 未采集到拓扑 profile 日志"
  sed -n '1,120p' /tmp/swarm_topology_fire_demo.log || true
  exit 2
fi

echo "[stress_fds_pipeline] ${profile_line}"

python3 - <<PY
import json
m = json.load(open('/tmp/fds_stress_metrics.json','r',encoding='utf-8'))
for k in [
  'api_ok','api_rtt_avg_ms','api_rtt_p95_ms','state_hz',
  'plan_latency_count','plan_latency_p50_ms','plan_latency_p95_ms',
  'exec_latency_count','exec_latency_p50_ms','exec_latency_p95_ms',
  'done_latency_count','done_latency_p50_ms','done_latency_p95_ms'
]:
  print(f"[stress_fds_pipeline] {k}={m.get(k)}")
PY

if [[ "${profile_line}" != *"status=PASS"* ]]; then
  echo "[stress_fds_pipeline] FAIL: 拓扑帧耗时未达标"
  exit 3
fi

if ! python3 - <<PY
import json,sys
m=json.load(open('/tmp/fds_stress_metrics.json','r',encoding='utf-8'))
if (m.get('state_hz') or 0.0) < 3.5:
  sys.exit(1)
if (m.get('plan_latency_count') or 0) < 1:
  sys.exit(2)
sys.exit(0)
PY
then
  echo "[stress_fds_pipeline] FAIL: 端到端链路指标未达标（state_hz 或 plan_latency）"
  exit 4
fi

echo "[stress_fds_pipeline] PASS: FDS 场景端到端压测通过"
echo "[stress_fds_pipeline] 详情见 /tmp/fds_stress_metrics.json"
