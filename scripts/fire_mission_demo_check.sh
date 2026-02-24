#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT_DIR}/scripts/demo_common.sh"
BASE_PORT="${1:-8899}"
INSTANCE_COUNT="${2:-4}"
TOTAL_CORES="${3:-4}"
TAG="fire_mission_demo_check"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

PORT="$(pick_available_port "${BASE_PORT}" 50)"
if [[ "${PORT}" == "-1" ]]; then
  log_error "${TAG}" "E_PORT_FULL" "从端口 ${BASE_PORT} 起未找到可用端口"
  exit 4
fi

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" >/dev/null 2>&1 || true
if ! "${ROOT_DIR}/scripts/fire_mission_demo_start.sh" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${PORT}" >/tmp/fire_demo_start.log 2>&1; then
  log_error "${TAG}" "E_START_FAIL" "启动 fire mission demo 失败"
  echo "--- start log ---"
  cat /tmp/fire_demo_start.log || true
  exit 1
fi

cleanup() {
  "${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

python3 - <<PY
import json
import time
import urllib.request

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import MissionPlan

PORT = int("${PORT}")
url = f"http://127.0.0.1:{PORT}/api/swarm_state"

proxy_handler = urllib.request.ProxyHandler({})
opener = urllib.request.build_opener(proxy_handler)

def fetch_state():
    data = opener.open(url, timeout=1.5).read().decode("utf-8")
    return json.loads(data)

def distance_m(a, b):
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return (dlat*dlat + dlon*dlon + dalt*dalt) ** 0.5

rclpy.init()
class PlanProbe(Node):
    def __init__(self):
        super().__init__("fire_demo_probe")
        self.seen = 0
        self.last = 0
        self.tracked_id = None
        self.target_position = None
        self.assign_time = None
        self.create_subscription(MissionPlan, "/swarm/mission_targets", self._cb, 10)
    def _cb(self, msg: MissionPlan):
        self.seen += 1
        self.last = len(msg.targets)
        if msg.targets:
            self.tracked_id = msg.targets[0].uav_id
            if self.target_position is None:
                self.target_position = list(msg.targets[0].position)
                self.assign_time = time.time()

probe = PlanProbe()
deadline = time.time() + 8
while time.time() < deadline and probe.seen == 0:
    rclpy.spin_once(probe, timeout_sec=0.2)

if probe.seen == 0 or probe.last == 0 or not probe.tracked_id or not probe.target_position:
    print("[fire_mission_demo_check] FAIL: mission planner 未产出目标")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

state0 = None
for _ in range(10):
    try:
        s = fetch_state()
        if s.get("uavs"):
            state0 = s
            break
    except Exception:
        pass
    time.sleep(0.5)

if state0 is None:
    print("[fire_mission_demo_check] FAIL: 无法获取初始 swarm state")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

tracked = probe.tracked_id
target_pos = probe.target_position
assign_time = probe.assign_time or time.time()
def pick_by_id(state, uid):
    for u in state.get("uavs", []):
        if u.get("id") == uid:
            return u["position"]
    return None

pos0 = pick_by_id(state0, tracked)
if pos0 is None:
    print("[fire_mission_demo_check] FAIL: 在初始状态中未找到被分配任务的 UAV")
    raise SystemExit(1)
completion_sec = None
deadline = time.time() + 10.0
pos1 = None
while time.time() < deadline:
    state1 = fetch_state()
    pos1 = pick_by_id(state1, tracked)
    if pos1 is None:
        time.sleep(0.2)
        continue
    if distance_m(pos1, target_pos) <= 8.0:
        completion_sec = time.time() - assign_time
        break
    time.sleep(0.2)

probe.destroy_node()
rclpy.shutdown()

if completion_sec is None:
    print("[fire_mission_demo_check][ERROR][E_MISSION_TIMEOUT] 未在窗口内完成任务")
    raise SystemExit(1)
if pos1 is None:
    print("[fire_mission_demo_check][ERROR][E_MISSION_TRACK_LOSS] 在后续状态中未找到被分配任务的 UAV")
    raise SystemExit(1)

move = distance_m(pos0, pos1)
print(
    f"mission_msgs={probe.seen} target_count={probe.last} tracked_uav={tracked} "
    f"moved_m={move:.2f} completion_s={completion_sec:.2f}"
)
if move < 2.0:
    print("[fire_mission_demo_check][ERROR][E_MOVE_SMALL] UAV 位移过小，未观察到任务驱动运动")
    raise SystemExit(1)
print(f"[fire_mission_demo_check][INFO] PASS: 火场任务链路有效 (port={PORT})")
PY
