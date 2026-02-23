#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_PORT="${1:-8899}"
INSTANCE_COUNT="${2:-4}"
TOTAL_CORES="${3:-4}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

PORT="$(python3 - <<PY
import socket
base = int("${BASE_PORT}")
for p in range(base, base + 50):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", p))
        print(p)
        break
    except OSError:
        pass
    finally:
        s.close()
else:
    print(base)
PY
)"

"${ROOT_DIR}/scripts/fire_mission_demo_stop.sh" >/dev/null 2>&1 || true
"${ROOT_DIR}/scripts/fire_mission_demo_start.sh" "${INSTANCE_COUNT}" "${TOTAL_CORES}" "${PORT}" >/tmp/fire_demo_start.log 2>&1

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
from swarm_interfaces.msg import MissionPlan, MissionStatus

PORT = int("${PORT}")
url = f"http://127.0.0.1:{PORT}/api/swarm_state"

proxy_handler = urllib.request.ProxyHandler({})
opener = urllib.request.build_opener(proxy_handler)

def fetch_state():
    data = opener.open(url, timeout=1.5).read().decode("utf-8")
    return json.loads(data)

def pick_one(state):
    if not state.get("uavs"):
        return None, None
    u = state["uavs"][0]
    return u["id"], u["position"]

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
        self.status_seen = 0
        self.status_done = 0
        self.status_fail = 0
        self.create_subscription(MissionPlan, "/swarm/mission_targets", self._cb, 10)
        self.create_subscription(MissionStatus, "/swarm/mission_status", self._cb_status, 10)
    def _cb(self, msg: MissionPlan):
        self.seen += 1
        self.last = len(msg.targets)
        if msg.targets:
            self.tracked_id = msg.targets[0].uav_id
    def _cb_status(self, msg: MissionStatus):
        self.status_seen += 1
        self.status_done = sum(1 for it in msg.items if int(it.state) == 3)
        self.status_fail = sum(1 for it in msg.items if int(it.state) == 4)

probe = PlanProbe()
deadline = time.time() + 8
while time.time() < deadline and (probe.seen == 0 or probe.status_seen == 0):
    rclpy.spin_once(probe, timeout_sec=0.2)

if probe.seen == 0 or probe.last == 0 or not probe.tracked_id:
    print("[fire_mission_demo_check] FAIL: mission planner 未产出目标")
    probe.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)
if probe.status_seen == 0:
    print("[fire_mission_demo_check] FAIL: mission status 未产出状态")
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
def pick_by_id(state, uid):
    for u in state.get("uavs", []):
        if u.get("id") == uid:
            return u["position"]
    return None

pos0 = pick_by_id(state0, tracked)
if pos0 is None:
    print("[fire_mission_demo_check] FAIL: 在初始状态中未找到被分配任务的 UAV")
    raise SystemExit(1)
time.sleep(3.0)
state1 = fetch_state()
pos1 = pick_by_id(state1, tracked)
probe.destroy_node()
rclpy.shutdown()

if pos1 is None:
    print("[fire_mission_demo_check] FAIL: 在后续状态中未找到被分配任务的 UAV")
    raise SystemExit(1)

move = distance_m(pos0, pos1)
print(
    f"mission_msgs={probe.seen} target_count={probe.last} mission_status_msgs={probe.status_seen} "
    f"done={probe.status_done} fail={probe.status_fail} tracked_uav={tracked} moved_m={move:.2f}"
)
if move < 2.0:
    print("[fire_mission_demo_check] FAIL: UAV 位移过小，未观察到任务驱动运动")
    raise SystemExit(1)
print(f"[fire_mission_demo_check] PASS: 火场任务链路有效 (port={PORT})")
PY
