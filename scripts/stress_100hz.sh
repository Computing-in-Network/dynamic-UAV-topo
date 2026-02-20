#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INSTANCE_COUNT="${1:-20}"
TOTAL_CORES="${2:-20}"
PUBLISH_HZ="${3:-100}"
DURATION_SECONDS="${4:-10}"
PROFILE_TARGET_MS="${5:-20.0}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

"${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true

cleanup() {
  kill "${TOPO_PID:-}" "${MANAGER_PID:-}" >/dev/null 2>&1 || true
  "${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p healthcheck_hz:=1 \
  -p demo_motion:=true \
  -p output_topic:=/swarm/state_raw \
  -p command_template:="sleep 1000" \
  > /tmp/swarm_manager_stress.log 2>&1 &
MANAGER_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:=/swarm/state_raw \
  -p output_topic:=/swarm/state \
  -p occlusion_mode:=altitude_gap \
  -p enable_profile_log:=true \
  -p profile_log_every_n:="${PUBLISH_HZ}" \
  -p profile_target_ms:="${PROFILE_TARGET_MS}" \
  > /tmp/swarm_topology_stress.log 2>&1 &
TOPO_PID=$!

sleep 1

ctxt_before="$(awk '/^ctxt /{print $2}' /proc/stat)"

msg_count="$(python3 - <<PY
import time
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import SwarmState

DURATION = float("${DURATION_SECONDS}")
TOPIC = "/swarm/state"

class Counter(Node):
    def __init__(self) -> None:
        super().__init__("swarm_stress_counter")
        self.count = 0
        self.create_subscription(SwarmState, TOPIC, self.cb, 50)
    def cb(self, _msg: SwarmState) -> None:
        self.count += 1

rclpy.init()
node = Counter()
end = time.time() + DURATION
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(node.count)
node.destroy_node()
rclpy.shutdown()
PY
)"

ctxt_after="$(awk '/^ctxt /{print $2}' /proc/stat)"

profile_line="$(grep 'profile frames=' /tmp/swarm_topology_stress.log | tail -n 1 || true)"
if [[ -z "${profile_line}" ]]; then
  echo "[stress_100hz] FAIL: 未采集到拓扑 profile 日志"
  cat /tmp/swarm_topology_stress.log || true
  exit 1
fi

obs_hz="$(python3 - <<PY
count = int("${msg_count}")
dur = float("${DURATION_SECONDS}")
print(f"{count / dur:.2f}")
PY
)"

ctxt_hz="$(python3 - <<PY
before = int("${ctxt_before}")
after = int("${ctxt_after}")
dur = float("${DURATION_SECONDS}")
print(f"{(after - before) / dur:.2f}")
PY
)"

min_hz="$(python3 - <<PY
target = float("${PUBLISH_HZ}")
print(f"{target * 0.80:.2f}")
PY
)"

echo "[stress_100hz] ${profile_line}"
echo "[stress_100hz] msg_count=${msg_count} observed_hz=${obs_hz} target_hz=${PUBLISH_HZ} min_hz=${min_hz}"
echo "[stress_100hz] context_switch_hz=${ctxt_hz}"

if [[ "${profile_line}" != *"status=PASS"* ]]; then
  echo "[stress_100hz] FAIL: 拓扑帧耗时未达标"
  exit 2
fi

if python3 - <<PY
obs = float("${obs_hz}")
min_hz = float("${min_hz}")
import sys
sys.exit(0 if obs >= min_hz else 1)
PY
then
  echo "[stress_100hz] PASS: 100Hz 压测通过（无明显阻塞）"
else
  echo "[stress_100hz] FAIL: 输出频率低于目标下限，存在阻塞风险"
  exit 3
fi
