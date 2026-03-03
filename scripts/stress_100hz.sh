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
  kill "${PLANNER_PID:-}" "${FIRE_PID:-}" "${TOPO_PID:-}" "${MANAGER_PID:-}" >/dev/null 2>&1 || true
  "${ROOT_DIR}/scripts/visual_demo_stop.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run swarm_uav_manager swarm_uav_manager_node --ros-args \
  -p instance_count:="${INSTANCE_COUNT}" \
  -p total_cores:="${TOTAL_CORES}" \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p healthcheck_hz:=1 \
  -p output_topic:=/swarm/state_raw \
  -p mission_follow_enabled:=true \
  -p mission_topic:=/swarm/mission_targets \
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

python3 "${ROOT_DIR}/scripts/fire_adapter_demo.py" \
  --ros-args -p output_topic:=/env/fire_state -p publish_hz:=2.0 \
  > /tmp/fire_adapter_stress.log 2>&1 &
FIRE_PID=$!

python3 "${ROOT_DIR}/scripts/mission_planner.py" \
  --ros-args -p fire_topic:=/env/fire_state -p swarm_topic:=/swarm/state -p plan_topic:=/swarm/mission_targets \
  > /tmp/mission_planner_stress.log 2>&1 &
PLANNER_PID=$!

sleep 1

ctxt_before="$(awk '/^ctxt /{print $2}' /proc/stat)"

stats_json="$(python3 - <<PY
import math
import time
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import MissionPlan, SwarmState

DURATION = float("${DURATION_SECONDS}")
TOPIC = "/swarm/state"
TARGET_HZ = float("${PUBLISH_HZ}")

class Counter(Node):
    def __init__(self) -> None:
        super().__init__("swarm_stress_counter")
        self.count = 0
        self.mission_msgs = 0
        self.latencies_ms = []
        self.intervals_ms = []
        self.last_stamp_ns = None
        self.latest_positions = {}
        self.tracked_uav = None
        self.tracked_baseline = None
        self.assign_time_ns = None
        self.mission_response_ms = None
        self.create_subscription(SwarmState, TOPIC, self.cb, 50)
        self.create_subscription(MissionPlan, "/swarm/mission_targets", self.on_mission, 50)

    def _distance_m(self, a, b) -> float:
        dlat = (a[0] - b[0]) * 111000.0
        dlon = (a[1] - b[1]) * 111000.0
        dalt = a[2] - b[2]
        return math.sqrt(dlat * dlat + dlon * dlon + dalt * dalt)

    def on_mission(self, msg: MissionPlan) -> None:
        self.mission_msgs += 1
        if self.assign_time_ns is not None or not msg.targets:
            return
        target = msg.targets[0]
        self.tracked_uav = target.uav_id
        pos = self.latest_positions.get(self.tracked_uav)
        if pos is None:
            return
        self.tracked_baseline = list(pos)
        self.assign_time_ns = time.time_ns()

    def cb(self, msg: SwarmState) -> None:
        self.count += 1
        for uav in msg.uavs:
            self.latest_positions[uav.id] = list(uav.position)
        if not (msg.stamp.sec or msg.stamp.nanosec):
            return
        stamp_ns = msg.stamp.sec * 1_000_000_000 + msg.stamp.nanosec
        now_ns = time.time_ns()
        latency_ms = max(0.0, (now_ns - stamp_ns) / 1_000_000.0)
        self.latencies_ms.append(latency_ms)
        if self.last_stamp_ns is not None:
            interval_ms = (stamp_ns - self.last_stamp_ns) / 1_000_000.0
            if interval_ms > 0.0:
                self.intervals_ms.append(interval_ms)
        self.last_stamp_ns = stamp_ns
        if (
            self.mission_response_ms is None and
            self.tracked_uav is not None and
            self.tracked_baseline is not None and
            self.assign_time_ns is not None
        ):
            pos = self.latest_positions.get(self.tracked_uav)
            if pos is not None and self._distance_m(self.tracked_baseline, pos) >= 5.0:
                self.mission_response_ms = (time.time_ns() - self.assign_time_ns) / 1_000_000.0

rclpy.init()
node = Counter()
end = time.time() + DURATION
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.01)

if node.count > 0 and node.latencies_ms:
    lat = sorted(node.latencies_ms)
    avg_latency = sum(node.latencies_ms) / len(node.latencies_ms)
    p50_latency = lat[len(lat)//2]
    p95_latency = lat[int(len(lat)*0.95)-1] if len(lat) > 1 else lat[0]
else:
    avg_latency = 0.0
    p50_latency = 0.0
    p95_latency = 0.0

if node.intervals_ms:
    interval_jitter = max(node.intervals_ms) - min(node.intervals_ms)
    max_gap = max(node.intervals_ms)
    avg_gap = sum(node.intervals_ms) / len(node.intervals_ms)
else:
    interval_jitter = 0.0
    max_gap = 0.0
    avg_gap = 0.0

expected = int(TARGET_HZ * DURATION)
drop_ratio = 0.0 if expected <= 0 else max(0.0, 1.0 - (node.count / float(expected)))
print(
    '{'
    f'"count": {node.count}, '
    f'"mission_msgs": {node.mission_msgs}, '
    f'"avg_latency_ms": {avg_latency:.3f}, '
    f'"p50_latency_ms": {p50_latency:.3f}, '
    f'"p95_latency_ms": {p95_latency:.3f}, '
    f'"drop_ratio": {drop_ratio:.6f}, '
    f'"interval_jitter_ms": {interval_jitter:.3f}, '
    f'"avg_interval_ms": {avg_gap:.3f}, '
    f'"max_interval_ms": {max_gap:.3f}, '
    f'"mission_response_ms": {(-1.0 if node.mission_response_ms is None else node.mission_response_ms):.3f}, '
    f'"tracked_uav": "{(node.tracked_uav or "")}"'
    + '}'
)
node.destroy_node()
rclpy.shutdown()
PY
)"

read -r msg_count mission_msgs e2e_avg e2e_p95 drop_ratio gap_jitter_ms gap_max_ms mission_response_ms tracked_uav <<< "$(python3 - <<PY
import json
stats = json.loads('''${stats_json}''')
print(
    f"{stats['count']} "
    f"{stats['mission_msgs']} "
    f"{stats['avg_latency_ms']:.3f} "
    f"{stats['p95_latency_ms']:.3f} "
    f"{stats['drop_ratio']:.4f} "
    f"{stats['interval_jitter_ms']:.3f} "
    f"{stats['max_interval_ms']:.3f} "
    f"{stats['mission_response_ms']:.3f} "
    f"{stats['tracked_uav'] or 'na'}"
)
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
echo "[stress_100hz] e2e_latency_ms avg=${e2e_avg} p95=${e2e_p95} drop_ratio=${drop_ratio}"
echo "[stress_100hz] interval_ms jitter=${gap_jitter_ms} max=${gap_max_ms}"
if python3 - <<PY
resp = float("${mission_response_ms}")
import sys
sys.exit(0 if resp >= 0.0 else 1)
PY
then
  echo "[stress_100hz] mission_loop_ms response=${mission_response_ms} mission_msgs=${mission_msgs} tracked_uav=${tracked_uav}"
else
  echo "[stress_100hz] WARN: mission_loop_ms response=unavailable mission_msgs=${mission_msgs} tracked_uav=${tracked_uav}"
fi
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

if python3 - <<PY
drop = float("${drop_ratio}")
max_drop = 0.20
import sys
sys.exit(0 if drop <= max_drop else 1)
PY
then
  echo "[stress_100hz] PASS: 掉包率 <= 0.20"
else
  echo "[stress_100hz] FAIL: 掉包率超过阈值(0.20)"
  exit 4
fi
