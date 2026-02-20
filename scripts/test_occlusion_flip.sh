#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT_TOPIC="${1:-/swarm/test_state_raw}"
OUTPUT_TOPIC="${2:-/swarm/test_state}"
ALT_GAP_M="${3:-35.0}"
TIMEOUT_SEC="${4:-8}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

cleanup() {
  kill "${TOPO_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:="${INPUT_TOPIC}" \
  -p output_topic:="${OUTPUT_TOPIC}" \
  -p occlusion_mode:=altitude_gap \
  -p occlusion_altitude_gap_m:="${ALT_GAP_M}" \
  > /tmp/swarm_occlusion_topo.log 2>&1 &
TOPO_PID=$!

python3 - <<PY
import sys
import time

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import SwarmState, UavState

INPUT_TOPIC = "${INPUT_TOPIC}"
OUTPUT_TOPIC = "${OUTPUT_TOPIC}"
TIMEOUT_SEC = float("${TIMEOUT_SEC}")


def to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Harness(Node):
    def __init__(self) -> None:
        super().__init__("occlusion_flip_harness")
        self.pub = self.create_publisher(SwarmState, INPUT_TOPIC, 10)
        self.last_msg = None
        self.sub = self.create_subscription(SwarmState, OUTPUT_TOPIC, self._cb, 10)

    def _cb(self, msg: SwarmState) -> None:
        self.last_msg = msg

    def build_msg(self, alt_delta: float) -> SwarmState:
        msg = SwarmState()
        msg.stamp = self.get_clock().now().to_msg()
        a = UavState()
        a.id = "uav_a"
        a.position = [39.9042, 116.4074, 100.0]
        a.velocity = [0.0, 0.0, 0.0]
        a.battery = 90.0
        a.status = UavState.STATUS_MISSION

        b = UavState()
        b.id = "uav_b"
        b.position = [39.9042, 116.4079, 100.0 + alt_delta]
        b.velocity = [0.0, 0.0, 0.0]
        b.battery = 90.0
        b.status = UavState.STATUS_MISSION

        msg.uavs = [a, b]
        msg.links = []
        return msg

    def publish_and_wait(self, alt_delta: float) -> bool:
        out_deadline = time.time() + TIMEOUT_SEC
        msg = self.build_msg(alt_delta)
        target_ns = to_ns(msg.stamp)
        while time.time() < out_deadline:
            self.pub.publish(msg)
            end_spin = time.time() + 0.25
            while time.time() < end_spin:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.last_msg is None:
                    continue
                if to_ns(self.last_msg.stamp) < target_ns:
                    continue
                if len(self.last_msg.links) == 0:
                    continue
                return bool(self.last_msg.links[0].is_occluded)
            msg.stamp = self.get_clock().now().to_msg()
            target_ns = to_ns(msg.stamp)
        raise TimeoutError("no valid topology output received")


def main() -> int:
    rclpy.init()
    node = Harness()
    try:
        time.sleep(0.8)
        first = node.publish_and_wait(80.0)   # expect occluded=True
        second = node.publish_and_wait(5.0)   # expect occluded=False
        print(f"occluded_first={first} occluded_second={second}")
        if first is True and second is False:
            print("[test_occlusion_flip] PASS: is_occluded 已从 true 翻转为 false")
            return 0
        print("[test_occlusion_flip] FAIL: is_occluded 未按预期翻转")
        return 1
    except Exception as ex:
        print(f"[test_occlusion_flip] FAIL: {ex}")
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


sys.exit(main())
PY
