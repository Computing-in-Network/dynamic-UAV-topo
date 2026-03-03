#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ALT_INPUT_TOPIC="${1:-/swarm/test_state_raw_compare_alt}"
ALT_OUTPUT_TOPIC="${2:-/swarm/test_state_compare_alt}"
TERRAIN_INPUT_TOPIC="${3:-/swarm/test_state_raw_compare_terrain}"
TERRAIN_OUTPUT_TOPIC="${4:-/swarm/test_state_compare_terrain}"
TERRAIN_CSV="${5:-${ROOT_DIR}/docs/examples/terrain_occlusion_sample.csv}"
TIMEOUT_SEC="${6:-8}"

set +u
source /opt/ros/humble/setup.bash
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

mkdir -p /tmp/roslog
export ROS_LOG_DIR=/tmp/roslog

cleanup() {
  kill "${ALT_PID:-}" "${TERRAIN_PID:-}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:="${ALT_INPUT_TOPIC}" \
  -p output_topic:="${ALT_OUTPUT_TOPIC}" \
  -p occlusion_mode:=altitude_gap \
  -p occlusion_altitude_gap_m:=35.0 \
  > /tmp/swarm_occlusion_compare_alt.log 2>&1 &
ALT_PID=$!

ros2 run swarm_topology_analyzer swarm_topology_analyzer_node --ros-args \
  -p input_topic:="${TERRAIN_INPUT_TOPIC}" \
  -p output_topic:="${TERRAIN_OUTPUT_TOPIC}" \
  -p occlusion_mode:=terrain_csv \
  -p occlusion_terrain_csv:="${TERRAIN_CSV}" \
  -p occlusion_terrain_clearance_m:=20.0 \
  -p occlusion_terrain_samples:=24 \
  > /tmp/swarm_occlusion_compare_terrain.log 2>&1 &
TERRAIN_PID=$!

python3 - <<PY
import sys
import time

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import SwarmState, UavState

ALT_INPUT_TOPIC = "${ALT_INPUT_TOPIC}"
ALT_OUTPUT_TOPIC = "${ALT_OUTPUT_TOPIC}"
TERRAIN_INPUT_TOPIC = "${TERRAIN_INPUT_TOPIC}"
TERRAIN_OUTPUT_TOPIC = "${TERRAIN_OUTPUT_TOPIC}"
TIMEOUT_SEC = float("${TIMEOUT_SEC}")


def to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CompareHarness(Node):
    def __init__(self) -> None:
        super().__init__("occlusion_compare_harness")
        self.alt_pub = self.create_publisher(SwarmState, ALT_INPUT_TOPIC, 10)
        self.terrain_pub = self.create_publisher(SwarmState, TERRAIN_INPUT_TOPIC, 10)
        self.alt_msg = None
        self.terrain_msg = None
        self.create_subscription(SwarmState, ALT_OUTPUT_TOPIC, self._on_alt, 10)
        self.create_subscription(SwarmState, TERRAIN_OUTPUT_TOPIC, self._on_terrain, 10)

    def _on_alt(self, msg: SwarmState) -> None:
        self.alt_msg = msg

    def _on_terrain(self, msg: SwarmState) -> None:
        self.terrain_msg = msg

    def build_msg(self, altitude: float, alt_delta: float = 0.0) -> SwarmState:
        msg = SwarmState()
        msg.stamp = self.get_clock().now().to_msg()
        a = UavState()
        a.id = "uav_a"
        a.position = [39.9042, 116.4074, altitude]
        a.velocity = [0.0, 0.0, 0.0]
        a.battery = 90.0
        a.status = UavState.STATUS_MISSION

        b = UavState()
        b.id = "uav_b"
        b.position = [39.9042, 116.4079, altitude + alt_delta]
        b.velocity = [0.0, 0.0, 0.0]
        b.battery = 90.0
        b.status = UavState.STATUS_MISSION

        msg.uavs = [a, b]
        msg.links = []
        return msg

    def publish_and_wait(self, altitude: float, alt_delta: float = 0.0) -> tuple[bool, bool]:
        out_deadline = time.time() + TIMEOUT_SEC
        alt_msg = self.build_msg(altitude, alt_delta)
        terrain_msg = self.build_msg(altitude, alt_delta)
        alt_target_ns = to_ns(alt_msg.stamp)
        terrain_target_ns = to_ns(terrain_msg.stamp)
        while time.time() < out_deadline:
            self.alt_pub.publish(alt_msg)
            self.terrain_pub.publish(terrain_msg)
            end_spin = time.time() + 0.25
            while time.time() < end_spin:
                rclpy.spin_once(self, timeout_sec=0.05)
                alt_ready = (
                    self.alt_msg is not None and
                    to_ns(self.alt_msg.stamp) >= alt_target_ns and
                    len(self.alt_msg.links) > 0
                )
                terrain_ready = (
                    self.terrain_msg is not None and
                    to_ns(self.terrain_msg.stamp) >= terrain_target_ns and
                    len(self.terrain_msg.links) > 0
                )
                if alt_ready and terrain_ready:
                    return (bool(self.alt_msg.links[0].is_occluded), bool(self.terrain_msg.links[0].is_occluded))
            alt_msg.stamp = self.get_clock().now().to_msg()
            terrain_msg.stamp = alt_msg.stamp
            alt_target_ns = to_ns(alt_msg.stamp)
            terrain_target_ns = alt_target_ns
        raise TimeoutError("no valid compare output received")


def main() -> int:
    rclpy.init()
    node = CompareHarness()
    try:
        time.sleep(0.8)
        flat_alt, flat_terrain = node.publish_and_wait(220.0, 0.0)
        ridge_alt, ridge_terrain = node.publish_and_wait(120.0, 0.0)
        gap_alt, gap_terrain = node.publish_and_wait(220.0, 80.0)
        print(f"scenario=clear_high altitude_gap={flat_alt} terrain_csv={flat_terrain}")
        print(f"scenario=terrain_ridge altitude_gap={ridge_alt} terrain_csv={ridge_terrain}")
        print(f"scenario=altitude_gap altitude_gap={gap_alt} terrain_csv={gap_terrain}")
        if (flat_alt, flat_terrain) != (False, False):
            print("[compare_occlusion_modes] FAIL: clear_high 场景输出异常")
            return 1
        if not (ridge_alt is False and ridge_terrain is True):
            print("[compare_occlusion_modes] FAIL: terrain_ridge 场景未体现模式差异")
            return 1
        if not (gap_alt is True and gap_terrain is False):
            print("[compare_occlusion_modes] FAIL: altitude_gap 场景未体现模式差异")
            return 1
        print("[compare_occlusion_modes] PASS: 两种模式均可工作，且差异符合预期")
        return 0
    except Exception as ex:
        print(f"[compare_occlusion_modes] FAIL: {ex}")
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


sys.exit(main())
PY
