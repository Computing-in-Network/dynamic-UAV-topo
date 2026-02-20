#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, Hotspot


@dataclass
class HotspotSeed:
    hotspot_id: str
    lat: float
    lon: float
    alt: float
    intensity: float
    spread_mps: float
    phase: float


class FireAdapterDemo(Node):
    def __init__(self) -> None:
        super().__init__("fire_adapter_demo")
        self.output_topic: str = self.declare_parameter("output_topic", "/env/fire_state").value
        self.publish_hz: float = float(self.declare_parameter("publish_hz", 1.0).value)
        self.base_lat: float = float(self.declare_parameter("base_lat", 39.9042).value)
        self.base_lon: float = float(self.declare_parameter("base_lon", 116.4074).value)
        self.motion_radius_deg: float = float(self.declare_parameter("motion_radius_deg", 0.0012).value)
        self.publisher = self.create_publisher(FireState, self.output_topic, 10)
        self.t0: float = self.get_clock().now().nanoseconds / 1e9
        self.seeds: List[HotspotSeed] = [
            HotspotSeed("hotspot_a", self.base_lat, self.base_lon, 100.0, 0.92, 0.8, 0.0),
            HotspotSeed("hotspot_b", self.base_lat + 0.0015, self.base_lon - 0.0010, 100.0, 0.75, 0.4, 1.2),
        ]
        period = 1.0 / max(0.1, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"fire_adapter_demo started output={self.output_topic} hz={self.publish_hz:.2f}"
        )

    def _tick(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9 - self.t0
        msg = FireState()
        msg.stamp = self.get_clock().now().to_msg()
        hotspots: List[Hotspot] = []
        for seed in self.seeds:
            hs = Hotspot()
            hs.id = seed.hotspot_id
            lat = seed.lat + math.sin(now_s * 0.08 + seed.phase) * self.motion_radius_deg
            lon = seed.lon + math.cos(now_s * 0.06 + seed.phase) * self.motion_radius_deg
            hs.position = [lat, lon, seed.alt]
            hs.intensity = float(max(0.1, min(1.0, seed.intensity + math.sin(now_s * 0.2 + seed.phase) * 0.1)))
            hs.spread_mps = float(seed.spread_mps)
            hotspots.append(hs)
        msg.hotspots = hotspots
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = FireAdapterDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
