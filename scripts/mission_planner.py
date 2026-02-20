#!/usr/bin/env python3
from __future__ import annotations

from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, MissionPlan, MissionTarget, SwarmState


def distance_score_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return (dlat * dlat + dlon * dlon + dalt * dalt) ** 0.5


class MissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner")
        self.fire_topic: str = self.declare_parameter("fire_topic", "/env/fire_state").value
        self.swarm_topic: str = self.declare_parameter("swarm_topic", "/swarm/state").value
        self.plan_topic: str = self.declare_parameter("plan_topic", "/swarm/mission_targets").value
        self.replan_hz: float = float(self.declare_parameter("replan_hz", 1.0).value)
        self.hotspot_per_uav: int = int(self.declare_parameter("hotspot_per_uav", 1).value)
        self.min_intensity: float = float(self.declare_parameter("min_intensity", 0.2).value)

        self.last_fire: Optional[FireState] = None
        self.last_swarm: Optional[SwarmState] = None

        self.sub_fire = self.create_subscription(FireState, self.fire_topic, self._on_fire, 10)
        self.sub_swarm = self.create_subscription(SwarmState, self.swarm_topic, self._on_swarm, 10)
        self.pub_plan = self.create_publisher(MissionPlan, self.plan_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, self.replan_hz), self._tick)
        self.get_logger().info(
            f"mission_planner started fire={self.fire_topic} swarm={self.swarm_topic} plan={self.plan_topic}"
        )

    def _on_fire(self, msg: FireState) -> None:
        self.last_fire = msg

    def _on_swarm(self, msg: SwarmState) -> None:
        self.last_swarm = msg

    def _tick(self) -> None:
        if self.last_fire is None or self.last_swarm is None:
            return
        hotspots = [h for h in self.last_fire.hotspots if h.intensity >= self.min_intensity]
        if not hotspots or not self.last_swarm.uavs:
            return
        hotspots.sort(key=lambda h: float(h.intensity), reverse=True)
        uavs = list(self.last_swarm.uavs)

        # Greedy assignment: each hotspot picks nearest unassigned UAV first.
        used_uav: Set[str] = set()
        assignments: Dict[str, Tuple[List[float], float, str]] = {}
        for hs in hotspots:
            best = None
            for u in uavs:
                if u.id in used_uav:
                    continue
                d = distance_score_m(list(u.position), list(hs.position))
                if best is None or d < best[0]:
                    best = (d, u.id)
            if best is None:
                continue
            _, uid = best
            used_uav.add(uid)
            assignments[uid] = (list(hs.position), float(hs.intensity), f"track:{hs.id}")
            if len(used_uav) >= max(1, self.hotspot_per_uav * len(hotspots)):
                break

        if not assignments:
            return

        out = MissionPlan()
        out.stamp = self.get_clock().now().to_msg()
        targets: List[MissionTarget] = []
        for uid, (pos, pri, reason) in assignments.items():
            t = MissionTarget()
            t.uav_id = uid
            t.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            t.priority = float(pri)
            t.reason = reason
            targets.append(t)
        out.targets = targets
        self.pub_plan.publish(out)


def main() -> None:
    rclpy.init()
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
