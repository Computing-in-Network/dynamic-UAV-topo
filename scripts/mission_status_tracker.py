#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import MissionPlan, MissionStatus, MissionStatusItem, SwarmState


def distance_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return (dlat * dlat + dlon * dlon + dalt * dalt) ** 0.5


@dataclass
class Assignment:
    uav_id: str
    target_pos: List[float]
    target_ref: str
    reason: str
    assigned_sec: float
    initial_distance_m: float


class MissionStatusTracker(Node):
    def __init__(self) -> None:
        super().__init__("mission_status_tracker")
        self.plan_topic: str = str(self.declare_parameter("plan_topic", "/swarm/mission_targets").value)
        self.swarm_topic: str = str(self.declare_parameter("swarm_topic", "/swarm/state").value)
        self.status_topic: str = str(self.declare_parameter("status_topic", "/swarm/mission_status").value)
        self.publish_hz: float = float(self.declare_parameter("publish_hz", 2.0).value)
        self.complete_radius_m: float = float(self.declare_parameter("complete_radius_m", 25.0).value)
        self.timeout_sec: float = float(self.declare_parameter("timeout_sec", 45.0).value)
        self.missing_uav_fail_sec: float = float(self.declare_parameter("missing_uav_fail_sec", 6.0).value)

        self._uav_pos: Dict[str, List[float]] = {}
        self._uav_last_seen_sec: Dict[str, float] = {}
        self._assignments: Dict[str, Assignment] = {}
        self._terminal: Dict[str, MissionStatusItem] = {}

        self.create_subscription(MissionPlan, self.plan_topic, self._on_plan, 10)
        self.create_subscription(SwarmState, self.swarm_topic, self._on_swarm, 10)
        self._pub = self.create_publisher(MissionStatus, self.status_topic, 10)
        self._timer = self.create_timer(1.0 / max(0.2, self.publish_hz), self._tick)
        self.get_logger().info(
            "mission_status_tracker started "
            f"plan={self.plan_topic} swarm={self.swarm_topic} status={self.status_topic} "
            f"complete_r={self.complete_radius_m:.1f} timeout={self.timeout_sec:.1f}s"
        )

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1_000_000_000.0

    @staticmethod
    def _extract_target_ref(reason: str) -> str:
        txt = str(reason or "").strip()
        if txt.startswith("track:"):
            parts = txt.split(":")
            if len(parts) >= 2 and parts[1]:
                return parts[1]
        return txt or "unknown"

    def _on_swarm(self, msg: SwarmState) -> None:
        now = self._now_sec()
        for u in msg.uavs:
            self._uav_pos[u.id] = [float(u.position[0]), float(u.position[1]), float(u.position[2])]
            self._uav_last_seen_sec[u.id] = now

    def _on_plan(self, msg: MissionPlan) -> None:
        now = self._now_sec()
        for t in msg.targets:
            uid = str(t.uav_id)
            target = [float(t.position[0]), float(t.position[1]), float(t.position[2])]
            ref = self._extract_target_ref(str(t.reason))
            pos = self._uav_pos.get(uid)
            init_d = distance_m(pos, target) if pos else 0.0
            self._assignments[uid] = Assignment(
                uav_id=uid,
                target_pos=target,
                target_ref=ref,
                reason=str(t.reason),
                assigned_sec=now,
                initial_distance_m=max(1.0, init_d),
            )
            self._terminal.pop(uid, None)

    def _emit_item(
        self,
        uid: str,
        target_ref: str,
        state: int,
        progress: float,
        dist: float,
        reason: str,
    ) -> MissionStatusItem:
        it = MissionStatusItem()
        it.uav_id = uid
        it.target_ref = target_ref
        it.state = state
        it.progress = float(max(0.0, min(1.0, progress)))
        it.distance_m = float(max(0.0, dist))
        it.reason = reason
        return it

    def _tick(self) -> None:
        now = self._now_sec()
        out = MissionStatus()
        out.stamp = self.get_clock().now().to_msg()

        items: List[MissionStatusItem] = []

        for uid, a in list(self._assignments.items()):
            pos = self._uav_pos.get(uid)
            if pos is None:
                last_seen = self._uav_last_seen_sec.get(uid, 0.0)
                if (now - last_seen) > self.missing_uav_fail_sec:
                    failed = self._emit_item(
                        uid,
                        a.target_ref,
                        MissionStatusItem.STATE_FAILED,
                        0.0,
                        0.0,
                        "uav_missing",
                    )
                    self._terminal[uid] = failed
                    items.append(failed)
                    self._assignments.pop(uid, None)
                else:
                    pending = self._emit_item(
                        uid,
                        a.target_ref,
                        MissionStatusItem.STATE_ASSIGNED,
                        0.0,
                        0.0,
                        "waiting_uav_state",
                    )
                    items.append(pending)
                continue

            d = distance_m(pos, a.target_pos)
            progress = 1.0 - (d / max(1.0, a.initial_distance_m))
            elapsed = now - a.assigned_sec

            if d <= self.complete_radius_m:
                done = self._emit_item(
                    uid,
                    a.target_ref,
                    MissionStatusItem.STATE_COMPLETED,
                    1.0,
                    d,
                    "arrived",
                )
                self._terminal[uid] = done
                items.append(done)
                self._assignments.pop(uid, None)
                continue

            if elapsed >= self.timeout_sec:
                failed = self._emit_item(
                    uid,
                    a.target_ref,
                    MissionStatusItem.STATE_FAILED,
                    progress,
                    d,
                    "timeout",
                )
                self._terminal[uid] = failed
                items.append(failed)
                self._assignments.pop(uid, None)
                continue

            executing = self._emit_item(
                uid,
                a.target_ref,
                MissionStatusItem.STATE_EXECUTING,
                progress,
                d,
                "enroute",
            )
            items.append(executing)

        for uid, t in self._terminal.items():
            if uid not in self._assignments:
                items.append(t)

        if not items:
            for uid in sorted(self._uav_pos.keys()):
                items.append(self._emit_item(uid, "", MissionStatusItem.STATE_IDLE, 0.0, 0.0, "idle"))

        out.items = items
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = MissionStatusTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
