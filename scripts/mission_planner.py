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
        self.revisit_bonus: float = float(self.declare_parameter("revisit_bonus", 0.25).value)
        self.revisit_cycle_sec: float = float(self.declare_parameter("revisit_cycle_sec", 45.0).value)
        self.hold_time_sec: float = float(self.declare_parameter("hold_time_sec", 4.0).value)
        self.hold_distance_m: float = float(self.declare_parameter("hold_distance_m", 20.0).value)
        self.hold_bonus: float = float(self.declare_parameter("hold_bonus", 0.35).value)
        self.reassign_penalty: float = float(self.declare_parameter("reassign_penalty", 0.60).value)
        self.distance_cost_scale: float = float(self.declare_parameter("distance_cost_scale", 0.0012).value)
        self.coverage_min_distance_m: float = float(self.declare_parameter("coverage_min_distance_m", 90.0).value)
        self.coverage_density_penalty: float = float(self.declare_parameter("coverage_density_penalty", 0.0006).value)
        self.state_ttl_sec: float = float(self.declare_parameter("state_ttl_sec", 120.0).value)

        self.last_fire: Optional[FireState] = None
        self.last_swarm: Optional[SwarmState] = None
        self._last_plan_stamp: Dict[str, float] = {}
        self._last_plan_hotspot: Dict[str, str] = {}
        self._last_plan_position: Dict[str, List[float]] = {}
        self._hotspot_last_assigned_stamp: Dict[str, float] = {}

        self.sub_fire = self.create_subscription(FireState, self.fire_topic, self._on_fire, 10)
        self.sub_swarm = self.create_subscription(SwarmState, self.swarm_topic, self._on_swarm, 10)
        self.pub_plan = self.create_publisher(MissionPlan, self.plan_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, self.replan_hz), self._tick)
        self.get_logger().info(
            "mission_planner started "
            f"fire={self.fire_topic} "
            f"swarm={self.swarm_topic} "
            f"plan={self.plan_topic}"
        )

    def _on_fire(self, msg: FireState) -> None:
        self.last_fire = msg

    def _on_swarm(self, msg: SwarmState) -> None:
        self.last_swarm = msg

    def _score_hotspot_target(self, now_sec: float, hs, uav) -> float:
        dist = distance_score_m(list(uav.position), list(hs.position))
        last_assigned = self._hotspot_last_assigned_stamp.get(hs.id)
        if last_assigned is None:
            revisit_ratio = 0.0
        else:
            revisit_ratio = min((now_sec - last_assigned) / max(self.revisit_cycle_sec, 1.0), 1.0)
        base = float(hs.intensity) * (1.0 + self.revisit_bonus * revisit_ratio)
        score = base - dist * self.distance_cost_scale

        if uav.id in self._last_plan_hotspot:
            last_hs = self._last_plan_hotspot[uav.id]
            last_ts = self._last_plan_stamp.get(uav.id, now_sec)
            if now_sec - last_ts <= self.hold_time_sec:
                if last_hs == hs.id:
                    score += self.hold_bonus
                    last_pos = self._last_plan_position.get(uav.id, [])
                    if len(last_pos) == 3 and distance_score_m(last_pos, list(uav.position)) <= self.hold_distance_m:
                        score += self.hold_bonus
                else:
                    score -= self.reassign_penalty
        return score

    def _cover_penalty(self, candidate: Tuple[float, float, str], assigned_hotspots: List[Tuple[float, float, str]]) -> float:
        if not assigned_hotspots:
            return 0.0
        min_dist = None
        cand_lat = float(candidate[0])
        cand_lon = float(candidate[1])
        for lat, lon, _ in assigned_hotspots:
            d = (((cand_lat - lat) * 111000.0) ** 2 + ((cand_lon - lon) * 111000.0) ** 2) ** 0.5
            if min_dist is None or d < min_dist:
                min_dist = d
        if min_dist is None:
            return 0.0
        if min_dist >= self.coverage_min_distance_m:
            return 0.0
        return (self.coverage_min_distance_m - min_dist) * self.coverage_density_penalty

    def _prune_memory(self, now_sec: float) -> None:
        stale_uav = [uid for uid, ts in self._last_plan_stamp.items() if now_sec - ts > self.state_ttl_sec]
        for uid in stale_uav:
            self._last_plan_stamp.pop(uid, None)
            self._last_plan_hotspot.pop(uid, None)
            self._last_plan_position.pop(uid, None)
        stale_hotspot = [hid for hid, ts in self._hotspot_last_assigned_stamp.items() if now_sec - ts > self.state_ttl_sec]
        for hid in stale_hotspot:
            self._hotspot_last_assigned_stamp.pop(hid, None)

    def _tick(self) -> None:
        if self.last_fire is None or self.last_swarm is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        hotspots = [h for h in self.last_fire.hotspots if h.intensity >= self.min_intensity]
        uavs = self.last_swarm.uavs
        if not hotspots or not uavs:
            return

        self._prune_memory(now_sec)
        if self.hotspot_per_uav <= 0:
            return
        max_assign = min(len(hotspots), len(uavs) * self.hotspot_per_uav)
        if max_assign <= 0:
            return

        candidates: List[Tuple[float, str, str, List[float]]] = []
        for hs in hotspots:
            for uav in uavs:
                score = self._score_hotspot_target(now_sec, hs, uav)
                candidates.append((score, str(uav.id), hs.id, [float(hs.position[0]), float(hs.position[1]), float(hs.position[2])]))

        candidates.sort(key=lambda item: item[0], reverse=True)

        assignments: Dict[str, Tuple[List[float], float, str, str]] = {}
        assigned_uav: Set[str] = set()
        assigned_hotspots: List[Tuple[float, float, str]] = []
        for raw_score, uid, hs_id, pos in candidates:
            if len(assignments) >= max_assign:
                break
            if uid in assigned_uav:
                continue
            if any(existing_id == hs_id for _, _, existing_id in assigned_hotspots):
                continue

            penalty = self._cover_penalty((pos[0], pos[1], hs_id), assigned_hotspots)
            final_score = raw_score - penalty
            if final_score <= 0.0:
                continue

            assignments[uid] = (pos, final_score, hs_id, f"track:{hs_id}|score:{final_score:.2f}")
            assigned_uav.add(uid)
            assigned_hotspots.append((pos[0], pos[1], hs_id))

        if not assignments:
            return

        out = MissionPlan()
        out.stamp = self.get_clock().now().to_msg()
        targets: List[MissionTarget] = []
        assigned_position_by_uav: Dict[str, List[float]] = {}
        for uid, (pos, pri, hs_id, reason) in assignments.items():
            t = MissionTarget()
            t.uav_id = uid
            t.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            t.priority = float(pri)
            t.reason = reason
            targets.append(t)
            assigned_position_by_uav[uid] = t.position
        out.targets = targets
        self.pub_plan.publish(out)

        for uid, (_, _, hs_id, __) in assignments.items():
            self._last_plan_stamp[uid] = now_sec
            self._last_plan_hotspot[uid] = hs_id
            self._last_plan_position[uid] = assigned_position_by_uav.get(uid, [0.0, 0.0, 0.0])
            self._hotspot_last_assigned_stamp[hs_id] = now_sec


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
