#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Set, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from swarm_interfaces.msg import FireState, MissionPlan, MissionTarget, SwarmState
except ImportError:
    rclpy = None
    Node = object
    FireState = MissionPlan = MissionTarget = SwarmState = None


def distance_score_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return (dlat * dlat + dlon * dlon + dalt * dalt) ** 0.5


@dataclass
class PlannerConfig:
    hotspot_per_uav: int = 1
    min_intensity: float = 0.2
    revisit_bonus: float = 0.25
    revisit_cycle_sec: float = 45.0
    hold_time_sec: float = 4.0
    hold_distance_m: float = 20.0
    hold_bonus: float = 0.35
    reassign_penalty: float = 0.60
    switch_cooldown_sec: float = 12.0
    switch_cooldown_penalty: float = 0.45
    distance_cost_scale: float = 0.0012
    coverage_min_distance_m: float = 90.0
    coverage_density_penalty: float = 0.0006
    coverage_recent_window_sec: float = 18.0
    coverage_repeat_penalty: float = 0.30
    hotspot_decay_per_visit: float = 0.08
    hotspot_decay_cap: float = 0.40
    state_ttl_sec: float = 120.0


class MissionPlannerCore:
    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self._last_plan_stamp: Dict[str, float] = {}
        self._last_plan_hotspot: Dict[str, str] = {}
        self._last_plan_position: Dict[str, List[float]] = {}
        self._hotspot_last_assigned_stamp: Dict[str, float] = {}
        self._hotspot_assignment_count: Dict[str, int] = {}

    def _score_hotspot_target(self, now_sec: float, hs: Any, uav: Any) -> float:
        dist = distance_score_m(list(uav.position), list(hs.position))
        last_assigned = self._hotspot_last_assigned_stamp.get(hs.id)
        if last_assigned is None:
            revisit_ratio = 1.0
            recent_coverage_penalty = 0.0
        else:
            elapsed = max(0.0, now_sec - last_assigned)
            revisit_ratio = min(elapsed / max(self.config.revisit_cycle_sec, 1.0), 1.0)
            recent_ratio = max(0.0, 1.0 - (elapsed / max(self.config.coverage_recent_window_sec, 1.0)))
            recent_coverage_penalty = self.config.coverage_repeat_penalty * recent_ratio

        visit_count = self._hotspot_assignment_count.get(hs.id, 0)
        visit_decay = min(visit_count * self.config.hotspot_decay_per_visit, self.config.hotspot_decay_cap)

        base = float(hs.intensity) * (1.0 + self.config.revisit_bonus * revisit_ratio)
        score = base - dist * self.config.distance_cost_scale
        score -= recent_coverage_penalty
        score -= visit_decay

        if uav.id in self._last_plan_hotspot:
            last_hs = self._last_plan_hotspot[uav.id]
            last_ts = self._last_plan_stamp.get(uav.id, now_sec)
            elapsed_since_plan = max(0.0, now_sec - last_ts)
            if elapsed_since_plan <= self.config.hold_time_sec:
                if last_hs == hs.id:
                    score += self.config.hold_bonus
                    last_pos = self._last_plan_position.get(uav.id, [])
                    if len(last_pos) == 3 and distance_score_m(last_pos, list(uav.position)) <= self.config.hold_distance_m:
                        score += self.config.hold_bonus
                else:
                    score -= self.config.reassign_penalty
            if last_hs != hs.id and elapsed_since_plan <= self.config.switch_cooldown_sec:
                score -= self.config.switch_cooldown_penalty
        return score

    def _cover_penalty(
        self, candidate: Tuple[float, float, str], assigned_hotspots: List[Tuple[float, float, str]]
    ) -> float:
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
        if min_dist >= self.config.coverage_min_distance_m:
            return 0.0
        return (self.config.coverage_min_distance_m - min_dist) * self.config.coverage_density_penalty

    def _prune_memory(self, now_sec: float) -> None:
        stale_uav = [uid for uid, ts in self._last_plan_stamp.items() if now_sec - ts > self.config.state_ttl_sec]
        for uid in stale_uav:
            self._last_plan_stamp.pop(uid, None)
            self._last_plan_hotspot.pop(uid, None)
            self._last_plan_position.pop(uid, None)
        stale_hotspot = [
            hid for hid, ts in self._hotspot_last_assigned_stamp.items() if now_sec - ts > self.config.state_ttl_sec
        ]
        for hid in stale_hotspot:
            self._hotspot_last_assigned_stamp.pop(hid, None)
            self._hotspot_assignment_count.pop(hid, None)

    def build_plan(self, now_sec: float, hotspots: List[Any], uavs: List[Any]) -> Dict[str, Tuple[List[float], float, str, str]]:
        if self.config.hotspot_per_uav <= 0:
            return {}
        self._prune_memory(now_sec)
        max_assign = min(len(hotspots), len(uavs) * self.config.hotspot_per_uav)
        if max_assign <= 0:
            return {}

        candidates: List[Tuple[float, str, str, List[float]]] = []
        for hs in hotspots:
            if float(hs.intensity) < self.config.min_intensity:
                continue
            for uav in uavs:
                score = self._score_hotspot_target(now_sec, hs, uav)
                candidates.append(
                    (
                        score,
                        str(uav.id),
                        hs.id,
                        [float(hs.position[0]), float(hs.position[1]), float(hs.position[2])],
                    )
                )

        assignments: Dict[str, Tuple[List[float], float, str, str]] = {}
        remaining_candidates = candidates
        assigned_uav: Set[str] = set()
        assigned_hotspots: List[Tuple[float, float, str]] = []
        while remaining_candidates and len(assignments) < max_assign:
            best_choice: Optional[Tuple[float, str, str, List[float]]] = None
            best_final_score: Optional[float] = None
            next_candidates: List[Tuple[float, str, str, List[float]]] = []
            for raw_score, uid, hs_id, pos in remaining_candidates:
                if uid in assigned_uav:
                    continue
                if any(existing_id == hs_id for _, _, existing_id in assigned_hotspots):
                    continue
                next_candidates.append((raw_score, uid, hs_id, pos))
                penalty = self._cover_penalty((pos[0], pos[1], hs_id), assigned_hotspots)
                final_score = raw_score - penalty
                if final_score <= 0.0:
                    continue
                if best_final_score is None or final_score > best_final_score:
                    best_choice = (raw_score, uid, hs_id, pos)
                    best_final_score = final_score

            if best_choice is None or best_final_score is None:
                break

            _, uid, hs_id, pos = best_choice
            assignments[uid] = (pos, best_final_score, hs_id, f"track:{hs_id}|score:{best_final_score:.2f}")
            assigned_uav.add(uid)
            assigned_hotspots.append((pos[0], pos[1], hs_id))

            remaining_candidates = [
                item
                for item in next_candidates
                if item[1] != uid and item[2] != hs_id
            ]

        for uid, (pos, _, hs_id, _) in assignments.items():
            self._last_plan_stamp[uid] = now_sec
            self._last_plan_hotspot[uid] = hs_id
            self._last_plan_position[uid] = list(pos)
            self._hotspot_last_assigned_stamp[hs_id] = now_sec
            self._hotspot_assignment_count[hs_id] = self._hotspot_assignment_count.get(hs_id, 0) + 1
        return assignments


class MissionPlanner(Node):
    def __init__(self) -> None:
        if rclpy is None:
            raise RuntimeError("rclpy is required to run mission_planner")
        super().__init__("mission_planner")
        self.fire_topic: str = self.declare_parameter("fire_topic", "/env/fire_state").value
        self.swarm_topic: str = self.declare_parameter("swarm_topic", "/swarm/state").value
        self.plan_topic: str = self.declare_parameter("plan_topic", "/swarm/mission_targets").value
        self.replan_hz: float = float(self.declare_parameter("replan_hz", 1.0).value)
        config = PlannerConfig(
            hotspot_per_uav=int(self.declare_parameter("hotspot_per_uav", 1).value),
            min_intensity=float(self.declare_parameter("min_intensity", 0.2).value),
            revisit_bonus=float(self.declare_parameter("revisit_bonus", 0.25).value),
            revisit_cycle_sec=float(self.declare_parameter("revisit_cycle_sec", 45.0).value),
            hold_time_sec=float(self.declare_parameter("hold_time_sec", 4.0).value),
            hold_distance_m=float(self.declare_parameter("hold_distance_m", 20.0).value),
            hold_bonus=float(self.declare_parameter("hold_bonus", 0.35).value),
            reassign_penalty=float(self.declare_parameter("reassign_penalty", 0.60).value),
            switch_cooldown_sec=float(self.declare_parameter("switch_cooldown_sec", 12.0).value),
            switch_cooldown_penalty=float(self.declare_parameter("switch_cooldown_penalty", 0.45).value),
            distance_cost_scale=float(self.declare_parameter("distance_cost_scale", 0.0012).value),
            coverage_min_distance_m=float(self.declare_parameter("coverage_min_distance_m", 90.0).value),
            coverage_density_penalty=float(self.declare_parameter("coverage_density_penalty", 0.0006).value),
            coverage_recent_window_sec=float(self.declare_parameter("coverage_recent_window_sec", 18.0).value),
            coverage_repeat_penalty=float(self.declare_parameter("coverage_repeat_penalty", 0.30).value),
            hotspot_decay_per_visit=float(self.declare_parameter("hotspot_decay_per_visit", 0.08).value),
            hotspot_decay_cap=float(self.declare_parameter("hotspot_decay_cap", 0.40).value),
            state_ttl_sec=float(self.declare_parameter("state_ttl_sec", 120.0).value),
        )
        self.core = MissionPlannerCore(config)

        self.last_fire: Optional[FireState] = None
        self.last_swarm: Optional[SwarmState] = None

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

    def _tick(self) -> None:
        if self.last_fire is None or self.last_swarm is None:
            return
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        hotspots = [h for h in self.last_fire.hotspots]
        uavs = self.last_swarm.uavs
        if not hotspots or not uavs:
            return
        assignments = self.core.build_plan(now_sec, hotspots, list(uavs))

        if not assignments:
            return

        out = MissionPlan()
        out.stamp = self.get_clock().now().to_msg()
        targets: List[MissionTarget] = []
        for uid, (pos, pri, hs_id, reason) in assignments.items():
            t = MissionTarget()
            t.uav_id = uid
            t.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            t.priority = float(pri)
            t.reason = reason
            targets.append(t)
        out.targets = targets
        self.pub_plan.publish(out)


def main() -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is required to run mission_planner")
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
