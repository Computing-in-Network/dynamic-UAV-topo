#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, MissionPlan, MissionTarget, SwarmState


def distance_score_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return (dlat * dlat + dlon * dlon + dalt * dalt) ** 0.5


@dataclass
class PlannerConfig:
    mode: str
    min_intensity: float
    hotspot_per_uav: int
    coverage_revisit_sec: float
    coverage_priority_decay: float
    coverage_distance_weight: float
    coverage_intensity_weight: float
    coverage_revisit_weight: float
    coverage_unseen_bonus: float
    coverage_continuity_bonus: float
    coverage_distance_norm_m: float
    coverage_retarget_threshold_m: float


class MissionPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mission_planner")
        self.fire_topic: str = self.declare_parameter("fire_topic", "/env/fire_state").value
        self.swarm_topic: str = self.declare_parameter("swarm_topic", "/swarm/state").value
        self.plan_topic: str = self.declare_parameter("plan_topic", "/swarm/mission_targets").value
        self.replan_hz: float = float(self.declare_parameter("replan_hz", 1.0).value)

        self.cfg = PlannerConfig(
            mode=str(self.declare_parameter("planner_mode", "greedy").value).strip().lower(),
            hotspot_per_uav=int(self.declare_parameter("hotspot_per_uav", 3).value),
            min_intensity=float(self.declare_parameter("min_intensity", 0.2).value),
            coverage_revisit_sec=float(self.declare_parameter("coverage_revisit_sec", 12.0).value),
            coverage_priority_decay=float(self.declare_parameter("coverage_priority_decay", 0.45).value),
            coverage_distance_weight=float(self.declare_parameter("coverage_distance_weight", 0.45).value),
            coverage_intensity_weight=float(self.declare_parameter("coverage_intensity_weight", 0.55).value),
            coverage_revisit_weight=float(self.declare_parameter("coverage_revisit_weight", 0.45).value),
            coverage_unseen_bonus=float(self.declare_parameter("coverage_unseen_bonus", 0.20).value),
            coverage_continuity_bonus=float(self.declare_parameter("coverage_continuity_bonus", 0.18).value),
            coverage_distance_norm_m=float(self.declare_parameter("coverage_distance_norm_m", 450.0).value),
            coverage_retarget_threshold_m=float(self.declare_parameter("coverage_retarget_threshold_m", 0.0).value),
        )

        self.last_fire: Optional[FireState] = None
        self.last_swarm: Optional[SwarmState] = None
        self.hotspot_last_visit_sec: Dict[str, float] = {}
        self.uav_last_hotspot: Dict[str, str] = {}

        self.sub_fire = self.create_subscription(FireState, self.fire_topic, self._on_fire, 10)
        self.sub_swarm = self.create_subscription(SwarmState, self.swarm_topic, self._on_swarm, 10)
        self.pub_plan = self.create_publisher(MissionPlan, self.plan_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, self.replan_hz), self._tick)
        self.get_logger().info(
            "mission_planner started "
            f"mode={self.cfg.mode} fire={self.fire_topic} swarm={self.swarm_topic} plan={self.plan_topic} "
            f"revisit={self.cfg.coverage_revisit_sec:.1f}s"
        )

    def _on_fire(self, msg: FireState) -> None:
        self.last_fire = msg

    def _on_swarm(self, msg: SwarmState) -> None:
        self.last_swarm = msg

    def _tick(self) -> None:
        if self.last_fire is None or self.last_swarm is None:
            return
        hotspots = [h for h in self.last_fire.hotspots if h.intensity >= self.cfg.min_intensity]
        if not hotspots or not self.last_swarm.uavs:
            return

        now_sec = float(self.get_clock().now().nanoseconds) / 1_000_000_000.0
        uavs = list(self.last_swarm.uavs)

        if self.cfg.mode == "coverage":
            assignments = self._plan_coverage(uavs, hotspots, now_sec)
        else:
            assignments = self._plan_greedy(uavs, hotspots)

        if not assignments:
            return
        assignments = self._spread_assignments(assignments)

        out = MissionPlan()
        out.stamp = self.get_clock().now().to_msg()
        targets: List[MissionTarget] = []
        next_uav_last_hotspot: Dict[str, str] = {}
        for uid, hs_id, pos, pri, reason in assignments:
            t = MissionTarget()
            t.uav_id = uid
            t.position = [float(pos[0]), float(pos[1]), float(pos[2])]
            t.priority = float(pri)
            t.reason = reason
            targets.append(t)
            self.hotspot_last_visit_sec[hs_id] = now_sec
            next_uav_last_hotspot[uid] = hs_id

        self.uav_last_hotspot = next_uav_last_hotspot
        out.targets = targets
        self.pub_plan.publish(out)

    @staticmethod
    def _offset_wgs84(pos: List[float], dx_m: float, dy_m: float) -> List[float]:
        lat = float(pos[0]) + dy_m / 111000.0
        lon_scale = 111000.0 * max(0.15, math.cos(math.radians(float(pos[0]))))
        lon = float(pos[1]) + dx_m / lon_scale
        return [lat, lon, float(pos[2])]

    def _spread_assignments(
        self, assignments: List[Tuple[str, str, List[float], float, str]]
    ) -> List[Tuple[str, str, List[float], float, str]]:
        # When many UAVs follow one hotspot, spread them on a small orbit ring
        # to avoid visual/physical crowding at exactly the same coordinate.
        by_hotspot: Dict[str, List[Tuple[str, str, List[float], float, str]]] = {}
        for item in assignments:
            by_hotspot.setdefault(item[1], []).append(item)

        out: List[Tuple[str, str, List[float], float, str]] = []
        for hs_id, items in by_hotspot.items():
            if len(items) <= 1:
                out.extend(items)
                continue
            n = len(items)
            ring_r = min(160.0, 40.0 + 14.0 * n)
            for i, (uid, _hid, pos, pri, reason) in enumerate(items):
                a = (2.0 * math.pi * i) / n
                dx = math.cos(a) * ring_r
                dy = math.sin(a) * ring_r
                p = self._offset_wgs84(pos, dx, dy)
                out.append((uid, hs_id, p, pri, f"{reason}:orbit"))
        return out

    def _plan_greedy(self, uavs: List, hotspots: List) -> List[Tuple[str, str, List[float], float, str]]:
        hotspots_sorted = sorted(hotspots, key=lambda h: float(h.intensity), reverse=True)
        max_targets = min(len(uavs), max(1, self.cfg.hotspot_per_uav * len(hotspots_sorted)))
        max_load_per_hotspot = max(1, math.ceil(len(uavs) / max(1, len(hotspots_sorted))))
        assign_count: Dict[str, int] = {}
        out: List[Tuple[str, str, List[float], float, str]] = []
        for u in uavs:
            best = None
            for hs in hotspots_sorted:
                load = assign_count.get(hs.id, 0)
                if load > max_load_per_hotspot:
                    continue
                d = distance_score_m(list(u.position), list(hs.position))
                load_penalty = 0.18 * load
                score = float(hs.intensity) - 0.35 * min(1.0, d / max(1.0, self.cfg.coverage_distance_norm_m)) - load_penalty
                if best is None or score > best[0]:
                    best = (score, hs)
            if best is None:
                continue
            _, hs = best
            out.append((u.id, hs.id, list(hs.position), float(hs.intensity), f"track:{hs.id}:greedy_follow"))
            assign_count[hs.id] = assign_count.get(hs.id, 0) + 1
            if len(out) >= max_targets:
                break
        return out

    def _coverage_priority(self, hs, now_sec: float) -> Tuple[float, float, float]:
        intensity = float(hs.intensity)
        last = self.hotspot_last_visit_sec.get(hs.id)
        unseen = last is None

        revisit_score = 1.0 if unseen else min(1.0, max(0.0, (now_sec - last) / max(1e-3, self.cfg.coverage_revisit_sec)))
        recent_factor = 1.0 - revisit_score
        decayed_intensity = intensity * (1.0 - self.cfg.coverage_priority_decay * recent_factor)

        priority = (
            self.cfg.coverage_intensity_weight * decayed_intensity
            + self.cfg.coverage_revisit_weight * revisit_score
            + (self.cfg.coverage_unseen_bonus if unseen else 0.0)
        )
        return priority, revisit_score, decayed_intensity

    def _pair_score(self, u, hs, now_sec: float) -> Tuple[float, float]:
        priority, revisit_score, _ = self._coverage_priority(hs, now_sec)
        dist = distance_score_m(list(u.position), list(hs.position))
        dist_penalty = self.cfg.coverage_distance_weight * min(1.0, dist / max(1.0, self.cfg.coverage_distance_norm_m))

        continuity_bonus = 0.0
        if self.uav_last_hotspot.get(u.id) == hs.id:
            continuity_bonus = self.cfg.coverage_continuity_bonus

        score = priority - dist_penalty + continuity_bonus
        priority_hint = 0.5 * float(hs.intensity) + 0.5 * revisit_score
        return score, priority_hint

    def _plan_coverage(self, uavs: List, hotspots: List, now_sec: float) -> List[Tuple[str, str, List[float], float, str]]:
        max_targets = min(len(uavs), max(1, self.cfg.hotspot_per_uav * len(hotspots)))

        # Step-1: select hotspots by urgency first, to avoid long-term starvation.
        hotspot_ranked = []
        for hs in hotspots:
            priority, revisit_score, _ = self._coverage_priority(hs, now_sec)
            urgency = 0.65 * revisit_score + 0.35 * priority
            hotspot_ranked.append((urgency, hs))
        hotspot_ranked.sort(key=lambda x: x[0], reverse=True)
        selected = [hs for _, hs in hotspot_ranked[:max_targets]]
        selected_by_id = {hs.id: hs for hs in selected}
        hotspot_by_id = {hs.id: hs for hs in hotspots}

        # Step-2: assign best UAV for each selected hotspot.
        remaining_uav = {u.id: u for u in uavs}
        out: List[Tuple[str, str, List[float], float, str]] = []

        # Step-2a: keep previous assignment when retarget distance is small (jitter control).
        if self.cfg.coverage_retarget_threshold_m > 0.0:
            for uid in list(remaining_uav.keys()):
                last_hid = self.uav_last_hotspot.get(uid)
                if not last_hid:
                    continue
                hs = hotspot_by_id.get(last_hid)
                if hs is None:
                    continue
                u = remaining_uav[uid]
                d = distance_score_m(list(u.position), list(hs.position))
                if d > self.cfg.coverage_retarget_threshold_m:
                    continue
                _, priority_hint = self._pair_score(u, hs, now_sec)
                out.append((u.id, hs.id, list(hs.position), priority_hint, f"track:{hs.id}:coverage_hold"))
                remaining_uav.pop(uid, None)
                selected_by_id.pop(hs.id, None)
                if len(out) >= max_targets:
                    return out

        # Step-2b: fill remaining slots by urgency-ranked hotspots.
        for hs in selected:
            if hs.id not in selected_by_id:
                continue
            if not remaining_uav:
                break
            best = None
            for u in remaining_uav.values():
                score, priority_hint = self._pair_score(u, hs, now_sec)
                if best is None or score > best[0]:
                    best = (score, priority_hint, u)
            if best is None:
                continue

            _, priority_hint, u = best
            out.append((u.id, hs.id, list(hs.position), priority_hint, f"track:{hs.id}:coverage"))
            remaining_uav.pop(u.id, None)

        # Step-2c: if UAVs still remain, allow multi-UAV follow on urgent hotspots.
        if remaining_uav and len(out) < max_targets:
            follow_pool = selected if selected else hotspots
            assign_count: Dict[str, int] = {}
            for _, hs_id, _, _, _ in out:
                assign_count[hs_id] = assign_count.get(hs_id, 0) + 1
            max_load_per_hotspot = max(1, math.ceil(len(uavs) / max(1, len(follow_pool))))
            for u in list(remaining_uav.values()):
                best = None
                for hs in follow_pool:
                    load = assign_count.get(hs.id, 0)
                    if load > max_load_per_hotspot:
                        continue
                    score, priority_hint = self._pair_score(u, hs, now_sec)
                    score -= 0.20 * load
                    if best is None or score > best[0]:
                        best = (score, priority_hint, hs)
                if best is None:
                    continue
                _, priority_hint, hs = best
                out.append((u.id, hs.id, list(hs.position), priority_hint, f"track:{hs.id}:coverage_follow"))
                assign_count[hs.id] = assign_count.get(hs.id, 0) + 1
                if len(out) >= max_targets:
                    break

        return out


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
