#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from mission_planner import MissionPlannerCore, PlannerConfig  # noqa: E402


def assert_true(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def make_hotspot(hotspot_id: str, lat: float, lon: float, intensity: float) -> SimpleNamespace:
    return SimpleNamespace(id=hotspot_id, position=[lat, lon, 100.0], intensity=float(intensity))


def make_uav(uav_id: str, lat: float, lon: float) -> SimpleNamespace:
    return SimpleNamespace(id=uav_id, position=[lat, lon, 100.0])


def planned_hotspot_id(result: dict[str, tuple[list[float], float, str, str]], uav_id: str) -> str:
    return result[uav_id][2]


def count_switches(picks: list[str]) -> int:
    switches = 0
    for prev, cur in zip(picks, picks[1:]):
        if prev != cur:
            switches += 1
    return switches


def print_metric(name: str, **values: float | int | str) -> None:
    metrics = " ".join(f"{key}={value}" for key, value in values.items())
    print(f"[test_mission_planner][metric] {name} {metrics}")


def test_single_hotspot_stays_stable() -> None:
    core = MissionPlannerCore(
        PlannerConfig(
            revisit_bonus=0.0,
            hold_time_sec=0.0,
            hold_bonus=0.0,
            reassign_penalty=0.0,
            switch_cooldown_sec=0.0,
            switch_cooldown_penalty=0.0,
            coverage_repeat_penalty=0.0,
            hotspot_decay_per_visit=0.0,
            hotspot_decay_cap=0.0,
        )
    )
    uav = make_uav("u1", 39.9000, 116.4000)
    picks: list[str] = []
    for now_sec in (0.0, 5.0, 10.0):
        result = core.build_plan(
            now_sec,
            [make_hotspot("hs_only", 39.9000, 116.4000, 0.90)],
            [uav],
        )
        picks.append(planned_hotspot_id(result, "u1"))

    switches = count_switches(picks)
    assert_true(picks == ["hs_only", "hs_only", "hs_only"], "单热点场景应持续锁定同一目标")
    assert_true(switches == 0, "单热点场景不应产生任务切换")
    print_metric("single_hotspot", switches=switches, assignments=",".join(picks))


def test_switch_cooldown_reduces_jitter() -> None:
    core = MissionPlannerCore(
        PlannerConfig(
            revisit_bonus=0.0,
            hold_time_sec=0.0,
            reassign_penalty=0.0,
            switch_cooldown_sec=12.0,
            switch_cooldown_penalty=0.75,
            coverage_repeat_penalty=0.0,
            hotspot_decay_per_visit=0.0,
            hotspot_decay_cap=0.0,
        )
    )
    uav = make_uav("u1", 39.9000, 116.4000)
    first = core.build_plan(
        0.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 0.80),
            make_hotspot("hs_b", 39.9000, 116.4010, 0.90),
        ],
        [uav],
    )
    second = core.build_plan(
        5.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 0.80),
            make_hotspot("hs_b", 39.9000, 116.4010, 1.15),
        ],
        [uav],
    )
    assert_true(planned_hotspot_id(first, "u1") == "hs_a", "首轮应先选择更近的目标")
    assert_true(planned_hotspot_id(second, "u1") == "hs_a", "冷却窗口内不应因小幅收益变化频繁改派")
    print_metric("hotspot_change", switches=count_switches(["hs_a", "hs_a"]), revisit_gap_s=5.0)


def test_recent_coverage_penalty_reduces_repeat_assignment() -> None:
    core = MissionPlannerCore(
        PlannerConfig(
            revisit_bonus=0.0,
            hold_time_sec=0.0,
            reassign_penalty=0.0,
            switch_cooldown_sec=0.0,
            switch_cooldown_penalty=0.0,
            coverage_recent_window_sec=30.0,
            coverage_repeat_penalty=0.45,
            hotspot_decay_per_visit=0.18,
            hotspot_decay_cap=0.36,
        )
    )
    uav = make_uav("u1", 39.9000, 116.4000)
    first = core.build_plan(
        0.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 1.00),
            make_hotspot("hs_b", 39.9000, 116.4000, 0.92),
        ],
        [uav],
    )
    second = core.build_plan(
        6.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 1.00),
            make_hotspot("hs_b", 39.9000, 116.4000, 0.92),
        ],
        [uav],
    )
    assert_true(planned_hotspot_id(first, "u1") == "hs_a", "首轮应命中更高强度热点")
    assert_true(planned_hotspot_id(second, "u1") == "hs_b", "近期已覆盖热点应被抑制，避免重复覆盖")
    print_metric("repeat_suppression", repeat_assignments=0, revisit_gap_s=6.0)


def test_multi_uav_spreads_assignments_across_nearby_hotspots() -> None:
    core = MissionPlannerCore(
        PlannerConfig(
            revisit_bonus=0.0,
            hold_time_sec=0.0,
            hold_bonus=0.0,
            reassign_penalty=0.0,
            switch_cooldown_sec=0.0,
            switch_cooldown_penalty=0.0,
            coverage_min_distance_m=90.0,
            coverage_density_penalty=0.013,
            coverage_repeat_penalty=0.0,
            hotspot_decay_per_visit=0.0,
            hotspot_decay_cap=0.0,
        )
    )
    result = core.build_plan(
        0.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 1.00),
            make_hotspot("hs_b", 39.9000, 116.4001, 0.99),
            make_hotspot("hs_c", 39.9000, 116.4020, 0.92),
        ],
        [
            make_uav("u1", 39.9000, 116.4000),
            make_uav("u2", 39.9000, 116.4000),
        ],
    )
    assert_true(planned_hotspot_id(result, "u1") == "hs_a", "首架 UAV 应锁定最优热点")
    assert_true(planned_hotspot_id(result, "u2") == "hs_c", "第二架 UAV 应被引导到更分散的热点，避免邻近重复覆盖")
    print_metric(
        "multi_hotspot",
        assigned_u1=planned_hotspot_id(result, "u1"),
        assigned_u2=planned_hotspot_id(result, "u2"),
        repeat_assignments=0,
    )


def test_state_ttl_prunes_history_before_replan() -> None:
    core = MissionPlannerCore(
        PlannerConfig(
            revisit_bonus=0.0,
            hold_time_sec=0.0,
            reassign_penalty=0.0,
            switch_cooldown_sec=0.0,
            switch_cooldown_penalty=0.0,
            coverage_recent_window_sec=30.0,
            coverage_repeat_penalty=0.55,
            hotspot_decay_per_visit=0.25,
            hotspot_decay_cap=0.50,
            state_ttl_sec=3.0,
        )
    )
    uav = make_uav("u1", 39.9000, 116.4000)
    first = core.build_plan(
        0.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 1.00),
            make_hotspot("hs_b", 39.9000, 116.4000, 0.92),
        ],
        [uav],
    )
    second = core.build_plan(
        5.0,
        [
            make_hotspot("hs_a", 39.9000, 116.4000, 1.00),
            make_hotspot("hs_b", 39.9000, 116.4000, 0.92),
        ],
        [uav],
    )
    assert_true(planned_hotspot_id(first, "u1") == "hs_a", "首轮应命中更高强度热点")
    assert_true(planned_hotspot_id(second, "u1") == "hs_a", "状态过期后应清除历史惩罚并恢复按当前得分选择")


def main() -> None:
    test_single_hotspot_stays_stable()
    test_switch_cooldown_reduces_jitter()
    test_recent_coverage_penalty_reduces_repeat_assignment()
    test_multi_uav_spreads_assignments_across_nearby_hotspots()
    test_state_ttl_prunes_history_before_replan()
    print("test_mission_planner: PASS")


if __name__ == "__main__":
    main()
