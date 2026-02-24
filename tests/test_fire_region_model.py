#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from fire_region_model import build_fire_regions  # noqa: E402


def assert_true(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def sample_hotspots() -> list[dict]:
    return [
        {
            "id": "hotspot_a",
            "position": [39.9042, 116.4074, 100.0],
            "intensity": 0.9,
            "spread_mps": 0.8,
        },
        {
            "id": "hotspot_b",
            "position": [39.9045, 116.4077, 100.0],
            "intensity": 0.7,
            "spread_mps": 0.6,
        },
        {
            "id": "hotspot_far",
            "position": [39.9120, 116.4200, 100.0],
            "intensity": 0.8,
            "spread_mps": 0.4,
        },
    ]


def test_cluster_and_growth() -> None:
    first_seen: dict[str, int] = {}
    hs = sample_hotspots()
    regions_t0 = build_fire_regions(hs, now_ms=1_000_000, first_seen_ms=first_seen, merge_distance_m=320.0)
    assert_true(len(regions_t0) == 2, "应形成两个火区（近处聚合 + 远处单独）")
    max_r0 = max(float(r["radius_m"]) for r in regions_t0)

    regions_t1 = build_fire_regions(hs, now_ms=1_020_000, first_seen_ms=first_seen, merge_distance_m=320.0)
    max_r1 = max(float(r["radius_m"]) for r in regions_t1)
    assert_true(max_r1 > max_r0, "随时间推进火区半径应增大")


def test_empty_input() -> None:
    first_seen: dict[str, int] = {}
    regions = build_fire_regions([], now_ms=10, first_seen_ms=first_seen)
    assert_true(regions == [], "空输入应返回空火区")


def main() -> None:
    test_cluster_and_growth()
    test_empty_input()
    print("test_fire_region_model: PASS")


if __name__ == "__main__":
    main()
