#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Dict, List, Tuple


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _distance_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    dlat = (a_lat - b_lat) * 111000.0
    mid_lat = (a_lat + b_lat) * 0.5
    dlon = (a_lon - b_lon) * 111000.0 * math.cos(math.radians(mid_lat))
    return math.hypot(dlat, dlon)


def _single_radius_m(hs: Dict[str, object], age_s: float) -> float:
    intensity = float(hs.get("intensity", 0.0))
    spread = float(hs.get("spread_mps", 0.0))
    base = 80.0 + intensity * 180.0
    grow = spread * max(0.0, age_s)
    return _clamp(base + grow, 80.0, 2500.0)


def build_fire_regions(
    hotspots: List[Dict[str, object]],
    now_ms: int,
    first_seen_ms: Dict[str, int],
    merge_distance_m: float = 320.0,
) -> List[Dict[str, object]]:
    if not hotspots:
        return []

    n = len(hotspots)
    parent = list(range(n))

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        pa = find(a)
        pb = find(b)
        if pa != pb:
            parent[pb] = pa

    for i in range(n):
        ai = hotspots[i]
        alat = float(ai["position"][0])
        alon = float(ai["position"][1])
        for j in range(i + 1, n):
            bj = hotspots[j]
            blat = float(bj["position"][0])
            blon = float(bj["position"][1])
            if _distance_m(alat, alon, blat, blon) <= merge_distance_m:
                union(i, j)

    groups: Dict[int, List[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    regions: List[Dict[str, object]] = []
    for root, idxs in groups.items():
        members = [hotspots[i] for i in idxs]
        weighted_sum = 0.0
        c_lat = 0.0
        c_lon = 0.0
        c_alt = 0.0
        max_intensity = 0.0
        total_spread = 0.0
        member_ids: List[str] = []

        for hs in members:
            hid = str(hs.get("id", ""))
            member_ids.append(hid)
            w = max(0.1, float(hs.get("intensity", 0.0)))
            c_lat += float(hs["position"][0]) * w
            c_lon += float(hs["position"][1]) * w
            c_alt += float(hs["position"][2]) * w
            weighted_sum += w
            max_intensity = max(max_intensity, float(hs.get("intensity", 0.0)))
            total_spread += float(hs.get("spread_mps", 0.0))

        c_lat /= weighted_sum
        c_lon /= weighted_sum
        c_alt /= weighted_sum

        max_radius = 0.0
        for hs in members:
            hid = str(hs.get("id", ""))
            if hid and hid not in first_seen_ms:
                first_seen_ms[hid] = now_ms
            seen_ms = first_seen_ms.get(hid, now_ms)
            age_s = max(0.0, (now_ms - seen_ms) / 1000.0)
            local_r = _single_radius_m(hs, age_s)
            dist = _distance_m(c_lat, c_lon, float(hs["position"][0]), float(hs["position"][1]))
            max_radius = max(max_radius, dist + local_r)

        avg_spread = total_spread / max(1, len(members))
        region_id = f"region_{root}"
        regions.append(
            {
                "id": region_id,
                "center": [c_lat, c_lon, c_alt],
                "radius_m": _clamp(max_radius, 120.0, 3200.0),
                "intensity": _clamp(max_intensity, 0.0, 1.0),
                "spread_mps": max(0.0, avg_spread),
                "member_count": len(members),
                "member_ids": member_ids,
            }
        )

    regions.sort(key=lambda x: float(x["radius_m"]), reverse=True)
    return regions
