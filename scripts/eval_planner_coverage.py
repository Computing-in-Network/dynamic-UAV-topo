#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class Hotspot:
    id: str
    pos: List[float]
    intensity: float


@dataclass
class Uav:
    id: str
    pos: List[float]


def dist_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    dlon = (a[1] - b[1]) * 111000.0
    dalt = a[2] - b[2]
    return math.sqrt(dlat * dlat + dlon * dlon + dalt * dalt)


def make_scene(seed: int, uav_n: int, hotspot_n: int) -> Tuple[List[Uav], List[Hotspot]]:
    rnd = random.Random(seed)
    base_lat, base_lon, base_alt = 39.9042, 116.4074, 120.0

    uavs = []
    for i in range(uav_n):
        lat = base_lat + rnd.uniform(-0.0025, 0.0025)
        lon = base_lon + rnd.uniform(-0.0025, 0.0025)
        alt = base_alt + rnd.uniform(-30.0, 30.0)
        uavs.append(Uav(id=f"uav_{i+1}", pos=[lat, lon, alt]))

    hotspots = []
    for i in range(hotspot_n):
        lat = base_lat + rnd.uniform(-0.0030, 0.0030)
        lon = base_lon + rnd.uniform(-0.0030, 0.0030)
        alt = base_alt + rnd.uniform(-10.0, 20.0)
        inten = rnd.uniform(0.35, 1.0)
        hotspots.append(Hotspot(id=f"h{i+1}", pos=[lat, lon, alt], intensity=inten))

    return uavs, hotspots


def evolve_hotspots(hs: List[Hotspot], tick: int, rnd: random.Random) -> None:
    for h in hs:
        h.pos[0] += rnd.uniform(-0.00008, 0.00008)
        h.pos[1] += rnd.uniform(-0.00008, 0.00008)
        h.intensity = max(0.2, min(1.0, h.intensity + rnd.uniform(-0.05, 0.08)))
        if tick % 12 == 0:
            h.intensity = min(1.0, h.intensity + 0.08)


def plan_greedy(uavs: List[Uav], hotspots: List[Hotspot]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    used_uav = set()
    for h in sorted(hotspots, key=lambda x: x.intensity, reverse=True):
        best_u = None
        best_d = None
        for u in uavs:
            if u.id in used_uav:
                continue
            d = dist_m(u.pos, h.pos)
            if best_d is None or d < best_d:
                best_d = d
                best_u = u
        if best_u is None:
            continue
        out[best_u.id] = h.id
        used_uav.add(best_u.id)
        if len(used_uav) >= len(uavs):
            break
    return out


def plan_coverage(
    uavs: List[Uav],
    hotspots: List[Hotspot],
    now_s: float,
    last_visit: Dict[str, float],
    last_uav_hotspot: Dict[str, str],
    revisit_sec: float,
) -> Dict[str, str]:
    remain_u = {u.id: u for u in uavs}
    out: Dict[str, str] = {}

    def hotspot_priority(h: Hotspot) -> float:
        lv = last_visit.get(h.id)
        unseen = lv is None
        revisit_score = 1.0 if unseen else min(1.0, max(0.0, (now_s - lv) / max(1e-3, revisit_sec)))
        recent = 1.0 - revisit_score
        decayed_intensity = h.intensity * (1.0 - 0.45 * recent)
        return 0.55 * decayed_intensity + 0.45 * revisit_score + (0.20 if unseen else 0.0)

    ranked = []
    for h in hotspots:
        pri = hotspot_priority(h)
        lv = last_visit.get(h.id)
        unseen = lv is None
        revisit_score = 1.0 if unseen else min(1.0, max(0.0, (now_s - lv) / max(1e-3, revisit_sec)))
        urgency = 0.65 * revisit_score + 0.35 * pri
        ranked.append((urgency, h))
    ranked.sort(key=lambda x: x[0], reverse=True)

    selected = [h for _, h in ranked[: len(uavs)]]
    selected_by_id = {h.id: h for h in selected}

    for h in selected:
        if h.id not in selected_by_id:
            continue
        if not remain_u:
            break
        best = None
        for u in remain_u.values():
            pri = hotspot_priority(h)
            d = dist_m(u.pos, h.pos)
            dp = 0.45 * min(1.0, d / 450.0)
            continuity = 0.18 if last_uav_hotspot.get(u.id) == h.id else 0.0
            score = pri - dp + continuity
            if best is None or score > best[0]:
                best = (score, u.id, h.id)
        if best is None:
            continue
        _, uid, hid = best
        out[uid] = hid
        remain_u.pop(uid, None)
    return out


def run(name: str, ticks: int, seed: int, uav_n: int, hotspot_n: int) -> Dict[str, float]:
    uavs, hotspots = make_scene(seed, uav_n, hotspot_n)
    rnd = random.Random(seed + 17)
    last_visit: Dict[str, float] = {}
    last_uav_hotspot: Dict[str, str] = {}

    coverage_hits = 0
    total_targets = 0
    revisit_overdue_hits = 0
    revisit_overdue_total = 0
    overdue_active = set()
    jitter_sum_m = 0.0
    jitter_n = 0
    visit_hist: Dict[str, List[int]] = {f"h{i+1}": [] for i in range(hotspot_n)}
    recent_assign: List[set] = []
    window_cov_sum = 0.0
    window_cov_n = 0

    for t in range(1, ticks + 1):
        now_s = float(t)
        evolve_hotspots(hotspots, t, rnd)
        hotspot_by_id = {h.id: h for h in hotspots}

        if name == "greedy":
            assign = plan_greedy(uavs, hotspots)
        else:
            assign = plan_coverage(uavs, hotspots, now_s, last_visit, last_uav_hotspot, revisit_sec=12.0)

        seen_in_tick = set(assign.values())
        for hid in seen_in_tick:
            visit_hist[hid].append(t)
        coverage_hits += len(seen_in_tick)
        total_targets += len(hotspots)
        recent_assign.append(seen_in_tick)
        if len(recent_assign) > 12:
            recent_assign.pop(0)
        if len(recent_assign) == 12:
            union = set()
            for s in recent_assign:
                union.update(s)
            window_cov_sum += len(union) / max(1, len(hotspots))
            window_cov_n += 1

        for hid in hotspots:
            lv = last_visit.get(hid.id)
            if lv is None:
                continue
            if (now_s - lv) > 12.0 and hid.id not in overdue_active:
                overdue_active.add(hid.id)
                revisit_overdue_total += 1

        for uid, hid in assign.items():
            h = hotspot_by_id[hid]
            u = next(x for x in uavs if x.id == uid)
            d = dist_m(u.pos, h.pos)
            step = min(65.0, d)
            if d > 1e-3:
                k = step / d
                old = list(u.pos)
                u.pos[0] += (h.pos[0] - u.pos[0]) * k
                u.pos[1] += (h.pos[1] - u.pos[1]) * k
                u.pos[2] += (h.pos[2] - u.pos[2]) * k
                jitter_sum_m += dist_m(old, u.pos)
                jitter_n += 1

            if hid in overdue_active:
                revisit_overdue_hits += 1
                overdue_active.remove(hid)
            last_visit[hid] = now_s

        last_uav_hotspot = dict(assign)

    coverage_rate = coverage_hits / max(1, total_targets)
    revisit_recovery = revisit_overdue_hits / max(1, revisit_overdue_total)
    jitter_avg = jitter_sum_m / max(1, jitter_n)
    window_coverage_rate = window_cov_sum / max(1, window_cov_n)
    gaps = []
    for times in visit_hist.values():
        if not times:
            gaps.append(float(ticks))
            continue
        prev = 0
        for ts in times:
            gaps.append(float(ts - prev))
            prev = ts
        gaps.append(float(ticks - prev))
    gaps.sort()
    gap_p95 = gaps[min(len(gaps) - 1, int(0.95 * (len(gaps) - 1)))] if gaps else 0.0
    return {
        "coverage_rate": coverage_rate,
        "revisit_recovery": revisit_recovery,
        "jitter_avg_m": jitter_avg,
        "window_coverage_rate": window_coverage_rate,
        "revisit_gap_p95_tick": gap_p95,
    }


def main() -> None:
    p = argparse.ArgumentParser(description="Compare greedy vs coverage planner strategy")
    p.add_argument("--ticks", type=int, default=120)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--uavs", type=int, default=8)
    p.add_argument("--hotspots", type=int, default=10)
    args = p.parse_args()

    g = run("greedy", args.ticks, args.seed, args.uavs, args.hotspots)
    c = run("coverage", args.ticks, args.seed, args.uavs, args.hotspots)

    print("=== Planner Compare ===")
    print(f"scene: ticks={args.ticks} seed={args.seed} uavs={args.uavs} hotspots={args.hotspots}")
    print(
        f"greedy   coverage_rate={g['coverage_rate']:.3f} "
        f"window_cov={g['window_coverage_rate']:.3f} "
        f"revisit_gap_p95={g['revisit_gap_p95_tick']:.1f} "
        f"jitter_avg_m={g['jitter_avg_m']:.2f}"
    )
    print(
        f"coverage coverage_rate={c['coverage_rate']:.3f} "
        f"window_cov={c['window_coverage_rate']:.3f} "
        f"revisit_gap_p95={c['revisit_gap_p95_tick']:.1f} "
        f"jitter_avg_m={c['jitter_avg_m']:.2f}"
    )
    print("--- delta (coverage - greedy) ---")
    print(f"coverage_rate={c['coverage_rate'] - g['coverage_rate']:+.3f}")
    print(f"window_cov={c['window_coverage_rate'] - g['window_coverage_rate']:+.3f}")
    print(f"revisit_gap_p95={c['revisit_gap_p95_tick'] - g['revisit_gap_p95_tick']:+.1f}")
    print(f"jitter_avg_m={c['jitter_avg_m'] - g['jitter_avg_m']:+.2f}")


if __name__ == "__main__":
    main()
