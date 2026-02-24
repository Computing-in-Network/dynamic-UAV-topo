#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Iterable, List, Tuple


def distance_m(a: List[float], b: List[float]) -> float:
    dlat = (a[0] - b[0]) * 111000.0
    mid_lat = (a[0] + b[0]) * 0.5
    dlon = (a[1] - b[1]) * 111000.0 * math.cos(math.radians(mid_lat))
    dalt = a[2] - b[2]
    return math.sqrt(dlat * dlat + dlon * dlon + dalt * dalt)


def evaluate_sync(
    pairs: Iterable[Tuple[int, List[float], int, List[float]]],
    max_dist_m: float,
    max_time_diff_ms: int,
) -> dict:
    rows = list(pairs)
    if not rows:
        return {
            "ok": False,
            "samples": 0,
            "max_dist_m": 0.0,
            "max_dt_ms": 0,
            "reason": "no_samples",
        }

    max_dist = 0.0
    max_dt = 0
    for vis_ts, vis_pos, focus_ts, focus_pos in rows:
        d = distance_m(vis_pos, focus_pos)
        dt = abs(int(vis_ts) - int(focus_ts))
        max_dist = max(max_dist, d)
        max_dt = max(max_dt, dt)

    ok = (max_dist <= max_dist_m) and (max_dt <= max_time_diff_ms)
    return {
        "ok": ok,
        "samples": len(rows),
        "max_dist_m": max_dist,
        "max_dt_ms": max_dt,
        "reason": "ok" if ok else "threshold_exceeded",
    }
