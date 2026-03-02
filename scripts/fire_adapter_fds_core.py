from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass
class RawHotspot:
    source_time_s: float
    hotspot_id: str
    x_m: float
    y_m: float
    z_m: float
    intensity: float
    spread_mps: float


@dataclass
class RawRegion:
    source_time_s: float
    region_id: str
    points_xyz: List[Tuple[float, float, float]]
    intensity: float
    spread_mps: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _to_float(raw: object, default: float = 0.0) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def _pick_first(raw: Dict[str, object], keys: Sequence[str], default: object = None) -> object:
    for key in keys:
        if key in raw and raw.get(key) not in (None, ""):
            return raw.get(key)
    return default


def _normalize_hotspot_record(raw: Dict[str, object], row_no: int, input_profile: str) -> Dict[str, object]:
    if input_profile == "fds_csv":
        hrr_kw = _to_float(_pick_first(raw, ("hrr_kw", "hrr", "q_dot_kw"), 0.0), 0.0)
        temp_c = _to_float(_pick_first(raw, ("temp_c", "temperature_c"), 20.0), 20.0)
        if _pick_first(raw, ("intensity", "hrr_rel"), None) not in (None, ""):
            intensity = _to_float(_pick_first(raw, ("intensity", "hrr_rel"), 0.0), 0.0)
        elif hrr_kw > 0.0:
            intensity = hrr_kw / 3000.0
        else:
            intensity = max(0.0, (temp_c - 80.0) / 400.0)
        cell_id = _pick_first(raw, ("cell_id", "id", "mesh_cell_id"), "")
        if not cell_id:
            grid_i = str(_pick_first(raw, ("grid_i", "i"), "0"))
            grid_j = str(_pick_first(raw, ("grid_j", "j"), "0"))
            grid_k = str(_pick_first(raw, ("grid_k", "k"), "0"))
            cell_id = f"cell_{grid_i}_{grid_j}_{grid_k}"
        return {
            "time_s": _pick_first(raw, ("time_s", "time", "sim_time_s"), 0.0),
            "id": cell_id,
            "x_m": _pick_first(raw, ("center_x_m", "x_m", "x"), 0.0),
            "y_m": _pick_first(raw, ("center_y_m", "y_m", "y"), 0.0),
            "z_m": _pick_first(raw, ("center_z_m", "z_m", "z"), 0.0),
            "intensity": intensity,
            "spread_mps": _pick_first(raw, ("spread_mps", "front_speed_mps", "spread"), 0.0),
        }
    return raw


def _normalize_region_record(raw: Dict[str, object], row_no: int, input_profile: str) -> Dict[str, object]:
    if input_profile == "fds_jsonl":
        points = _pick_first(raw, ("points", "vertices", "boundary"), [])
        if isinstance(points, Sequence) and not isinstance(points, (str, bytes)):
            normalized_points = []
            for pt in points:
                if isinstance(pt, dict):
                    normalized_points.append(
                        [
                            _to_float(_pick_first(pt, ("x_m", "x"), 0.0), 0.0),
                            _to_float(_pick_first(pt, ("y_m", "y"), 0.0), 0.0),
                            _to_float(_pick_first(pt, ("z_m", "z"), 0.0), 0.0),
                        ]
                    )
                else:
                    normalized_points.append(pt)
        else:
            normalized_points = points
        return {
            "time_s": _pick_first(raw, ("time_s", "time", "sim_time_s"), 0.0),
            "id": _pick_first(raw, ("region_id", "id", "mesh_region_id"), f"region_{row_no}"),
            "intensity": _pick_first(raw, ("intensity", "hrr_rel"), 0.5),
            "spread_mps": _pick_first(raw, ("spread_mps", "front_speed_mps", "spread"), 0.0),
            "vertices": normalized_points,
        }
    return raw


def parse_hotspot_record(
    raw: Dict[str, object],
    row_no: int,
    source_time_scale: float = 1.0,
    input_profile: str = "normalized",
) -> RawHotspot:
    raw = _normalize_hotspot_record(raw, row_no, input_profile)
    source_time_s = _to_float(raw.get("time_s", raw.get("time", 0.0))) * max(1e-6, source_time_scale)
    hotspot_id = str(raw.get("id", raw.get("hotspot_id", f"hotspot_{row_no}")))
    x_m = _to_float(raw.get("x_m", raw.get("x", 0.0)))
    y_m = _to_float(raw.get("y_m", raw.get("y", 0.0)))
    z_m = _to_float(raw.get("z_m", raw.get("z", 0.0)))
    intensity = _clamp(_to_float(raw.get("intensity", 0.5)), 0.0, 1.0)
    spread_mps = max(0.0, _to_float(raw.get("spread_mps", raw.get("spread", 0.0))))
    return RawHotspot(
        source_time_s=source_time_s,
        hotspot_id=hotspot_id,
        x_m=x_m,
        y_m=y_m,
        z_m=z_m,
        intensity=intensity,
        spread_mps=spread_mps,
    )


def parse_region_record(
    raw: Dict[str, object],
    row_no: int,
    source_time_scale: float = 1.0,
    input_profile: str = "normalized",
) -> Optional[RawRegion]:
    raw = _normalize_region_record(raw, row_no, input_profile)
    source_time_s = _to_float(raw.get("time_s", raw.get("time", 0.0))) * max(1e-6, source_time_scale)
    region_id = str(raw.get("id", raw.get("region_id", f"region_{row_no}")))
    intensity = _clamp(_to_float(raw.get("intensity", 0.5)), 0.0, 1.0)
    spread_mps = max(0.0, _to_float(raw.get("spread_mps", raw.get("spread", 0.0))))

    points_xyz: List[Tuple[float, float, float]] = []
    vertices = raw.get("vertices", raw.get("boundary", []))
    if isinstance(vertices, str):
        for chunk in vertices.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            nums = [x.strip() for x in chunk.split(":")]
            if len(nums) < 2:
                continue
            x_m = _to_float(nums[0], 0.0)
            y_m = _to_float(nums[1], 0.0)
            z_m = _to_float(nums[2], 0.0) if len(nums) >= 3 else 0.0
            points_xyz.append((x_m, y_m, z_m))
    elif isinstance(vertices, Sequence) and not isinstance(vertices, (str, bytes)):
        for pt in vertices:
            if not isinstance(pt, Sequence) or isinstance(pt, (str, bytes)):
                continue
            if len(pt) < 2:
                continue
            x_m = _to_float(pt[0], 0.0)
            y_m = _to_float(pt[1], 0.0)
            z_m = _to_float(pt[2], 0.0) if len(pt) >= 3 else 0.0
            points_xyz.append((x_m, y_m, z_m))
    if len(points_xyz) < 3:
        return None

    return RawRegion(
        source_time_s=source_time_s,
        region_id=region_id,
        points_xyz=points_xyz,
        intensity=intensity,
        spread_mps=spread_mps,
    )


def load_hotspot_rows(
    path: Path,
    input_format: str = "csv",
    has_header: bool = True,
    source_time_scale: float = 1.0,
    input_profile: str = "normalized",
) -> List[RawHotspot]:
    rows: List[RawHotspot] = []
    with path.open("r", encoding="utf-8") as f:
        if input_format == "jsonl":
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                raw = json.loads(line)
                if not isinstance(raw, dict):
                    continue
                rows.append(parse_hotspot_record(raw, idx + 1, source_time_scale, input_profile))
            return rows

        if has_header:
            reader = csv.DictReader(f)
            for idx, raw in enumerate(reader):
                if raw is None:
                    continue
                rows.append(parse_hotspot_record(raw, idx + 1, source_time_scale, input_profile))
            return rows

        reader = csv.reader(f)
        for idx, cols in enumerate(reader):
            if not cols:
                continue
            raw = {
                "time_s": cols[0] if len(cols) > 0 else 0.0,
                "id": cols[1] if len(cols) > 1 else f"hotspot_{idx}",
                "x_m": cols[2] if len(cols) > 2 else 0.0,
                "y_m": cols[3] if len(cols) > 3 else 0.0,
                "z_m": cols[4] if len(cols) > 4 else 0.0,
                "intensity": cols[5] if len(cols) > 5 else 0.5,
                "spread_mps": cols[6] if len(cols) > 6 else 0.0,
            }
            rows.append(parse_hotspot_record(raw, idx + 1, source_time_scale, input_profile))
    return rows


def load_region_rows(
    path: Path,
    input_format: str = "jsonl",
    has_header: bool = True,
    source_time_scale: float = 1.0,
    input_profile: str = "normalized",
) -> List[RawRegion]:
    rows: List[RawRegion] = []
    with path.open("r", encoding="utf-8") as f:
        if input_format == "jsonl":
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                raw = json.loads(line)
                if not isinstance(raw, dict):
                    continue
                row = parse_region_record(raw, idx + 1, source_time_scale, input_profile)
                if row is not None:
                    rows.append(row)
            return rows

        if has_header:
            reader = csv.DictReader(f)
            for idx, raw in enumerate(reader):
                if raw is None:
                    continue
                row = parse_region_record(raw, idx + 1, source_time_scale, input_profile)
                if row is not None:
                    rows.append(row)
            return rows

        reader = csv.reader(f)
        for idx, cols in enumerate(reader):
            if not cols:
                continue
            raw = {
                "time_s": cols[0] if len(cols) > 0 else 0.0,
                "id": cols[1] if len(cols) > 1 else f"region_{idx}",
                "vertices": cols[2] if len(cols) > 2 else "",
                "intensity": cols[3] if len(cols) > 3 else 0.5,
                "spread_mps": cols[4] if len(cols) > 4 else 0.0,
            }
            row = parse_region_record(raw, idx + 1, source_time_scale, input_profile)
            if row is not None:
                rows.append(row)
    return rows


def group_hotspot_rows(rows: List[RawHotspot]) -> Tuple[List[float], List[List[RawHotspot]]]:
    grouped: Dict[float, List[RawHotspot]] = {}
    for row in rows:
        grouped.setdefault(row.source_time_s, []).append(row)
    ts = sorted(grouped.keys())
    return ts, [grouped[t] for t in ts]


def group_region_rows(rows: List[RawRegion]) -> Tuple[List[float], List[List[RawRegion]]]:
    grouped: Dict[float, List[RawRegion]] = {}
    for row in rows:
        grouped.setdefault(row.source_time_s, []).append(row)
    ts = sorted(grouped.keys())
    return ts, [grouped[t] for t in ts]
