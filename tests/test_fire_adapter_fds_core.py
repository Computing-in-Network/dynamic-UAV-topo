#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

from fire_adapter_fds_core import group_hotspot_rows, load_hotspot_rows, load_region_rows  # noqa: E402


def test_load_hotspot_rows_csv() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "hotspots.csv"
        path.write_text(
            "time_s,id,x_m,y_m,z_m,intensity,spread_mps\n"
            "1.0,hs_a,10,20,3,1.2,0.4\n"
            "1.0,hs_b,11,21,4,-0.1,-1.0\n"
            "2.5,hs_c,12,22,5,0.6,0.2\n",
            encoding="utf-8",
        )
        rows = load_hotspot_rows(path, input_format="csv", has_header=True, source_time_scale=2.0)
        assert len(rows) == 3
        assert rows[0].source_time_s == 2.0
        assert rows[0].hotspot_id == "hs_a"
        assert rows[0].intensity == 1.0
        assert rows[1].intensity == 0.0
        assert rows[1].spread_mps == 0.0
        ts, grouped = group_hotspot_rows(rows)
        assert ts == [2.0, 5.0]
        assert len(grouped[0]) == 2
        assert grouped[1][0].hotspot_id == "hs_c"


def test_load_region_rows_jsonl() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "regions.jsonl"
        payload = [
            {
                "time_s": 3,
                "id": "r1",
                "intensity": 0.7,
                "spread_mps": 0.3,
                "vertices": [[0, 0, 0], [5, 0, 0], [5, 5, 0]],
            },
            {
                "time_s": 4,
                "id": "bad",
                "vertices": [[0, 0, 0], [1, 1, 0]],
            },
            {
                "time_s": 4,
                "id": "r2",
                "intensity": 2.0,
                "spread_mps": -2.0,
                "vertices": "0:0:0;2:0:0;2:2:0",
            },
        ]
        path.write_text("".join(json.dumps(row) + "\n" for row in payload), encoding="utf-8")
        rows = load_region_rows(path, input_format="jsonl", source_time_scale=0.5)
        assert len(rows) == 2
        assert rows[0].region_id == "r1"
        assert rows[0].source_time_s == 1.5
        assert len(rows[0].points_xyz) == 3
        assert rows[1].region_id == "r2"
        assert rows[1].source_time_s == 2.0
        assert rows[1].intensity == 1.0
        assert rows[1].spread_mps == 0.0


def test_load_hotspot_rows_fds_csv_profile() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "fds_cells.csv"
        path.write_text(
            "time,grid_i,grid_j,grid_k,center_x_m,center_y_m,center_z_m,hrr_kw,front_speed_mps\n"
            "3,10,20,0,15,25,1,1500,0.5\n"
            "4,11,21,0,18,28,1,3000,0.8\n",
            encoding="utf-8",
        )
        rows = load_hotspot_rows(path, input_format="csv", input_profile="fds_csv")
        assert len(rows) == 2
        assert rows[0].hotspot_id == "cell_10_20_0"
        assert rows[0].intensity == 0.5
        assert rows[1].intensity == 1.0
        assert rows[1].spread_mps == 0.8


def test_load_region_rows_fds_jsonl_profile() -> None:
    with tempfile.TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "fds_regions.jsonl"
        payload = [
            {
                "time": 2,
                "region_id": "mesh_a",
                "hrr_rel": 0.9,
                "front_speed_mps": 0.4,
                "points": [
                    {"x": 0, "y": 0, "z": 0},
                    {"x": 2, "y": 0, "z": 0},
                    {"x": 2, "y": 2, "z": 0},
                ],
            }
        ]
        path.write_text("".join(json.dumps(row) + "\n" for row in payload), encoding="utf-8")
        rows = load_region_rows(path, input_format="jsonl", input_profile="fds_jsonl")
        assert len(rows) == 1
        assert rows[0].region_id == "mesh_a"
        assert rows[0].intensity == 0.9
        assert rows[0].spread_mps == 0.4
        assert rows[0].points_xyz[2] == (2.0, 2.0, 0.0)


def main() -> None:
    test_load_hotspot_rows_csv()
    test_load_region_rows_jsonl()
    test_load_hotspot_rows_fds_csv_profile()
    test_load_region_rows_fds_jsonl_profile()
    print("test_fire_adapter_fds_core: PASS")


if __name__ == "__main__":
    main()
