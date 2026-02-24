#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, Hotspot


@dataclass
class RawHotspot:
    source_time_s: float
    hotspot_id: str
    x_m: float
    y_m: float
    z_m: float
    intensity: float
    spread_mps: float


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _to_float(raw: object, default: float = 0.0) -> float:
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


class FireAdapterFds(Node):
    def __init__(self) -> None:
        super().__init__("fire_adapter_fds")
        self.output_topic: str = self.declare_parameter("output_topic", "/env/fire_state").value
        self.publish_hz: float = float(self.declare_parameter("publish_hz", 1.0).value)
        self.input_path: str = self.declare_parameter("input_path", "").value
        self.input_format: str = self.declare_parameter("input_format", "csv").value
        self.time_mode: str = self.declare_parameter("time_mode", "source_offset").value
        self.origin_lat: float = float(self.declare_parameter("origin_lat", 39.9042).value)
        self.origin_lon: float = float(self.declare_parameter("origin_lon", 116.4074).value)
        self.origin_alt_m: float = float(self.declare_parameter("origin_alt_m", 0.0).value)
        self.xy_swap: bool = bool(self.declare_parameter("xy_swap", False).value)
        self.has_header: bool = bool(self.declare_parameter("has_header", True).value)
        self.republish_last_on_stale: bool = bool(
            self.declare_parameter("republish_last_on_stale", True).value
        )

        self.publisher = self.create_publisher(FireState, self.output_topic, 10)
        self.first_source_time_s: Optional[float] = None
        self.first_ros_time_s: Optional[float] = None
        self.last_payload: Optional[List[RawHotspot]] = None
        self.last_file_state: Tuple[int, int] = (-1, -1)

        period = 1.0 / max(0.2, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            "fire_adapter_fds started output=%s input=%s format=%s time_mode=%s hz=%.2f",
            self.output_topic,
            self.input_path if self.input_path else "<empty>",
            self.input_format,
            self.time_mode,
            self.publish_hz,
        )

    def _tick(self) -> None:
        payload = self._load_latest_hotspots()
        if payload is None:
            if self.republish_last_on_stale and self.last_payload:
                self._publish(self.last_payload)
            return
        self.last_payload = payload
        self._publish(payload)

    def _load_latest_hotspots(self) -> Optional[List[RawHotspot]]:
        if not self.input_path:
            self.get_logger().warn("input_path is empty, skip", throttle_duration_sec=5.0)
            return None
        p = Path(self.input_path)
        if not p.exists():
            self.get_logger().warn("input file not found: %s", str(p), throttle_duration_sec=5.0)
            return None
        try:
            st = p.stat()
        except OSError as ex:
            self.get_logger().warn("stat file failed: %s", str(ex), throttle_duration_sec=5.0)
            return None

        file_state = (int(st.st_mtime_ns), int(st.st_size))
        if file_state == self.last_file_state and self.last_payload is not None:
            return None

        try:
            if self.input_format == "jsonl":
                rows = self._parse_jsonl(p)
            else:
                rows = self._parse_csv(p)
        except Exception as ex:
            self.get_logger().error("parse input failed: %s", str(ex))
            return None

        if not rows:
            self.get_logger().warn("no valid hotspot rows from %s", str(p), throttle_duration_sec=5.0)
            return None

        latest_ts = max(row.source_time_s for row in rows)
        latest_rows = [row for row in rows if abs(row.source_time_s - latest_ts) < 1e-6]
        self.last_file_state = file_state
        return latest_rows

    def _parse_csv(self, p: Path) -> List[RawHotspot]:
        rows: List[RawHotspot] = []
        with p.open("r", encoding="utf-8") as f:
            if self.has_header:
                reader = csv.DictReader(f)
                for idx, raw in enumerate(reader):
                    if raw is None:
                        continue
                    row = self._to_row(raw, idx + 1)
                    if row is not None:
                        rows.append(row)
            else:
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
                    row = self._to_row(raw, idx + 1)
                    if row is not None:
                        rows.append(row)
        return rows

    def _parse_jsonl(self, p: Path) -> List[RawHotspot]:
        rows: List[RawHotspot] = []
        with p.open("r", encoding="utf-8") as f:
            for idx, line in enumerate(f):
                line = line.strip()
                if not line:
                    continue
                raw = json.loads(line)
                if not isinstance(raw, dict):
                    continue
                row = self._to_row(raw, idx + 1)
                if row is not None:
                    rows.append(row)
        return rows

    def _to_row(self, raw: Dict[str, object], row_no: int) -> Optional[RawHotspot]:
        source_time_s = _to_float(raw.get("time_s", raw.get("time", 0.0)))
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

    def _xy_to_wgs84(self, x_m: float, y_m: float, z_m: float) -> Tuple[float, float, float]:
        east_m = y_m if self.xy_swap else x_m
        north_m = x_m if self.xy_swap else y_m
        dlat = north_m / 111000.0
        lon_scale = max(1e-6, math.cos(math.radians(self.origin_lat)))
        dlon = east_m / (111000.0 * lon_scale)
        lat = self.origin_lat + dlat
        lon = self.origin_lon + dlon
        alt = self.origin_alt_m + z_m
        return lat, lon, alt

    def _source_to_time_msg(self, source_time_s: float):
        if self.time_mode == "ros_now":
            return self.get_clock().now().to_msg()

        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.first_source_time_s is None:
            self.first_source_time_s = source_time_s
            self.first_ros_time_s = now_s

        assert self.first_ros_time_s is not None
        assert self.first_source_time_s is not None
        out_s = self.first_ros_time_s + max(0.0, source_time_s - self.first_source_time_s)
        sec = int(out_s)
        nanosec = int((out_s - sec) * 1e9)
        t = self.get_clock().now().to_msg()
        t.sec = sec
        t.nanosec = nanosec
        return t

    def _publish(self, hotspots_raw: List[RawHotspot]) -> None:
        msg = FireState()
        ts = hotspots_raw[0].source_time_s if hotspots_raw else 0.0
        msg.stamp = self._source_to_time_msg(ts)
        hotspots: List[Hotspot] = []
        for row in hotspots_raw:
            hs = Hotspot()
            hs.id = row.hotspot_id
            lat, lon, alt = self._xy_to_wgs84(row.x_m, row.y_m, row.z_m)
            hs.position = [lat, lon, alt]
            hs.intensity = float(row.intensity)
            hs.spread_mps = float(row.spread_mps)
            hotspots.append(hs)
        msg.hotspots = hotspots
        self.publisher.publish(msg)


def main() -> None:
    os.environ.setdefault("ROS_LOG_DIR", "/tmp/roslog")
    Path(os.environ["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = FireAdapterFds()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
