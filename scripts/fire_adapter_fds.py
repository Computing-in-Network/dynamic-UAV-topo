#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import os
from bisect import bisect_right
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
        self.xy_yaw_deg: float = float(self.declare_parameter("xy_yaw_deg", 0.0).value)
        self.x_scale: float = float(self.declare_parameter("x_scale", 1.0).value)
        self.y_scale: float = float(self.declare_parameter("y_scale", 1.0).value)
        self.z_scale: float = float(self.declare_parameter("z_scale", 1.0).value)
        self.x_offset_m: float = float(self.declare_parameter("x_offset_m", 0.0).value)
        self.y_offset_m: float = float(self.declare_parameter("y_offset_m", 0.0).value)
        self.z_offset_m: float = float(self.declare_parameter("z_offset_m", 0.0).value)
        self.has_header: bool = bool(self.declare_parameter("has_header", True).value)
        self.source_time_scale: float = max(
            1e-6, float(self.declare_parameter("source_time_scale", 1.0).value)
        )
        self.playback_mode: str = self.declare_parameter("playback_mode", "timeline").value
        self.replay_speed: float = max(0.05, float(self.declare_parameter("replay_speed", 1.0).value))
        self.loop_timeline: bool = bool(self.declare_parameter("loop_timeline", True).value)
        self.enforce_monotonic_stamp: bool = bool(
            self.declare_parameter("enforce_monotonic_stamp", True).value
        )
        self.republish_last_on_stale: bool = bool(
            self.declare_parameter("republish_last_on_stale", True).value
        )

        self.publisher = self.create_publisher(FireState, self.output_topic, 10)
        self.first_source_time_s: Optional[float] = None
        self.first_ros_time_s: Optional[float] = None
        self.last_out_stamp_s: Optional[float] = None
        self.last_payload: Optional[List[RawHotspot]] = None
        self.last_source_time_s: Optional[float] = None
        self.last_file_state: Tuple[int, int] = (-1, -1)
        self.timeline_ts: List[float] = []
        self.timeline_rows: List[List[RawHotspot]] = []
        self.timeline_duration_s: float = 0.0
        self.timeline_base_ros_s: Optional[float] = None
        self.last_published_idx: int = -1

        period = 1.0 / max(0.2, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"fire_adapter_fds started output={self.output_topic} input={self.input_path or '<empty>'} "
            f"format={self.input_format} time_mode={self.time_mode} playback={self.playback_mode} "
            f"loop={self.loop_timeline} replay_speed={self.replay_speed:.2f} hz={self.publish_hz:.2f}"
        )

    def _tick(self) -> None:
        self._refresh_timeline_from_file()
        frame = self._pick_frame()
        if frame is None:
            if self.republish_last_on_stale and self.last_payload is not None:
                self._publish(self.last_source_time_s or 0.0, self.last_payload)
            return
        frame_ts, payload, frame_idx = frame
        self.last_payload = payload
        self.last_source_time_s = frame_ts
        self.last_published_idx = frame_idx
        self._publish(frame_ts, payload)

    def _refresh_timeline_from_file(self) -> None:
        if not self.input_path:
            self.get_logger().warn("input_path is empty, skip", throttle_duration_sec=5.0)
            return
        p = Path(self.input_path)
        if not p.exists():
            self.get_logger().warn("input file not found: %s", str(p), throttle_duration_sec=5.0)
            return
        try:
            st = p.stat()
        except OSError as ex:
            self.get_logger().warn("stat file failed: %s", str(ex), throttle_duration_sec=5.0)
            return

        file_state = (int(st.st_mtime_ns), int(st.st_size))
        if file_state == self.last_file_state and self.timeline_rows:
            return

        try:
            if self.input_format == "jsonl":
                rows = self._parse_jsonl(p)
            else:
                rows = self._parse_csv(p)
        except Exception as ex:
            self.get_logger().error("parse input failed: %s", str(ex))
            return

        if not rows:
            self.get_logger().warn("no valid hotspot rows from %s", str(p), throttle_duration_sec=5.0)
            return

        grouped: Dict[float, List[RawHotspot]] = {}
        for row in rows:
            bucket = grouped.setdefault(row.source_time_s, [])
            bucket.append(row)
        self.timeline_ts = sorted(grouped.keys())
        self.timeline_rows = [grouped[ts] for ts in self.timeline_ts]
        self.timeline_duration_s = max(0.0, self.timeline_ts[-1] - self.timeline_ts[0])
        self.timeline_base_ros_s = self._now_s()
        self.last_published_idx = -1
        self.last_file_state = file_state
        self.get_logger().info(
            f"loaded input frames={len(self.timeline_ts)} hotspots={len(rows)} duration_s={self.timeline_duration_s:.2f}"
        )

    def _pick_frame(self) -> Optional[Tuple[float, List[RawHotspot], int]]:
        if not self.timeline_rows:
            return None
        if self.playback_mode not in ("timeline", "latest"):
            self.get_logger().warn(
                "unsupported playback_mode=%s, fallback to latest",
                self.playback_mode,
                throttle_duration_sec=5.0,
            )
            self.playback_mode = "latest"

        if self.playback_mode == "latest":
            idx = len(self.timeline_rows) - 1
            if idx == self.last_published_idx and not self.republish_last_on_stale:
                return None
            return self.timeline_ts[idx], self.timeline_rows[idx], idx

        idx = self._timeline_index_at_now()
        if idx == self.last_published_idx and not self.republish_last_on_stale:
            return None
        return self.timeline_ts[idx], self.timeline_rows[idx], idx

    def _timeline_index_at_now(self) -> int:
        if len(self.timeline_ts) == 1:
            return 0
        now_s = self._now_s()
        if self.timeline_base_ros_s is None:
            self.timeline_base_ros_s = now_s

        start_ts = self.timeline_ts[0]
        end_ts = self.timeline_ts[-1]
        elapsed_source = (now_s - self.timeline_base_ros_s) * self.replay_speed
        source_t = start_ts + elapsed_source
        if self.loop_timeline and self.timeline_duration_s > 0.0:
            source_t = start_ts + (elapsed_source % self.timeline_duration_s)
        elif source_t >= end_ts:
            return len(self.timeline_ts) - 1

        idx = bisect_right(self.timeline_ts, source_t) - 1
        return max(0, min(idx, len(self.timeline_ts) - 1))

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
        source_time_s = _to_float(raw.get("time_s", raw.get("time", 0.0))) * self.source_time_scale
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
        x_loc = y_m if self.xy_swap else x_m
        y_loc = x_m if self.xy_swap else y_m
        x_loc = x_loc * self.x_scale + self.x_offset_m
        y_loc = y_loc * self.y_scale + self.y_offset_m
        z_loc = z_m * self.z_scale + self.z_offset_m

        yaw = math.radians(self.xy_yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        east_m = x_loc * c - y_loc * s
        north_m = x_loc * s + y_loc * c
        dlat = north_m / 111000.0
        lon_scale = max(1e-6, math.cos(math.radians(self.origin_lat)))
        dlon = east_m / (111000.0 * lon_scale)
        lat = self.origin_lat + dlat
        lon = self.origin_lon + dlon
        alt = self.origin_alt_m + z_loc
        return lat, lon, alt

    def _source_to_time_msg(self, source_time_s: float):
        if self.time_mode == "ros_now":
            return self.get_clock().now().to_msg()

        now_s = self._now_s()
        if self.first_source_time_s is None:
            self.first_source_time_s = source_time_s
            self.first_ros_time_s = now_s

        assert self.first_ros_time_s is not None
        assert self.first_source_time_s is not None
        out_s = self.first_ros_time_s + max(0.0, source_time_s - self.first_source_time_s)
        if self.enforce_monotonic_stamp and self.last_out_stamp_s is not None and out_s <= self.last_out_stamp_s:
            out_s = self.last_out_stamp_s + 1e-6
        self.last_out_stamp_s = out_s
        sec = int(out_s)
        nanosec = int((out_s - sec) * 1e9)
        t = self.get_clock().now().to_msg()
        t.sec = sec
        t.nanosec = nanosec
        return t

    def _publish(self, source_time_s: float, hotspots_raw: List[RawHotspot]) -> None:
        msg = FireState()
        msg.stamp = self._source_to_time_msg(source_time_s)
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

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


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
