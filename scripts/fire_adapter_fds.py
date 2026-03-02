#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from bisect import bisect_right
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireRegion, FireRegionState, FireState, Hotspot
from fire_adapter_fds_core import (
    RawHotspot,
    RawRegion,
    group_hotspot_rows,
    group_region_rows,
    load_hotspot_rows,
    load_region_rows,
)


class FireAdapterFds(Node):
    def __init__(self) -> None:
        super().__init__("fire_adapter_fds")
        self.output_topic: str = self.declare_parameter("output_topic", "/env/fire_state").value
        self.publish_hz: float = float(self.declare_parameter("publish_hz", 1.0).value)
        self.input_path: str = self.declare_parameter("input_path", "").value
        self.input_format: str = self.declare_parameter("input_format", "csv").value
        self.input_profile: str = self.declare_parameter("input_profile", "normalized").value
        self.region_output_topic: str = self.declare_parameter(
            "region_output_topic", "/env/fire_region_state"
        ).value
        self.region_input_path: str = self.declare_parameter("region_input_path", "").value
        self.region_input_format: str = self.declare_parameter("region_input_format", "jsonl").value
        self.region_input_profile: str = self.declare_parameter("region_input_profile", "normalized").value
        self.publish_regions: bool = bool(self.declare_parameter("publish_regions", True).value)
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
        self.region_publisher = self.create_publisher(FireRegionState, self.region_output_topic, 10)
        self.first_source_time_s: Optional[float] = None
        self.first_ros_time_s: Optional[float] = None
        self.last_out_stamp_s: Optional[float] = None
        self.last_payload: Optional[List[RawHotspot]] = None
        self.last_source_time_s: Optional[float] = None
        self.last_file_state: Tuple[int, int] = (-1, -1)
        self.last_region_file_state: Tuple[int, int] = (-1, -1)
        self.timeline_ts: List[float] = []
        self.timeline_rows: List[List[RawHotspot]] = []
        self.region_timeline_ts: List[float] = []
        self.region_timeline_rows: List[List[RawRegion]] = []
        self.playback_ts: List[float] = []
        self.timeline_duration_s: float = 0.0
        self.timeline_base_ros_s: Optional[float] = None
        self.last_published_idx: int = -1

        period = 1.0 / max(0.2, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"fire_adapter_fds started output={self.output_topic} input={self.input_path or '<empty>'} "
            f"format={self.input_format} profile={self.input_profile} "
            f"region_output={self.region_output_topic} region_input={self.region_input_path or '<empty>'} "
            f"region_format={self.region_input_format} region_profile={self.region_input_profile} "
            f"time_mode={self.time_mode} playback={self.playback_mode} "
            f"loop={self.loop_timeline} replay_speed={self.replay_speed:.2f} hz={self.publish_hz:.2f}"
        )

    def _tick(self) -> None:
        self._refresh_timeline_from_file()
        self._refresh_region_timeline_from_file()
        frame = self._pick_frame()
        if frame is None:
            if self.republish_last_on_stale and self.last_payload is not None:
                stamp = self._source_to_time_msg(self.last_source_time_s or 0.0)
                self._publish(self.last_payload, stamp)
            return
        frame_ts, payload, frame_idx = frame
        self.last_payload = payload
        self.last_source_time_s = frame_ts
        self.last_published_idx = frame_idx
        self.get_logger().info(
            f"publish frame idx={frame_idx}/{max(0, len(self.playback_ts)-1)} source_t={frame_ts:.3f} hotspots={len(payload)}",
            throttle_duration_sec=2.0,
        )
        stamp = self._source_to_time_msg(frame_ts)
        self._publish(payload, stamp)
        if self.publish_regions:
            self._publish_regions(self._pick_region_rows(frame_ts), stamp)

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
            rows = load_hotspot_rows(
                p,
                input_format=self.input_format,
                has_header=self.has_header,
                source_time_scale=self.source_time_scale,
                input_profile=self.input_profile,
            )
        except Exception as ex:
            self.get_logger().error("parse input failed: %s", str(ex))
            return

        if not rows:
            self.get_logger().warn("no valid hotspot rows from %s", str(p), throttle_duration_sec=5.0)
            return

        self.timeline_ts, self.timeline_rows = group_hotspot_rows(rows)
        self._rebuild_playback_timeline()
        self.timeline_base_ros_s = self._now_s()
        self.last_published_idx = -1
        self.last_file_state = file_state
        self.get_logger().info(
            f"loaded input frames={len(self.timeline_ts)} hotspots={len(rows)} duration_s={self.timeline_duration_s:.2f}"
        )

    def _refresh_region_timeline_from_file(self) -> None:
        if not self.publish_regions:
            return
        if not self.region_input_path:
            return
        p = Path(self.region_input_path)
        if not p.exists():
            self.get_logger().warn("region input file not found: %s", str(p), throttle_duration_sec=5.0)
            return
        try:
            st = p.stat()
        except OSError as ex:
            self.get_logger().warn("stat region file failed: %s", str(ex), throttle_duration_sec=5.0)
            return

        file_state = (int(st.st_mtime_ns), int(st.st_size))
        if file_state == self.last_region_file_state and self.region_timeline_rows:
            return

        try:
            rows = load_region_rows(
                p,
                input_format=self.region_input_format,
                has_header=self.has_header,
                source_time_scale=self.source_time_scale,
                input_profile=self.region_input_profile,
            )
        except Exception as ex:
            self.get_logger().error("parse region input failed: %s", str(ex))
            return

        if not rows:
            self.get_logger().warn("no valid region rows from %s", str(p), throttle_duration_sec=5.0)
            return

        self.region_timeline_ts, self.region_timeline_rows = group_region_rows(rows)
        self.last_region_file_state = file_state
        self._rebuild_playback_timeline()
        self.timeline_base_ros_s = self._now_s()
        self.last_published_idx = -1
        self.get_logger().info(
            f"loaded region frames={len(self.region_timeline_ts)} regions={len(rows)}"
        )

    def _pick_frame(self) -> Optional[Tuple[float, List[RawHotspot], int]]:
        if not self.playback_ts:
            return None
        if self.playback_mode not in ("timeline", "latest"):
            self.get_logger().warn(
                "unsupported playback_mode=%s, fallback to latest",
                self.playback_mode,
                throttle_duration_sec=5.0,
            )
            self.playback_mode = "latest"

        if self.playback_mode == "latest":
            idx = len(self.playback_ts) - 1
            if idx == self.last_published_idx and not self.republish_last_on_stale:
                return None
            frame_ts = self.playback_ts[idx]
            return frame_ts, self._pick_hotspot_rows(frame_ts), idx

        idx = self._timeline_index_at_now()
        if idx == self.last_published_idx and not self.republish_last_on_stale:
            return None
        frame_ts = self.playback_ts[idx]
        return frame_ts, self._pick_hotspot_rows(frame_ts), idx

    def _timeline_index_at_now(self) -> int:
        if len(self.playback_ts) == 1:
            return 0
        now_s = self._now_s()
        if self.timeline_base_ros_s is None:
            self.timeline_base_ros_s = now_s

        start_ts = self.playback_ts[0]
        end_ts = self.playback_ts[-1]
        elapsed_source = (now_s - self.timeline_base_ros_s) * self.replay_speed
        source_t = start_ts + elapsed_source
        if self.loop_timeline and self.timeline_duration_s > 0.0:
            source_t = start_ts + (elapsed_source % self.timeline_duration_s)
        elif source_t >= end_ts:
            return len(self.playback_ts) - 1

        idx = bisect_right(self.playback_ts, source_t) - 1
        return max(0, min(idx, len(self.playback_ts) - 1))

    def _rebuild_playback_timeline(self) -> None:
        merged = sorted(set(self.timeline_ts + self.region_timeline_ts))
        self.playback_ts = merged
        if merged:
            self.timeline_duration_s = max(0.0, merged[-1] - merged[0])
        else:
            self.timeline_duration_s = 0.0

    def _pick_hotspot_rows(self, frame_ts: float) -> List[RawHotspot]:
        if not self.timeline_ts:
            return []
        idx = bisect_right(self.timeline_ts, frame_ts) - 1
        idx = max(0, min(idx, len(self.timeline_ts) - 1))
        return self.timeline_rows[idx]

    def _pick_region_rows(self, frame_ts: float) -> List[RawRegion]:
        if not self.region_timeline_ts:
            return []
        idx = bisect_right(self.region_timeline_ts, frame_ts) - 1
        idx = max(0, min(idx, len(self.region_timeline_ts) - 1))
        return self.region_timeline_rows[idx]

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

    def _publish(self, hotspots_raw: List[RawHotspot], stamp) -> None:
        msg = FireState()
        msg.stamp = stamp
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

    def _publish_regions(self, regions_raw: List[RawRegion], stamp) -> None:
        msg = FireRegionState()
        msg.stamp = stamp
        out_regions: List[FireRegion] = []
        for row in regions_raw:
            region = FireRegion()
            region.id = row.region_id
            boundary: List[float] = []
            sum_lat = 0.0
            sum_lon = 0.0
            sum_alt = 0.0
            for x_m, y_m, z_m in row.points_xyz:
                lat, lon, alt = self._xy_to_wgs84(x_m, y_m, z_m)
                boundary.extend([lat, lon, alt])
                sum_lat += lat
                sum_lon += lon
                sum_alt += alt
            n = max(1, len(row.points_xyz))
            region.boundary = boundary
            region.center = [sum_lat / n, sum_lon / n, sum_alt / n]
            region.intensity = float(row.intensity)
            region.spread_mps = float(row.spread_mps)
            region.member_count = int(len(row.points_xyz))
            out_regions.append(region)
        msg.regions = out_regions
        self.region_publisher.publish(msg)

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
