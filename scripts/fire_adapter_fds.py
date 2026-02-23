#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import FireState, Hotspot


@dataclass
class Origin:
    lat0: float
    lon0: float
    alt0_m: float


@dataclass
class LocalHotspot:
    hotspot_id: str
    x_m: float
    y_m: float
    z_m: float
    intensity_01: float
    spread_mps: float


@dataclass
class Frame:
    timestamp_ms: int
    sim_time_s: float
    hotspots: list[LocalHotspot]


def clamp01(v: float) -> float:
    return max(0.0, min(1.0, v))


def local_to_wgs84(origin: Origin, x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    lat = origin.lat0 + (y_m / 111000.0)
    lon_scale = 111000.0 * math.cos(math.radians(origin.lat0))
    lon = origin.lon0 + (x_m / max(1e-6, lon_scale))
    alt = origin.alt0_m + z_m
    return lat, lon, alt


def parse_json_frames(path: Path) -> tuple[Origin, list[Frame]]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    origin_obj = obj.get("origin_wgs84", {})
    origin = Origin(
        lat0=float(origin_obj.get("lat0", 39.9042)),
        lon0=float(origin_obj.get("lon0", 116.4074)),
        alt0_m=float(origin_obj.get("alt0_m", 80.0)),
    )

    def to_hotspots(raw_list: list[dict[str, Any]]) -> list[LocalHotspot]:
        out = []
        for item in raw_list:
            out.append(
                LocalHotspot(
                    hotspot_id=str(item["id"]),
                    x_m=float(item["x_m"]),
                    y_m=float(item["y_m"]),
                    z_m=float(item.get("z_m", 0.0)),
                    intensity_01=float(item.get("intensity_01", 0.0)),
                    spread_mps=float(item.get("spread_mps", 0.0)),
                )
            )
        return out

    frames: list[Frame] = []
    if isinstance(obj.get("frames"), list):
        for f in obj["frames"]:
            frames.append(
                Frame(
                    timestamp_ms=int(f["timestamp_ms"]),
                    sim_time_s=float(f.get("sim_time_s", 0.0)),
                    hotspots=to_hotspots(list(f.get("hotspots_local_m", []))),
                )
            )
    else:
        frames.append(
            Frame(
                timestamp_ms=int(obj["timestamp_ms"]),
                sim_time_s=float(obj.get("sim_time_s", 0.0)),
                hotspots=to_hotspots(list(obj.get("hotspots_local_m", []))),
            )
        )
    if not frames:
        raise ValueError("json input has no frames")
    return origin, frames


def parse_csv_frames(path: Path) -> tuple[Origin, list[Frame]]:
    # CSV uses fixed default origin, can be overridden by node parameters.
    origin = Origin(lat0=39.9042, lon0=116.4074, alt0_m=80.0)
    grouped: dict[int, list[LocalHotspot]] = {}
    sim_time: dict[int, float] = {}
    with path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            ts = int(float(row["timestamp_ms"]))
            grouped.setdefault(ts, []).append(
                LocalHotspot(
                    hotspot_id=str(row["id"]),
                    x_m=float(row["x_m"]),
                    y_m=float(row["y_m"]),
                    z_m=float(row.get("z_m", 0.0)),
                    intensity_01=float(row.get("intensity_01", 0.0)),
                    spread_mps=float(row.get("spread_mps", 0.0)),
                )
            )
            sim_time[ts] = float(row.get("sim_time_s", 0.0))
    frames = [Frame(timestamp_ms=ts, sim_time_s=sim_time.get(ts, 0.0), hotspots=grouped[ts]) for ts in sorted(grouped)]
    if not frames:
        raise ValueError("csv input has no frames")
    return origin, frames


class FireAdapterFDS(Node):
    def __init__(self) -> None:
        super().__init__("fire_adapter_fds")
        self.input_path = Path(str(self.declare_parameter("input_path", "data/fds_samples/fds_fire_snapshot.sample.json").value))
        self.input_format = str(self.declare_parameter("input_format", "auto").value).lower()
        self.output_topic = str(self.declare_parameter("output_topic", "/env/fire_state").value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.origin_lat0 = float(self.declare_parameter("origin_lat0", 39.9042).value)
        self.origin_lon0 = float(self.declare_parameter("origin_lon0", 116.4074).value)
        self.origin_alt0_m = float(self.declare_parameter("origin_alt0_m", 80.0).value)

        self.publisher = self.create_publisher(FireState, self.output_topic, 10)
        self.frames: list[Frame] = []
        self.origin = Origin(self.origin_lat0, self.origin_lon0, self.origin_alt0_m)
        self.index = 0
        self._load()

        period = 1.0 / max(0.1, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"fire_adapter_fds started input={self.input_path} format={self.input_format} "
            f"frames={len(self.frames)} output={self.output_topic} hz={self.publish_hz:.2f}"
        )

    def _load(self) -> None:
        if not self.input_path.exists():
            raise FileNotFoundError(f"input file not found: {self.input_path}")
        fmt = self.input_format
        if fmt == "auto":
            fmt = "json" if self.input_path.suffix.lower() == ".json" else "csv"
        if fmt == "json":
            origin, frames = parse_json_frames(self.input_path)
            self.origin = origin
            self.frames = frames
            return
        if fmt == "csv":
            origin, frames = parse_csv_frames(self.input_path)
            # csv origin default can be overridden by ros params.
            self.origin = Origin(self.origin_lat0 or origin.lat0, self.origin_lon0 or origin.lon0, self.origin_alt0_m or origin.alt0_m)
            self.frames = frames
            return
        raise ValueError(f"unsupported input_format={self.input_format}")

    def _tick(self) -> None:
        if not self.frames:
            return
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
            else:
                return
        frame = self.frames[self.index]
        self.index += 1

        msg = FireState()
        msg.stamp = self.get_clock().now().to_msg()
        hotspots: list[Hotspot] = []
        for hs in frame.hotspots:
            lat, lon, alt = local_to_wgs84(self.origin, hs.x_m, hs.y_m, hs.z_m)
            h = Hotspot()
            h.id = hs.hotspot_id
            h.position = [float(lat), float(lon), float(alt)]
            h.intensity = float(clamp01(hs.intensity_01))
            h.spread_mps = float(max(0.0, hs.spread_mps))
            hotspots.append(h)
        msg.hotspots = hotspots
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = FireAdapterFDS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
