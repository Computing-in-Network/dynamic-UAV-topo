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


@dataclass
class ResolvedFrame:
    origin: Origin
    frame: Frame


@dataclass
class DevcColumnConfig:
    name: str
    hotspot_id: str
    x_m: float
    y_m: float
    z_m: float
    value_min: float
    value_max: float
    spread_base: float
    spread_scale: float


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


def _to_float(value: Any, fallback: float) -> float:
    try:
        return float(value)
    except Exception:
        return fallback


def normalize_stream_glob(raw: str) -> str:
    v = str(raw).strip()
    if not v:
        return "*.json"
    if "*" in v or "?" in v or "[" in v:
        return v
    if v.startswith("."):
        return f"*{v}"
    return v


def parse_fds_devc_mapping(path: Path) -> tuple[Origin | None, list[DevcColumnConfig]]:
    obj = json.loads(path.read_text(encoding="utf-8"))
    origin_obj = obj.get("origin_wgs84")
    origin = None
    if isinstance(origin_obj, dict):
        origin = Origin(
            lat0=float(origin_obj.get("lat0", 39.9042)),
            lon0=float(origin_obj.get("lon0", 116.4074)),
            alt0_m=float(origin_obj.get("alt0_m", 80.0)),
        )

    columns_obj = obj.get("columns", [])
    if not isinstance(columns_obj, list) or not columns_obj:
        raise ValueError("fds devc mapping columns is empty")

    columns: list[DevcColumnConfig] = []
    for i, item in enumerate(columns_obj):
        if not isinstance(item, dict):
            raise ValueError(f"fds devc mapping columns[{i}] must be object")
        name = str(item.get("name", "")).strip()
        if not name:
            raise ValueError(f"fds devc mapping columns[{i}].name is required")
        hotspot_id = str(item.get("id", name)).strip() or name
        value_min = _to_float(item.get("value_min", 0.0), 0.0)
        value_max = _to_float(item.get("value_max", 1.0), 1.0)
        if value_max <= value_min:
            raise ValueError(f"fds devc mapping columns[{i}] value_max must be greater than value_min")
        columns.append(
            DevcColumnConfig(
                name=name,
                hotspot_id=hotspot_id,
                x_m=_to_float(item.get("x_m", 0.0), 0.0),
                y_m=_to_float(item.get("y_m", 0.0), 0.0),
                z_m=_to_float(item.get("z_m", 0.0), 0.0),
                value_min=value_min,
                value_max=value_max,
                spread_base=max(0.0, _to_float(item.get("spread_base", 0.2), 0.2)),
                spread_scale=max(0.0, _to_float(item.get("spread_scale", 0.8), 0.8)),
            )
        )
    return origin, columns


def parse_fds_devc_csv_frames(path: Path, mapping_path: Path) -> tuple[Origin | None, list[Frame]]:
    origin, mapping_cols = parse_fds_devc_mapping(mapping_path)

    with path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = [row for row in reader if row]
    if not rows:
        raise ValueError("fds devc csv is empty")

    # FDS devc.csv may place unit row before/after header row.
    # Find the first row whose first column is "Time".
    header_row_idx = -1
    for i, row in enumerate(rows[:8]):
        if not row:
            continue
        probe = row[0].strip().strip('"').lower()
        if probe == "time":
            header_row_idx = i
            break
    if header_row_idx < 0:
        raise ValueError("fds devc csv header must contain Time column row")

    header = [c.strip().strip('"') for c in rows[header_row_idx]]
    data_start_idx = header_row_idx + 1
    if data_start_idx < len(rows):
        probe = rows[data_start_idx][0].strip().strip('"').lower()
        if probe in ("s", "sec", "second", "seconds", "time"):
            data_start_idx += 1

    name_to_idx: dict[str, int] = {}
    for i, name in enumerate(header):
        if i == 0:
            continue
        name_to_idx[name] = i
        name_to_idx[name.lower()] = i

    missing = [c.name for c in mapping_cols if c.name not in name_to_idx and c.name.lower() not in name_to_idx]
    if missing:
        raise ValueError(f"fds devc mapping columns missing in csv: {missing}")

    frames: list[Frame] = []
    for row in rows[data_start_idx:]:
        if not row:
            continue
        time_s = _to_float(row[0], float("nan"))
        if math.isnan(time_s):
            continue
        timestamp_ms = int(time_s * 1000.0)
        hotspots: list[LocalHotspot] = []
        for col in mapping_cols:
            idx = name_to_idx.get(col.name, name_to_idx.get(col.name.lower(), -1))
            if idx < 0 or idx >= len(row):
                continue
            value = _to_float(row[idx], float("nan"))
            if math.isnan(value):
                continue
            intensity = clamp01((value - col.value_min) / max(1e-6, (col.value_max - col.value_min)))
            spread = col.spread_base + intensity * col.spread_scale
            hotspots.append(
                LocalHotspot(
                    hotspot_id=col.hotspot_id,
                    x_m=col.x_m,
                    y_m=col.y_m,
                    z_m=col.z_m,
                    intensity_01=intensity,
                    spread_mps=spread,
                )
            )
        if hotspots:
            frames.append(Frame(timestamp_ms=timestamp_ms, sim_time_s=time_s, hotspots=hotspots))

    if not frames:
        raise ValueError("fds devc csv has no valid rows")
    return origin, frames


class FireAdapterFDS(Node):
    def __init__(self) -> None:
        super().__init__("fire_adapter_fds")
        self.input_path = Path(str(self.declare_parameter("input_path", "data/fds_samples/fds_fire_snapshot.sample.json").value))
        self.input_format = str(self.declare_parameter("input_format", "auto").value).lower()
        self.mode = str(self.declare_parameter("mode", "offline").value).lower()
        self.output_topic = str(self.declare_parameter("output_topic", "/env/fire_state").value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 1.0).value)
        self.loop = bool(self.declare_parameter("loop", True).value)
        self.origin_lat0 = float(self.declare_parameter("origin_lat0", 39.9042).value)
        self.origin_lon0 = float(self.declare_parameter("origin_lon0", 116.4074).value)
        self.origin_alt0_m = float(self.declare_parameter("origin_alt0_m", 80.0).value)
        self.fds_devc_mapping = Path(
            str(self.declare_parameter("fds_devc_mapping", "data/fds_samples/fds_devc_mapping.sample.json").value)
        )

        self.stream_dir = Path(str(self.declare_parameter("stream_dir", "data/fds_samples/stream").value))
        self.stream_glob = normalize_stream_glob(str(self.declare_parameter("stream_glob", ".json").value))
        self.checkpoint_file = Path(
            str(self.declare_parameter("checkpoint_file", "/tmp/fire_adapter_fds_checkpoint.json").value)
        )
        self.fail_on_bad_file = bool(self.declare_parameter("fail_on_bad_file", False).value)

        self.publisher = self.create_publisher(FireState, self.output_topic, 10)
        self.frames: list[Frame] = []
        self.origin = Origin(self.origin_lat0, self.origin_lon0, self.origin_alt0_m)
        self.index = 0
        self.pending: list[ResolvedFrame] = []
        self.last_file = ""
        self._load_checkpoint()
        self._load()

        period = 1.0 / max(0.1, self.publish_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(
            f"fire_adapter_fds started mode={self.mode} input={self.input_path} format={self.input_format} "
            f"frames={len(self.frames)} output={self.output_topic} hz={self.publish_hz:.2f} "
            f"stream_dir={self.stream_dir} checkpoint={self.checkpoint_file}"
        )

    def _load_checkpoint(self) -> None:
        if not self.checkpoint_file.exists():
            return
        try:
            obj = json.loads(self.checkpoint_file.read_text(encoding="utf-8"))
            self.last_file = str(obj.get("last_file", "")).strip()
        except Exception:
            self.last_file = ""

    def _save_checkpoint(self) -> None:
        try:
            self.checkpoint_file.parent.mkdir(parents=True, exist_ok=True)
            self.checkpoint_file.write_text(json.dumps({"last_file": self.last_file}, ensure_ascii=False), encoding="utf-8")
        except Exception as ex:
            self.get_logger().warn(f"save checkpoint failed: {ex}")

    def _parse_file_frames(self, path: Path) -> list[ResolvedFrame]:
        fmt = self.input_format
        if fmt == "auto":
            fmt = "json" if path.suffix.lower() == ".json" else "csv"
        if fmt == "json":
            origin, frames = parse_json_frames(path)
        elif fmt == "csv":
            origin, frames = parse_csv_frames(path)
            origin = Origin(
                self.origin_lat0 if self.origin_lat0 != 0 else origin.lat0,
                self.origin_lon0 if self.origin_lon0 != 0 else origin.lon0,
                self.origin_alt0_m if self.origin_alt0_m != 0 else origin.alt0_m,
            )
        elif fmt == "fds_devc_csv":
            mapped_origin, frames = parse_fds_devc_csv_frames(path, self.fds_devc_mapping)
            origin = Origin(
                mapped_origin.lat0 if mapped_origin else self.origin_lat0,
                mapped_origin.lon0 if mapped_origin else self.origin_lon0,
                mapped_origin.alt0_m if mapped_origin else self.origin_alt0_m,
            )
        else:
            raise ValueError(f"unsupported input_format={self.input_format}")
        return [ResolvedFrame(origin=origin, frame=f) for f in frames]

    def _scan_stream_dir(self) -> None:
        files = sorted(self.stream_dir.glob(self.stream_glob))
        for file_path in files:
            name = file_path.name
            if self.last_file and name <= self.last_file:
                continue
            try:
                resolved = self._parse_file_frames(file_path)
                self.pending.extend(resolved)
                self.get_logger().info(f"loaded stream file={name} frames={len(resolved)}")
            except Exception as ex:
                self.get_logger().error(f"parse stream file failed: {name}: {ex}")
                if self.fail_on_bad_file:
                    raise
            self.last_file = name
            self._save_checkpoint()

    def _load(self) -> None:
        if self.mode == "stream":
            if not self.stream_dir.exists():
                raise FileNotFoundError(f"stream_dir not found: {self.stream_dir}")
            self._scan_stream_dir()
            return

        if self.mode != "offline":
            raise ValueError(f"unsupported mode={self.mode}")

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
            self.origin = Origin(
                self.origin_lat0 or origin.lat0,
                self.origin_lon0 or origin.lon0,
                self.origin_alt0_m or origin.alt0_m,
            )
            self.frames = frames
            return
        if fmt == "fds_devc_csv":
            mapped_origin, frames = parse_fds_devc_csv_frames(self.input_path, self.fds_devc_mapping)
            self.origin = Origin(
                mapped_origin.lat0 if mapped_origin else self.origin_lat0,
                mapped_origin.lon0 if mapped_origin else self.origin_lon0,
                mapped_origin.alt0_m if mapped_origin else self.origin_alt0_m,
            )
            self.frames = frames
            return
        raise ValueError(f"unsupported input_format={self.input_format}")

    def _publish_frame(self, origin: Origin, frame: Frame) -> None:
        msg = FireState()
        msg.stamp = self.get_clock().now().to_msg()
        hotspots: list[Hotspot] = []
        for hs in frame.hotspots:
            lat, lon, alt = local_to_wgs84(origin, hs.x_m, hs.y_m, hs.z_m)
            h = Hotspot()
            h.id = hs.hotspot_id
            h.position = [float(lat), float(lon), float(alt)]
            h.intensity = float(clamp01(hs.intensity_01))
            h.spread_mps = float(max(0.0, hs.spread_mps))
            hotspots.append(h)
        msg.hotspots = hotspots
        self.publisher.publish(msg)

    def _tick(self) -> None:
        if self.mode == "stream":
            self._scan_stream_dir()
            if not self.pending:
                return
            item = self.pending.pop(0)
            self._publish_frame(item.origin, item.frame)
            return

        if not self.frames:
            return
        if self.index >= len(self.frames):
            if self.loop:
                self.index = 0
            else:
                return
        frame = self.frames[self.index]
        self.index += 1
        self._publish_frame(self.origin, frame)


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
