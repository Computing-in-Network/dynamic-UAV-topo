#!/usr/bin/env python3
import json
import ctypes
import math
import os
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from glob import glob

from fire_region_model import build_fire_regions

ros_lib_paths = [
    "/opt/ros/humble/lib",
    "/opt/ros/humble/local/lib",
]
ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
for p in ros_lib_paths:
    if p not in ld_library_path.split(":"):
        ld_library_path = f"{p}:{ld_library_path}" if ld_library_path else p
os.environ["LD_LIBRARY_PATH"] = ld_library_path

try:
    import rclpy
    from rclpy.node import Node
    from swarm_interfaces.msg import FireRegionState, FireState, MissionPlan, SwarmState
except ModuleNotFoundError:
    ros_py_paths = [
        "/opt/ros/humble/local/lib/python3.10/dist-packages",
        "/opt/ros/humble/lib/python3.10/site-packages",
    ]
    for p in ros_py_paths:
        if p not in sys.path:
            sys.path.insert(0, p)
    for so in sorted(glob("/opt/ros/humble/lib/librcl*.so")):
        try:
            ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass
    for so in sorted(glob("/opt/ros/humble/lib/librcutils*.so")):
        try:
            ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass
    for so in sorted(glob("/opt/ros/humble/lib/librmw*.so")):
        try:
            ctypes.CDLL(so, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass
    import rclpy
    from rclpy.node import Node
    from swarm_interfaces.msg import FireRegionState, FireState, MissionPlan, SwarmState


class SwarmStateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._fire_first_seen_ms: dict[str, int] = {}
        self._real_region_stamp_ms: int = 0
        self._payload = {
            "timestamp": 0,
            "uavs": [],
            "links": [],
            "fire_hotspots": [],
            "fire_regions": [],
            "mission_targets": [],
        }

    def update_from_msg(self, msg: SwarmState) -> None:
        uavs = []
        for uav in msg.uavs:
            uavs.append(
                {
                    "id": uav.id,
                    "position": [float(uav.position[0]), float(uav.position[1]), float(uav.position[2])],
                    "velocity": [float(uav.velocity[0]), float(uav.velocity[1]), float(uav.velocity[2])],
                    "battery": float(uav.battery),
                    "status": int(uav.status),
                }
            )

        links = [
            {
                "source": link.source,
                "target": link.target,
                "weight": float(link.weight),
                "is_occluded": bool(link.is_occluded),
            }
            for link in msg.links
        ]

        payload = {
            "timestamp": int(msg.stamp.sec * 1000 + msg.stamp.nanosec / 1_000_000),
            "uavs": uavs,
            "links": links,
            "fire_hotspots": self._payload.get("fire_hotspots", []),
            "fire_regions": self._payload.get("fire_regions", []),
            "mission_targets": self._payload.get("mission_targets", []),
        }

        with self._lock:
            self._payload = payload

    def update_fire(self, msg: FireState) -> None:
        stamp_ms = int(msg.stamp.sec * 1000 + msg.stamp.nanosec / 1_000_000)
        fire = [
            {
                "id": hs.id,
                "position": [float(hs.position[0]), float(hs.position[1]), float(hs.position[2])],
                "intensity": float(hs.intensity),
                "spread_mps": float(hs.spread_mps),
            }
            for hs in msg.hotspots
        ]
        regions = build_fire_regions(fire, stamp_ms, self._fire_first_seen_ms)
        with self._lock:
            self._payload["fire_hotspots"] = fire
            if self._real_region_stamp_ms + 5000 < stamp_ms:
                self._payload["fire_regions"] = regions

    def update_fire_regions(self, msg: FireRegionState) -> None:
        stamp_ms = int(msg.stamp.sec * 1000 + msg.stamp.nanosec / 1_000_000)
        regions = []
        for r in msg.regions:
            boundary = list(r.boundary)
            center = [float(r.center[0]), float(r.center[1]), float(r.center[2])]
            radius_m = 0.0
            if len(boundary) >= 6:
                for i in range(0, len(boundary), 3):
                    lat = float(boundary[i])
                    lon = float(boundary[i + 1])
                    dlat = (lat - center[0]) * 111000.0
                    dlon = (lon - center[1]) * 111000.0 * max(1e-6, math.cos(math.radians(center[0])))
                    radius_m = max(radius_m, (dlat * dlat + dlon * dlon) ** 0.5)
            regions.append(
                {
                    "id": r.id,
                    "center": center,
                    "radius_m": max(60.0, radius_m),
                    "intensity": float(r.intensity),
                    "spread_mps": float(r.spread_mps),
                    "member_count": int(r.member_count),
                    "boundary": boundary,
                    "source": "fds_region",
                }
            )
        with self._lock:
            self._real_region_stamp_ms = stamp_ms
            self._payload["fire_regions"] = regions

    def update_mission(self, msg: MissionPlan) -> None:
        targets = [
            {
                "uav_id": t.uav_id,
                "position": [float(t.position[0]), float(t.position[1]), float(t.position[2])],
                "priority": float(t.priority),
                "reason": t.reason,
            }
            for t in msg.targets
        ]
        with self._lock:
            self._payload["mission_targets"] = targets

    def get(self) -> dict:
        # Avoid hanging the HTTP API forever if the publisher thread holds the
        # cache lock unexpectedly long; a best-effort snapshot is sufficient.
        if self._lock.acquire(timeout=0.05):
            try:
                return dict(self._payload)
            finally:
                self._lock.release()
        return dict(self._payload)

class SwarmSubscriber(Node):
    def __init__(self, cache: SwarmStateCache) -> None:
        super().__init__("swarm_visual_subscriber")
        self._cache = cache
        swarm_topic = os.environ.get("SWARM_STATE_TOPIC", "/swarm/state")
        fire_topic = os.environ.get("FIRE_STATE_TOPIC", "/env/fire_state")
        mission_topic = os.environ.get("MISSION_TOPIC", "/swarm/mission_targets")
        fire_region_topic = os.environ.get("FIRE_REGION_TOPIC", "/env/fire_region_state")
        self._swarm_sub = self.create_subscription(SwarmState, swarm_topic, self._on_swarm, 10)
        self._fire_sub = self.create_subscription(FireState, fire_topic, self._on_fire, 10)
        self._fire_region_sub = self.create_subscription(
            FireRegionState, fire_region_topic, self._on_fire_region, 10
        )
        self._mission_sub = self.create_subscription(MissionPlan, mission_topic, self._on_mission, 10)
        self.get_logger().info(
            f"subscribe swarm={swarm_topic} fire={fire_topic} fire_region={fire_region_topic} mission={mission_topic}"
        )

    def _on_swarm(self, msg: SwarmState) -> None:
        self._cache.update_from_msg(msg)

    def _on_fire(self, msg: FireState) -> None:
        self._cache.update_fire(msg)

    def _on_fire_region(self, msg: FireRegionState) -> None:
        self._cache.update_fire_regions(msg)

    def _on_mission(self, msg: MissionPlan) -> None:
        self._cache.update_mission(msg)


WEB_DIR = Path(__file__).resolve().parent.parent / "web"
INDEX_HTML = (WEB_DIR / "index.html").read_text(encoding="utf-8")
LEGACY_HTML = INDEX_HTML
CESIUM_HTML = (WEB_DIR / "cesium_deck.html").read_text(encoding="utf-8")


class Handler(BaseHTTPRequestHandler):
    cache: SwarmStateCache

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html", "/cesium"):
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(CESIUM_HTML.encode("utf-8"))
            return

        if self.path == "/legacy":
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(LEGACY_HTML.encode("utf-8"))
            return

        if self.path == "/api/swarm_state":
            payload = json.dumps(self.cache.get()).encode("utf-8")
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(payload)
            return

        self.send_response(HTTPStatus.NOT_FOUND)
        self.end_headers()

    def log_message(self, format: str, *args) -> None:
        return


def main() -> None:
    ros_log_dir = os.environ.get("ROS_LOG_DIR", "/tmp/roslog")
    os.makedirs(ros_log_dir, exist_ok=True)
    os.environ["ROS_LOG_DIR"] = ros_log_dir

    cache = SwarmStateCache()

    rclpy.init()
    subscriber = SwarmSubscriber(cache)
    spin_thread = threading.Thread(target=rclpy.spin, args=(subscriber,), daemon=True)
    spin_thread.start()

    Handler.cache = cache
    port = int(os.environ.get("SWARM_VIS_PORT", "8876"))
    server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    print(f"[visual] serving http://127.0.0.1:{port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
