#!/usr/bin/env python3
import json
import ctypes
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
    from swarm_interfaces.msg import FireState, MissionPlan, SwarmState
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
    from swarm_interfaces.msg import FireState, MissionPlan, SwarmState


class SwarmStateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._fire_first_seen_ms: dict[str, int] = {}
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
        with self._lock:
            return dict(self._payload)

class SwarmSubscriber(Node):
    def __init__(self, cache: SwarmStateCache) -> None:
        super().__init__("swarm_visual_subscriber")
        self._cache = cache
        swarm_topic = os.environ.get("SWARM_STATE_TOPIC", "/swarm/state")
        fire_topic = os.environ.get("FIRE_STATE_TOPIC", "/env/fire_state")
        mission_topic = os.environ.get("MISSION_TOPIC", "/swarm/mission_targets")
        self._swarm_sub = self.create_subscription(SwarmState, swarm_topic, self._on_swarm, 10)
        self._fire_sub = self.create_subscription(FireState, fire_topic, self._on_fire, 10)
        self._mission_sub = self.create_subscription(MissionPlan, mission_topic, self._on_mission, 10)
        self.get_logger().info(f"subscribe swarm={swarm_topic} fire={fire_topic} mission={mission_topic}")

    def _on_swarm(self, msg: SwarmState) -> None:
        self._cache.update_from_msg(msg)

    def _on_fire(self, msg: FireState) -> None:
        self._cache.update_fire(msg)

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
