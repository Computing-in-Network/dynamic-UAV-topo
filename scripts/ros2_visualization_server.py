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
    from swarm_interfaces.msg import SwarmState
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
    from swarm_interfaces.msg import SwarmState


class SwarmStateCache:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._payload = {
            "timestamp": 0,
            "uavs": [],
            "links": [],
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
        }

        with self._lock:
            self._payload = payload

    def get(self) -> dict:
        with self._lock:
            return dict(self._payload)

class SwarmSubscriber(Node):
    def __init__(self, cache: SwarmStateCache) -> None:
        super().__init__("swarm_visual_subscriber")
        self._cache = cache
        topic = os.environ.get("SWARM_STATE_TOPIC", "/swarm/state")
        self._sub = self.create_subscription(SwarmState, topic, self._on_msg, 10)
        self.get_logger().info(f"subscribe topic={topic}")

    def _on_msg(self, msg: SwarmState) -> None:
        self._cache.update_from_msg(msg)


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
