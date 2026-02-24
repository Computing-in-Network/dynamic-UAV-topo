#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


VIS_PORT = int(os.environ.get("DUAL_VIS_PORT", "8899"))
FOCUS_PORT = int(os.environ.get("DUAL_FOCUS_PORT", "8900"))
TARGET_UAV_ID = os.environ.get("DUAL_UAV_ID", "").strip()
VIS_URL = os.environ.get("DUAL_VIS_URL", f"http://127.0.0.1:{VIS_PORT}/api/swarm_state")

_proxy_handler = urllib.request.ProxyHandler({})
_opener = urllib.request.build_opener(_proxy_handler)


def fetch_swarm_state() -> dict:
    raw = _opener.open(VIS_URL, timeout=1.2).read().decode("utf-8")
    obj = json.loads(raw)
    if not isinstance(obj, dict):
        return {}
    return obj


def pick_uav(state: dict) -> dict | None:
    uavs = state.get("uavs", [])
    if not isinstance(uavs, list) or not uavs:
        return None
    if TARGET_UAV_ID:
        for u in uavs:
            if isinstance(u, dict) and u.get("id") == TARGET_UAV_ID:
                return u
        return None
    for u in uavs:
        if isinstance(u, dict) and "id" in u and "position" in u:
            return u
    return None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path != "/api/gazebo_focus_state":
            self.send_response(HTTPStatus.NOT_FOUND)
            self.end_headers()
            return

        try:
            state = fetch_swarm_state()
            u = pick_uav(state)
            if u is None:
                payload = {
                    "ok": False,
                    "error": "no_uav",
                    "source": "mock_from_visual",
                }
            else:
                payload = {
                    "ok": True,
                    "source": "mock_from_visual",
                    "timestamp": int(state.get("timestamp", 0)),
                    "uav": {
                        "id": u.get("id", ""),
                        "position": list(u.get("position", [0.0, 0.0, 0.0])),
                    },
                }
        except Exception as ex:
            payload = {
                "ok": False,
                "error": str(ex),
                "source": "mock_from_visual",
            }

        raw = json.dumps(payload).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(raw)

    def log_message(self, format: str, *args) -> None:
        return


def main() -> None:
    server = ThreadingHTTPServer(("0.0.0.0", FOCUS_PORT), Handler)
    print(f"[dual_focus_mock] serving http://127.0.0.1:{FOCUS_PORT}/api/gazebo_focus_state")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
