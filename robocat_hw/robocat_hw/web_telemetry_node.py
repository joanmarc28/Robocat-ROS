import json
import socket
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

try:
    import requests
except ImportError:
    requests = None


class WebTelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("web_telemetry_node")
        self.declare_parameter("telemetry_url", "https://europerobotics.jmprojects.cat/api/telemetry")
        self.declare_parameter("server_url", "https://europerobotics.jmprojects.cat")
        self.declare_parameter("robot_id", socket.gethostname())
        self.declare_parameter("publish_hz", 1.0)
        self.declare_parameter("access_token_path", "/home/robocat-v2/.robocat/access_token.json")
        self.declare_parameter("identity_path", "/home/robocat-v2/.robocat/identity.json")

        self._latest: Optional[Dict[str, str]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._access_token: Optional[str] = None
        self._access_token_mtime: Optional[float] = None
        self._identity_robot_id: Optional[str] = None
        self._identity_mtime: Optional[float] = None

        self.create_subscription(DiagnosticArray, "pi_status", self._on_status, 10)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _on_status(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        metrics = {item.key: item.value for item in status.values}
        metrics["status_name"] = status.name
        metrics["status_level"] = str(status.level)
        metrics["status_message"] = status.message
        metrics["hardware_id"] = status.hardware_id
        with self._lock:
            self._latest = metrics

    def _make_payload(self) -> Optional[Dict[str, object]]:
        with self._lock:
            if self._latest is None:
                return None
            snapshot = dict(self._latest)
        return {
            "snapshot": snapshot,
        }

    def _load_robot_id(self) -> Optional[str]:
        path = Path(self.get_parameter("identity_path").value)
        try:
            stat = path.stat()
        except OSError:
            self._identity_robot_id = None
            self._identity_mtime = None
            return None
        if self._identity_mtime == stat.st_mtime and self._identity_robot_id:
            return self._identity_robot_id
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            self._identity_robot_id = None
            self._identity_mtime = None
            return None
        robot_id = data.get("robot_id")
        if not robot_id:
            self._identity_robot_id = None
            self._identity_mtime = None
            return None
        self._identity_robot_id = robot_id
        self._identity_mtime = stat.st_mtime
        return robot_id

    def _load_access_token(self) -> Optional[str]:
        path = Path(self.get_parameter("access_token_path").value)
        try:
            stat = path.stat()
        except OSError:
            self._access_token = None
            self._access_token_mtime = None
            return None
        if self._access_token_mtime == stat.st_mtime and self._access_token:
            return self._access_token
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            self._access_token = None
            self._access_token_mtime = None
            return None
        token = data.get("access_token")
        if not token:
            self._access_token = None
            self._access_token_mtime = None
            return None
        self._access_token = token
        self._access_token_mtime = stat.st_mtime
        return token

    def _run(self) -> None:
        self._post_loop()

    def _post_loop(self) -> None:
        if requests is None:
            self.get_logger().error("python3-requests is not installed.")
            return
        while not self._stop.is_set():
            url = self.get_parameter("telemetry_url").value or self.get_parameter("server_url").value
            if not url:
                time.sleep(1.0)
                continue
            token = self._load_access_token()
            if not token:
                time.sleep(1.0)
                continue
            interval = 1.0 / float(self.get_parameter("publish_hz").value)
            headers = {
                "content-type": "application/json",
                "x-robot-token": token,
            }
            while not self._stop.is_set():
                new_token = self._load_access_token()
                if not new_token:
                    break
                if new_token != token:
                    token = new_token
                    headers["x-robot-token"] = token
                payload = self._make_payload()
                if payload is not None:
                    try:
                        response = requests.post(url, json=payload, headers=headers, timeout=5)
                        if response.status_code >= 400:
                            self.get_logger().warning(f"Telemetry POST error: {response.status_code}")
                    except requests.RequestException as exc:
                        self.get_logger().warning(f"Telemetry POST error: {exc}")
                        break
                time.sleep(interval)


def main() -> None:
    rclpy.init()
    node = WebTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
