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
        self._last_warning_at: float = 0.0

        self.declare_parameter("pi_status_topic", "pi_status")
        self.declare_parameter("sensors_topic", "robot_sensors")
        self.create_subscription(DiagnosticArray, self.get_parameter("pi_status_topic").value, self._on_status, 10)
        self.create_subscription(DiagnosticArray, self.get_parameter("sensors_topic").value, self._on_status, 10)

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _on_status(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        updates = {}
        for status in msg.status:
            prefix = status.name or "status"
            for item in status.values:
                updates[f"{prefix}.{item.key}"] = item.value
            updates[f"{prefix}.status_level"] = str(status.level)
            updates[f"{prefix}.status_message"] = status.message
            updates[f"{prefix}.hardware_id"] = status.hardware_id
        with self._lock:
            if self._latest is None:
                self._latest = {}
            self._latest.update(updates)

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
                self._warn_throttled("telemetry_url is empty.")
                time.sleep(1.0)
                continue
            token = self._load_access_token()
            if not token:
                self._warn_throttled("No access_token.json yet. Waiting for pairing/auth.")
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
                            detail = response.text.strip()
                            self.get_logger().warning(
                                f"Telemetry POST error: {response.status_code} {detail}"
                            )
                    except requests.RequestException as exc:
                        self.get_logger().warning(f"Telemetry POST error: {exc}")
                        break
                time.sleep(interval)

    def _warn_throttled(self, message: str) -> None:
        now = time.time()
        if now - self._last_warning_at < 10.0:
            return
        self._last_warning_at = now
        self.get_logger().warning(message)


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
