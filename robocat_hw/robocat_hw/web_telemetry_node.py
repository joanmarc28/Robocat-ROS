import asyncio
import json
import socket
import threading
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

try:
    import websockets
except ImportError:
    websockets = None


class WebTelemetryNode(Node):
    def __init__(self) -> None:
        super().__init__("web_telemetry_node")
        self.declare_parameter("server_url", "")
        self.declare_parameter("robot_id", socket.gethostname())
        self.declare_parameter("publish_hz", 1.0)

        self._latest: Optional[Dict[str, str]] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

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
            "robot_id": self.get_parameter("robot_id").value,
            "metrics": snapshot,
        }

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._connect_loop())

    async def _connect_loop(self) -> None:
        if websockets is None:
            self.get_logger().error("python3-websockets is not installed.")
            return
        while not self._stop.is_set():
            url = self.get_parameter("server_url").value
            if not url:
                await asyncio.sleep(1.0)
                continue
            try:
                async with websockets.connect(url) as websocket:
                    self.get_logger().info("Connected to telemetry server.")
                    interval = 1.0 / float(self.get_parameter("publish_hz").value)
                    while not self._stop.is_set():
                        payload = self._make_payload()
                        if payload is not None:
                            await websocket.send(json.dumps(payload))
                        await asyncio.sleep(interval)
            except Exception as exc:
                self.get_logger().warning(f"Websocket error: {exc}")
                await asyncio.sleep(2.0)


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
