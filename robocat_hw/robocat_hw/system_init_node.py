import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray


class SystemInitNode(Node):
    def __init__(self) -> None:
        super().__init__("system_init_node")
        self.declare_parameter("server_host", "")
        self.declare_parameter("server_port", 443)
        self.declare_parameter("pi_status_timeout_sec", 5.0)

        self._pub = self.create_publisher(String, "oled_text", 10)
        self._pi_status_event = threading.Event()

        self.create_subscription(
            DiagnosticArray,
            "pi_status",
            self._on_pi_status_ready,
            10,
        )

        self._thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._thread.start()

    def _on_pi_status_ready(self, _msg: DiagnosticArray) -> None:
        self._pi_status_event.set()

    def _publish(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._pub.publish(msg)

    def _check_internet(self, host: str, port: int, timeout: float = 2.0) -> bool:
        try:
            with socket.create_connection((host, port), timeout=timeout):
                return True
        except OSError:
            return False

    def _wait_for_pi_status(self, timeout: float) -> bool:
        return self._pi_status_event.wait(timeout=timeout)

    def _run_sequence(self) -> None:
        self._publish("Initializing...")
        time.sleep(0.5)

        host = self.get_parameter("server_host").value
        port = int(self.get_parameter("server_port").value)
        internet_ok = True
        if host:
            internet_ok = self._check_internet(host, port)
            self._publish(f"Internet: {'ok' if internet_ok else 'fail'}")
            time.sleep(0.5)

        timeout = float(self.get_parameter("pi_status_timeout_sec").value)
        has_status = self._wait_for_pi_status(timeout)
        self._publish(f"Pi status: {'ok' if has_status else 'fail'}")
        time.sleep(0.5)

        if internet_ok and has_status:
            self._publish("System ready")
        else:
            self._publish("Check system")


def main() -> None:
    rclpy.init()
    node = SystemInitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
