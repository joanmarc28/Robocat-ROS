import shlex
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node


class WebRtcStreamerNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_streamer_node")
        self.declare_parameter("webrtc_command", "")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("restart_on_exit", True)
        self.declare_parameter("restart_delay_sec", 2.0)

        self._process: Optional[subprocess.Popen] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        command = str(self.get_parameter("webrtc_command").value).strip()
        if not command:
            self.get_logger().warning("webrtc_command is empty. WebRTC not started.")
            return
        if not bool(self.get_parameter("auto_start").value):
            self.get_logger().info("auto_start is false. WebRTC not started.")
            return
        restart_on_exit = bool(self.get_parameter("restart_on_exit").value)
        restart_delay = float(self.get_parameter("restart_delay_sec").value)

        while not self._stop.is_set():
            try:
                args = command if isinstance(command, str) else shlex.split(command)
                self.get_logger().info("Starting WebRTC process.")
                self._process = subprocess.Popen(
                    args,
                    shell=isinstance(args, str),
                )
                self._process.wait()
                if self._stop.is_set():
                    break
                if not restart_on_exit:
                    break
                self.get_logger().warning("WebRTC process exited. Restarting.")
                time.sleep(restart_delay)
            except Exception as exc:
                self.get_logger().error(f"WebRTC start failed: {exc}")
                if not restart_on_exit:
                    break
                time.sleep(restart_delay)

    def stop(self) -> None:
        self._stop.set()
        if self._process and self._process.poll() is None:
            try:
                self._process.terminate()
                self._process.wait(timeout=5)
            except Exception:
                try:
                    self._process.kill()
                except Exception:
                    pass


def main() -> None:
    rclpy.init()
    node = WebRtcStreamerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
