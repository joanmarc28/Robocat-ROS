import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionEventNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_event_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("manual_event_topic", "/vision/manual_event")
        self.declare_parameter("events_topic", "/vision/events")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("publish_fake_events", False)
        self.declare_parameter("fake_event_interval_sec", 10.0)

        self._pub_events = self.create_publisher(
            String, self.get_parameter("events_topic").value, 10
        )
        self._pub_detections = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )

        self.create_subscription(
            String, self.get_parameter("manual_event_topic").value, self._on_manual_event, 10
        )

        if bool(self.get_parameter("publish_fake_events").value):
            self._timer = self.create_timer(
                float(self.get_parameter("fake_event_interval_sec").value),
                self._publish_fake_event,
            )
        else:
            self._timer = None

        state = "enabled" if bool(self.get_parameter("enabled").value) else "disabled"
        self.get_logger().info(f"Vision event node ready ({state}).")

    def _on_manual_event(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        # allow raw JSON or simple "type=foo"
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = {"type": raw}
        self._publish_event(data)

    def _publish_event(self, data: dict) -> None:
        payload = json.dumps(data, ensure_ascii=False)
        msg = String()
        msg.data = payload
        self._pub_events.publish(msg)

    def _publish_fake_event(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            return
        event = {
            "type": "human_emotion",
            "score": 0.7,
            "data": {"emotion": "happy"},
            "ts": time.time(),
        }
        self._publish_event(event)


def main() -> None:
    rclpy.init()
    node = VisionEventNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
