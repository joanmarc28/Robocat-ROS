import json
import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import cv2
    _CV2_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    cv2 = None  # type: ignore
    _CV2_ERROR = exc

try:
    from robocat_vision.plate_detection import PlateDetection
    from robocat_vision.container_detection import ContainerDetection
    _VISION_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    PlateDetection = None  # type: ignore
    ContainerDetection = None  # type: ignore
    _VISION_IMPORT_ERROR = exc


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15)
        self.declare_parameter("detect_interval_sec", 2.0)
        self.declare_parameter("detect_plate", True)
        self.declare_parameter("detect_container", True)
        self.declare_parameter("ocr_enabled", False)
        self.declare_parameter("events_topic", "/vision/events")
        self.declare_parameter("detections_topic", "/vision/detections")

        self._pub_events = self.create_publisher(
            String, self.get_parameter("events_topic").value, 10
        )
        self._pub_detections = self.create_publisher(
            String, self.get_parameter("detections_topic").value, 10
        )

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("Vision node disabled (enabled=false).")
            return
        if _CV2_ERROR is not None:
            self.get_logger().error(f"OpenCV not installed: {_CV2_ERROR}")
            return
        if _VISION_IMPORT_ERROR is not None:
            self.get_logger().error(f"Vision deps missing: {_VISION_IMPORT_ERROR}")
            return

        self._cap = self._open_camera()
        if self._cap is None:
            self.get_logger().error("Camera not available for vision.")
            return

        interval = float(self.get_parameter("detect_interval_sec").value)
        self._timer = self.create_timer(max(0.1, interval), self._tick)
        self.get_logger().info("Vision node ready.")

    def _open_camera(self):
        device = str(self.get_parameter("camera_device").value)
        cap = cv2.VideoCapture(device)  # type: ignore[arg-type]
        if not cap or not cap.isOpened():
            return None
        width = int(self.get_parameter("camera_width").value)
        height = int(self.get_parameter("camera_height").value)
        fps = int(self.get_parameter("camera_fps").value)
        if width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps > 0:
            cap.set(cv2.CAP_PROP_FPS, fps)
        return cap

    def _publish(self, topic: String, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        topic.publish(msg)

    def _emit_event(self, payload: Dict[str, Any]) -> None:
        self._publish(self._pub_events, payload)
        if "boxes" in payload or "detections" in payload:
            self._publish(self._pub_detections, payload)

    def _tick(self) -> None:
        if not self._cap:
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return

        h, w = frame.shape[:2]
        boxes = []

        if bool(self.get_parameter("detect_plate").value):
            car, car_bbox = PlateDetection.detect_car(frame)  # type: ignore[call-arg]
            if car is not None and car_bbox:
                plate, plate_bbox = PlateDetection.detect_plate(car, car_bbox)  # type: ignore[call-arg]
                if plate is not None and plate_bbox:
                    cx1, cy1, _, _ = car_bbox
                    px1, py1, px2, py2 = plate_bbox
                    x1 = (cx1 + px1) / w
                    y1 = (cy1 + py1) / h
                    x2 = (cx1 + px2) / w
                    y2 = (cy1 + py2) / h
                    boxes.append(
                        {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "label": "plate"}
                    )
                    event = {"type": "plate_detected", "score": 0.8, "data": {}}
                    if bool(self.get_parameter("ocr_enabled").value):
                        try:
                            plate_text, _, _ = PlateDetection.detect_ocr(plate)  # type: ignore[call-arg]
                            event["data"]["plate"] = plate_text
                        except Exception:
                            pass
                    self._emit_event(event)

        if bool(self.get_parameter("detect_container").value):
            container, container_bbox = ContainerDetection.detect_container(frame)  # type: ignore[call-arg]
            if container is not None and container_bbox:
                x1, y1, x2, y2 = container_bbox
                boxes.append(
                    {"x1": x1 / w, "y1": y1 / h, "x2": x2 / w, "y2": y2 / h, "label": "container"}
                )
                self._emit_event({"type": "container_detected", "score": 0.7, "data": {}})

        if boxes:
            self._emit_event({"boxes": boxes, "ts": time.time()})


def main() -> None:
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "_cap") and node._cap is not None:
            node._cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
