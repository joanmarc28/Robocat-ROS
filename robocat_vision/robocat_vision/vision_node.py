import json
import time
from typing import Any, Dict, List, Optional, Tuple

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
    _PLATE_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    PlateDetection = None  # type: ignore
    _PLATE_IMPORT_ERROR = exc

try:
    from robocat_vision.container_detection import ContainerDetection
    _CONTAINER_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    ContainerDetection = None  # type: ignore
    _CONTAINER_IMPORT_ERROR = exc


class VisionNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_node")
        self.declare_parameter("enabled", True)
        self.declare_parameter("mode_topic", "/robot/mode")
        self.declare_parameter("camera_device", "/dev/video2")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 15)
        self.declare_parameter("detect_interval_sec", 2.0)
        self.declare_parameter("detect_plate", True)
        self.declare_parameter("detect_container", True)
        self.declare_parameter("detect_human_emotion", True)
        self.declare_parameter("ocr_enabled", False)
        self.declare_parameter("emotion_backend", "cascade")
        self.declare_parameter("emotion_interval_sec", 0.5)
        self.declare_parameter("emotion_cooldown_sec", 1.5)
        self.declare_parameter("emotion_min_confidence", 0.55)
        self.declare_parameter("overlay_face_label_prefix", "face")
        self.declare_parameter("overlay_face_label_default", "face")
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
        self._base_plate_enabled = bool(self.get_parameter("detect_plate").value)
        self._base_container_enabled = bool(self.get_parameter("detect_container").value)
        self._base_emotion_enabled = bool(self.get_parameter("detect_human_emotion").value)
        self._plate_enabled = self._base_plate_enabled
        self._container_enabled = self._base_container_enabled
        self._emotion_enabled = self._base_emotion_enabled
        self._mode_name = "cat"
        if self._plate_enabled and _PLATE_IMPORT_ERROR is not None:
            self.get_logger().warning(
                f"Plate detection disabled (missing deps): {_PLATE_IMPORT_ERROR}"
            )
            self._base_plate_enabled = False
            self._plate_enabled = False
        if self._container_enabled and _CONTAINER_IMPORT_ERROR is not None:
            self.get_logger().warning(
                f"Container detection disabled (missing deps): {_CONTAINER_IMPORT_ERROR}"
            )
            self._base_container_enabled = False
            self._container_enabled = False

        self._last_emotion: str = ""
        self._last_emotion_ts: float = 0.0
        self._next_emotion_ts: float = 0.0
        self._face_cascade = None
        self._smile_cascade = None
        self._setup_emotion_backend()
        self._update_detection_flags()

        self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self._on_mode,
            10,
        )

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

    def _setup_emotion_backend(self) -> None:
        if not self._base_emotion_enabled:
            return
        backend = str(self.get_parameter("emotion_backend").value).strip().lower()
        if backend != "cascade":
            self.get_logger().warning(
                f"Emotion backend '{backend}' is not implemented yet. Using 'cascade'."
            )
        if cv2 is None:
            return
        face_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        smile_path = cv2.data.haarcascades + "haarcascade_smile.xml"
        self._face_cascade = cv2.CascadeClassifier(face_path)
        self._smile_cascade = cv2.CascadeClassifier(smile_path)
        if self._face_cascade.empty():
            self._face_cascade = None
            self.get_logger().warning("Face cascade not available; emotion detection disabled.")
        if self._smile_cascade.empty():
            self._smile_cascade = None
            self.get_logger().warning("Smile cascade not available; happy detection degraded.")

    def _on_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if not raw:
            return
        mode = raw.split(":", 1)[0].strip() if ":" in raw else raw
        if not mode:
            return
        if mode == self._mode_name:
            return
        self._mode_name = mode
        self._update_detection_flags()

    def _update_detection_flags(self) -> None:
        # Fixed simple rules by mode:
        # police -> plate, city -> container, human/cat -> emotion.
        mode = self._mode_name
        if mode == "police":
            want_plate, want_container, want_emotion = True, False, False
        elif mode == "city":
            want_plate, want_container, want_emotion = False, True, False
        else:
            # human, cat and unknown fallback
            want_plate, want_container, want_emotion = False, False, True

        self._plate_enabled = self._base_plate_enabled and want_plate
        self._container_enabled = self._base_container_enabled and want_container
        self._emotion_enabled = self._base_emotion_enabled and want_emotion
        self.get_logger().info(
            f"Vision mode={mode} -> plate={self._plate_enabled} container={self._container_enabled} emotion={self._emotion_enabled}"
        )

    def _publish(self, topic: String, payload: Dict[str, Any]) -> None:
        def _json_default(value: Any) -> Any:
            # Convert numpy scalar types (bool_, int32, float32, etc.) to native Python.
            if hasattr(value, "item"):
                try:
                    return value.item()
                except Exception:
                    pass
            raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, default=_json_default)
        topic.publish(msg)

    def _emit_event(self, payload: Dict[str, Any]) -> None:
        self._publish(self._pub_events, payload)
        if "boxes" in payload or "detections" in payload:
            self._publish(self._pub_detections, payload)

    def _select_primary_face(
        self, faces: List[Tuple[int, int, int, int]]
    ) -> Optional[Tuple[int, int, int, int]]:
        if not faces:
            return None
        return sorted(faces, key=lambda b: b[2] * b[3], reverse=True)[0]

    def _detect_human_emotion(
        self, frame
    ) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
        if self._face_cascade is None:
            return None, []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self._face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60)
        )
        if len(faces) == 0:
            return None, []

        h, w = frame.shape[:2]
        selected = self._select_primary_face(list(faces))
        if selected is None:
            return None, []

        x, y, fw, fh = selected
        roi = gray[y : y + fh, x : x + fw]

        smile_found = False
        if self._smile_cascade is not None and roi.size > 0:
            smiles = self._smile_cascade.detectMultiScale(
                roi, scaleFactor=1.7, minNeighbors=20, minSize=(20, 20)
            )
            smile_found = len(smiles) > 0

        emotion = "happy" if smile_found else "default"
        confidence = 0.82 if smile_found else 0.58
        min_conf = float(self.get_parameter("emotion_min_confidence").value)
        if confidence < min_conf:
            return None, []

        now = time.time()
        cooldown = float(self.get_parameter("emotion_cooldown_sec").value)
        if emotion == self._last_emotion and (now - self._last_emotion_ts) < cooldown:
            return None, []
        self._last_emotion = emotion
        self._last_emotion_ts = now

        center_x = x + (fw / 2.0)
        center_y = y + (fh / 2.0)
        cx_norm = center_x / float(w)
        cy_norm = center_y / float(h)
        attention = bool(0.35 <= cx_norm <= 0.65 and 0.25 <= cy_norm <= 0.75)
        eye_contact = bool(attention)

        payload = {
            "emotion": emotion,
            "attention": attention,
            "eye_contact": eye_contact,
            "gesture": "unknown",
            "aggression": "none",
            "head_pose": {"pitch": 0.0, "yaw": 0.0, "roll": 0.0},
            "face_bbox": {"x1": x, "y1": y, "x2": x + fw, "y2": y + fh},
        }
        prefix = str(self.get_parameter("overlay_face_label_prefix").value).strip() or "face"
        default_label = str(self.get_parameter("overlay_face_label_default").value).strip() or prefix
        face_label = default_label if emotion == "default" else f"{prefix}:{emotion}"
        boxes = [
            {
                "x1": x / float(w),
                "y1": y / float(h),
                "x2": (x + fw) / float(w),
                "y2": (y + fh) / float(h),
                "label": face_label,
            }
        ]
        return payload, boxes

    def _tick(self) -> None:
        if not self._cap:
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return

        h, w = frame.shape[:2]
        boxes = []

        if self._emotion_enabled:
            now = time.time()
            if now < self._next_emotion_ts:
                emotion_data = None
                emotion_boxes = []
            else:
                emotion_data, emotion_boxes = self._detect_human_emotion(frame)
                self._next_emotion_ts = now + float(
                    self.get_parameter("emotion_interval_sec").value
                )
            if emotion_data is not None:
                score = 0.82 if emotion_data.get("emotion") == "happy" else 0.58
                self._emit_event(
                    {"type": "human_emotion", "score": score, "data": emotion_data}
                )
            if emotion_boxes:
                boxes.extend(emotion_boxes)

        if self._plate_enabled:
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

        if self._container_enabled:
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
