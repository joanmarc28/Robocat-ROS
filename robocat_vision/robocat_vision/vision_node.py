import json
import time
import pickle
from pathlib import Path
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

try:
    import numpy as np
    _NP_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    np = None  # type: ignore
    _NP_ERROR = exc

try:
    from keras.models import load_model as keras_load_model
    _KERAS_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    keras_load_model = None  # type: ignore
    _KERAS_ERROR = exc


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
        self.declare_parameter("emotion_keras_model_path", "")
        self.declare_parameter("emotion_keras_label_encoder_path", "")
        self.declare_parameter(
            "emotion_keras_labels_json",
            '["angry","disgusted","fearful","happy","neutral","sad","surprised"]',
        )
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
        self._emotion_model = None
        self._emotion_label_encoder = None
        self._emotion_model_hw: Tuple[int, int] = (48, 48)
        self._emotion_channels_last: bool = True
        self._emotion_backend: str = "cascade"
        self._emotion_class_names: List[str] = []
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
        backend = str(self.get_parameter("emotion_backend").value).strip().lower() or "cascade"
        self._emotion_backend = backend
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

        if backend != "keras":
            if backend != "cascade":
                self.get_logger().warning(
                    f"Emotion backend '{backend}' is not implemented. Using 'cascade'."
                )
                self._emotion_backend = "cascade"
            return

        if _KERAS_ERROR is not None or keras_load_model is None:
            self.get_logger().warning(
                f"Keras backend unavailable ({_KERAS_ERROR}). Using cascade."
            )
            self._emotion_backend = "cascade"
            return
        if _NP_ERROR is not None or np is None:
            self.get_logger().warning(
                f"Numpy unavailable ({_NP_ERROR}). Using cascade."
            )
            self._emotion_backend = "cascade"
            return

        model_path, encoder_path = self._resolve_emotion_assets()
        if not model_path:
            self.get_logger().warning("Emotion model not found. Using cascade.")
            self._emotion_backend = "cascade"
            return

        try:
            self._emotion_model = keras_load_model(str(model_path), compile=False, safe_mode=False)
            if encoder_path and encoder_path.exists():
                with open(encoder_path, "rb") as f:
                    self._emotion_label_encoder = pickle.load(f)
            if self._emotion_label_encoder is not None and not self._has_emotion_labels(self._emotion_label_encoder):
                self.get_logger().warning(
                    "Loaded keras labels do not look like emotion classes. Using configured label mapping."
                )
                self._emotion_class_names = self._load_emotion_labels_from_param()
                self._emotion_label_encoder = None
            elif self._emotion_label_encoder is None:
                self.get_logger().warning(
                    "No valid emotion label encoder found. Using configured label mapping."
                )
                self._emotion_class_names = self._load_emotion_labels_from_param()
            else:
                classes = getattr(self._emotion_label_encoder, "classes_", [])
                self._emotion_class_names = [str(c).strip().lower() for c in classes]
            self._configure_emotion_input()
            self.get_logger().info(
                f"Keras emotion backend ready (model={model_path.name})."
            )
        except Exception as exc:
            self.get_logger().warning(f"Failed to load keras emotion backend: {exc}. Using cascade.")
            self._emotion_backend = "cascade"
            self._emotion_model = None
            self._emotion_label_encoder = None
            self._emotion_class_names = []

    def _has_emotion_labels(self, encoder: Any) -> bool:
        classes = getattr(encoder, "classes_", None)
        if classes is None:
            return False
        known = {
            "angry",
            "disgust",
            "disgusted",
            "fear",
            "scared",
            "happy",
            "sad",
            "surprise",
            "surprised",
            "neutral",
            "default",
        }
        values = {str(v).strip().lower() for v in classes}
        return len(values.intersection(known)) >= 2

    def _load_emotion_labels_from_param(self) -> List[str]:
        raw = str(self.get_parameter("emotion_keras_labels_json").value or "").strip()
        fallback = ["angry", "disgusted", "fearful", "happy", "neutral", "sad", "surprised"]
        if not raw:
            return fallback
        try:
            parsed = json.loads(raw)
            if isinstance(parsed, list):
                labels = [str(v).strip().lower() for v in parsed if str(v).strip()]
                return labels or fallback
        except Exception:
            pass
        return fallback

    def _resolve_emotion_assets(self) -> Tuple[Optional[Path], Optional[Path]]:
        model_cfg = str(self.get_parameter("emotion_keras_model_path").value).strip()
        encoder_cfg = str(self.get_parameter("emotion_keras_label_encoder_path").value).strip()

        model_candidates: List[Path] = []
        encoder_candidates: List[Path] = []
        if model_cfg:
            model_candidates.append(Path(model_cfg))
        if encoder_cfg:
            encoder_candidates.append(Path(encoder_cfg))

        module_assets = Path(__file__).resolve().parents[1] / "assets"
        repo_assets = Path.cwd() / "robocat_vision" / "assets"
        for base in [module_assets, repo_assets]:
            model_candidates.append(base / "models" / "cnn.keras")
            encoder_candidates.append(base / "label_encoder" / "label_encoder.pkl")
            encoder_candidates.append(base / "models" / "label_encoder" / "label_encoder.pkl")

        model_path = next((p for p in model_candidates if p.exists()), None)
        encoder_path = next((p for p in encoder_candidates if p.exists()), None)
        return model_path, encoder_path

    def _configure_emotion_input(self) -> None:
        if self._emotion_model is None:
            return
        input_shape = getattr(self._emotion_model, "input_shape", None)
        if isinstance(input_shape, list):
            input_shape = input_shape[0]
        # Expected patterns: (None, H, W, C) or (None, C, H, W).
        if isinstance(input_shape, tuple) and len(input_shape) == 4:
            if input_shape[-1] in (1, 3):
                self._emotion_channels_last = True
                self._emotion_model_hw = (int(input_shape[1] or 48), int(input_shape[2] or 48))
            elif input_shape[1] in (1, 3):
                self._emotion_channels_last = False
                self._emotion_model_hw = (int(input_shape[2] or 48), int(input_shape[3] or 48))

    def _map_emotion_label(self, label: str) -> str:
        value = (label or "").strip().lower()
        mapping = {
            "neutral": "default",
            "default": "default",
            "happy": "happy",
            "surprise": "surprised",
            "surprised": "surprised",
            "sad": "sad",
            "angry": "angry",
            "fear": "scared",
            "scared": "scared",
            "disgust": "disgusted",
            "disgusted": "disgusted",
        }
        return mapping.get(value, "default")

    def _predict_keras_emotion(self, roi_gray) -> Tuple[str, float]:
        if self._emotion_model is None or np is None:
            return "default", 0.0

        target_h, target_w = self._emotion_model_hw
        resized = cv2.resize(roi_gray, (target_w, target_h), interpolation=cv2.INTER_AREA)
        arr = resized.astype("float32") / 255.0

        if self._emotion_channels_last:
            batch = arr.reshape(1, target_h, target_w, 1)
        else:
            batch = arr.reshape(1, 1, target_h, target_w)

        pred = self._emotion_model.predict(batch, verbose=0)
        if isinstance(pred, list):
            pred = pred[0]
        pred_vec = pred[0] if hasattr(pred, "__len__") and len(pred) else pred
        if hasattr(pred_vec, "tolist"):
            pred_vec = pred_vec.tolist()
        idx = int(np.argmax(pred_vec))
        conf = float(pred_vec[idx])

        label = "default"
        if self._emotion_label_encoder is not None:
            try:
                label = str(self._emotion_label_encoder.inverse_transform([idx])[0])
            except Exception:
                classes = getattr(self._emotion_label_encoder, "classes_", None)
                if classes is not None and len(classes) > idx:
                    label = str(classes[idx])
        elif self._emotion_class_names and idx < len(self._emotion_class_names):
            label = self._emotion_class_names[idx]
        return self._map_emotion_label(label), conf

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
        # police -> plate, city -> container, cat/unknown -> emotion.
        mode = self._mode_name
        if mode == "police":
            want_plate, want_container, want_emotion = True, False, False
        elif mode == "city":
            want_plate, want_container, want_emotion = False, True, False
        else:
            # cat and unknown fallback
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

        if self._emotion_backend == "keras":
            emotion, confidence = self._predict_keras_emotion(roi)
        else:
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
            "confidence": confidence,
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
                score = float(emotion_data.get("confidence") or 0.0)
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
