import logging
from pathlib import Path

from ultralytics import YOLO

logger = logging.getLogger(__name__)

_REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = _REPO_ROOT / "robocat_vision" / "assets"


class ContainerDetection:
    def detect_container(frame, camera=None):
        logger.info("Detectant container...")
        model = YOLO(
            str(
                ASSETS_DIR
                / "yolo"
                / "containers_model"
                / "weights"
                / "YOLO11seg_contenedores_batch1n_1216.pt"
            )
        )
        results = model.predict(frame, save=False, imgsz=416)
        if results:
            for result in results:
                if not result.boxes:
                    continue
                box = result.boxes[0]
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                if camera is not None:
                    camera.add_overlay_box((x1, y1, x2, y2), label="container")
                container = frame[y1:y2, x1:x2]
                return container, (x1, y1, x2, y2)
        return None, None
