import logging
import os
import pickle
from pathlib import Path

import cv2
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from ultralytics import YOLO

logger = logging.getLogger(__name__)

_REPO_ROOT = Path(__file__).resolve().parents[2]
ASSETS_DIR = _REPO_ROOT / "robocat_vision" / "assets"


class PlateDetection:
    # Funcions de train (nomes s'executen un cop)
    def car_train():
        model = YOLO(str(ASSETS_DIR / "models" / "yolov8n.pt"))
        results = model.train(
            data=str(ASSETS_DIR / "yolo" / "car_model" / "config_car.yaml"),
            epochs=15,
            imgsz=416,
            batch=8,
            device="cpu",
            cache=True,
            workers=4,
            project=str(ASSETS_DIR / "yolo" / "car_model" / "runs" / "train_fast"),
            name="yolov8n_cotxes",
        )
        return results

    def plate_train():
        model = YOLO(str(ASSETS_DIR / "models" / "yolov8n.pt"))
        results = model.train(
            data=str(ASSETS_DIR / "yolo" / "plate_model" / "config_plate.yaml"),
            epochs=15,
            imgsz=416,
            batch=8,
            device="cpu",
            cache=True,
            workers=4,
            project=str(ASSETS_DIR / "yolo" / "plate_model" / "runs" / "train_fast"),
            name="yolov8n_plates",
        )
        return results

    def ocr_train():
        try:
            from keras.models import Sequential
            from tensorflow.keras.layers import Conv2D, Dense, Dropout, Flatten, MaxPooling2D
            from tensorflow.keras.optimizers import Adam
            from tensorflow.keras.preprocessing.image import ImageDataGenerator
            from tensorflow.keras.utils import to_categorical
        except Exception as exc:
            raise RuntimeError(f"OCR training requires keras/tensorflow: {exc}") from exc

        dataset_path = str(ASSETS_DIR / "database" / "plate_ocr" / "chars74k")
        images = []
        labels = []

        for class_name in os.listdir(dataset_path):
            class_dir = os.path.join(dataset_path, class_name)
            if not os.path.isdir(class_dir):
                continue
            for img_name in os.listdir(class_dir):
                img_path = os.path.join(class_dir, img_name)
                img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                if img is None:
                    continue
                img = cv2.resize(img, (32, 32))
                images.append(img)
                labels.append(class_name)

        images = np.array(images) / 255.0
        images = images.reshape(-1, 32, 32, 1)

        label_encoder = LabelEncoder()
        labels_encoded = label_encoder.fit_transform(labels)
        labels_categorical = to_categorical(labels_encoded)

        with open(ASSETS_DIR / "label_encoder" / "label_encoder.pkl", "wb") as f:
            pickle.dump(label_encoder, f)

        X_train, X_test, y_train, y_test = train_test_split(
            images, labels_categorical, test_size=0.2, random_state=42
        )

        datagen = ImageDataGenerator(
            rotation_range=5,
            width_shift_range=0.05,
            height_shift_range=0.05,
            zoom_range=0.1,
            shear_range=2,
            fill_mode="nearest",
        )

        cnn = Sequential(
            [
                Conv2D(64, (3, 3), activation="relu", input_shape=(32, 32, 1)),
                MaxPooling2D((2, 2)),
                Conv2D(128, (3, 3), activation="relu"),
                MaxPooling2D((2, 2)),
                Flatten(),
                Dense(256, activation="relu"),
                Dropout(0.5),
                Dense(len(label_encoder.classes_), activation="softmax"),
            ]
        )

        cnn.compile(optimizer=Adam(), loss="categorical_crossentropy", metrics=["accuracy"])
        cnn.fit(
            datagen.flow(X_train, y_train, batch_size=16),
            epochs=20,
            validation_data=(X_test, y_test),
        )
        cnn.save(str(ASSETS_DIR / "models" / "cnn_plate.keras"), save_format="keras")

    # Funcions de deteccio
    def detect_car(frame, camera=None):
        logger.info("Detectant cotxe...")
        model = YOLO(
            str(
                ASSETS_DIR
                / "yolo"
                / "car_model"
                / "runs"
                / "train_fast"
                / "yolov8n_cotxes"
                / "weights"
                / "best.pt"
            )
        )
        results = model.predict(frame, save=False, imgsz=416)
        if results:
            for result in results:
                if result.boxes:
                    box = result.boxes[0]
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    if camera is not None:
                        camera.add_overlay_box((x1, y1, x2, y2), label="car")
                    car = frame[y1:y2, x1:x2]
                    return car, (x1, y1, x2, y2)
        return None, None

    def detect_plate(car, car_bbox=None, camera=None):
        model = YOLO(
            str(
                ASSETS_DIR
                / "yolo"
                / "plate_model"
                / "runs"
                / "train_fast"
                / "yolov8n_plates4"
                / "weights"
                / "best.pt"
            )
        )
        results = model.predict(car, save=False, imgsz=416)
        if results:
            for result in results:
                if result.boxes:
                    box = result.boxes[0]
                    px1, py1, px2, py2 = map(int, box.xyxy[0].tolist())
                    if camera is not None and car_bbox is not None:
                        cx1, cy1, _, _ = car_bbox
                        camera.add_overlay_box(
                            (cx1 + px1, cy1 + py1, cx1 + px2, cy1 + py2),
                            label="plate",
                            color=(0, 0, 255),
                        )
                    plate = car[py1:py2, px1:px2]
                    if plate is not None:
                        only_plate = PlateDetection.crop_nationality(plate)
                        return only_plate, (px1, py1, px2, py2)
        return None, None

    def detect_ocr(plate):
        result, char_imgs, chars = PlateDetection.predict_plate_text(plate)
        return result, char_imgs, chars

    # Helpers
    def preprocess(img):
        img = cv2.resize(img, (32, 32))
        img = img / 255.0
        return img.reshape(1, 32, 32, 1)

    def resize_img(img, size=32):
        h, w = img.shape
        scale = size / max(h, w)
        new_w, new_h = int(w * scale), int(h * scale)
        resized = cv2.resize(img, (new_w, new_h))
        canvas = np.zeros((size, size), dtype=np.uint8) * 255
        x_offset = (size - new_w) // 2
        y_offset = (size - new_h) // 2
        canvas[y_offset : y_offset + new_h, x_offset : x_offset + new_w] = resized
        return canvas

    def segment_characters(plate_img):
        gray = cv2.cvtColor(plate_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thr = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        char_regions = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if 15 < h < 100 and 5 < w < 80 and h > w:
                char_regions.append((x, y, w, h))

        char_regions = sorted(char_regions, key=lambda b: b[0])
        char_imgs = []
        for x, y, w, h in char_regions:
            char = thr[y : y + h, x : x + w]
            char = PlateDetection.resize_img(char)
            char = 255 - char
            char_imgs.append(char)
        return char_imgs

    def predict_plate_text(
        plate_img,
        model_path=str(ASSETS_DIR / "models" / "cnn.keras"),
        label_encoder_path=str(ASSETS_DIR / "label_encoder" / "label_encoder.pkl"),
    ):
        try:
            from keras.models import load_model
        except Exception as exc:
            raise RuntimeError(f"OCR inference requires keras/tensorflow: {exc}") from exc

        char_imgs = PlateDetection.segment_characters(plate_img)
        text = ""
        model = load_model(model_path, compile=False, safe_mode=False)
        with open(label_encoder_path, "rb") as f:
            label_encoder = pickle.load(f)
        for char_img in char_imgs:
            processed = PlateDetection.preprocess(char_img)
            prediction = model.predict(processed)
            predicted_label = label_encoder.inverse_transform([np.argmax(prediction)])[0]
            text += predicted_label

        corrected = PlateDetection.correct_plate_format(text)
        return corrected, char_imgs, list(corrected)

    def correct_plate_format(text):
        if len(text) != 7:
            return "No trobada"
        corrected = ""
        for i in range(4):
            if text[i].isalpha() and text[i].upper() in ["L", "S", "G", "B"]:
                corrected += {"L": "4", "S": "5", "G": "6", "B": "8"}[text[i].upper()]
            else:
                corrected += text[i]
        for i in range(3):
            if text[i + 4].isdigit() and text[i + 4] in ["4", "5", "6", "8"]:
                map_num_to_letter = {"4": "L", "5": "S", "6": "G", "8": "B"}
                corrected += map_num_to_letter.get(text[i + 4], "A")
            else:
                corrected += text[i + 4].upper()
        return corrected

    def crop_nationality(img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_pixels = cv2.countNonZero(mask)
        h, w, _ = img.shape
        blue_ratio = blue_pixels / (h * int(w * 0.15))
        if blue_ratio > 0.05:
            plate_w = int(w * 0.12)
            return img[:, plate_w:]
        return img
