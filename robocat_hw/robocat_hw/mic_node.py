import json
import queue
import threading
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robocat_msgs.srv import SetLanguage

try:
    import speech_recognition as sr
    _SR_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    sr = None  # type: ignore
    _SR_IMPORT_ERROR = exc

try:
    import sounddevice as sd
    from vosk import KaldiRecognizer, Model
    _VOSK_IMPORT_ERROR: Optional[Exception] = None
except Exception as exc:  # pragma: no cover
    sd = None  # type: ignore
    KaldiRecognizer = None  # type: ignore
    Model = None  # type: ignore
    _VOSK_IMPORT_ERROR = exc


class MicNode(Node):
    def __init__(self) -> None:
        super().__init__("mic_node")
        self.declare_parameter("enabled", False)
        self.declare_parameter("stt_backend", "vosk")  # vosk | google
        self.declare_parameter("wake_word", "hola")
        self.declare_parameter("language", "ca-ES")
        self.declare_parameter("device_index", -1)
        self.declare_parameter("vosk_model_path", "/home/robocat-v2/.vosk/model")
        self.declare_parameter("vosk_model_map_json", "")
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("timeout", 6.0)
        self.declare_parameter("phrase_time_limit", 5.0)
        self.declare_parameter("publish_only_on_wake", False)
        self.declare_parameter("wake_threshold", 0.5)
        self.declare_parameter("pause_sec", 0.3)

        self._pub_heard = self.create_publisher(String, "audio/heard", 10)
        self._pub_wake = self.create_publisher(String, "audio/wake", 10)

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._model_lock = threading.Lock()
        self._vosk_reload = threading.Event()
        self._current_model_path: Optional[str] = None

        if not bool(self.get_parameter("enabled").value):
            self.get_logger().info("Mic node disabled (enabled=false).")
            return

        backend = str(self.get_parameter("stt_backend").value).strip().lower()
        if backend == "vosk":
            if _VOSK_IMPORT_ERROR is not None:
                self.get_logger().error(
                    "vosk not installed; mic node disabled."
                )
                return
        else:
            if _SR_IMPORT_ERROR is not None:
                self.get_logger().error(
                    "speech_recognition not installed; mic node disabled."
                )
                return

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

        self.create_service(SetLanguage, "audio/set_language", self._on_set_language)

    def destroy_node(self) -> bool:
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _select_device(self) -> int:
        device_index = int(self.get_parameter("device_index").value)
        if device_index >= 0:
            return device_index
        backend = str(self.get_parameter("stt_backend").value).strip().lower()
        if backend == "vosk":
            return -1
        if sr is None:
            return -1
        try:
            for i, _name in enumerate(sr.Microphone.list_microphone_names()):
                try:
                    with sr.Microphone(device_index=i) as _source:
                        return i
                except Exception:
                    continue
        except Exception:
            return -1
        return -1

    def _listen_once(
        self, recognizer: "sr.Recognizer", device_index: int
    ) -> Optional[str]:
        language = str(self.get_parameter("language").value)
        timeout = float(self.get_parameter("timeout").value)
        phrase_time_limit = float(self.get_parameter("phrase_time_limit").value)
        try:
            with sr.Microphone(device_index=device_index) as source:
                audio = recognizer.listen(
                    source, timeout=timeout, phrase_time_limit=phrase_time_limit
                )
            result = recognizer.recognize_google(audio, language=language, show_all=True)
            if isinstance(result, dict) and "alternative" in result:
                text = result["alternative"][0].get("transcript", "").lower()
                return text
        except Exception:
            return None
        return None

    def _match_wake(self, text: str) -> bool:
        wake = str(self.get_parameter("wake_word").value).lower().strip()
        if not wake:
            return False
        # Comparacio simple
        return wake in text

    def _resolve_model_path(self, language: str) -> Optional[str]:
        model_map = None
        raw = self.get_parameter("vosk_model_map_json").value
        if isinstance(raw, str) and raw.strip():
            try:
                model_map = json.loads(raw)
            except Exception:
                model_map = None
        if isinstance(model_map, dict) and language in model_map:
            return str(model_map[language])
        # fallback to ~/.vosk/<lang>
        home = Path.home()
        candidate = home / ".vosk" / language
        if candidate.exists():
            return str(candidate)
        # fallback to default path
        default_path = str(self.get_parameter("vosk_model_path").value).strip()
        if default_path:
            return default_path
        return None

    def _set_model_path(self, path: str) -> None:
        with self._model_lock:
            self._current_model_path = path
        self._vosk_reload.set()

    def _get_model_path(self) -> Optional[str]:
        with self._model_lock:
            return self._current_model_path

    def _run_vosk(self) -> None:
        if sd is None or Model is None or KaldiRecognizer is None:
            return
        sample_rate = int(self.get_parameter("sample_rate").value)
        q: "queue.Queue[bytes]" = queue.Queue()

        def _callback(indata, frames, time_info, status) -> None:
            if status:
                return
            q.put(bytes(indata))

        while not self._stop.is_set():
            model_path = self._get_model_path()
            if not model_path:
                model_path = str(self.get_parameter("vosk_model_path").value).strip()
            if not model_path:
                self.get_logger().warning("Vosk model path not set.")
                time.sleep(1.0)
                continue
            try:
                model = Model(model_path)
            except Exception as exc:
                self.get_logger().warning(f"Vosk model load failed: {exc}")
                time.sleep(1.0)
                continue

            recognizer = KaldiRecognizer(model, sample_rate)
            try:
                with sd.RawInputStream(
                    samplerate=sample_rate,
                    blocksize=8000,
                    dtype="int16",
                    channels=1,
                    callback=_callback,
                ):
                    self.get_logger().info(f"Vosk mic stream started ({model_path}).")
                    while not self._stop.is_set():
                        if self._vosk_reload.is_set():
                            self._vosk_reload.clear()
                            break
                        try:
                            data = q.get(timeout=0.5)
                        except Exception:
                            continue
                        if recognizer.AcceptWaveform(data):
                            result = recognizer.Result()
                            try:
                                text = json.loads(result).get("text", "").strip()
                            except Exception:
                                text = ""
                            if text:
                                self._publish_text(text)
            except Exception as exc:
                self.get_logger().warning(f"Vosk stream failed: {exc}")
                time.sleep(1.0)

    def _publish_text(self, text: str) -> None:
        only_on_wake = bool(self.get_parameter("publish_only_on_wake").value)
        is_wake = self._match_wake(text)
        if is_wake:
            msg = String()
            msg.data = text
            self._pub_wake.publish(msg)
        if (not only_on_wake) or is_wake:
            msg = String()
            msg.data = text
            self._pub_heard.publish(msg)

    def _run(self) -> None:
        backend = str(self.get_parameter("stt_backend").value).strip().lower()
        if backend == "vosk":
            # initialize model path with current language
            self._set_model_path(self._resolve_model_path(
                str(self.get_parameter("language").value)
            ) or "")
            self._run_vosk()
            return
        if sr is None:
            return
        recognizer = sr.Recognizer()
        device_index = self._select_device()
        if device_index < 0:
            self.get_logger().warning("No microphone device detected.")
            return

        self.get_logger().info(f"Microphone device index: {device_index}")
        while not self._stop.is_set():
            text = self._listen_once(recognizer, device_index)
            if text:
                self._publish_text(text)
            time.sleep(float(self.get_parameter("pause_sec").value))

    def _on_set_language(self, request: SetLanguage.Request, response: SetLanguage.Response) -> SetLanguage.Response:
        language = request.language.strip().lower()
        if not language:
            response.success = False
            response.message = "language is empty"
            return response
        model_path = self._resolve_model_path(language)
        if not model_path or not Path(model_path).exists():
            response.success = False
            response.message = f"model not found for language '{language}'"
            return response
        self.set_parameters([rclpy.parameter.Parameter("language", rclpy.Parameter.Type.STRING, language)])
        self._set_model_path(model_path)
        response.success = True
        response.message = f"language set to {language}"
        return response


def main() -> None:
    rclpy.init()
    node = MicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
