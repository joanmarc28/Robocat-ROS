import os
import queue
import random
import subprocess
import threading
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeakerNode(Node):
    def __init__(self) -> None:
        super().__init__("speaker_node")
        self.declare_parameter("sounds_dir", "")
        self.declare_parameter("audio_device", "default")
        self.declare_parameter("tts_enabled", True)
        self.declare_parameter("tts_cmd", "espeak")
        self.declare_parameter("tts_voice", "ca")
        self.declare_parameter("tts_speed", 140)
        self.declare_parameter("emotion_sounds", {})

        self._queue: "queue.Queue[Tuple[str, str]]" = queue.Queue()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._sub_play = self.create_subscription(
            String, "audio/play_file", self._on_play_file, 10
        )
        self._sub_say = self.create_subscription(
            String, "audio/say", self._on_say_text, 10
        )
        self._sub_emotion = self.create_subscription(
            String, "audio/emotion", self._on_emotion, 10
        )

        self.get_logger().info("Speaker node ready.")

    def _start_thread_if_needed(self) -> None:
        with self._lock:
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._worker, daemon=True)
                self._thread.start()

    def _enqueue(self, kind: str, payload: str) -> None:
        self._queue.put((kind, payload))
        self._start_thread_if_needed()

    def _worker(self) -> None:
        while not self._queue.empty():
            kind, payload = self._queue.get()
            try:
                if kind == "file":
                    self._play_file(payload)
                elif kind == "tts":
                    self._say_text(payload)
            finally:
                self._queue.task_done()

    def _resolve_sound_path(self, name: str) -> Optional[Path]:
        path = Path(name)
        if not path.is_absolute():
            sounds_dir = str(self.get_parameter("sounds_dir").value).strip()
            if sounds_dir:
                path = Path(sounds_dir) / name
        if not path.exists():
            self.get_logger().warning(f"Sound not found: {path}")
            return None
        return path

    def _play_file(self, name: str) -> None:
        path = self._resolve_sound_path(name)
        if not path:
            return
        device = str(self.get_parameter("audio_device").value).strip()
        cmd = ["aplay"]
        if device:
            cmd += ["-D", device]
        cmd.append(str(path))
        try:
            subprocess.run(cmd, check=True)
        except Exception as exc:
            self.get_logger().warning(f"Failed to play {path.name}: {exc}")

    def _say_text(self, text: str) -> None:
        if not bool(self.get_parameter("tts_enabled").value):
            return
        cmd = str(self.get_parameter("tts_cmd").value).strip() or "espeak"
        voice = str(self.get_parameter("tts_voice").value).strip()
        speed = int(self.get_parameter("tts_speed").value)
        try:
            subprocess.run([cmd, f"-v{voice}", f"-s{speed}", text], check=True)
        except Exception as exc:
            self.get_logger().warning(f"TTS failed: {exc}")

    def _get_emotion_sounds(self, emotion: str) -> List[str]:
        sounds = self.get_parameter("emotion_sounds").value
        if isinstance(sounds, dict):
            value = sounds.get(emotion, [])
            if isinstance(value, list):
                return [str(v) for v in value]
        return []

    def _on_play_file(self, msg: String) -> None:
        if msg.data.strip():
            self._enqueue("file", msg.data.strip())

    def _on_say_text(self, msg: String) -> None:
        if msg.data.strip():
            self._enqueue("tts", msg.data.strip())

    def _on_emotion(self, msg: String) -> None:
        emotion = msg.data.strip()
        if not emotion:
            return
        options = self._get_emotion_sounds(emotion)
        if not options:
            self.get_logger().info(f"No sounds for emotion: {emotion}")
            return
        self._enqueue("file", random.choice(options))


def main() -> None:
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
