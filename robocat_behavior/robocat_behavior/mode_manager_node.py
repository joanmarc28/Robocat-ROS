import json
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class Action:
    movement: Optional[str] = None
    oled_anim: Optional[str] = None
    oled_text: Optional[str] = None
    audio_emotion: Optional[str] = None
    audio_say: Optional[str] = None


class ModeManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("mode_manager_node")
        self.declare_parameter("default_mode", "cat")
        self.declare_parameter("default_submode", "default")
        self.declare_parameter("events_topic", "/vision/events")
        self.declare_parameter("command_topic", "/robocat/cmd")
        self.declare_parameter("mode_topic", "/robot/mode")
        self.declare_parameter("movement_topic", "/robocat/cmd")
        self.declare_parameter("oled_anim_topic", "/oled_anim")
        self.declare_parameter("oled_text_topic", "/oled_text")
        self.declare_parameter("audio_emotion_topic", "/audio/emotion")
        self.declare_parameter("audio_say_topic", "/audio/say")
        self.declare_parameter("min_repeat_sec", 1.0)
        self.declare_parameter("event_hold_sec", 2.0)
        self.declare_parameter("actions_enabled", True)

        self._mode = str(self.get_parameter("default_mode").value).strip().lower()
        self._submode = str(self.get_parameter("default_submode").value).strip().lower()

        self._last_event_priority = -1
        self._last_event_time = 0.0

        self._pub_mode = self.create_publisher(
            String, self.get_parameter("mode_topic").value, 10
        )
        self._pub_move = self.create_publisher(
            String, self.get_parameter("movement_topic").value, 10
        )
        self._pub_oled_anim = self.create_publisher(
            String, self.get_parameter("oled_anim_topic").value, 10
        )
        self._pub_oled_text = self.create_publisher(
            String, self.get_parameter("oled_text_topic").value, 10
        )
        self._pub_audio_emotion = self.create_publisher(
            String, self.get_parameter("audio_emotion_topic").value, 10
        )
        self._pub_audio_say = self.create_publisher(
            String, self.get_parameter("audio_say_topic").value, 10
        )

        self._last_sent: Dict[str, tuple[float, str]] = {}

        self.create_subscription(
            String, self.get_parameter("events_topic").value, self._on_event, 10
        )
        self.create_subscription(
            String, self.get_parameter("command_topic").value, self._on_command, 10
        )

        self.create_timer(0.5, self._tick)
        self._publish_mode()
        self.get_logger().info("Mode manager ready.")

    def _publish_mode(self) -> None:
        msg = String()
        msg.data = f"{self._mode}:{self._submode}"
        self._pub_mode.publish(msg)

    def _send(self, topic: str, value: str, publisher) -> None:
        if not value:
            return
        min_repeat = float(self.get_parameter("min_repeat_sec").value)
        last_time, last_value = self._last_sent.get(topic, (0.0, ""))
        now = time.time()
        if value == last_value and now - last_time < min_repeat:
            return
        msg = String()
        msg.data = value
        publisher.publish(msg)
        self._last_sent[topic] = (now, value)

    def _emit(self, action: Action) -> None:
        if action.movement:
            self._send("movement", action.movement, self._pub_move)
        if action.oled_anim:
            self._send("oled_anim", action.oled_anim, self._pub_oled_anim)
        if action.oled_text:
            self._send("oled_text", action.oled_text, self._pub_oled_text)
        if action.audio_emotion:
            self._send("audio_emotion", action.audio_emotion, self._pub_audio_emotion)
        if action.audio_say:
            self._send("audio_say", action.audio_say, self._pub_audio_say)

    def _on_command(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        lower = raw.lower()
        if lower.startswith("mode:") or lower.startswith("mode "):
            mode = lower.split(":", 1)[1] if ":" in lower else lower.split(" ", 1)[1]
            self._set_mode(mode.strip())
            return
        if lower.startswith("submode:") or lower.startswith("submode "):
            submode = lower.split(":", 1)[1] if ":" in lower else lower.split(" ", 1)[1]
            self._set_submode(submode.strip())
            return
        if lower.startswith("emotion:"):
            self._emit(Action(audio_emotion=lower.split(":", 1)[1].strip()))
            return
        if lower.startswith("say:"):
            self._emit(Action(audio_say=raw.split(":", 1)[1].strip()))
            return

        # default: treat as movement command passthrough
        self._emit(Action(movement=raw))

    def _set_mode(self, mode: str) -> None:
        if not mode:
            return
        if mode == self._mode:
            return
        self._mode = mode
        self._submode = "default"
        self._publish_mode()

    def _set_submode(self, submode: str) -> None:
        if not submode:
            return
        if submode == self._submode:
            return
        self._submode = submode
        self._publish_mode()

    def _event_priority(self, event_type: str) -> int:
        priority = [
            "obstacle_close",
            "fall_detected",
            "low_battery",
            "plate_detected",
            "alert",
            "human_emotion_angry",
            "human_emotion_scared",
            "human_emotion_disgusted",
            "human_emotion_happy",
            "human_emotion_sad",
            "container_detected",
            "idle",
        ]
        try:
            return len(priority) - priority.index(event_type)
        except ValueError:
            return 0

    def _on_event(self, msg: String) -> None:
        if not bool(self.get_parameter("actions_enabled").value):
            return
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            event = json.loads(raw)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid event JSON: {raw}")
            return

        event_type = str(event.get("type") or "").strip().lower()
        data = event.get("data") or {}
        score = float(event.get("score") or 0.0)

        if not event_type:
            return

        # normalize emotion events
        if event_type == "human_emotion":
            emotion = str(data.get("emotion") or "default").strip().lower()
            event_type = f"human_emotion_{emotion}"

        priority = self._event_priority(event_type)
        now = time.time()
        if priority < self._last_event_priority and (now - self._last_event_time) < float(
            self.get_parameter("event_hold_sec").value
        ):
            return

        action = self._map_event_to_action(event_type, data, score)
        if action:
            self._last_event_priority = priority
            self._last_event_time = now
            self._emit(action)

    def _map_event_to_action(self, event_type: str, data: Dict[str, Any], score: float) -> Optional[Action]:
        mode = self._mode
        if event_type in {"obstacle_close", "fall_detected"}:
            return Action(movement="stop", oled_text="Obstacle", audio_emotion="sad")
        if event_type == "low_battery":
            return Action(movement="sit", oled_text="Low battery", audio_emotion="sad")

        if mode == "police":
            if event_type == "plate_detected":
                plate = str((data or {}).get("plate") or "").strip().upper()
                text = f"Matricula: {plate}" if plate else "Matricula detectada"
                return Action(
                    movement="rotar",
                    oled_anim="patrol",
                    oled_text=text,
                    audio_emotion="siren",
                )
            if event_type.startswith("human_emotion_angry"):
                return Action(movement="walk_back", oled_anim="angry", audio_emotion="angry")
            return Action(movement="patrol", oled_anim="patrol")

        if mode == "human":
            if event_type.startswith("human_emotion_happy"):
                return Action(movement="maneta", oled_anim="happy", audio_emotion="happy")
            if event_type.startswith("human_emotion_sad"):
                return Action(movement="sit", oled_anim="sad", audio_emotion="sad")
            if event_type.startswith("human_emotion_angry"):
                return Action(movement="walk_back", oled_anim="angry", audio_emotion="angry")
            if event_type.startswith("human_emotion_surprised"):
                return Action(movement="strech", oled_anim="surprised", audio_emotion="surprised")
            return Action(oled_anim="default")

        if mode == "city":
            if event_type == "container_detected":
                return Action(movement="approach", oled_anim="surprised", audio_emotion="happy")
            return Action(movement="patrol", oled_anim="default")

        # cat/default
        if event_type.startswith("human_emotion_happy"):
            return Action(movement="maneta", oled_anim="happy", audio_emotion="happy")
        if event_type.startswith("human_emotion_sad"):
            return Action(movement="normal", oled_anim="sad", audio_emotion="sad")
        if event_type.startswith("human_emotion_angry"):
            return Action(movement="walk_back", oled_anim="angry", audio_emotion="angry")
        if event_type.startswith("human_emotion_surprised"):
            return Action(movement="strech", oled_anim="surprised", audio_emotion="surprised")
        return Action(oled_anim="default")

    def _tick(self) -> None:
        hold = float(self.get_parameter("event_hold_sec").value)
        if self._last_event_priority <= 0:
            return
        if time.time() - self._last_event_time > hold:
            self._last_event_priority = -1
            self._emit(Action(oled_anim="default"))


def main() -> None:
    rclpy.init()
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
