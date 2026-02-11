import json
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"true", "1", "yes", "si", "s", "y", "on"}


def _to_float01(value: Any) -> float:
    try:
        if isinstance(value, (int, float)):
            v = float(value)
        else:
            s = str(value).strip().lower()
            if s.endswith("%"):
                v = float(s[:-1]) / 100.0
            elif s in {"", "none", "null"}:
                v = 0.0
            elif s in {"low", "baixa"}:
                v = 0.2
            elif s in {"medium", "mitjana"}:
                v = 0.5
            elif s in {"high", "alta"}:
                v = 0.8
            else:
                v = float(s)
    except Exception:
        v = 0.0
    return max(0.0, min(1.0, v))


def _parse_head_pose(value: Any) -> Dict[str, float]:
    if isinstance(value, dict):
        return {
            "pitch": float(value.get("pitch", 0.0)),
            "yaw": float(value.get("yaw", 0.0)),
            "roll": float(value.get("roll", 0.0)),
        }
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        return {"pitch": float(value[0]), "yaw": float(value[1]), "roll": float(value[2]) if len(value) > 2 else 0.0}
    if isinstance(value, str):
        s = value.strip().lower()
        if s in {"left"}:
            return {"pitch": 0.0, "yaw": -30.0, "roll": 0.0}
        if s in {"right"}:
            return {"pitch": 0.0, "yaw": 30.0, "roll": 0.0}
        if s in {"up"}:
            return {"pitch": -20.0, "yaw": 0.0, "roll": 0.0}
        if s in {"down"}:
            return {"pitch": 20.0, "yaw": 0.0, "roll": 0.0}
    return {"pitch": 0.0, "yaw": 0.0, "roll": 0.0}


class BehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("behavior_node")
        self.declare_parameter("mode_topic", "/robot/mode")
        self.declare_parameter("events_topic", "/vision/events")
        self.declare_parameter("movement_topic", "/robocat/cmd")
        self.declare_parameter("oled_anim_topic", "/oled_anim")
        self.declare_parameter("oled_anim_state_topic", "/oled_anim_state")
        self.declare_parameter("oled_text_topic", "/oled_text")
        self.declare_parameter("audio_emotion_topic", "/audio/emotion")
        self.declare_parameter("min_repeat_sec", 1.0)
        self.declare_parameter("event_anim_hold_sec", 2.5)
        self.declare_parameter("cat_idle_anim", "default")
        self.declare_parameter("city_idle_anim", "default")
        self.declare_parameter("police_idle_anim", "default")
        self.declare_parameter("city_container_anim", "surprised")
        self.declare_parameter("police_detect_anim", "patrol")
        self.declare_parameter("plate_similarity_threshold", 0.85)
        self.declare_parameter("plate_max_history", 5)

        self._mode = "cat"
        self._submode = "default"

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

        self._last_sent: Dict[str, Tuple[float, str]] = {}
        self._plate_history: List[str] = []
        self._event_anim_active: bool = False
        self._event_anim_deadline: float = 0.0
        self._event_anim_name: str = ""

        self.create_subscription(
            String, self.get_parameter("mode_topic").value, self._on_mode, 10
        )
        self.create_subscription(
            String, self.get_parameter("oled_anim_state_topic").value, self._on_oled_anim_state, 10
        )
        self.create_subscription(
            String, self.get_parameter("events_topic").value, self._on_event, 10
        )
        self.create_timer(0.5, self._tick)

        self.get_logger().info("Behavior node ready.")

    def _send(self, key: str, value: str, publisher) -> None:
        if not value:
            return
        min_repeat = float(self.get_parameter("min_repeat_sec").value)
        last_time, last_value = self._last_sent.get(key, (0.0, ""))
        now = time.time()
        if value == last_value and now - last_time < min_repeat:
            return
        msg = String()
        msg.data = value
        publisher.publish(msg)
        self._last_sent[key] = (now, value)

    def _on_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if not raw:
            return
        if ":" in raw:
            mode, submode = raw.split(":", 1)
        else:
            mode, submode = raw, "default"
        # Keep backward compatibility: old "human" mode maps to "cat".
        if mode == "human":
            mode = "cat"
        self._mode = mode or "cat"
        self._submode = submode or "default"
        self._send_idle_anim(force=True)

    def _idle_anim(self) -> str:
        if self._mode == "police":
            return str(self.get_parameter("police_idle_anim").value)
        if self._mode == "city":
            return str(self.get_parameter("city_idle_anim").value)
        return str(self.get_parameter("cat_idle_anim").value)

    def _set_event_anim(self, anim: str) -> None:
        # Do not interrupt an active one-shot emotion animation.
        if self._event_anim_active:
            return
        # one-shot event animation: play full sequence once, then return to idle
        self._send("oled_anim", f"{anim}|once", self._pub_oled_anim)
        self._event_anim_active = True
        self._event_anim_name = (anim or "").strip().lower()
        hold = float(self.get_parameter("event_anim_hold_sec").value)
        # hold <= 0 means "no timeout": wait for OLED done/stopped state.
        if hold > 0.0:
            self._event_anim_deadline = time.time() + hold
        else:
            self._event_anim_deadline = 0.0

    def _on_oled_anim_state(self, msg: String) -> None:
        state = (msg.data or "").strip().lower()
        if not state:
            return
        if not (state.startswith("done:") or state.startswith("stopped:")):
            return
        if not self._event_anim_active:
            return

        # Example states:
        #   done:happy
        #   stopped:happy|once
        #   stopped:default
        anim_state = state.split(":", 1)[1].strip()
        anim_name = anim_state.split("|", 1)[0].strip()
        if not anim_name:
            return
        if self._event_anim_name and anim_name != self._event_anim_name:
            # Ignore unrelated idle/default stop events.
            return

        self._event_anim_active = False
        self._event_anim_deadline = 0.0
        self._event_anim_name = ""
        self._send_idle_anim(force=True)

    def _send_idle_anim(self, force: bool = False) -> None:
        if self._event_anim_active:
            # Fallback only: in case OLED state messages are missing, eventually recover.
            if self._event_anim_deadline > 0.0 and time.time() >= self._event_anim_deadline:
                self._event_anim_active = False
                self._event_anim_deadline = 0.0
                self._event_anim_name = ""
            elif not force:
                return
        self._send("oled_anim", self._idle_anim(), self._pub_oled_anim)

    def _plates_are_similar(self, plate1: str, plate2: str) -> bool:
        if not plate1 or not plate2:
            return False
        if len(plate1) != len(plate2):
            return False
        matches = sum(c1 == c2 for c1, c2 in zip(plate1, plate2))
        similarity = matches / len(plate1)
        threshold = float(self.get_parameter("plate_similarity_threshold").value)
        return similarity >= threshold

    def _handle_police(self, event_type: str, data: Dict[str, Any]) -> None:
        if event_type != "plate_detected":
            return
        plate = str(data.get("plate") or "").strip().upper()
        if plate:
            for prev in self._plate_history:
                if self._plates_are_similar(plate, prev):
                    self.get_logger().info(f"Matrícula similar detectada, s'ignora: {plate}")
                    return
            self._plate_history.append(plate)
            if len(self._plate_history) > int(self.get_parameter("plate_max_history").value):
                self._plate_history.pop(0)
            self._send("oled_text", f"Matricula: {plate}", self._pub_oled_text)
        else:
            self._send("oled_text", "Matricula detectada", self._pub_oled_text)
        self._set_event_anim(str(self.get_parameter("police_detect_anim").value))
        self._send("audio_emotion", "siren", self._pub_audio_emotion)
        self._send("movement", "rotar", self._pub_move)

    def _handle_city(self, event_type: str) -> None:
        if event_type != "container_detected":
            return
        self._set_event_anim(str(self.get_parameter("city_container_anim").value))
        self._send("audio_emotion", "happy", self._pub_audio_emotion)
        self._send("movement", "endavant", self._pub_move)

    def _handle_emotion(self, emotion: str, context: Dict[str, Any]) -> None:
        raw_emotion = (emotion or "default").strip().lower()
        normalize = {
            "neutral": "default",
            "none": "default",
            "surprise": "surprised",
            "surprised": "surprised",
            "fear": "scared",
            "fearful": "scared",
            "disgust": "disgusted",
            "disgusted": "disgusted",
            "angry": "angry",
            "anger": "angry",
            "happy": "happy",
            "happiness": "happy",
            "sadness": "sad",
            "sad": "sad",
        }
        emotion = normalize.get(raw_emotion, raw_emotion)
        self.get_logger().info(f"Emotion event: raw={raw_emotion} normalized={emotion} mode={self._mode}")
        attention = _to_bool(context.get("attention"))
        eye_contact = _to_bool(context.get("eye_contact"))
        gesture = str(context.get("gesture") or "unknown").lower()
        aggr_raw = str(context.get("aggression") or context.get("aggression_signals") or "").strip().lower()
        aggression = aggr_raw not in {"", "none", "no", "false"}
        head_pose = _parse_head_pose(context.get("head_pose"))
        pitch = float(head_pose.get("pitch", 0.0))
        yaw = float(head_pose.get("yaw", 0.0))
        cat_mode = self._mode == "cat"

        if aggression:
            self._send("audio_emotion", "scared", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("scared")
            self._send("movement", "enrere", self._pub_move)
            return

        if emotion == "angry":
            self._send("audio_emotion", "angry", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("angry")
            self._send("movement", "enrere", self._pub_move)
            return

        if emotion == "disgusted":
            self._send("audio_emotion", "disgusted", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("disgusted")
            return

        if emotion == "scared":
            self._send("audio_emotion", "scared", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("scared")
            self._send("movement", "enrere", self._pub_move)
            return

        if emotion == "surprised":
            self._send("audio_emotion", "surprised", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("surprised")
            if attention and eye_contact and (abs(pitch) > 20 or abs(yaw) > 25):
                self._send("movement", "strech", self._pub_move)
            return

        friendly = {"wave", "thumbs_up", "ok", "open_hand", "peace"}
        if gesture in friendly or emotion == "happy":
            self._send("audio_emotion", "happy", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("happy")
            self._send("movement", "maneta", self._pub_move)
            return

        if emotion == "sad":
            self._send("audio_emotion", "sad", self._pub_audio_emotion)
            if cat_mode:
                self._set_event_anim("sad")
            self._send("movement", "normal", self._pub_move)
            return

        # Neutral/default should keep idle behavior without forcing reactions.
        if emotion == "default":
            return

    def _on_event(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        try:
            event = json.loads(raw)
        except json.JSONDecodeError:
            return
        if not isinstance(event, dict):
            return
        event_type = str(event.get("type") or "").strip().lower()
        data = event.get("data") or {}
        if not event_type:
            return

        if self._mode == "police":
            self._handle_police(event_type, data)
            return
        if self._mode == "city":
            self._handle_city(event_type)
            return

        # Cat mode uses human_emotion events from vision.
        if event_type == "human_emotion":
            emotion = str(data.get("emotion") or "default").strip().lower()
            self._handle_emotion(emotion, data)
            return

    def _tick(self) -> None:
        self._send_idle_anim()


def main() -> None:
    rclpy.init()
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
