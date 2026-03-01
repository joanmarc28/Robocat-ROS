import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robocat_control.movement.motors import EstructuraPotes
from robocat_control.movement.simulation_data import position_states


class DevLegNode(Node):
    def __init__(self) -> None:
        super().__init__("robocat_dev_leg")
        self.declare_parameter("command_topic", "/robocat/dev_leg_cmd")
        self.declare_parameter("status_topic", "/robocat/dev_leg_status")
        self.declare_parameter("default_move_sec", 0.35)

        self._lock = threading.Lock()
        self._estructura: Optional[EstructuraPotes] = None
        try:
            self._estructura = EstructuraPotes()
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize motors: {exc}")

        command_topic = str(self.get_parameter("command_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, command_topic, self._on_cmd, 10)

        self._publish_status(
            "ready | commands: help, states, where, all <state> [t], leg <0-3> <state> [t], calibrar [t]"
        )

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._pub.publish(msg)
        self.get_logger().info(text)

    def _parse_time(self, raw: str) -> Optional[float]:
        try:
            t = float(raw)
        except Exception:
            return None
        if t <= 0:
            return None
        return t

    def _help(self) -> str:
        return (
            "help | states | where | "
            "all <state> [t] | leg <0-3> <state> [t] | calibrar [t]"
        )

    def _on_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if not raw:
            return
        tokens = raw.split()
        if not tokens:
            return

        with self._lock:
            if self._estructura is None:
                self._publish_status("error: motors not initialized")
                return

            if tokens[0] == "help":
                self._publish_status(self._help())
                return

            if tokens[0] == "states":
                self._publish_status("states: " + ",".join(sorted(position_states.keys())))
                return

            if tokens[0] == "where":
                self._publish_status(f"where: {self._estructura.get_states()}")
                return

            if tokens[0] == "calibrar":
                t = float(self.get_parameter("default_move_sec").value)
                if len(tokens) > 1:
                    t_parsed = self._parse_time(tokens[1])
                    if t_parsed is None:
                        self._publish_status("error: invalid t")
                        return
                    t = t_parsed
                self._estructura.init_bot(t=t)
                self._publish_status(f"ok: calibrar t={t}")
                return

            if tokens[0] == "all":
                if len(tokens) < 2:
                    self._publish_status("error: use all <state> [t]")
                    return
                state = tokens[1]
                if state not in position_states:
                    self._publish_status(f"error: invalid state '{state}'")
                    return
                t = float(self.get_parameter("default_move_sec").value)
                if len(tokens) >= 3:
                    t_parsed = self._parse_time(tokens[2])
                    if t_parsed is None:
                        self._publish_status("error: invalid t")
                        return
                    t = t_parsed
                for leg in self._estructura.legs:
                    leg.set_state(state)
                self._estructura._move_legs(self._estructura.legs, t=t, inter_method="linear")
                self._publish_status(f"ok: all -> {state} t={t}")
                return

            if tokens[0] == "leg":
                if len(tokens) < 3:
                    self._publish_status("error: use leg <0-3> <state> [t]")
                    return
                try:
                    idx = int(tokens[1])
                except Exception:
                    self._publish_status("error: invalid leg index")
                    return
                if idx < 0 or idx >= len(self._estructura.legs):
                    self._publish_status("error: leg index out of range (0..3)")
                    return
                state = tokens[2]
                if state not in position_states:
                    self._publish_status(f"error: invalid state '{state}'")
                    return
                t = float(self.get_parameter("default_move_sec").value)
                if len(tokens) >= 4:
                    t_parsed = self._parse_time(tokens[3])
                    if t_parsed is None:
                        self._publish_status("error: invalid t")
                        return
                    t = t_parsed
                leg = self._estructura.legs[idx]
                leg.set_state(state)
                self._estructura._move_legs([leg], t=t, inter_method="linear")
                self._publish_status(f"ok: leg {idx} -> {state} t={t}")
                return

            self._publish_status("error: unknown command | " + self._help())


def main() -> None:
    rclpy.init()
    node = DevLegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
