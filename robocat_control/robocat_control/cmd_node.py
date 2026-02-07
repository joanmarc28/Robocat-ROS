import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from robocat_control.movement.motors import EstructuraPotes
from robocat_control.movement.simulation_data import (
    maneta_states,
    rot_states,
    walk_back_states,
    walk_states,
)

class CmdNode(Node):
    def __init__(self):
        super().__init__("robocat_cmd")
        self.sub = self.create_subscription(String, "/robocat/cmd", self.on_cmd, 10)
        self._queue: "queue.Queue[str]" = queue.Queue()
        self._last_enqueued_action: str | None = None
        self._last_enqueued_ts: float = 0.0
        self._last_motion_action: str | None = None
        self._last_motion_ts: float = 0.0
        self._motion_switch_cooldown_sec = 1.2
        self._estructura = None
        try:
            self._estructura = EstructuraPotes()
        except Exception as exc:
            self.get_logger().error(f"No s'ha pogut inicialitzar EstructuraPotes: {exc}")
        self._worker = threading.Thread(target=self._run_queue, daemon=True)
        self._worker.start()

    def on_cmd(self, msg: String):
        action = (msg.data or "").strip().lower()
        if not action:
            return
        now = time.time()
        if action == self._last_enqueued_action and (now - self._last_enqueued_ts) < 0.4:
            return
        self._last_enqueued_action = action
        self._last_enqueued_ts = now
        self._queue.put(action)

    def _run_queue(self) -> None:
        while rclpy.ok():
            action = self._queue.get()
            try:
                self._handle_action(action)
            except Exception as exc:
                self.get_logger().error(f"Error executant '{action}': {exc}")
            finally:
                self._queue.task_done()

    def _handle_action(self, action: str) -> None:
        if not self._estructura:
            self.get_logger().warning(f"Accio '{action}' ignorada: estructura no disponible.")
            return

        if action in {"endavant", "enrere"}:
            now = time.time()
            opposite = (
                (action == "endavant" and self._last_motion_action == "enrere")
                or (action == "enrere" and self._last_motion_action == "endavant")
            )
            if opposite and (now - self._last_motion_ts) < self._motion_switch_cooldown_sec:
                self.get_logger().warning(
                    f"Ignorant canvi rapid de moviment ({self._last_motion_action} -> {action})"
                )
                return
            self._last_motion_action = action
            self._last_motion_ts = now

        if action == "endavant":
            self._estructura.follow_sequance(walk_states, cycles=6, t=0.2)
            return
        if action == "enrere":
            self._estructura.follow_sequance(walk_back_states, cycles=6, t=0.8)
            return
        if action == "rotar":
            self._estructura.follow_sequance(rot_states, cycles=6, t=0.8)
            return
        if action == "maneta":
            self._estructura.follow_sequance(maneta_states, cycles=6, t=0.8)
            return
        if action == "ajupir":
            self._estructura.set_position("sit")
            return
        if action == "normal":
            self._estructura.set_position("normal")
            return
        if action == "hind_sit":
            self._estructura.sit_hind_legs()
            return
        if action == "recte":
            self._estructura.set_position("recte")
            return
        if action == "strech":
            self._estructura.strech()
            return
        if action == "up":
            self._estructura.set_position("up")
            return
        if action == "calibrar":
            self._estructura.init_bot()
            return
        if action == "demo":
            self._estructura.set_position("sit")
            time.sleep(5)
            self._estructura.set_position("normal")
            time.sleep(5)
            self._estructura.set_position("up")
            time.sleep(5)
            self._estructura.sit_hind_legs()
            time.sleep(5)
            self._estructura.strech()
            time.sleep(5)
            self._estructura.set_position("sit")
            time.sleep(5)
            self._estructura.set_position("normal")
            time.sleep(5)
            self._estructura.set_position("up")
            time.sleep(5)
            self._estructura.sit_hind_legs()
            time.sleep(5)
            self._estructura.strech()
            return

        self.get_logger().warning(f"Accio '{action}' no implementada en ROS.")

def main():
    rclpy.init()
    node = CmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
