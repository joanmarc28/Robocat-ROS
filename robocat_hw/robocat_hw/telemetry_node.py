import rclpy
from rclpy.node import Node
from robocat_msgs.msg import Telemetry

class TelemetryNode(Node):
    def __init__(self):
        super().__init__("robocat_telemetry")
        self.pub = self.create_publisher(Telemetry, "/robocat/telemetry", 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.mode = "boot"
        self.batt = 12.4

    def tick(self):
        msg = Telemetry()
        msg.stamp = self.get_clock().now().to_msg()
        self.batt = max(10.5, self.batt - 0.01)
        msg.battery_v = float(self.batt)
        msg.current_a = 1.2
        msg.temperature_c = 42.0
        msg.mode = self.mode
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TelemetryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
