import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CmdNode(Node):
    def __init__(self):
        super().__init__("robocat_cmd")
        self.sub = self.create_subscription(String, "/robocat/cmd", self.on_cmd, 10)

    def on_cmd(self, msg: String):
        self.get_logger().info(f"CMD: {msg.data}")

def main():
    rclpy.init()
    node = CmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
