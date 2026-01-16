from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="robocat_hw", executable="telemetry_node", output="screen"),
        Node(package="robocat_control", executable="cmd_node", output="screen"),
    ])
