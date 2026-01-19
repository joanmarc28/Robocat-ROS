from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pairing_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "pairing.yaml"]
    )
    telemetry_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "telemetry.yaml"]
    )
    return LaunchDescription([
        Node(
            package="robocat_pairing",
            executable="pairing_node",
            output="screen",
            parameters=[pairing_params],
        ),
        Node(package="robocat_hw", executable="oled_message_node", output="screen"),
        Node(package="robocat_hw", executable="telemetry_node", output="screen"),
        Node(
            package="robocat_hw",
            executable="web_telemetry_node",
            output="screen",
            parameters=[telemetry_params],
        ),
        Node(package="robocat_control", executable="cmd_node", output="screen"),
    ])
