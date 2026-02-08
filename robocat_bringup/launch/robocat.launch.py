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
    webrtc_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "webrtc.yaml"]
    )
    audio_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "audio.yaml"]
    )
    command_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "commands.yaml"]
    )
    oled_assets_path = PathJoinSubstitution(
        [FindPackageShare("robocat_hw"), "assets", "eyes_img"]
    )
    behavior_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "behavior.yaml"]
    )
    vision_params = PathJoinSubstitution(
        [FindPackageShare("robocat_bringup"), "config", "vision.yaml"]
    )
    return LaunchDescription([
        Node(
            package="robocat_pairing",
            executable="pairing_node",
            output="screen",
            parameters=[pairing_params],
        ),
        Node(package="robocat_hw", executable="pi_status_node", output="screen"),
        Node(package="robocat_hw", executable="sensors_node", output="screen"),
        Node(
            package="robocat_hw",
            executable="oled_message_node",
            output="screen",
            parameters=[{
                "assets_path": oled_assets_path,
                "anim_delay": 0.06,
                "anim_loop": True,
            }],
        ),
        Node(
            package="robocat_hw",
            executable="web_telemetry_node",
            output="screen",
            parameters=[telemetry_params],
        ),
        Node(
            package="robocat_hw",
            executable="speaker_node",
            output="screen",
            parameters=[audio_params],
        ),
        Node(
            package="robocat_hw",
            executable="mic_node",
            output="screen",
            parameters=[audio_params],
        ),
        Node(
            package="robocat_hw",
            executable="ws_command_node",
            output="screen",
            parameters=[command_params],
        ),
        Node(
            package="robocat_video",
            executable="webrtc_streamer_node",
            output="screen",
            parameters=[webrtc_params],
        ),
        Node(
            package="robocat_video",
            executable="webrtc_signaling_node",
            output="screen",
            parameters=[webrtc_params],
        ),
        Node(
            package="robocat_vision",
            executable="vision_event_node",
            output="screen",
            parameters=[vision_params],
        ),
        Node(
            package="robocat_vision",
            executable="vision_node",
            output="screen",
            parameters=[vision_params],
        ),
        Node(
            package="robocat_behavior",
            executable="mode_manager_node",
            output="screen",
            parameters=[behavior_params],
        ),
        Node(
            package="robocat_behavior",
            executable="behavior_node",
            output="screen",
            parameters=[behavior_params],
        ),
        Node(package="robocat_control", executable="cmd_node", output="screen"),
    ])
