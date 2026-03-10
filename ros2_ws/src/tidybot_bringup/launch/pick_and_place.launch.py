"""
Launch file for the Pick-and-Place pipeline.

Launches:
- AprilTag detector node (from tidybot_perception) for bin finding
- pick_and_place.py — integrated find/grasp/bin/drop state machine

External nodes (run separately):
- ros2 run object_classification classifier   (YOLO detection)
- ros2 run tidybot_bringup voice_command.py   (voice commands, optional)

Usage:
    ros2 launch tidybot_bringup pick_and_place.launch.py
    ros2 launch tidybot_bringup pick_and_place.launch.py target_object:=apple
    ros2 launch tidybot_bringup pick_and_place.launch.py skip_voice:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    skip_voice_arg = DeclareLaunchArgument(
        'skip_voice', default_value='true',
        description='Skip voice commands and use hardcoded target')
    target_object_arg = DeclareLaunchArgument(
        'target_object', default_value='banana',
        description='Object name when skip_voice is true')
    user_command_arg = DeclareLaunchArgument(
        'user_command', default_value='get',
        description='Action verb when skip_voice is true')
    arm_name_arg = DeclareLaunchArgument(
        'arm_name', default_value='right',
        description='Which arm to use (right or left)')

    skip_voice = LaunchConfiguration('skip_voice')
    target_object = LaunchConfiguration('target_object')
    user_command = LaunchConfiguration('user_command')
    arm_name = LaunchConfiguration('arm_name')

    return LaunchDescription([
        skip_voice_arg,
        target_object_arg,
        user_command_arg,
        arm_name_arg,

        # 1. AprilTag Detector — detects tag36h11 tags for bin finding
        Node(
            package='tidybot_perception',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen',
        ),

        # 2. Pick-and-Place Node — integrated state machine
        # Subscribes to /objbbox (YOLO) + /apriltag_detections + depth
        # Publishes /cmd_vel, arm/gripper commands
        Node(
            package='tidybot_bringup',
            executable='pick_and_place.py',
            name='pick_and_place',
            output='screen',
            parameters=[
                {'skip_voice': skip_voice},
                {'target_object': target_object},
                {'user_command': user_command},
                {'arm_name': arm_name},
                {'use_sim_time': False},
            ]
        ),

        # External nodes (run in separate terminals):
        #   ros2 run object_classification classifier
        #   ros2 run tidybot_bringup voice_command.py
    ])
