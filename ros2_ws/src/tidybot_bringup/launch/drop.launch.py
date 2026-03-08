"""
Launch file for the Find Bin & Drop pipeline.

Launches:
- AprilTag detector node (from tidybot_perception)
- Find bin and drop script

Usage:
    ros2 launch tidybot_bringup drop.launch.py
    ros2 launch tidybot_bringup drop.launch.py target_fruit:=apple
    ros2 launch tidybot_bringup drop.launch.py target_fruit:=orange arm_name:=left
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    target_fruit_arg = DeclareLaunchArgument(
        'target_fruit', default_value='banana',
        description='Which fruit bin to find (banana, apple, orange)')
    arm_name_arg = DeclareLaunchArgument(
        'arm_name', default_value='right',
        description='Which arm is holding the object (right or left)')

    target_fruit = LaunchConfiguration('target_fruit')
    arm_name = LaunchConfiguration('arm_name')

    return LaunchDescription([
        target_fruit_arg,
        arm_name_arg,

        # 1. AprilTag Detector — detects tag36h11 tags in camera frames
        Node(
            package='tidybot_perception',
            executable='apriltag_detector_node',
            name='apriltag_detector',
            output='screen',
        ),

        # 2. Find Bin & Drop — state machine that scans, approaches, and drops
        Node(
            package='tidybot_bringup',
            executable='find_bin_and_drop.py',
            name='find_bin_and_drop',
            output='screen',
            parameters=[
                {'target_fruit': target_fruit},
                {'arm_name': arm_name},
                {'use_sim_time': False},
            ]
        ),
    ])
