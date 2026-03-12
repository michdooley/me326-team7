import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
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

    skip_voice = LaunchConfiguration('skip_voice')
    target_object = LaunchConfiguration('target_object')
    user_command = LaunchConfiguration('user_command')

    return LaunchDescription([
        skip_voice_arg,
        target_object_arg,
        user_command_arg,

        # 1. The YOLO Object Classifier — run separately:
        #   ros2 run object_classification classifier

        # 2. The Explore and Find Node (The "Brain")
        # Subscribes /objbbox + depth, publishes /cmd_vel
        Node(
            package='tidybot_bringup',
            executable='explore_and_find_object.py',
            name='explorer',
            output='screen',
            parameters=[
                {'skip_voice': skip_voice},
                {'target_object': target_object},
                {'user_command': user_command},
                {'use_sim_time': False}
            ]
        ),

        # 3. Voice Command Node — run in a separate interactive terminal:
        #   ros2 run tidybot_bringup voice_command.py
        # Or manually publish:
        #   ros2 topic pub --once /user_command std_msgs/msg/String "data: 'get'"
        #   ros2 topic pub --once /target_object std_msgs/msg/String "data: 'banana'"
    ])
