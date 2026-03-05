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

        # 1. The YOLO Object Classifier (The "Eyes")
        # Subscribes /camera/color/image_raw, publishes /objbbox
        Node(
            package='object_classification',
            executable='classifier',
            name='classifier',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

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

        # 3. Voice Command Node (The "Ears")
        # Only launched when skip_voice is false.
        # Publishes /target_object and /user_command from mic input.
        Node(
            package='tidybot_bringup',
            executable='voice_command.py',
            name='voice_command',
            output='screen',
            condition=UnlessCondition(skip_voice),
        ),
    ])
