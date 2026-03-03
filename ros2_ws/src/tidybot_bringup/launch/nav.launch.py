import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The YOLO Object Classifier (The "Eyes")
        # This node takes /camera/color/image_raw and outputs /objbbox
        Node(
            package='object_classification', # Your package name
            executable='object_classifier',   # Your python script/entry point
            name='classifier',
            output='screen',
            parameters=[{'use_sim_time': False}] # Set to True if in MuJoCo
        ),

        # 2. The Explore and Find Node (The "Brain")
        # This node takes /objbbox and /camera/depth/image_raw to move the robot
        Node(
            package='tidybot_bringup',
            executable='explore_and_find_object.py',
            name='explorer',
            output='screen',
            parameters=[
                {'target_class_id': 46}, # 46 is 'banana' in YOLOv8
                {'use_sim_time': False}
            ]
        )
    ])