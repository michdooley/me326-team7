"""
Perception Launch File for TidyBot2.

Launches all perception nodes in one terminal: YOLO classifier,
face recognition, AprilTag detector, and a unified image overlay.

Usage:
    # All perception nodes:
    ros2 launch tidybot_bringup perception.launch.py

    # Task 1 (YOLO only):
    ros2 launch tidybot_bringup perception.launch.py use_face_recognition:=false use_apriltag:=false

    # Task 2 (YOLO + AprilTag):
    ros2 launch tidybot_bringup perception.launch.py use_face_recognition:=false

    # Task 3 (all):
    ros2 launch tidybot_bringup perception.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_use_classifier = DeclareLaunchArgument(
        'use_classifier', default_value='true',
        description='Launch YOLO object classifier')
    declare_use_face = DeclareLaunchArgument(
        'use_face_recognition', default_value='true',
        description='Launch face recognition node')
    declare_use_apriltag = DeclareLaunchArgument(
        'use_apriltag', default_value='true',
        description='Launch AprilTag detector node')
    declare_use_overlay = DeclareLaunchArgument(
        'use_overlay', default_value='true',
        description='Launch unified perception overlay node')

    classifier_node = Node(
        package='object_classification',
        executable='classifier',
        name='object_classifier',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_classifier')),
    )

    face_recognition_node = Node(
        package='tidybot_perception',
        executable='face_recognition_node',
        name='face_recognition_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_face_recognition')),
    )

    apriltag_detector_node = Node(
        package='tidybot_perception',
        executable='apriltag_detector_node',
        name='apriltag_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_apriltag')),
    )

    overlay_node = Node(
        package='tidybot_perception',
        executable='perception_overlay_node',
        name='perception_overlay_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_overlay')),
    )

    return LaunchDescription([
        declare_use_classifier,
        declare_use_face,
        declare_use_apriltag,
        declare_use_overlay,
        classifier_node,
        face_recognition_node,
        apriltag_detector_node,
        overlay_node,
    ])
