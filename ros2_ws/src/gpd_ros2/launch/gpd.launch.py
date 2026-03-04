"""
Standalone launch file for GPD grasp detection server.

Usage:
    ros2 launch gpd_ros2 gpd.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default config path (source tree)
    default_config = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'eigen_params.cfg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='Path to GPD eigen_params.cfg'),
        DeclareLaunchArgument(
            'publish_rviz_markers', default_value='true',
            description='Publish RViz visualization markers'),

        Node(
            package='gpd_ros2',
            executable='gpd_detect_grasps_server',
            name='gpd_detect_grasps_server',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'frame_id': 'base_link',
                'publish_rviz_markers': LaunchConfiguration('publish_rviz_markers'),
            }],
        ),
    ])
