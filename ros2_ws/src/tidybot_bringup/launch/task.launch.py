"""
Task Launch File for TidyBot2.

Launches perception nodes on top of the base simulation or real hardware.
Designed to be launched AFTER sim.launch.py or real.launch.py.

Launches:
- Object detector (YOLO)
- Object localizer (depth → 3D)
- Grasp planner (GraspNet)

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Start perception stack
    ros2 launch tidybot_bringup task.launch.py

    # Terminal 3: Run a task
    ros2 run tidybot_bringup task1_retrieve.py

    # Or disable specific nodes:
    ros2 launch tidybot_bringup task.launch.py use_grasp_planner:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Declare arguments
    declare_use_detector = DeclareLaunchArgument(
        'use_detector', default_value='true',
        description='Launch YOLO object detection node'
    )
    declare_use_localizer = DeclareLaunchArgument(
        'use_localizer', default_value='true',
        description='Launch depth-to-3D object localization node'
    )
    declare_use_grasp_planner = DeclareLaunchArgument(
        'use_grasp_planner', default_value='true',
        description='Launch GraspNet grasp planning node'
    )
    declare_model_path = DeclareLaunchArgument(
        'detector_model_path', default_value='',
        description='Path to YOLO model weights'
    )
    declare_grasp_model_path = DeclareLaunchArgument(
        'grasp_model_path', default_value='',
        description='Path to GraspNet model weights'
    )

    # Perception nodes
    detector_node = Node(
        package='tidybot_perception',
        executable='detector_node',
        name='detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_detector')),
        parameters=[{
            'model_path': LaunchConfiguration('detector_model_path'),
            'confidence_threshold': 0.5,
            'publish_rate': 10.0,
            'device': 'cpu',
        }]
    )

    object_localizer_node = Node(
        package='tidybot_perception',
        executable='object_localizer_node',
        name='object_localizer_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_localizer')),
        parameters=[{
            'depth_scale': 0.001,
            'max_depth': 3.0,
            'target_frame': 'base_link',
        }]
    )

    grasp_planner_node = Node(
        package='tidybot_perception',
        executable='grasp_planner_node',
        name='grasp_planner_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_grasp_planner')),
        parameters=[{
            'model_path': LaunchConfiguration('grasp_model_path'),
            'approach_offset': 0.10,
            'default_arm': 'right',
        }]
    )

    return LaunchDescription([
        # Arguments
        declare_use_detector,
        declare_use_localizer,
        declare_use_grasp_planner,
        declare_model_path,
        declare_grasp_model_path,
        # Nodes
        detector_node,
        object_localizer_node,
        grasp_planner_node,
    ])
