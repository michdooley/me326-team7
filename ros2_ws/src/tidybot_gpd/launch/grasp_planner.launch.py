import os
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch the grasp planner node using GPD.
    
    Usage:
        ros2 launch tidybot_gpd grasp_planner.launch.py
        
    Optional arguments:
        config_file: path to GPD config file (default: share/tidybot_gpd/config/gpd_params.cfg)
        approach_offset: pre-grasp offset in meters (default: 0.10)
        default_arm: arm to use when "auto" is requested (default: "right")
    """
    
    # Get the package path
    package_path = Path(__file__).parent.parent
    config_dir = os.path.join(package_path, 'config')
    default_config = os.path.join(config_dir, 'gpd_params.cfg')
    
    # Declare arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to GPD configuration file'
    )
    
    declare_approach_offset = DeclareLaunchArgument(
        'approach_offset',
        default_value='0.10',
        description='Pre-grasp offset above grasp pose in meters'
    )
    
    declare_default_arm = DeclareLaunchArgument(
        'default_arm',
        default_value='right',
        description='Default arm to use when "auto" is requested'
    )
    
    # Grasp planner node
    grasp_planner_node = Node(
        package='tidybot_gpd',
        executable='grasp_planner_node',
        name='grasp_planner_node',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file'),
                'approach_offset': LaunchConfiguration('approach_offset'),
                'default_arm': LaunchConfiguration('default_arm'),
                'point_cloud_timeout': 2.0,
            }
        ],
        remappings=[
            ('/camera/depth/cloud', '/camera/depth/cloud'),
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_approach_offset,
        declare_default_arm,
        grasp_planner_node,
    ])
