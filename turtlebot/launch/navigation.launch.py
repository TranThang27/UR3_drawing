"""Localize on a saved map and run the official TurtleBot 4 Nav2 configuration."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4_navigation')) / 'launch'
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to saved occupancy map YAML'),
        DeclareLaunchArgument('navigation', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'localization.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'nav2.launch.py')),
            condition=IfCondition(LaunchConfiguration('navigation')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': str(share.parent / 'config/nav2.yaml'),
            }.items()),
    ])
