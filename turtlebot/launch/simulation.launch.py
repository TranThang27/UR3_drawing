"""Reuse TurtleBot 4 Humble bringup; visualize simulation in the odom frame."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    config = LaunchConfiguration('rviz_config').perform(context)
    # Capture before the upstream include sets its own rviz=false. Keep that
    # include unscoped: Create 3's deferred controller spawners use its context.
    show_rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    world = LaunchConfiguration('world').perform(context)
    if world == 'simple_square':
        # Gazebo needs a file path; sensor bridges need the SDF world name.
        # Keep official spawning/controllers and use a camera above the room.
        root = Path(__file__).resolve().parents[1]
        package = lambda name: Path(get_package_share_directory(name))
        actions = [
            SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
                str(root / 'worlds'), str(share / 'worlds'),
                str(package('irobot_create_ignition_bringup') / 'worlds'),
                str(package('turtlebot4_description').parent),
                str(package('irobot_create_description').parent)])),
            SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join([
                str(package('turtlebot4_ignition_gui_plugins') / 'lib'),
                str(package('irobot_create_ignition_plugins') / 'lib')])),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(
                    package('ros_ign_gazebo') / 'launch/ign_gazebo.launch.py')),
                launch_arguments={'ign_args':
                    f'{root / "worlds/simple_square.sdf"} -r -v 4 '
                    f'--gui-config {root / "config/simple_square_gui.config"}'}.items()),
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                 output='screen'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / 'launch/turtlebot4_spawn.launch.py')),
                launch_arguments={
                    'world': 'simple_square',
                    'model': LaunchConfiguration('model'), 'namespace': '',
                    'rviz': 'false', 'slam': 'false', 'nav2': 'false',
                    'localization': 'false', 'use_sim_time': 'true',
                }.items())]
    else:
        actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'turtlebot4_ignition.launch.py')),
            launch_arguments={
                'world': world,
                'model': LaunchConfiguration('model'),
                'rviz': 'false', 'slam': 'false', 'nav2': 'false',
                'localization': 'false', 'use_sim_time': 'true', 'namespace': '',
            }.items(),
        )]
    if show_rviz:
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', str(config)],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='simple_square'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz_config', default_value=str(
            Path(__file__).resolve().parents[1] / 'config/simulation.rviz')),
        OpaqueFunction(function=launch_setup),
    ])
