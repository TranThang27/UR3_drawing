"""Launch a UR3/UR3e in the simple tabletop pick-and-place world."""

from launch import LaunchDescription
from pathlib import Path
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": launch_rviz,
            "description_package": "ur_simulation_gz",
            "description_file": "ur3e_gripper.urdf.xacro",
            "world_file": LaunchConfiguration("world_file"),
            # The table surface is at z=0.75 m.
            "robot_x": "-0.27",
            "robot_y": "0.0",
            "robot_z": "0.75",
            "robot_yaw": "0.0",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(
                Path(__file__).resolve().parents[1] / "worlds/camera_table.sdf")),
            # Gazebo converts package:// mesh URIs to model:// URIs. Add the
            # parent share directory so the official Robotiq meshes resolve.
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH",
                PathJoinSubstitution(
                    [FindPackagePrefix("robotiq_description"), "share"]
                ),
            ),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="Robot model to spawn.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Start the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "launch_rviz", default_value="false", description="Start RViz."
            ),
            simulation,
        ]
    )
