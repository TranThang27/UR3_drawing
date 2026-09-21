from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    defaults = {
        "ur_type": "ur3",
        "gazebo_gui": "true",
        "x": "0.30",
        "y": "-0.08",
        "z": "0.40",
        "radius": "0.08",
        "step": "0.004",
        "speed_scale": "0.25",
        "execute": "true",
        "launch_rviz": "true",
    }
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    parameters = {
        k: ParameterValue(LaunchConfiguration(k), value_type=float)
        for k in ("x", "y", "z", "radius", "step", "speed_scale")
    }
    parameters.update({
        "use_sim_time": True,
        "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
    })
    return LaunchDescription(arguments + [
        SetEnvironmentVariable("LC_ALL", "C.UTF-8"),
        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("ur_simulation_gz"),
                        "launch",
                        "ur_sim_control.launch.py",
                    ])),
                    launch_arguments={
                        "ur_type": LaunchConfiguration("ur_type"),
                        "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                        "launch_rviz": "false",
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare("ur_moveit_config"),
                        "launch",
                        "ur_moveit.launch.py",
                    ])),
                    launch_arguments={
                        "ur_type": LaunchConfiguration("ur_type"),
                        "launch_rviz": "false",
                        "use_sim_time": "true",
                    }.items(),
                ),
            ],
        ),
        Node(
            package="ur_drawing",
            executable="draw_p.py",
            output="screen",
            parameters=[parameters],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz_draw_p",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("ur_drawing"),
                "config",
                "draw_p.rviz",
            ])],
            parameters=[{"use_sim_time": True}],
            additional_env={"LC_ALL": "C.UTF-8"},
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ])
