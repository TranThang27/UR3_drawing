"""Launch the tabletop simulation together with MoveIt and its planning scene."""

import fcntl
import os
import tempfile
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.actions import RegisterEventHandler, EmitEvent
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_launch_lock = None


def require_single_launch(context):
    """Keep accidental second launches from publishing conflicting states."""
    global _launch_lock
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    path = os.path.join(tempfile.gettempdir(), f"ur3_pick_place_{os.getuid()}_{domain}.lock")
    lock = open(path, "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError(
            "pick_place_moveit.launch.py is already running. Use the existing "
            "Gazebo/RViz window, or stop its launch with Ctrl+C before restarting."
        )
    _launch_lock = lock
    return []


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    moveit_rviz = LaunchConfiguration("moveit_rviz")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(root / "launch/table.launch.py")
        ),
        launch_arguments={
            "ur_type": ur_type,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
            "world_file": LaunchConfiguration("world_file"),
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_servo": "false",
            "description_package": "ur_simulation_gz",
            "description_file": "ur3e_gripper.urdf.xacro",
            "launch_rviz": "false",
        }.items(),
    )

    # Keep a project-owned RViz configuration: show the measured robot by
    # default, without the orange planning-goal preview covering its pose.
    rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit",
        condition=IfCondition(moveit_rviz), output="log",
        arguments=["-d", str(root / "config/camera.rviz")],
        parameters=[
            {"use_sim_time": True,
             "robot_description": ParameterValue(Command([
                 FindExecutable(name="xacro"), " ", PathJoinSubstitution([
                     FindPackageShare("ur_simulation_gz"), "urdf", "ur3e_gripper.urdf.xacro",
                 ]), " name:=ur ur_type:=", ur_type,
             ]), value_type=str),
             "robot_description_semantic": ParameterValue(Command([
                 FindExecutable(name="xacro"), " ", PathJoinSubstitution([
                     FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro",
                 ]), " name:=ur",
             ]), value_type=str)},
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"]),
        ],
    )

    scene_loader = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(cmd=[sys.executable, str(root / "scene_loader.py")], output="screen")
        ],
    )

    return LaunchDescription(
        [
            OpaqueFunction(function=require_single_launch),
            DeclareLaunchArgument("world_file", default_value=str(root / "worlds/camera_table.sdf")),
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur3e",
                choices=["ur3", "ur3e"],
                description="Robot model to spawn and configure in MoveIt.",
            ),
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Start the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "moveit_rviz", default_value="true", description="Start MoveIt RViz."
            ),
            simulation,
            Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=[
                "/hri_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/hri_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/hri_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            ], output="log"),
            ExecuteProcess(cmd=[sys.executable, str(root / "perception_node.py")], output="screen"),
            moveit,
            RegisterEventHandler(OnProcessExit(
                target_action=rviz,
                on_exit=[EmitEvent(event=Shutdown(reason="RViz closed; stopping simulation"))],
            )),
            TimerAction(period=5.0, actions=[rviz]),
            Node(package="ur_simulation_gz", executable="gripper_servo.py",
                 output="screen", parameters=[{"use_sim_time": True}]),
            scene_loader,
        ]
    )
