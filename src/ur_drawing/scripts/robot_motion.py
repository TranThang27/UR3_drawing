import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class RobotMotion:
    def __init__(self, node, config):
        self.node = node
        self.config = config
        self.cartesian = node.create_client(
            GetCartesianPath, "/compute_cartesian_path")
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, "/execute_trajectory")
        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        self.controller_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )

    def wait_result(self, future, timeout=60.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError(
                "MoveIt request timed out; check move_group and simulation")
        return future.result()

    def wait_for_cartesian_service(self):
        if not self.cartesian.wait_for_service(timeout_sec=30.0):
            raise RuntimeError(
                "/compute_cartesian_path unavailable; MoveIt did not start")

    def wait_for_execute_server(self):
        if not self.execute_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/execute_trajectory unavailable")

    def compute_cartesian(self, poses, header, label):
        request = GetCartesianPath.Request()
        request.header = header
        request.start_state.is_diff = True
        request.group_name = self.config["group_name"]
        request.link_name = self.config["ee_link"]
        request.waypoints = poses
        request.max_step = self.config["step"]
        request.jump_threshold = 5.0
        request.revolute_jump_threshold = 1.0
        request.avoid_collisions = True
        if hasattr(request, "max_velocity_scaling_factor"):
            request.max_velocity_scaling_factor = self.config["speed_scale"]
            request.max_acceleration_scaling_factor = self.config["speed_scale"]
        response = self.wait_result(self.cartesian.call_async(request))
        self.node.get_logger().info(f"{label}: {response.fraction:.1%}")
        if (response.error_code.val != MoveItErrorCodes.SUCCESS
                or response.fraction < 0.999):
            raise RuntimeError(
                f"{label} is incomplete; robot will not move. "
                "Adjust x/y/z/radius to keep the circle reachable and "
                "collision-free.")
        if not response.solution.joint_trajectory.points:
            raise RuntimeError(
                f"MoveIt returned an empty trajectory for {label}")
        return response.solution

    def move_to_start(self):
        if not self.controller_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(
                "joint_trajectory_controller action server is not ready; "
                "check the controller spawner")
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("/move_action unavailable; MoveIt did not start")
        self.node.get_logger().info("Controller action server is ready")
        time.sleep(1.0)

        target = Pose()
        target.position.x = self.config["x"]
        target.position.y = self.config["y"]
        target.position.z = self.config["z"]
        target.orientation.x = 1.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 0.0

        region = BoundingVolume()
        region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=[0.003],
        )]
        region.primitive_poses = [target]

        header = PoseStamped().header
        header.frame_id = self.config["frame_id"]
        constraints = Constraints()
        constraints.name = "circle_start"
        constraints.position_constraints = [PositionConstraint(
            header=header,
            link_name=self.config["ee_link"],
            constraint_region=region,
            weight=1.0,
        )]
        constraints.orientation_constraints = [OrientationConstraint(
            header=header,
            link_name=self.config["ee_link"],
            orientation=target.orientation,
            absolute_x_axis_tolerance=0.02,
            absolute_y_axis_tolerance=0.02,
            absolute_z_axis_tolerance=0.02,
            weight=1.0,
        )]

        goal = MoveGroup.Goal()
        goal.request.group_name = self.config["group_name"]
        goal.request.pipeline_id = "move_group"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        approach_scale = min(self.config["speed_scale"], 0.05)
        goal.request.max_velocity_scaling_factor = approach_scale
        goal.request.max_acceleration_scaling_factor = approach_scale
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3
        handle = self.wait_result(
            self.move_client.send_goal_async(goal), timeout=30.0)
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the approach goal")
        result = self.wait_result(handle.get_result_async(), timeout=90.0)
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            if result.result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
                raise RuntimeError(
                    "The approach was planned, but Gazebo could not track its "
                    "final goal; see the joint_trajectory_controller tolerance "
                    "error above")
            raise RuntimeError(
                f"MoveIt could not reach the circle start (code "
                f"{result.result.error_code.val}); adjust x/y/z")
        self.node.get_logger().info("Reached the circle start using OMPL")

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self.wait_result(self.execute_client.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected trajectory execution")
        result_future = handle.get_result_async()
        try:
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            timeout = max(
                60.0, duration.sec + duration.nanosec / 1e9 + 30.0)
            result = self.wait_result(result_future, timeout)
        except (RuntimeError, KeyboardInterrupt):
            self.wait_result(handle.cancel_goal_async(), timeout=5.0)
            raise
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"Execution failed: MoveIt code "
                f"{result.result.error_code.val}")
