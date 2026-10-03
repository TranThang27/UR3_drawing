from __future__ import annotations

import atexit
import json
import math
import os
import time
from typing import Dict, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
    PlanningSceneWorld,
    RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, String
from vision import require_fresh, OBJECTS, ZONES

CUBE_COLORS = {
    "red_cube": (1.0, 0.03, 0.03, 1.0),
    "yellow_cube": (1.0, 0.85, 0.02, 1.0),
    "blue_cube": (0.02, 0.15, 1.0, 1.0),
    "green_cube": (0.02, 1.0, 0.03, 1.0),
    "purple_cube": (0.6, 0.02, 1.0, 1.0),
}


def _motion_speed_scale() -> float:
    raw_value = os.environ.get("HRI_SPEED_SCALE", "2.0")
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise RuntimeError("HRI_SPEED_SCALE must be a number from 0.5 to 3.0") from exc
    if not 0.5 <= value <= 3.0:
        raise RuntimeError("HRI_SPEED_SCALE must be between 0.5 and 3.0")
    return value


SPEED_SCALE = _motion_speed_scale()
MOVE_GROUP_SCALE = min(0.2 * SPEED_SCALE, 0.6)
TRAJECTORY_TIME_SCALE = 5.0 / SPEED_SCALE


def _cube_color(name: str) -> ObjectColor:
    color = ObjectColor(id=name)
    color.color.r, color.color.g, color.color.b, color.color.a = CUBE_COLORS[name]
    return color


HOME_JOINTS = {
    "shoulder_pan_joint": -0.48707,
    "shoulder_lift_joint": -1.77750,
    "elbow_joint": 1.67467,
    "wrist_1_joint": -1.46797,
    "wrist_2_joint": -1.57080,
    "wrist_3_joint": 1.08373,
}

OPEN_POSITION = 0.0
CLOSED_POSITION = 0.7929
APPROACH_HEIGHT = 0.32
GRASP_HEIGHT = 0.195
CUBE_HALF_SIZE = 0.025
TOOL_TO_CUBE_CENTER = GRASP_HEIGHT - CUBE_HALF_SIZE
APPROACH_CLEARANCE = APPROACH_HEIGHT - GRASP_HEIGHT
GRIPPER_BASE_LINK = "robotiq_85_base_link"
GRIPPER_TOUCH_LINKS = [
    "robotiq_85_left_finger_link",
    "robotiq_85_right_finger_link",
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link",
]


def _normalise_object(name: str) -> str:
    value = str(name).strip().lower().replace(" ", "_")
    aliases = {name.removesuffix("_cube"): name for name in OBJECTS}
    value = aliases.get(value, value)
    if value not in OBJECTS:
        raise ValueError(f"Unknown object {name!r}; expected one of {sorted(OBJECTS)}")
    return value


def _normalise_zone(name: str) -> str:
    value = str(name).strip().lower().replace(" ", "_")
    aliases = {"a": "zone_a", "b": "zone_b", "c": "zone_c"}
    value = aliases.get(value, value)
    if value not in ZONES:
        raise ValueError(f"Unknown zone {name!r}; expected Zone A, Zone B or Zone C")
    return value


class RobotActions(Node):
    def __init__(self) -> None:
        super().__init__("hri_robot_actions")
        self._move_group = ActionClient(self, MoveGroup, "/move_action")
        self._scene = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._get_scene = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self._cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self._fk = self.create_client(GetPositionFK, "/compute_fk")
        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        self._execute = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self._gripper = self.create_publisher(
            Float64, "/gripper/target", 10
        )
        self._joint_positions: Dict[str, float] = {}
        self._joint_state_subscription = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        self.held_object: Optional[str] = None
        self._camera_state = None
        self._temporary = None
        self.create_subscription(String, "/hri2/world_state", self._on_camera_state, 10)
        self.get_logger().info(f"Motion speed: {SPEED_SCALE:.1f}x")

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_positions.update(zip(message.name, message.position))

    def _on_camera_state(self, message):
        self._camera_state = json.loads(message.data)

    def observe(self, complete=True, timeout=10.0):
        deadline = time.monotonic() + timeout
        started = time.monotonic()
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            state = self._camera_state
            if state and state["observed_at"] > started:
                require_fresh(state)
                if not complete or state["complete"]:
                    return state
        missing = self._camera_state.get("missing") if self._camera_state else "camera offline"
        raise RuntimeError(f"PERCEPTION_INCOMPLETE: {missing}")

    def sync_environment(self):
        state = self.observe()
        for name, item in state["objects"].items():
            if name != self.held_object:
                self._add_world_cube(name, item["position"])
        return state

    def reachable_temporary(self, position):
        if not self._ik.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MoveIt IK service is unavailable")
        state = self._planning_scene(PlanningSceneComponents.ROBOT_STATE).robot_state
        for height in (APPROACH_HEIGHT, GRASP_HEIGHT):
            request = GetPositionIK.Request()
            request.ik_request.group_name = "ur_manipulator"
            request.ik_request.ik_link_name = "tool0"
            request.ik_request.robot_state = state
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout.sec = 1
            request.ik_request.pose_stamped.header.frame_id = "world"
            pose = request.ik_request.pose_stamped.pose
            pose.position.x, pose.position.y, pose.position.z = position[0], position[1], height
            pose.orientation.y = 1.0
            response = self._wait_future(self._ik.call_async(request), 3.0, "checking temporary IK")
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                return False
            state = response.solution
        return True

    @staticmethod
    def _rotate_vector(q, x: float, y: float, z: float):
        tx = 2.0 * (q.y * z - q.z * y)
        ty = 2.0 * (q.z * x - q.x * z)
        tz = 2.0 * (q.x * y - q.y * x)
        return (
            x + q.w * tx + q.y * tz - q.z * ty,
            y + q.w * ty + q.z * tx - q.x * tz,
            z + q.w * tz + q.x * ty - q.y * tx,
        )

    def _wait_future(self, future, timeout: float, description: str):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"Timed out while {description}")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"Failed while {description}: {exception}")
        return future.result()

    def _sleep(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.05, deadline - time.monotonic()))

    def _assert_single_move_action_server(self) -> None:
        status_topic = "/move_action/_action/status"
        servers = self.get_publishers_info_by_topic(status_topic)
        if len(servers) <= 1:
            return

        names = ", ".join(
            f"{item.node_namespace.rstrip('/')}/{item.node_name}"
            for item in servers
        )
        raise RuntimeError(
            f"Detected {len(servers)} servers for /move_action ({names}). "
            "The Gazebo/MoveIt launch file is running more than once. Stop "
            "the duplicate launch and keep exactly one move_group process."
        )

    def _execute_constraints(self, constraints: Constraints) -> bool:
        if not self._move_group.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("MoveIt action /move_action is not available")
        self._assert_single_move_action_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = MOVE_GROUP_SCALE
        goal.request.max_acceleration_scaling_factor = MOVE_GROUP_SCALE
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        handle = self._wait_future(
            self._move_group.send_goal_async(goal), 20.0, "sending a MoveIt goal"
        )
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected the motion goal")
        wrapped_result = self._wait_future(
            handle.get_result_async(), 120.0, "planning/executing the motion"
        )
        code = wrapped_result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"MoveIt failed with error code {code}")
        return True

    def _move_joints(self, values: Dict[str, float]) -> bool:
        constraints = Constraints(name="joint_goal")
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, value in values.items()
        ]
        return self._execute_constraints(constraints)

    def _planning_scene(self, components: int) -> PlanningScene:
        if not self._get_scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                "Cannot connect to MoveIt /get_planning_scene. Start "
                "'python3 run_demo.py' from LLM_HRI_2 "
                "in another terminal and wait for it to finish loading. "
                "If Gazebo/RViz is already open, restart that launch; an old "
                "process may still be running without ROS discovery. Both "
                "terminals must use the same ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY."
            )
        request = GetPlanningScene.Request()
        request.components.components = components
        response = self._wait_future(
            self._get_scene.call_async(request), 10.0, "reading the planning scene"
        )
        return response.scene

    def _move_tool(self, x: float, y: float, z: float) -> bool:
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation.y = 1.0
        target.orientation.w = 0.0

        if not self._cartesian.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt Cartesian service is unavailable")
        self._assert_single_move_action_server()
        state = self._planning_scene(
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        ).robot_state
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.start_state = state
        request.group_name = "ur_manipulator"
        request.link_name = "tool0"
        request.waypoints = [target]
        request.max_step = 0.003
        request.jump_threshold = 2.0
        request.revolute_jump_threshold = 0.15
        request.avoid_collisions = True
        response = self._wait_future(
            self._cartesian.call_async(request), 15.0, "planning Cartesian motion"
        )
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or response.fraction < 0.999
        ):
            raise RuntimeError(
                f"Cartesian path only {response.fraction:.1%} complete to "
                f"({x:.3f}, {y:.3f}, {z:.3f}); refusing partial path or IK branch flip"
            )
        trajectory = response.solution
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("MoveIt returned an empty Cartesian path")
        measured = dict(zip(state.joint_state.name, state.joint_state.position))
        for index, name in enumerate(trajectory.joint_trajectory.joint_names):
            offset = 2 * math.pi * round(
                (measured[name] - points[0].positions[index]) / (2 * math.pi)
            )
            for point in points:
                point.positions[index] += offset
                if abs(point.positions[index]) > 2 * math.pi:
                    raise RuntimeError(
                        f"Cartesian path exceeds joint range for {name}; "
                        "return home first"
                    )
        travel = [0.0] * len(points[0].positions)
        for before, after in zip(points, points[1:]):
            for index, (a, b) in enumerate(zip(before.positions, after.positions)):
                if abs(b - a) > 0.35:
                    raise RuntimeError("Cartesian path changes IK branch")
                travel[index] += abs(b - a)
        if max(travel) > 1.5:
            raise RuntimeError("Cartesian segment would rotate a joint more than 1.5 rad")
        for point in points:
            source_nanos = (
                point.time_from_start.sec * 10**9
                + point.time_from_start.nanosec
            )
            nanos = int(source_nanos * TRAJECTORY_TIME_SCALE)
            point.time_from_start = Duration(
                sec=nanos // 10**9, nanosec=nanos % 10**9
            )
            point.velocities = [
                value / TRAJECTORY_TIME_SCALE for value in point.velocities
            ]
            point.accelerations = [
                value / TRAJECTORY_TIME_SCALE**2
                for value in point.accelerations
            ]
        if not self._execute.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt trajectory execution server is unavailable")
        self.get_logger().info(
            f"Cartesian -> ({x:.3f}, {y:.3f}, {z:.3f}), "
            "full path, no joint jumps"
        )
        handle = self._wait_future(
            self._execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)),
            10.0, "sending Cartesian trajectory",
        )
        if not handle.accepted:
            raise RuntimeError("MoveIt rejected Cartesian trajectory")
        result = self._wait_future(
            handle.get_result_async(), 120.0, "executing Cartesian trajectory"
        )
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"Cartesian execution failed: {result.result.error_code.val}")
        self._sleep(0.3)
        actual = self._tool_pose()
        settle_deadline = time.monotonic() + 3.0
        while (
            math.dist(
                (actual.position.x, actual.position.y, actual.position.z),
                (x, y, z),
            )
            > 0.005
            and time.monotonic() < settle_deadline
        ):
            self._sleep(0.2)
            actual = self._tool_pose()
        if (
            math.dist(
                (actual.position.x, actual.position.y, actual.position.z),
                (x, y, z),
            )
            > 0.005
        ):
            raise RuntimeError(
                f"Actual tool ({actual.position.x:.4f}, {actual.position.y:.4f}, "
                f"{actual.position.z:.4f}) missed Cartesian goal by more than 5 mm"
            )
        if 2.0 * math.acos(min(1.0, abs(actual.orientation.y))) > 0.05:
            raise RuntimeError("Tool did not retain the requested downward orientation")
        return True

    def _tool_pose(self) -> Pose:
        if not self._fk.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("MoveIt FK service is unavailable")
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = ["tool0"]
        request.robot_state = self._planning_scene(PlanningSceneComponents.ROBOT_STATE).robot_state
        response = self._wait_future(self._fk.call_async(request), 5.0, "reading tool pose")
        if response.error_code.val != MoveItErrorCodes.SUCCESS or not response.pose_stamped:
            raise RuntimeError("Could not read tool pose")
        return response.pose_stamped[0].pose

    def _camera_pose(self, name: str) -> Pose:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            state = self.observe(complete=False)
            item = state["objects"].get(name) or state["zones"].get(name)
            if item is not None:
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = item["position"]
                pose.orientation.z = math.sin(item["yaw"] / 2)
                pose.orientation.w = math.cos(item["yaw"] / 2)
                return pose
        raise RuntimeError(f"PERCEPTION_OCCLUDED: camera cannot locate {name}")

    def _sync_cube(self, name: str) -> Tuple[float, float, float]:
        pose = self._camera_pose(name)
        xyz = (pose.position.x, pose.position.y, pose.position.z)
        self._add_world_cube(name, xyz, pose)
        return xyz

    def _command_gripper(self, position: float) -> bool:
        if not OPEN_POSITION <= position <= CLOSED_POSITION:
            raise ValueError(
                f"Robotiq joint position must be in [{OPEN_POSITION}, {CLOSED_POSITION}] rad"
            )
        deadline = time.monotonic() + 10.0
        while (
            self._gripper.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._gripper.get_subscription_count() == 0:
            raise RuntimeError(
                "Gripper servo /gripper/target is unavailable; "
                "start LLM_HRI_2/launch/camera_moveit.launch.py"
            )

        command = Float64(data=position)
        for _ in range(5):
            self._gripper.publish(command)
            self._sleep(0.1)
        self._sleep(0.8)
        return True

    def _apply_scene(self, scene: PlanningScene) -> None:
        if not self._scene.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("MoveIt service /apply_planning_scene is unavailable")
        response = self._wait_future(
            self._scene.call_async(ApplyPlanningScene.Request(scene=scene)),
            10.0,
            "updating the planning scene",
        )
        if not response.success:
            raise RuntimeError("MoveIt rejected the planning-scene update")

    def _remove_world_object(self, name: str) -> None:
        item = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        self._apply_scene(
            PlanningScene(
                is_diff=True,
                world=PlanningSceneWorld(collision_objects=[item]),
            )
        )

    def _allow_object_contact(self, name: str, allowed: bool) -> None:
        scene = self._planning_scene(PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        matrix = scene.allowed_collision_matrix
        contact_names = [*GRIPPER_TOUCH_LINKS, "work_table", *ZONES]

        for entry_name in [name, *contact_names]:
            if entry_name in matrix.entry_names:
                continue
            matrix.entry_names.append(entry_name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names))
            )

        cube_index = matrix.entry_names.index(name)
        for other_name in contact_names:
            other_index = matrix.entry_names.index(other_name)
            matrix.entry_values[cube_index].enabled[other_index] = allowed
            matrix.entry_values[other_index].enabled[cube_index] = allowed

        self._apply_scene(
            PlanningScene(is_diff=True, allowed_collision_matrix=matrix)
        )

    def _attach_cube(self, name: str) -> None:
        cube = CollisionObject(id=name, operation=CollisionObject.ADD)
        attached = AttachedCollisionObject(
            link_name=GRIPPER_BASE_LINK,
            object=cube,
            touch_links=GRIPPER_TOUCH_LINKS,
        )
        self._apply_scene(
            PlanningScene(
                is_diff=True,
                object_colors=[_cube_color(name)],
                robot_state=RobotState(
                    is_diff=True, attached_collision_objects=[attached]
                ),
            )
        )

    def _detach_cube(self, name: str) -> None:
        remove = AttachedCollisionObject(link_name=GRIPPER_BASE_LINK)
        remove.object.id = name
        remove.object.operation = CollisionObject.REMOVE
        self._apply_scene(
            PlanningScene(
                is_diff=True,
                robot_state=RobotState(
                    is_diff=True, attached_collision_objects=[remove]
                ),
            )
        )
        self._remove_world_object(name)

    def _add_world_cube(
        self, name: str, position_xyz: Tuple[float, float, float], measured_pose=None
    ) -> None:
        cube = CollisionObject()
        cube.header.frame_id = "world"
        cube.id = name
        cube.operation = CollisionObject.ADD
        cube.primitives = [
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05])
        ]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position_xyz
        pose.orientation.w = 1.0
        if measured_pose is not None:
            pose = measured_pose
        cube.primitive_poses = [pose]
        self._apply_scene(
            PlanningScene(
                is_diff=True,
                object_colors=[_cube_color(name)],
                world=PlanningSceneWorld(collision_objects=[cube]),
            )
        )

    def home(self) -> bool:
        self._recover_held_state()
        if self.held_object is None:
            self.open_gripper()
        self.get_logger().info("HOME: moving to initial joints")
        result = self._move_joints(HOME_JOINTS)
        if self.held_object is None:
            self.sync_environment()
        return result

    def _recover_held_state(self) -> None:
        scene = self._planning_scene(PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        held = [item.object.id for item in scene.robot_state.attached_collision_objects
                if item.object.id in OBJECTS]
        if len(held) > 1:
            raise RuntimeError("Planning scene contains multiple held cubes; reset the scene")
        self.held_object = held[0] if held else None

    def open_gripper(self) -> bool:
        return self._command_gripper(OPEN_POSITION)

    def close_gripper(self) -> bool:
        return self._command_gripper(CLOSED_POSITION)

    def _require_gripper_contact(self, name: str) -> float:
        deadline = time.monotonic() + 3.0
        joint_name = "robotiq_85_left_knuckle_joint"
        while joint_name not in self._joint_positions and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if joint_name not in self._joint_positions:
            raise RuntimeError("No gripper joint feedback; cannot confirm grasp contact")

        position = self._joint_positions[joint_name]
        self.get_logger().info(f"Measured gripper aperture joint: {position:.4f} rad")
        if not 0.15 < position < CLOSED_POSITION - 0.06:
            raise RuntimeError(
                "Grasp failed: aperture is inconsistent with a 50-mm cube"
            )
        cube = self._camera_pose(name)
        tool = self._tool_pose()
        expected = self._rotate_vector(tool.orientation, 0.0, 0.0, TOOL_TO_CUBE_CENTER)
        error = math.dist(
            (cube.position.x, cube.position.y, cube.position.z),
            (
                tool.position.x + expected[0],
                tool.position.y + expected[1],
                tool.position.z + expected[2],
            ),
        )
        tolerance = 0.025 if self.held_object == name else 0.012
        if error > tolerance:
            raise RuntimeError(
                f"Cube is {error * 1000:.1f} mm away from grasp centre; "
                "stopping motion"
            )
        return position

    def pick(self, object: str) -> bool:
        name = _normalise_object(object)
        self._recover_held_state()
        if self.held_object is not None:
            raise RuntimeError(f"The gripper is already holding {self.held_object}")
        object_xyz = self._sync_cube(name)
        x, y, cube_z = object_xyz
        grasp_z = cube_z + TOOL_TO_CUBE_CENTER
        approach_z = grasp_z + APPROACH_CLEARANCE
        self.open_gripper()
        self.get_logger().info(f"PICK {name}: approach above measured cube")
        self._move_tool(x, y, approach_z)
        self._allow_object_contact(name, True)
        attached = False
        try:
            self.get_logger().info("PICK: descend vertically, then close fingers")
            self._move_tool(x, y, grasp_z)
            self.close_gripper()
            self._require_gripper_contact(name)
            self._sync_cube(name)
            self._attach_cube(name)
            attached = True
            self.held_object = name
            self.get_logger().info("PICK: physical lift; no Gazebo fixed attachment")
            self._move_tool(x, y, approach_z)
            lifted = self._camera_pose(name)
            if lifted.position.z < cube_z + APPROACH_CLEARANCE - 0.015:
                raise RuntimeError("Physical grasp failed: cube did not rise with the fingers")
            self.get_logger().info(
                f"PICK verified: cube rose {lifted.position.z - cube_z:.3f} m"
            )
        except Exception:
            if attached:
                actual = self._camera_pose(name)
                if actual.position.z > cube_z + 0.03:
                    self.get_logger().error(
                        "Pick interrupted while cube is lifted; "
                        "keeping grip and collision body"
                    )
                    raise
                self._detach_cube(name)
                self.open_gripper()
                self.held_object = None
            self._sync_cube(name)
            self._allow_object_contact(name, False)
            raise
        return True

    def place(self, object: str, zone: str) -> bool:
        name = _normalise_object(object)
        zone_name = "temporary_position" if zone == "temporary_position" else _normalise_zone(zone)
        self._recover_held_state()
        if self.held_object != name:
            raise RuntimeError(
                f"Cannot place {name}: gripper currently holds {self.held_object or 'nothing'}"
            )
        if zone_name == "temporary_position":
            if self._temporary is None:
                raise RuntimeError("No camera-validated temporary position reserved")
            zone_x, zone_y = self._temporary[:2]
        else:
            zone_pose = self._camera_pose(zone_name)
            zone_x, zone_y = zone_pose.position.x, zone_pose.position.y
        target_xyz = (zone_x, zone_y, CUBE_HALF_SIZE)
        grasp_z = target_xyz[2] + TOOL_TO_CUBE_CENTER
        approach_z = grasp_z + APPROACH_CLEARANCE
        self.get_logger().info(f"PLACE {name}: transfer above {zone_name}")
        self._require_gripper_contact(name)
        self._move_tool(zone_x, zone_y, approach_z)
        self._require_gripper_contact(name)
        tool = self._tool_pose()
        cube = self._camera_pose(name)
        place_x = zone_x + tool.position.x - cube.position.x
        place_y = zone_y + tool.position.y - cube.position.y
        grasp_z = CUBE_HALF_SIZE + tool.position.z - cube.position.z + 0.003
        self._move_tool(place_x, place_y, approach_z)
        self._allow_object_contact(name, True)
        self.get_logger().info("PLACE: descend vertically with cube collision geometry")
        self._move_tool(place_x, place_y, grasp_z)
        self.get_logger().info("PLACE: release cube, then retreat vertically")
        self.open_gripper()
        self._sleep(0.8)
        self._detach_cube(name)
        self.held_object = None
        self._sync_cube(name)
        self._move_tool(place_x, place_y, approach_z)
        actual = self._sync_cube(name)
        self._allow_object_contact(name, False)
        if math.dist(actual[:2], target_xyz[:2]) > 0.015 or abs(actual[2] - CUBE_HALF_SIZE) > 0.008:
            raise RuntimeError(f"Place missed zone centre: measured cube pose {actual}")
        self.get_logger().info(f"PLACE verified at {actual}")
        return True


_robot: Optional[RobotActions] = None
_owns_rclpy = False


def _get_robot() -> RobotActions:
    global _robot, _owns_rclpy
    if _robot is None:
        if not rclpy.ok():
            rclpy.init()
            _owns_rclpy = True
        _robot = RobotActions()
    return _robot


def shutdown() -> None:
    global _robot, _owns_rclpy
    if _robot is not None:
        _robot.destroy_node()
        _robot = None
    if _owns_rclpy and rclpy.ok():
        rclpy.shutdown()
    _owns_rclpy = False


def home() -> bool:
    return _get_robot().home()


def pick(object: str) -> bool:
    return _get_robot().pick(object)


def place(object: str, zone: str) -> bool:
    return _get_robot().place(object, zone)


def open_gripper() -> bool:
    return _get_robot().open_gripper()


def close_gripper() -> bool:
    return _get_robot().close_gripper()


atexit.register(shutdown)
