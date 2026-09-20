import copy
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformException, TransformListener

from circle_geometry import circle_points
from robot_motion import RobotMotion


class DrawCircle(Node):
    def __init__(self):
        super().__init__("draw_circle")
        defaults = {
            "frame_id": "base_link",
            "ee_link": "tool0",
            "group_name": "ur_manipulator",
            "x": 0.30,
            "y": -0.08,
            "z": 0.18,
            "radius": 0.08,
            "step": 0.004,
            "speed_scale": 0.25,
            "execute": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config = {
            name: self.get_parameter(name).value for name in defaults
        }
        self.config["use_sim_time"] = self.get_parameter(
            "use_sim_time").value
        if not 0 < self.config["speed_scale"] <= 1:
            raise ValueError("speed_scale must be in (0, 1]")

        qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(
            Path, "/draw_p/target_path", qos)
        self.trace_pub = self.create_publisher(
            Path, "/draw_p/actual_path", qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.motion = RobotMotion(self, self.config)
        self.trace = Path()
        self.trace.header.frame_id = self.config["frame_id"]
        self.tracing = False
        self.create_timer(0.05, self.sample_trace)

    def tool_pose(self):
        transform = self.tf_buffer.lookup_transform(
            self.config["frame_id"],
            self.config["ee_link"],
            rclpy.time.Time(),
        )
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def sample_trace(self):
        if not self.tracing:
            return
        try:
            pose = self.tool_pose()
        except TransformException:
            return
        if self.trace.poses:
            old = self.trace.poses[-1].pose.position
            new = pose.position
            distance = math.dist(
                (old.x, old.y, old.z), (new.x, new.y, new.z))
            if distance < 0.001:
                return
        stamped = PoseStamped()
        stamped.header.frame_id = self.config["frame_id"]
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.pose = pose
        self.trace.poses.append(stamped)
        self.trace.header.stamp = stamped.header.stamp
        self.trace_pub.publish(self.trace)

    def rotate_tool(self, orientation, angle):
        half = -0.25 * angle
        cosine = math.cos(half)
        sine = math.sin(half)
        rotated = copy.deepcopy(orientation)
        rotated.x = cosine * orientation.x - sine * orientation.y
        rotated.y = cosine * orientation.y + sine * orientation.x
        rotated.z = cosine * orientation.z + sine * orientation.w
        rotated.w = cosine * orientation.w - sine * orientation.z
        return rotated

    def make_target_path(self, points, orientation, rotate_tool=False):
        target = Path()
        target.header.frame_id = self.config["frame_id"]
        target.header.stamp = self.get_clock().now().to_msg()
        denominator = max(1, len(points) - 1)
        for index, (x, y, z) in enumerate(points):
            stamped = PoseStamped()
            stamped.header = copy.deepcopy(target.header)
            stamped.pose.position.x = x
            stamped.pose.position.y = y
            stamped.pose.position.z = z
            if rotate_tool:
                angle = 2.0 * math.pi * index / denominator
                stamped.pose.orientation = self.rotate_tool(
                    orientation, angle)
            else:
                stamped.pose.orientation = copy.deepcopy(orientation)
            target.poses.append(stamped)
        self.target_pub.publish(target)
        return target

    def circle(self):
        keys = ("x", "y", "z", "radius", "step")
        return circle_points(*(self.config[key] for key in keys))

    def wait_for_tool_pose(self):
        deadline = time.monotonic() + 30.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.tool_pose()
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError(
                        "No tool TF: check robot_state_publisher and "
                        "joint_states")
        return None

    def execute_drawing(self, trajectory):
        self.tracing = True
        self.sample_trace()
        try:
            self.motion.execute(trajectory)
        finally:
            self.sample_trace()
            self.tracing = False

    def validate_origin(self, pose):
        reached = (pose.position.x, pose.position.y, pose.position.z)
        configured = (
            self.config["x"], self.config["y"], self.config["z"])
        error = math.dist(reached, configured)
        self.get_logger().info(f"Drawing-origin error: {error:.4f} m")
        if error > 0.01:
            raise RuntimeError(
                f"TCP stopped {error:.3f} m from the configured drawing "
                "origin")
        orientation = pose.orientation
        tool_z_world_z = 1.0 - 2.0 * (
            orientation.x ** 2 + orientation.y ** 2)
        tilt_error = math.acos(max(-1.0, min(1.0, -tool_z_world_z)))
        self.get_logger().info(
            f"Tool perpendicular error: {math.degrees(tilt_error):.2f} deg")
        if tilt_error > 0.05:
            raise RuntimeError(
                "tool0 is not perpendicular to the X-Y drawing plane")

    def run(self):
        if not self.config["use_sim_time"]:
            raise RuntimeError(
                "This node is for simulation; set use_sim_time:=true")
        current = self.wait_for_tool_pose()
        if current is None:
            return

        target = self.make_target_path(
            self.circle(), current.orientation, rotate_tool=True)
        self.get_logger().info(
            "Published circle on /draw_p/target_path")
        self.motion.wait_for_cartesian_service()
        if not self.config["execute"]:
            self.get_logger().info(
                "Preview only; execute:=false, so robot will not move")
            return

        self.motion.move_to_start()
        current = self.tool_pose()
        self.motion.wait_for_execute_server()
        self.validate_origin(current)
        target = self.make_target_path(
            self.circle(), current.orientation, rotate_tool=True)
        drawing = self.motion.compute_cartesian(
            [pose.pose for pose in target.poses],
            target.header,
            "Circle path",
        )
        self.execute_drawing(drawing)
        self.get_logger().info(
            "Finished drawing circle. Paths remain visible while this node "
            "runs.")
