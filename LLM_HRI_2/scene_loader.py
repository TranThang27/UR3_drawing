#!/usr/bin/env python3
"""Add the tabletop environment to the MoveIt planning scene."""

import sys
import time
from itertools import combinations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    CollisionObject,
    ObjectColor,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, DurabilityPolicy
import json
from std_msgs.msg import String
from vision import require_fresh


PLANNING_FRAME = "world"


def box_primitive(size, position):
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)

    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.w = 1.0
    return primitive, pose


def collision_object(object_id, boxes):
    obj = CollisionObject()
    obj.header.frame_id = PLANNING_FRAME
    obj.id = object_id
    obj.operation = CollisionObject.ADD
    for size, position in boxes:
        primitive, pose = box_primitive(size, position)
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
    return obj


def object_color(object_id, rgba):
    color = ObjectColor()
    color.id = object_id
    color.color.r, color.color.g, color.color.b, color.color.a = rgba
    return color


def build_scene():
    """Build geometry in the robot's planning frame.

    Gazebo places the robot at (-0.27, 0, 0.75). The UR description keeps its
    planning frame at the robot base, so all Gazebo world coordinates are
    translated into that frame here.
    """
    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True

    scene.world.collision_objects = [
        collision_object(
            "work_table",
            [
                ((1.20, 0.90, 0.08), (0.42, 0.0, -0.04)),
                ((0.07, 0.07, 0.70), (-0.08, -0.35, -0.40)),
                ((0.07, 0.07, 0.70), (-0.08, 0.35, -0.40)),
                ((0.07, 0.07, 0.70), (0.92, -0.35, -0.40)),
                ((0.07, 0.07, 0.70), (0.92, 0.35, -0.40)),
            ],
        ),

    ]

    scene.object_colors = [
        object_color("work_table", (0.55, 0.36, 0.19, 1.0)),

    ]
    return scene


class SceneLoader(Node):
    def __init__(self):
        super().__init__("pick_place_scene_loader")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.camera_state = None
        self.create_subscription(String, "/hri2/world_state", self.on_camera, 10)
        self.labels = self.create_publisher(MarkerArray, "/zone_labels", QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_camera(self, message):
        self.camera_state = json.loads(message.data)

    def publish_labels(self):
        markers = MarkerArray()
        for index, (label, y) in enumerate((("A", -0.16), ("B", 0.0), ("C", 0.16))):
            marker = Marker()
            marker.header.frame_id = PLANNING_FRAME
            marker.ns = "zone_labels"
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            if not self.camera_state or "zone_" + label.lower() not in self.camera_state["zones"]:
                continue
            pos = self.camera_state["zones"]["zone_" + label.lower()]["position"]
            marker.pose.position.x = pos[0] + 0.10
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.04
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.045
            marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
            marker.text = label
            markers.markers.append(marker)
        self.labels.publish(markers)

    def allow_mounted_adapter_collision(self, scene):
        """Preserve MoveIt's ACM and allow the fixed UR/Robotiq overlap."""
        if not self.get_scene.wait_for_service(timeout_sec=120.0):
            self.get_logger().error("MoveIt service /get_planning_scene is unavailable")
            return False

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error("Could not read MoveIt's allowed-collision matrix")
            return False

        matrix = future.result().scene.allowed_collision_matrix
        gripper_links = [
            "ur_to_robotiq_link",
            "robotiq_mounting_link",
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
            "robotiq_85_left_inner_knuckle_link",
            "robotiq_85_right_inner_knuckle_link",
            "robotiq_85_left_finger_tip_link",
            "robotiq_85_right_finger_tip_link",
        ]
        allowed_pairs = list(combinations(gripper_links, 2))
        allowed_pairs.append(("wrist_3_link", "ur_to_robotiq_link"))

        for link in {name for pair in allowed_pairs for name in pair}:
            if link in matrix.entry_names:
                continue
            matrix.entry_names.append(link)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(
                AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names))
            )

        for first_link, second_link in allowed_pairs:
            first = matrix.entry_names.index(first_link)
            second = matrix.entry_names.index(second_link)
            matrix.entry_values[first].enabled[second] = True
            matrix.entry_values[second].enabled[first] = True
        scene.allowed_collision_matrix = matrix
        return True

    def apply(self):
        self.get_logger().info("Waiting for MoveIt planning scene service...")
        if not self.client.wait_for_service(timeout_sec=120.0):
            self.get_logger().error("MoveIt service /apply_planning_scene is unavailable")
            return False

        scene = build_scene()
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.camera_state and self.camera_state["complete"]:
                require_fresh(self.camera_state)
                break
        else:
            self.get_logger().error("Camera cannot observe all five cubes and three zones")
            return False
        scene.world.collision_objects = [scene.world.collision_objects[0]]
        scene.object_colors = [scene.object_colors[0]]
        colors = {"red_cube": (1., .03, .03, 1.), "yellow_cube": (1., .85, .02, 1.),
                  "blue_cube": (.02, .15, 1., 1.), "green_cube": (.02, 1., .03, 1.),
                  "purple_cube": (.6, .02, 1., 1.), "zone_a": (0., 1., 1., 1.),
                  "zone_b": (1., .48, .03, 1.), "zone_c": (1., 0., .65, 1.)}
        for name, data in {**self.camera_state["objects"], **self.camera_state["zones"]}.items():
            size = (.05, .05, .05) if name.endswith("cube") else (.12, .12, .002)
            scene.world.collision_objects.append(collision_object(name, [(size, data["position"])]))
            scene.object_colors.append(object_color(name, colors[name]))
        if not self.allow_mounted_adapter_collision(scene):
            return False
        applied = False
        for attempt in range(1, 6):
            request = ApplyPlanningScene.Request()
            request.scene = scene
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            if future.done() and future.result() is not None and future.result().success:
                applied = True
                break
            self.get_logger().warning(
                f"Planning scene attempt {attempt}/5 timed out; retrying..."
            )
            time.sleep(1.0)

        if not applied:
            self.get_logger().error("MoveIt did not accept the planning scene")
            return False

        # Also broadcast the diff. A ROS graph can temporarily contain more
        # than one move_group service with the same name; a service request is
        # then handled by only one server, while this topic update reaches all
        # active planning-scene monitors (including the one used by RViz).
        for _ in range(3):
            self.publisher.publish(scene)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.2)

        self.get_logger().info(
            "Added table plus five camera-detected cubes and three camera-detected zones to MoveIt"
        )
        return True


def main():
    rclpy.init()
    node = SceneLoader()
    success = node.apply()
    if success:
        node.publish_labels()
        node.create_timer(1.0, node.publish_labels)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
