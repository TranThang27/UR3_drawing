#!/usr/bin/env python3
"""Bounded effort servo for the official Robotiq mechanism in Fortress.

Finger-pad orientation follows the measured knuckles, including when an
object blocks closure. No object attachment or model pose commands are used.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

JOINTS = [
    "robotiq_85_left_knuckle_joint", "robotiq_85_right_knuckle_joint",
    "robotiq_85_left_inner_knuckle_joint", "robotiq_85_right_inner_knuckle_joint",
    "robotiq_85_left_finger_tip_joint", "robotiq_85_right_finger_tip_joint",
]


class GripperServo(Node):
    def __init__(self):
        super().__init__("gripper_servo")
        self.target = 0.0
        self.positions = {}
        self.velocities = {}
        self.last_state = None
        self.publisher = self.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)
        self.create_subscription(JointState, "/joint_states", self.state, qos_profile_sensor_data)
        self.create_subscription(Float64, "/gripper/target", self.command, 10)
        self.create_timer(0.004, self.update)

    def state(self, message):
        self.positions.update(zip(message.name, message.position))
        self.velocities.update(zip(message.name, message.velocity))
        self.last_state = self.get_clock().now()

    def command(self, message):
        if math.isfinite(message.data) and 0.0 <= message.data <= 0.7929:
            self.target = message.data

    def update(self):
        if any(name not in self.positions for name in JOINTS):
            return
        if (self.get_clock().now() - self.last_state).nanoseconds > 500_000_000:
            self.publisher.publish(Float64MultiArray(data=[0.0] * 6))
            return
        left = self.positions[JOINTS[0]]
        right = self.positions[JOINTS[1]]
        targets = [self.target, -self.target, left, right, -left, -right]
        effort = []
        for index, (name, target) in enumerate(zip(JOINTS, targets)):
            gain = 1.5 if index < 2 else 2.0
            value = gain * (target - self.positions[name]) - 0.02 * self.velocities.get(name, 0.0)
            effort.append(float(max(-0.5, min(0.5, value))))
        self.publisher.publish(Float64MultiArray(data=effort))


def main():
    rclpy.init()
    node = GripperServo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
