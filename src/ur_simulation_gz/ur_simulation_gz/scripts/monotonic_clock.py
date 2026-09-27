#!/usr/bin/env python3
"""Relay Gazebo simulation time while rejecting backward clock samples."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class MonotonicClock(Node):
    def __init__(self):
        super().__init__("monotonic_clock")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Clock, "/clock", qos)
        self.subscription = self.create_subscription(Clock, "/raw_clock", self.relay, qos)
        self.last_ns = -1
        self.dropped = 0

    def relay(self, message):
        stamp = message.clock
        current_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if current_ns < self.last_ns:
            self.dropped += 1
            if self.dropped == 1 or self.dropped % 1000 == 0:
                self.get_logger().warning(
                    f"Dropped backward Gazebo clock sample #{self.dropped}: "
                    f"{current_ns} < {self.last_ns} ns"
                )
            return

        self.last_ns = current_ns
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = MonotonicClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
