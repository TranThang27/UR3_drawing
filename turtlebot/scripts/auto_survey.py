#!/usr/bin/env python3
"""Survey simple_square with slow velocity commands and simulator pose feedback.

Run with simulation and SLAM active, after undock, and with keyboard teleop paused.
This route is specific to worlds/simple_square.sdf, not a general exploration planner.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
WAYPOINTS = [(-2.1, 0), (-2.1, -2.1), (2.1, -2.1), (2.1, 2.1),
             (0, 2.1), (-2.1, 2.1), (-2.1, 0), (-0.5, 0),
             (-0.5, 0.7), (0.2, 0.7), (0.2, 2.1), (0.2, 0.7),
             (-0.5, 0.7), (-0.5, -0.7), (-1.7, -0.7), (-1.7, 0), (-0.4, 0)]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Survey(Node):
    def __init__(self):
        super().__init__('simple_square_auto_survey')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.pose = None
        self.odom_pose = None
        self.scan = None
        self.last_odom = self.last_scan = 0
        self.bump_messages = 0
        self.distance = 0
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_yaw = 0.0
        self.subs = [
            self.create_subscription(Odometry, '/sim_ground_truth_pose', self.odom, qos_profile_sensor_data),
            self.create_subscription(Odometry, '/odom', self.wheel_odom, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.lidar, qos_profile_sensor_data),
            self.create_subscription(HazardDetectionVector, '/hazard_detection',
                                     self.hazards, qos_profile_sensor_data)]

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        if self.pose:
            step = math.hypot(p.x-self.pose[0], p.y-self.pose[1])
            if step < 0.5:
                self.distance += step
        self.pose = (p.x, p.y, yaw)
        self.last_odom = time.monotonic()

    def lidar(self, msg):
        self.scan = msg
        self.last_scan = time.monotonic()

    def wheel_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pose = [p.x, p.y]

    def hazards(self, msg):
        if any(h.type == HazardDetection.BUMP for h in msg.detections):
            self.bump_messages += 1

    def command(self, forward=0.0, turn=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(forward), float(turn)
        self.velocity.publish(cmd)

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.02)

    def go(self, target):
        print(f'Survey waypoint {target}', flush=True)
        deadline = time.monotonic() + 100
        mode = 'rotate'
        while time.monotonic() < deadline:
            self.spin(0.05)
            now = time.monotonic()
            if now - self.last_odom > 1 or now - self.last_scan > 1:
                raise RuntimeError('Lost fresh odometry or LiDAR; stopping.')
            if self.bump_messages:
                raise RuntimeError('Bumper contact detected; stopping.')
            x, y, yaw = self.pose
            distance = math.hypot(target[0]-x, target[1]-y)
            if distance < 0.07:
                self.command()
                self.spin(0.3)
                return
            error = wrap(math.atan2(target[1]-y, target[0]-x)-yaw)
            if mode == 'rotate' and abs(error) < 0.06:
                mode = 'drive'
            if abs(error) > 0.35:
                mode = 'rotate'
            if mode == 'rotate':
                self.command(turn=max(-0.45, min(0.45, 1.2*error)))
            else:
                # Forward sector in base coordinates; stop before a physical obstacle.
                scan = self.scan
                front = [r for i, r in enumerate(scan.ranges)
                         if abs(wrap(self.laser_yaw+scan.angle_min+i*scan.angle_increment)) < 0.25
                         and math.isfinite(r) and r > scan.range_min]
                if front and min(front) < 0.38:
                    raise RuntimeError(f'Obstacle within {min(front):.2f} m in front; stopping.')
                self.command(forward=min(0.22, max(0.06, 0.5*distance)),
                             turn=max(-0.25, min(0.25, 1.5*error)))
        raise RuntimeError(f'Waypoint timeout: {target}')


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Survey()
    report = {'world': 'simple_square', 'waypoints_completed': [], 'success': False}
    try:
        deadline = time.monotonic() + 10
        while (node.pose is None or node.scan is None or
               not node.tf_buffer.can_transform('base_link', node.scan.header.frame_id,
                                                rclpy.time.Time())):
            if time.monotonic() > deadline:
                raise RuntimeError('No sensor feedback.')
            node.spin(0.1)
        q = node.tf_buffer.lookup_transform('base_link', node.scan.header.frame_id,
                                             rclpy.time.Time()).transform.rotation
        node.laser_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if math.hypot(node.pose[0]+0.35, node.pose[1]) > 0.3:
            raise RuntimeError('Expected freshly restarted robot after undock near (-0.35, 0).')
        for target in WAYPOINTS:
            node.go(target)
            report['waypoints_completed'].append(target)
        # Finish facing the starting direction and let loop closure/map update settle.
        deadline = time.monotonic() + 12
        while abs(wrap(node.pose[2])) > 0.05 and time.monotonic() < deadline:
            node.command(turn=max(-0.4, min(0.4, wrap(-node.pose[2]))))
            node.spin(0.05)
        node.command()
        node.spin(5)
        report['success'] = True
    except (RuntimeError, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'Interrupted'
        print(report['error'], flush=True)
    finally:
        for _ in range(8):
            node.command()
            node.spin(0.05)
        report.update(distance_m=node.distance, bump_messages=node.bump_messages,
                      final_gazebo_pose=node.pose, final_odom_xy=node.odom_pose)
        path = ROOT / 'reports/auto_survey.json'
        path.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
