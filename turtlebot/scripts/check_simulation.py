#!/usr/bin/env python3
"""Check live simulation data; optionally verify cmd_vel using odometry feedback."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class SimulationCheck(Node):
    def __init__(self):
        super().__init__('turtlebot4_simulation_check')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.messages = {}
        self.arrivals = {name: [] for name in ('scan', 'odom', 'clock')}
        self.subscriptions_kept = [
            self.create_subscription(msg_type, '/' + name,
                                     lambda msg, key=name: self.receive(key, msg),
                                     qos_profile_sensor_data)
            for name, msg_type in [('scan', LaserScan), ('odom', Odometry), ('clock', Clock)]
        ]
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clock_values = []

    def receive(self, key, msg):
        self.messages[key] = msg
        self.arrivals[key].append(time.monotonic())
        if key == 'clock':
            self.clock_values.append(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def spin_for(self, seconds):
        until = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.05)

    def inspect(self):
        now = time.monotonic()
        result = {}
        for key, arrivals in self.arrivals.items():
            rate = ((len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
                    if len(arrivals) > 1 and arrivals[-1] > arrivals[0] else 0.0)
            result['/' + key] = {
                'ok': len(arrivals) >= 2 and now - arrivals[-1] < 2.0,
                'messages': len(arrivals), 'observed_hz': round(rate, 2),
            }
        result['clock_advances'] = {
            'ok': len(self.clock_values) >= 2 and self.clock_values[-1] > self.clock_values[0]
        }
        scan = self.messages.get('scan')
        result['scan_data'] = {'ok': bool(
            scan and scan.header.frame_id and scan.ranges
            and scan.range_max > scan.range_min >= 0
            and any((math.isfinite(r) and scan.range_min <= r <= scan.range_max)
                    or r == math.inf for r in scan.ranges))}
        usable_hits = (sum(math.isfinite(r) and scan.range_min + 0.01 < r <= scan.range_max
                           for r in scan.ranges) if scan else 0)
        result['lidar_obstacles'] = {
            'ok': usable_hits > 0, 'usable_hits': usable_hits,
            'hint': 'World mặc định có tường/vật cản; scan toàn range_min là lỗi GPU rendering.'}
        odom = self.messages.get('odom')
        result['odom_data'] = {'ok': bool(
            odom and odom.header.frame_id and odom.child_frame_id
            and all(math.isfinite(value) for value in (
                odom.pose.pose.position.x, odom.pose.pose.position.y,
                odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w))
            and abs(sum(value * value for value in (
                odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)) - 1) < 0.1)}
        frames = {'odom_to_base': ('odom', 'base_link')}
        if scan:
            frames['odom_to_lidar'] = ('odom', scan.header.frame_id)
        else:
            frames['odom_to_lidar'] = ('odom', 'rplidar_link')
        for label, (target, source) in frames.items():
            result[label] = {'ok': bool(self.tf_buffer.can_transform(target, source, rclpy.time.Time())),
                             'target': target, 'source': source}
        subscribers = self.count_subscribers('/cmd_vel')
        result['cmd_vel_receiver'] = {'ok': subscribers > 0, 'subscribers': subscribers}
        return result

    def motion_check(self):
        # A short rotation avoids assuming that a path ahead is clear.
        before = self.messages['odom'].pose.pose.orientation
        initial_yaw = quaternion_yaw(before)
        command = Twist()
        command.angular.z = 0.2
        start = time.monotonic()
        try:
            while rclpy.ok() and time.monotonic() - start < 3.0:
                self.velocity.publish(command)
                self.spin_for(0.1)
        finally:
            # Always stop, including Ctrl+C during the motion test.
            if rclpy.ok():
                for _ in range(5):
                    self.velocity.publish(Twist())
                    self.spin_for(0.1)
        after = self.messages['odom'].pose.pose.orientation
        delta = quaternion_yaw(after) - initial_yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))
        fresh = self.arrivals['odom'][-1] > start
        return {'ok': fresh and abs(delta) > 0.05,
                'yaw_change_rad': round(delta, 4), 'fresh_odometry': fresh,
                'hint': 'Nếu thất bại: undock trước; dừng teleop/Nav2; kiểm tra Gazebo đang Play.'}


def quaternion_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=15.0,
                        help='Observation window in wall-clock seconds')
    parser.add_argument('--motion', action='store_true', help='Rotate at 0.2 rad/s for 3 seconds')
    parser.add_argument('--report', type=Path, help='Write a JSON report')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout < 2.0:
        parser.error('--timeout must be finite and >= 2 seconds')
    # Keep the context alive during Python's Ctrl+C handling so finally can stop the robot.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = SimulationCheck()
    try:
        node.spin_for(args.timeout)
        report = node.inspect()
        if args.motion:
            if all(item['ok'] for item in report.values()):
                report['motion_response'] = node.motion_check()
            else:
                report['motion_response'] = {'ok': False, 'hint': 'Prerequisites failed; motion skipped.'}
        for name, detail in report.items():
            print(f"{'PASS' if detail['ok'] else 'FAIL'} {name}: {json.dumps(detail, ensure_ascii=False)}")
        print('Kiểm tra hiển thị robot/sensor trong Gazebo và RViz2 bằng mắt theo README.')
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        return 0 if all(item['ok'] for item in report.values()) else 1
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
