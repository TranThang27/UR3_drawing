#!/usr/bin/env python3
"""Record teleoperation distance and publish the live trajectory in the map frame."""
import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformException, TransformListener


class SurveyRecorder(Node):
    def __init__(self, output):
        super().__init__('tb4_survey_recorder')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.started = time.monotonic()
        self.distance = 0.0
        self.previous_odom = None
        self.map_info = None
        self.rows = []
        self.path = RosPath()
        self.path.header.frame_id = 'map'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(RosPath, '/survey_path', latched)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.on_map, latched)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.sample_timer = self.create_timer(1.0, self.sample)
        self.report_timer = self.create_timer(5.0, self.write_report)

    def on_odom(self, message):
        point = message.pose.pose.position
        current = point.x, point.y
        if self.previous_odom:
            step = math.dist(current, self.previous_odom)
            if math.isfinite(step) and step < 1.0:
                self.distance += step
        self.previous_odom = current

    def on_map(self, message):
        known = sum(value >= 0 for value in message.data)
        free = sum(0 <= value <= 25 for value in message.data)
        self.map_info = {
            'width_px': message.info.width, 'height_px': message.info.height,
            'resolution_m': message.info.resolution,
            'known_area_m2': round(known * message.info.resolution ** 2, 2),
            'free_area_m2': round(free * message.info.resolution ** 2, 2),
        }

    def sample(self):
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return
        p = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        if self.rows:
            previous = self.rows[-1]
            angle = math.atan2(math.sin(yaw - previous[3]), math.cos(yaw - previous[3]))
            if math.dist((p.x, p.y), (previous[1], previous[2])) < 0.05 and abs(angle) < 0.1:
                return
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x, pose.pose.position.y = p.x, p.y
        pose.pose.orientation = q
        self.path.poses.append(pose)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.path)
        self.rows.append((self.get_clock().now().nanoseconds * 1e-9, p.x, p.y, yaw))

    def write_report(self):
        report = {
            'distance_traveled_m': round(self.distance, 2),
            'elapsed_wall_seconds': round(time.monotonic() - self.started, 1),
            'trajectory_samples': len(self.rows), 'map': self.map_info,
            'coverage_verified': False,
            'note': 'Quãng đường và diện tích map không tự chứng minh đã khảo sát phần lớn môi trường. '
                    'Đối chiếu các khu vực trên Gazebo/RViz2; xem lại sau loop closure.',
        }
        temporary = self.output / '.survey.json.tmp'
        temporary.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        temporary.replace(self.output / 'survey.json')
        with (self.output / 'trajectory.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('sim_seconds', 'map_x_m', 'map_y_m', 'map_yaw_rad'))
            writer.writerows(self.rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SurveyRecorder(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
