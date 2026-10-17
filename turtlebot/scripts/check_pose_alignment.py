#!/usr/bin/env python3
"""Read-only comparison of AMCL TF with Gazebo ground truth; no velocity publisher."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(); node = Node('pose_alignment_observer')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    data = []
    sub = node.create_subscription(Odometry, '/sim_ground_truth_pose', data.append, qos_profile_sensor_data)
    buffer = Buffer(node=node); listener = TransformListener(buffer, node)
    deadline = time.monotonic()+30
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if data and buffer.can_transform('map', 'base_link', rclpy.time.Time()):
            break
    report = {'passed': False, 'read_only': True}
    if data and buffer.can_transform('map', 'base_link', rclpy.time.Time()):
        gt = data[-1].pose.pose; pose = buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
        def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        error = math.hypot(pose.translation.x-gt.position.x, pose.translation.y-gt.position.y)
        dyaw = yaw(pose.rotation)-yaw(gt.orientation)
        angle = abs(math.atan2(math.sin(dyaw),math.cos(dyaw)))
        report.update(gazebo_pose=[gt.position.x,gt.position.y,yaw(gt.orientation)],
                      amcl_map_pose=[pose.translation.x,pose.translation.y,yaw(pose.rotation)],
                      position_error_m=error, yaw_error_rad=angle,
                      passed=error<.25 and angle<.35)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,indent=2)+'\n'); print(json.dumps(report),flush=True)
    node.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
