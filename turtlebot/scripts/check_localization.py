#!/usr/bin/env python3
"""Check AMCL against Gazebo after slow motion in the simple_square world.

Start simulation/localization, set an initial pose and undock first. This is
localization verification, not a Nav2 navigation test. Simulator feedback is
used only for the test route and independent measurement, never fed to AMCL.
"""
import hashlib
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped
from auto_survey import Survey, wrap

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--initial-pose', type=float, nargs=3, required=True,
                        help='Approximate known x y yaw in the saved map, after undock')
    parser.add_argument('--map', type=Path, default=ROOT/'maps/map.yaml')
    parser.add_argument('--report', type=Path, default=ROOT/'reports/part3/localization_check.json')
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Survey()
    amcl = []
    subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', amcl.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    report = {'map': str(args.map.resolve()), 'world': 'simple_square',
              'initial_estimate': args.initial_pose, 'samples': [], 'passed': False,
              'thresholds': {'position_m': .2, 'yaw_rad': .2617993878},
              'note': 'AMCL uses saved map, scan and wheel odometry. Ground truth is used '
                      'only to drive the verification route and independently compare poses. '
                      'Comparisons are stationary, with map axes corresponding to world axes '
                      'from the original SLAM start. This is not a navigation-goal test.'}
    import yaml
    metadata=yaml.safe_load(args.map.read_text())
    for path in (args.map, args.map.parent/metadata['image']):
        report[path.name+'_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()

    def capture(label):
        node.command()
        node.spin(3)
        tf = node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        t, q = tf.transform.translation, tf.transform.rotation
        estimate_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        gt = list(node.pose)
        error = math.hypot(t.x-gt[0], t.y-gt[1])
        yaw_error = abs(wrap(estimate_yaw-gt[2]))
        sample = {'label': label, 'gazebo_xy_yaw': gt,
                  'amcl_map_base_xy_yaw': [t.x, t.y, estimate_yaw],
                  'position_error_m': error, 'yaw_error_rad': yaw_error,
                  'yaw_error_deg': math.degrees(yaw_error),
                  'amcl_pose_messages_seen': len(amcl),
                  'tf_stamp': [tf.header.stamp.sec, tf.header.stamp.nanosec],
                  'passed': error < .2 and yaw_error < .2617993878}
        report['samples'].append(sample)
        print(json.dumps(sample), flush=True)

    try:
        initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose',
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic()+30
        while not (initial.get_subscription_count() and node.pose and
                   node.get_clock().now().nanoseconds > 0):
            if time.monotonic()>deadline:
                raise RuntimeError('Initial-pose subscriber or simulator pose is unavailable')
            node.spin(.1)
        seed = PoseWithCovarianceStamped()
        seed.header.frame_id = 'map'
        seed.header.stamp = (node.get_clock().now()-rclpy.duration.Duration(seconds=.1)).to_msg()
        seed.pose.pose.position.x, seed.pose.pose.position.y, yaw = args.initial_pose
        seed.pose.pose.orientation.z, seed.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        seed.pose.covariance[0] = seed.pose.covariance[7] = .25
        seed.pose.covariance[35] = .1
        initial.publish(seed)
        next_publish = time.monotonic()+2
        deadline = time.monotonic()+30
        while not any((m.header.stamp.sec, m.header.stamp.nanosec) >=
                      (seed.header.stamp.sec, seed.header.stamp.nanosec) for m in amcl):
            if time.monotonic()>deadline:
                raise RuntimeError('AMCL did not acknowledge the initial estimate with a fresh pose')
            node.spin(.1)
            if time.monotonic() >= next_publish:
                initial.publish(seed)
                next_publish = time.monotonic()+2
        deadline = time.monotonic()+45
        while not (node.pose and node.scan and amcl and
                   node.tf_buffer.can_transform('map','base_link',rclpy.time.Time())):
            if time.monotonic()>deadline:
                raise RuntimeError('Missing AMCL or map/base_link TF; set initial pose first')
            node.spin(.1)
        nodes = node.get_node_names_and_namespaces()
        report['slam_stopped'] = not any(name=='slam_toolbox' for name, ns in nodes)
        if not report['slam_stopped']:
            raise RuntimeError('SLAM is still running')
        q = node.tf_buffer.lookup_transform('base_link', node.scan.header.frame_id,
                                             rclpy.time.Time()).transform.rotation
        node.laser_yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        capture('after_initial_estimate')
        for label, target in [('west',(-1.6,0)), ('south_west',(-1.6,-1.6))]:
            node.go(target)
            capture(label)
        report['passed'] = all(s['passed'] for s in report['samples']) and not node.bump_messages
    except (RuntimeError, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'Interrupted'
    finally:
        node.command(); node.spin(.5)
        report['distance_m'] = node.distance
        report['bump_messages'] = node.bump_messages
        path = args.report
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2)+'\n')
        print('Localization check passed:', report['passed'], flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
