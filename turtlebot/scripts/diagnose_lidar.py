#!/usr/bin/env python3
"""Compare stationary simple_square scans with ray intersections of its SDF.

Uses simulator ground truth and the full 3D base-to-laser transform. Does not
run SLAM or change the map. Robot and dock geometry are excluded from the
reference: short/self returns are reported separately.
"""
import argparse
import json
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]


def rotation(q):
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def xyz(p):
    return np.array([p.x, p.y, p.z])


def reference_ranges(origin, directions):
    result = np.full(len(directions), np.inf)
    world = ET.parse(ROOT / 'worlds/simple_square.sdf').getroot().find('world')
    for model in world.findall('model'):
        model_pose = np.array([float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()])
        for link in model.findall('link'):
            link_pose = np.array([float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()])
            # The training world has axis-aligned shapes and no rotated links.
            assert np.allclose(model_pose[3:], 0) and np.allclose(link_pose[3:], 0)
            center = model_pose[:3] + link_pose[:3]
            geometry = link.find('collision/geometry')
            if geometry.find('box') is not None:
                size = np.array([float(v) for v in geometry.findtext('box/size').split()])
                with np.errstate(divide='ignore', invalid='ignore'):
                    a = (center-size/2-origin)/directions
                    b = (center+size/2-origin)/directions
                enter = np.max(np.minimum(a, b), axis=1)
                leave = np.min(np.maximum(a, b), axis=1)
                distance = np.where((leave >= enter) & (enter > 0), enter, np.inf)
            else:
                radius = float(geometry.findtext('cylinder/radius'))
                height = float(geometry.findtext('cylinder/length'))
                offset = origin[:2]-center[:2]
                a = np.sum(directions[:, :2]**2, axis=1)
                b = 2*directions[:, :2].dot(offset)
                c = offset.dot(offset)-radius**2
                discriminant = b*b-4*a*c
                distance = (-b-np.sqrt(np.maximum(discriminant, 0)))/(2*a)
                z = origin[2]+distance*directions[:, 2]
                distance = np.where((discriminant >= 0) & (distance > 0) &
                                    (abs(z-center[2]) <= height/2), distance, np.inf)
            result = np.minimum(result, distance)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='reports/lidar_investigation/stationary')
    args = parser.parse_args()
    rclpy.init()
    node = Node('stationary_lidar_diagnostic')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest = {}
    scans = []
    node.create_subscription(Odometry, '/sim_ground_truth_pose',
                             lambda m: latest.update(gt=m), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan',
                             lambda m: scans.append(m), qos_profile_sensor_data)
    deadline = time.monotonic()+20
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if len(scans) >= 30 and 'gt' in latest and buffer.can_transform(
                    'base_link', scans[-1].header.frame_id, rclpy.time.Time()):
                break
        if len(scans) < 30 or 'gt' not in latest:
            raise RuntimeError('Missing scan or simulator pose')
        scan = scans[-1]
        pose = latest['gt'].pose.pose
        transform = buffer.lookup_transform('base_link', scan.header.frame_id,
                                            rclpy.time.Time()).transform
        base_rotation = rotation(pose.orientation)
        origin = xyz(pose.position)+base_rotation.dot(xyz(transform.translation))
        angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
        local = np.column_stack([np.cos(angles), np.sin(angles), np.zeros(len(angles))])
        directions = local.dot((base_rotation @ rotation(transform.rotation)).T)
        expected = reference_ranges(origin, directions)
        measured = np.array(scan.ranges)
        valid = np.isfinite(measured) & (measured > scan.range_min+.01)
        error = measured-expected
        bad = valid & (np.abs(error) > .1)
        points = origin+measured[:, None]*directions
        outside = valid & ((np.abs(points[:, 0]) > 3.1) | (np.abs(points[:, 1]) > 3.1))
        samples = np.array([s.ranges for s in scans[-30:]])
        with np.errstate(invalid='ignore'):
            span = np.max(samples, axis=0)-np.min(samples, axis=0)
        stamp = lambda s: s.sec+s.nanosec*1e-9
        report = dict(
            reference='SDF geometry, full 3D ground truth and static laser TF; no SLAM',
            scan_count=30, rays=len(measured), base_xyz=xyz(pose.position).tolist(),
            base_rpy=Rotation.from_matrix(base_rotation).as_euler('xyz').tolist(),
            laser_xyz=origin.tolist(),
            scan_pose_time_difference_s=abs(stamp(scan.header.stamp)-stamp(latest['gt'].header.stamp)),
            valid_rays=int(valid.sum()), error_over_10cm_rays=int(bad.sum()),
            outside_wall_rays=int(outside.sum()),
            median_absolute_error_m=float(np.median(np.abs(error[valid]))),
            largest_absolute_error_m=float(np.max(np.abs(error[valid]))),
            median_range_span_30_scans_m=float(np.nanmedian(span[valid])),
            errors=[dict(index=int(i), angle_deg=float(np.degrees(angles[i])),
                         measured_m=float(measured[i]), expected_m=float(expected[i]),
                         xyz=points[i].tolist()) for i in np.flatnonzero(bad)])
        output = ROOT / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')
        np.savez(output.with_suffix('.npz'), origin=origin, directions=directions,
                 measured=measured, expected=expected, samples=samples)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        reference = origin+expected[:, None]*directions
        axes[0].scatter(reference[:, 0], reference[:, 1], s=5, c='black', label='SDF reference')
        axes[0].scatter(points[valid, 0], points[valid, 1], s=4, c='red', label='/scan')
        axes[0].plot(*origin[:2], 'bo', label='LiDAR')
        axes[0].set_aspect('equal'); axes[0].set_xlim(-5,5); axes[0].set_ylim(-5,5)
        axes[0].legend(); axes[0].grid(); axes[0].set_xlabel('x (m)'); axes[0].set_ylabel('y (m)')
        axes[1].plot(np.degrees(angles), expected, label='SDF reference')
        axes[1].plot(np.degrees(angles), measured, label='/scan')
        axes[1].set_xlabel('LiDAR angle (degrees)'); axes[1].set_ylabel('Range (m)')
        axes[1].legend(); axes[1].grid()
        fig.suptitle('Stationary robot, SLAM disabled'); fig.tight_layout()
        fig.savefig(output.with_suffix('.png'), dpi=150)
        print(json.dumps({k:v for k,v in report.items() if k != 'errors'}, indent=2))
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
