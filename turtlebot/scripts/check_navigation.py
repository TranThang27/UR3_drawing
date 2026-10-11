#!/usr/bin/env python3
"""Send sequential Nav2 goals and record results, final TF poses and bumper data."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=3, action='append', required=True,
                        metavar=('X', 'Y', 'YAW'), help='Pose in map, yaw in radians; repeat for each goal')
    parser.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW'),
                        help='Only supply when this pose is known in your saved map')
    parser.add_argument('--timeout', type=float, default=120, help='Wall seconds per goal')
    parser.add_argument('--report', type=Path, default=Path('reports/navigation_goals.json'))
    args = parser.parse_args()
    values = [v for goal in args.goal for v in goal] + (args.initial_pose or [])
    if not all(math.isfinite(v) for v in values) or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('Coordinates must be finite; timeout must be positive.')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('tb4_navigation_check')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    state_client = node.create_client(GetState, '/bt_navigator/get_state')
    initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    observations = {'hazard_messages': 0, 'bump_messages': 0, 'plan_messages': 0}
    amcl = []

    def hazards(msg):
        observations['hazard_messages'] += 1
        if any(d.type == HazardDetection.BUMP for d in msg.detections):
            observations['bump_messages'] += 1

    def plan(msg):
        if msg.poses:
            observations['plan_messages'] += 1

    subscriptions = [
        node.create_subscription(HazardDetectionVector, '/hazard_detection', hazards, qos_profile_sensor_data),
        node.create_subscription(NavPath, '/plan', plan, 10),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def wait(condition, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(node, timeout_sec=0.1)
        return bool(condition())

    report = {'goals': [], 'observations': observations, 'passed': False,
              'note': 'Bumper data detects reported contacts; also inspect Gazebo and RViz2. '
                      'Final map TF alone does not establish ground-truth localization accuracy.'}
    handle = None
    try:
        if args.initial_pose:
            if not wait(lambda: initial.get_subscription_count() > 0, 30):
                raise RuntimeError('AMCL chưa nhận /initialpose.')
            x, y, yaw = args.initial_pose
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.04
            msg.pose.covariance[35] = 0.04
            initial.publish(msg)
        if not wait(lambda: bool(amcl) and buffer.can_transform('map', 'base_link', rclpy.time.Time()), 30):
            raise RuntimeError('Chưa localization: đặt 2D Pose Estimate và kiểm tra LiDAR khớp map.')
        if not client.wait_for_server(timeout_sec=60):
            raise RuntimeError('Nav2 action chưa sẵn sàng.')
        if not state_client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Không có lifecycle service cho bt_navigator.')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            state = state_client.call_async(GetState.Request())
            if wait(state.done, 5) and state.result().current_state.id == 3:
                break
            wait(lambda: False, 0.5)
        else:
            raise RuntimeError('bt_navigator chưa active.')
        for x, y, yaw in args.goal:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            before = observations.copy()
            entry = {'goal': [x, y, yaw], 'succeeded': False}
            report['goals'].append(entry)
            sent = client.send_goal_async(goal)
            if not wait(sent.done, 15):
                raise RuntimeError('Gửi goal hết thời gian.')
            handle = sent.result()
            if not handle.accepted:
                raise RuntimeError('Goal bị từ chối.')
            print(f'Nav2 đang đi tới ({x}, {y}); không dùng teleop.', flush=True)
            result = handle.get_result_async()
            if not wait(result.done, args.timeout):
                raise RuntimeError('Goal hết thời gian; hủy goal và dừng kiểm tra.')
            entry['status'] = result.result().status
            handle = None
            pose = buffer.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
            entry['final_xy'] = [pose.translation.x, pose.translation.y]
            entry['position_error_m'] = math.hypot(x - pose.translation.x, y - pose.translation.y)
            entry['hazard_messages'] = observations['hazard_messages'] - before['hazard_messages']
            entry['bump_messages'] = observations['bump_messages'] - before['bump_messages']
            entry['plan_messages'] = observations['plan_messages'] - before['plan_messages']
            entry['succeeded'] = (entry['status'] == GoalStatus.STATUS_SUCCEEDED
                                  and entry['position_error_m'] < 0.35
                                  and entry['hazard_messages'] > 0
                                  and entry['bump_messages'] == 0 and entry['plan_messages'] > 0)
            print(json.dumps(entry, ensure_ascii=False), flush=True)
            if not entry['succeeded']:
                raise RuntimeError('Goal chưa đạt kiểm tra; xem report.')
        report['passed'] = True
    except (RuntimeError, TransformException, KeyboardInterrupt) as error:
        report['error'] = str(error) or 'Interrupted'
        print(report['error'], flush=True)
    finally:
        if handle is not None:
            canceled = handle.cancel_goal_async()
            wait(canceled.done, 10)
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
