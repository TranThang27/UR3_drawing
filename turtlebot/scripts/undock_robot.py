#!/usr/bin/env python3
"""Wait for dock state and confirm the robot is undocked before returning."""
import time

import rclpy
from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    rclpy.init()
    node = Node('tb4_undock_helper')
    state = []
    subscription = node.create_subscription(
        DockStatus, '/dock_status', lambda msg: state.append(msg), qos_profile_sensor_data)
    client = ActionClient(node, Undock, '/undock')
    deadline = time.monotonic() + 90

    def wait_until(condition):
        while rclpy.ok() and time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(node, timeout_sec=0.1)
        return False

    try:
        if not wait_until(lambda: bool(state)):
            raise RuntimeError('Không nhận được /dock_status.')
        if not state[-1].is_docked:
            print('Robot đã ở ngoài dock.')
            return 0
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Không tìm thấy action /undock.')
        sent = client.send_goal_async(Undock.Goal())
        if not wait_until(sent.done):
            raise RuntimeError('Hết thời gian gửi undock goal.')
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('Undock goal bị từ chối.')
        result = handle.get_result_async()
        if not wait_until(result.done):
            handle.cancel_goal_async()
            raise RuntimeError('Undock không hoàn tất trong 90 giây.')
        outcome = result.result()
        print(f'Undock action status={outcome.status}, is_docked={outcome.result.is_docked}')
        if outcome.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Undock action không thành công; kiểm tra log và hazard_detection.')
        if outcome.result.is_docked or not wait_until(lambda: not state[-1].is_docked):
            raise RuntimeError('Robot vẫn ở trên dock.')
        print('Undock thành công; robot sẵn sàng nhận /cmd_vel.')
        return 0
    except (RuntimeError, KeyboardInterrupt) as error:
        print(error)
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
