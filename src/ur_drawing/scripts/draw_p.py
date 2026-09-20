#!/usr/bin/env python3

import rclpy

from drawing_node import DrawCircle


def main():
    rclpy.init()
    node = None
    try:
        node = DrawCircle()
        try:
            node.run()
        except (RuntimeError, ValueError) as exc:
            node.get_logger().error(str(exc))
            return
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
