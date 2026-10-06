"""Minimal ROS 2 Humble node for Lab 01."""

import rclpy
from rclpy.node import Node


def main(args=None):
    """Log a greeting and release the ROS resources."""
    rclpy.init(args=args)
    node = Node('hello_node')
    try:
        node.get_logger().info('Hello from ROS 2 Humble!')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
