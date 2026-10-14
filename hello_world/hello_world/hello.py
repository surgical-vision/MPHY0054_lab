#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = Node("hello_mphy0054")

    while rclpy.ok():
        node.get_logger().info("Hello MPHY0054!")
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()
