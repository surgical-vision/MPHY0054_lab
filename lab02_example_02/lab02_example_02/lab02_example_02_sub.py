#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.subscriber = self.create_subscription(String, 'chatter', self.listener_callback, 10)


    def listener_callback(self, msg):
        self.get_logger().info("I am attending: %s" % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode('lab02_example_02_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
