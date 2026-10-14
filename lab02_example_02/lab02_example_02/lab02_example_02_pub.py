#!/usr/bin/env python3

from typing import Type

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
1. type
2. declare and createthe publisher
3. context of the publisher


"""



class PublisherNode(Node):
    def __init__(self,name):
        super().__init__(name)
        # 2. declare and create the publisher
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "MPHY0054_Robotic Systems Engineering"
        self.publisher.publish(msg)
        self.get_logger().info("Publishing: %s" % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode('lab02_example_02_pub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
