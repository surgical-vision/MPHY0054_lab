#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from lab03_mes.msg import Mphy0054


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(Mphy0054, 'chatter', 10)
        self.create_timer(0.1, self.callback)  # 10 Hz

    def callback(self):
        msg = Mphy0054()
        msg.name = "ROS2 Foxy MPHY"
        msg.student_number = 54
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: name={msg.name}, student number={msg.student_number}')

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
