#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(Int64, 'chatter', 10)
        self.count = 0
        self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        msg = Int64()
        msg.data = random.randint(0, self.count)
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(f'Publishing: {msg.data}')

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
