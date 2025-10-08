#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(Int64, 'chatter', self.cb, 10)

    def cb(self, msg: Int64):
        self.get_logger().info(f'Got: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
