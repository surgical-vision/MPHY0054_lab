#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float64MultiArray

# keep a global node reference for logging inside the callback (preserve function signature)
node = None

def listen(msg):
    # log received data
    if node is not None:
        node.get_logger().info(str(msg.data))

def listener():
    # initialize node and create a subscriber
    global node
    rclpy.init()
    node = rclpy.create_node('listener')
    node.create_subscription(Float64MultiArray, 'chatter', listen, 10)

    # keep the node alive
    rclpy.spin(node)

    # cleanup
    node.destroy_node()
    rclpy.shutdown()

def main():
    # entry point that preserves original structure
    listener()

if __name__ == '__main__':
    main()
