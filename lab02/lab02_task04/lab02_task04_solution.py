#!/usr/bin/env python3

import rclpy
import math
import time
from std_msgs.msg import Float64


def joint_pub():
    # initialize ROS2 node
    rclpy.init()
    node = rclpy.create_node('trajectory_generator')

    # define rate (10 Hz)
    class Rate:
        def __init__(self, hz: float):
            self._period = 1.0 / hz
        def sleep(self):
            time.sleep(self._period)
    rate = Rate(10)

    ##TODO: Define each joint publisher, by identifying the appropriate publishing topic.
    

    ##TODO: Define the messages to be published.
    

    while rclpy.ok():
        # current time in seconds (float)
        t = node.get_clock().now().nanoseconds / 1e9

        ##TODO: Define the joint trajectories.
        

        ##TODO: Publish all messages.
        

    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        joint_pub()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
