#!/usr/bin/env python3

import rclpy
import math
import random
import time
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped


def talker():
    ##TODO: Define a publisher, initialize the node, and define a rate
    rclpy.init()
    node = rclpy.create_node('talker')
    pub = node.create_publisher(Vector3Stamped, 'chatter', 10)

    # small helper to preserve the original `rate.sleep()` structure
    class Rate:
        def __init__(self, hz: float):
            self._period = 1.0 / hz
        def sleep(self):
            time.sleep(self._period)

    rate = Rate(10)  # 10hz

    ##TODO: Define the unit circle radius, and the message to be published
    radius = 1.0
    vector_for_publish = Vector3Stamped()

    while rclpy.ok():
        # The VectorStamped type needs a timestamp. This timestamp can be the current time.
        time_stamp = node.get_clock().now().to_msg()

        ##TODO: Define the position (x,y,z) on the unit circle
        angle = random.uniform(-2 * math.pi, 2 * math.pi)
        x = radius * math.cos(angle)
        y = 0.0
        z = radius * math.sin(angle)

        ##TODO: Input the data into your message.
        vector_for_publish.header.stamp = time_stamp
        vector_for_publish.vector.x = x
        vector_for_publish.vector.y = y
        vector_for_publish.vector.z = z

        ##TODO: Publish the message
        pub.publish(vector_for_publish)
        print(f'{x:.3f}, {y:.3f}, {z:.3f}')
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        talker()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
