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
    pub1 = node.create_publisher(Float64, '/EffortJointInterface_J1_controller/command', 100)
    pub2 = node.create_publisher(Float64, '/EffortJointInterface_J2_controller/command', 100)
    pub3 = node.create_publisher(Float64, '/EffortJointInterface_J3_controller/command', 100)
    pub4 = node.create_publisher(Float64, '/EffortJointInterface_J4_controller/command', 100)

    ##TODO: Define the messages to be published.
    joint1 = Float64()
    joint2 = Float64()
    joint3 = Float64()
    joint4 = Float64()

    while rclpy.ok():
        # current time in seconds (float)
        t = node.get_clock().now().nanoseconds / 1e9

        ##TODO: Define the joint trajectories.
        joint1.data = 200 * math.pi / 180 * math.sin(2 * math.pi * t / 10)
        joint2.data = 50 * math.pi / 180 * math.sin(2 * math.pi * t / 12)
        joint3.data = -80 * math.pi / 180 * math.sin(2 * math.pi * t / 15)
        joint4.data = 60 * math.pi / 180 * math.sin(2 * math.pi * t / 11)

        ##TODO: Publish all messages.
        pub1.publish(joint1)
        pub2.publish(joint2)
        pub3.publish(joint3)
        pub4.publish(joint4)

        print(f"{joint1.data:.3f}, {joint2.data:.3f}, {joint3.data:.3f}, {joint4.data:.3f}")

        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        joint_pub()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
