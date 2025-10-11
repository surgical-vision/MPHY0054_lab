#!/usr/bin/env python3

import rclpy
import time
from std_msgs.msg import Float64MultiArray

def talker():
    # initialize the ROS2 Python client library
    rclpy.init()
    # create a node named 'talker'
    node = rclpy.create_node('talker')

    # create a publisher for Float64MultiArray messages on topic 'chatter'
    pub = node.create_publisher(Float64MultiArray, 'chatter', 10)

    # define a rate of 10Hz
    rate_hz = 10.0
    period = 1.0 / rate_hz

    # create the message to be published
    msg = Float64MultiArray()
    msg.data = []

    # initialize counter
    i_count = 0

    # loop until the node is shut down
    while rclpy.ok():
        # append current count to the message data
        msg.data.append(float(i_count))
        i_count += 1

        # publish the message
        pub.publish(msg)

        # print message data to the log
        node.get_logger().info(str(msg.data))

        # sleep to maintain the loop rate
        time.sleep(period)

    # clean up and shut down the node
    node.destroy_node()
    rclpy.shutdown()

def main():
    # keep the same structure: call talker() from main
    try:
        talker()
    except KeyboardInterrupt:
        exit(1)

if __name__ == '__main__':
    main()