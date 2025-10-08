#!/usr/bin/env python3

import rclpy
import math
import random
import time
##TODO: include the library for a msg typed VectorStamped



def talker():
    ##TODO: Define a publisher, initialize the node, and define a rate
    

    # small helper to preserve the original `rate.sleep()` structure
    class Rate:
        def __init__(self, hz: float):
            self._period = 1.0 / hz
        def sleep(self):
            time.sleep(self._period)

    rate = Rate(10)  # 10hz

    ##TODO: Define the unit circle radius, and the message to be published
    

    while rclpy.ok():
        # The VectorStamped type needs a timestamp. This timestamp can be the current time.
        time_stamp = node.get_clock().now().to_msg()

        ##TODO: Define the position (x,y,z) on the unit circle
        

        ##TODO: Input the data into your message.
        

        ##TODO: Publish the message
        

    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        talker()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
