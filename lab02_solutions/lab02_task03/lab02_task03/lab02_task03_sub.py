#!/usr/bin/env python3
import rclpy
import random
from functools import partial
##TODO: include the library for a msg typed VectorStamped
from geometry_msgs.msg import Vector3Stamped


##TODO: complete your callback function.
def callback(msg, offsets):
    ##TODO: Add an offset to the incoming x and z coordinates.
    x_new = msg.vector.x + offsets[0]
    z_new = msg.vector.z + offsets[1]

    ##TODO: Print the updated coordinates.
    print('The current point is ')
    print(str(x_new))
    print(str(msg.vector.y))
    print(str(z_new))


def listener():
    ##TODO: Initialize the subscriber node.
    rclpy.init()
    node = rclpy.create_node('listener')

    ##TODO: Create two random numbers between 0 and 1.
    offset1 = random.randint(0, 1)
    offset2 = random.randint(0, 1)

    ##TODO: Define the subscriber with the offset arguments
    node.create_subscription(
        Vector3Stamped,
        "chatter",
        partial(callback, offsets=(offset1, offset2)),
        10
    )

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        listener()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
