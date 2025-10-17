#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from lab03_example_msg.msg import Test


def subscriberCallback(msg):
    I = np.identity(3)
    theta = np.sqrt(
        np.power(msg.rotx.data, 2) +
        np.power(msg.roty.data, 2) +
        np.power(msg.rotz.data, 2)
    )
    omega_x = msg.rotx.data / theta
    omega_y = msg.roty.data / theta
    omega_z = msg.rotz.data / theta

    K = np.zeros((3, 3))
    K[0, 1] = -omega_z
    K[1, 0] = omega_z
    K[0, 2] = omega_y
    K[2, 0] = -omega_y
    K[1, 2] = -omega_x
    K[2, 1] = omega_x

    rot_max = I + np.sin(theta) * K + (1 - np.cos(theta)) * (np.dot(K, K))
    print(rot_max)


def rot_converter():
    rclpy.init(args=None)
    node = Node('rotation_subscriber')
    sub = node.create_subscription(Test, 'publish_rotation', subscriberCallback, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    try:
        rot_converter()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
