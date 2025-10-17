#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time

from lab03_ser.srv import Pointrot


def point_rotation_client():
    rclpy.init(args=None)
    node = Node('point_rotation_client')

    node.get_logger().info('Waiting for service rotate_pt...')
    client = node.create_client(Pointrot, 'rotate_pt')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = Pointrot.Request()

    while rclpy.ok():

        request.p.x = random.uniform(-2.0, 2.0)
        request.p.y = random.uniform(-2.0, 2.0)
        request.p.z = random.uniform(-2.0, 2.0)

        print(request)

        quaternion = np.random.rand(4)
        quaternion /= np.linalg.norm(quaternion)

        request.q.x = quaternion[0]
        request.q.y = quaternion[1]
        request.q.z = quaternion[2]
        request.q.w = quaternion[3]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()
            print(response)
        else:
            node.get_logger().error('Service call failed')

        time.sleep(3)

    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    try:
        point_rotation_client()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
