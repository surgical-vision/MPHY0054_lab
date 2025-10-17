#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from lab03_ser.srv import Pointrot


def handle_point_rotation(request, response):
    px = request.p.x
    py = request.p.y
    pz = request.p.z

    qx = request.q.x
    qy = request.q.y
    qz = request.q.z
    qw = request.q.w

    
    qxs = np.power(qx, 2)
    qys = np.power(qy, 2)
    qzs = np.power(qz, 2)

 
    response.out_p.x = (
        px * (1 - 2 * qys - 2 * qzs)
        + py * (2 * (qx * qy - qz * qw))
        + pz * (2 * (qx * qz + qy * qw))
    )

    response.out_p.y = (
        px * (2 * (qx * qy + qz * qw))
        + py * (1 - 2 * qxs - 2 * qzs)
        + pz * (2 * (qy * qz - qx * qw))
    )

    response.out_p.z = (
        px * (2 * (qx * qz - qy * qw))
        + py * (2 * (qy * qz + qx * qw))
        + pz * (1 - 2 * qxs - 2 * qys)
    )

    return response


def rotation_point_service():
    rclpy.init()
    node = Node('rotate_point')
    srv = node.create_service(Pointrot, 'rotate_pt', handle_point_rotation)
    node.get_logger().info('Rotate point service is ready.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


def main():
    try:
        rotation_point_service()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
