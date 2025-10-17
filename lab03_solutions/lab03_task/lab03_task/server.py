#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# TODO: Include all the required service classes
from lab03_ser.srv import Rotmat2quat

class RotationConverter(Node):
    def __init__(self):
        super().__init__('rotation_converter')

        # TODO: Initialise the service
        self.srv = self.create_service(Rotmat2quat, 'rotmat2quat', self.convert_rotmat2quat)

    def convert_rotmat2quat(self, request, response):
        """Callback ROS service function to convert a rotation matrix into a quaternion

        Args:
            request (Rotmat2quat.Request): lab03_task service message, containing
            the rotation matrix you need to convert.

        Returns:
            Rotmat2quat.Response: lab03_task service response, in which 
            you store the requested quaternion
        """

        # TODO: complete the function to transform a rotation matrix to quaternion
        rotation_matrix = np.array(request.r.data).reshape(3, 3)
        trace = np.trace(rotation_matrix)
        theta = np.arccos((trace - 1) / 2)

        if theta == 0:
            response.q.x = 0
            response.q.y = 0
            response.q.z = 0
            response.q.w = 1
        elif theta == np.pi or theta == -np.pi:
            K = 0.5 * (rotation_matrix + np.eye(3))
            sth2 = np.sin(theta / 2)
            response.q.x = np.sqrt(K[0, 0]) * sth2
            response.q.y = np.sqrt(K[1, 1]) * sth2
            response.q.z = np.sqrt(K[2, 2]) * sth2
            response.q.w = 0
        else:
            sth = np.sin(theta)
            sth2 = np.sin(theta / 2)
            cth2 = np.cos(theta / 2)

            rx = 1 / (2 * sth) * (rotation_matrix[2, 1] - rotation_matrix[1, 2])
            ry = 1 / (2 * sth) * (rotation_matrix[0, 2] - rotation_matrix[2, 0])
            rz = 1 / (2 * sth) * (rotation_matrix[1, 0] - rotation_matrix[0, 1])

            response.q.x = rx * sth2
            response.q.y = ry * sth2
            response.q.z = rz * sth2
            response.q.w = cth2

        return response

def main(args=None):
    rclpy.init(args=args)
    node = RotationConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
