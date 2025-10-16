#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# TODO: Include all the required service classes


class RotationConverter(Node):
    def __init__(self):
        super().__init__('rotation_converter')

        # TODO: Initialise the service
        

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
        




        
def main(args=None):
    rclpy.init(args=args)
    node = RotationConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
