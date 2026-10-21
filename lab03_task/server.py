#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# Import the Rotmat2quat service type from lab03_task.srv.
############################################################################
# BEGIN STUDENT CODE: service_imports






# END STUDENT CODE: service_imports
############################################################################


class RotationConverter(Node):
    def __init__(self):
        super().__init__('rotation_converter', namespace='lab03_task')

        # Create self.srv for the relative service name 'rotmat2quat'.
        ############################################################################
        # BEGIN STUDENT CODE: service_initialization



        raise NotImplementedError("Lab 03: initialize the rotation conversion service")



        # END STUDENT CODE: service_initialization
        ############################################################################
        assert self.srv in self.services, "Store the service as self.srv"


    def convert_rotmat2quat(self, request, response):
        """Callback ROS service function to convert a rotation matrix into a quaternion

        Args:
            request (Rotmat2quat.Request): lab03_task service message, containing
            the rotation matrix you need to convert.

        Returns:
            Rotmat2quat.Response: lab03_task service response, in which
            you store the requested quaternion
        """

        # Input: request.r.data contains nine row-major rotation entries.
        # Output: response.q is a finite unit quaternion in x, y, z, w order.
        ############################################################################
        # BEGIN STUDENT CODE: rotation_matrix_to_quaternion



        raise NotImplementedError("Lab 03: fill response.q from the rotation matrix")



        # END STUDENT CODE: rotation_matrix_to_quaternion
        ############################################################################
        values = [response.q.x, response.q.y, response.q.z, response.q.w]
        assert np.all(np.isfinite(values)), "Quaternion entries must be finite"
        assert np.isclose(np.linalg.norm(values), 1., atol=1e-6), "Quaternion must have unit norm"
        return response


############################################################################
# BEGIN STUDENT CODE: helpers






# END STUDENT CODE: helpers
############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RotationConverter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
