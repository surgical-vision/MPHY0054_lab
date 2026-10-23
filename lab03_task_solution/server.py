#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# Import the Rotmat2quat service type from lab03_task_solution.srv.
############################################################################
# BEGIN STUDENT CODE: service_imports



from lab03_task_solution.srv import Rotmat2quat



# END STUDENT CODE: service_imports
############################################################################


class RotationConverter(Node):
    def __init__(self):
        super().__init__('rotation_converter', namespace='lab03_task_solution')

        # Create self.srv for the relative service name 'rotmat2quat'.
        ############################################################################
        # BEGIN STUDENT CODE: service_initialization



        self.srv = self.create_service(Rotmat2quat, 'rotmat2quat', self.convert_rotmat2quat)



        # END STUDENT CODE: service_initialization
        ############################################################################
        assert self.srv in self.services, "Store the service as self.srv"


    def convert_rotmat2quat(self, request, response):
        """Callback ROS service function to convert a rotation matrix into a quaternion

        Args:
            request (Rotmat2quat.Request): lab03_task_solution service message, containing
            the rotation matrix you need to convert.

        Returns:
            Rotmat2quat.Response: lab03_task_solution service response, in which
            you store the requested quaternion
        """

        # Input: request.r.data contains nine row-major rotation entries.
        # Output: response.q is a finite unit quaternion in x, y, z, w order.
        ############################################################################
        # BEGIN STUDENT CODE: rotation_matrix_to_quaternion



        rotation = np.asarray(request.r.data, dtype=float).reshape(3, 3)
        quaternion = rotation_matrix_to_quaternion(rotation)
        response.q.x, response.q.y, response.q.z, response.q.w = map(float, quaternion)



        # END STUDENT CODE: rotation_matrix_to_quaternion
        ############################################################################
        values = [response.q.x, response.q.y, response.q.z, response.q.w]
        assert np.all(np.isfinite(values)), "Quaternion entries must be finite"
        assert np.isclose(np.linalg.norm(values), 1., atol=1e-6), "Quaternion must have unit norm"
        return response


############################################################################
# BEGIN STUDENT CODE: helpers



def rotation_matrix_to_quaternion(rotation):
    """Return a unit quaternion in x, y, z, w order, including half turns."""
    rotation = np.asarray(rotation, dtype=float)
    if (rotation.shape != (3, 3) or not np.all(np.isfinite(rotation))
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-6, rtol=0)):
        raise ValueError('Expected a finite, proper 3x3 rotation matrix')
    r = rotation
    matrix = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ]) / 3.
    _, vectors = np.linalg.eigh(matrix)
    quaternion = vectors[:, -1]
    if quaternion[3] < 0:
        quaternion = -quaternion
    return quaternion



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
