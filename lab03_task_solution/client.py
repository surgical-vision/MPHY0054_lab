#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time

# Import the Rotmat2quat service type from lab03_task_solution.srv.
############################################################################
# BEGIN STUDENT CODE: service_imports



from lab03_task_solution.srv import Rotmat2quat



# END STUDENT CODE: service_imports
############################################################################


class Rot2QuatClient(Node):
    def __init__(self):
        super().__init__('rot2quat_client', namespace='lab03_task_solution')

        # Create self.client for the relative service name 'rotmat2quat'.
        ############################################################################
        # BEGIN STUDENT CODE: client_initialization



        self.client = self.create_client(Rotmat2quat, 'rotmat2quat')



        # END STUDENT CODE: client_initialization
        ############################################################################
        assert self.client in self.clients, "Store the client as self.client"

        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        """Create a proper 3x3 rotation matrix, send it and display the response."""
        ############################################################################
        # BEGIN STUDENT CODE: send_request



        request = Rotmat2quat.Request()
        request.r.data = random_rotation_matrix().ravel().tolist()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Request: {request.r.data}')
        response = future.result()
        if response is not None:
            self.get_logger().info(f'Response: {response}')
        else:
            self.get_logger().error('Service call failed.')
        return response



        # END STUDENT CODE: send_request
        ############################################################################


############################################################################
# BEGIN STUDENT CODE: helpers



def random_rotation_matrix():
    """Generate an orthogonal matrix with determinant +1."""
    rotation, _ = np.linalg.qr(np.random.normal(size=(3, 3)))
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1
    return rotation



# END STUDENT CODE: helpers
############################################################################


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Rot2QuatClient()
        while rclpy.ok():
            node.send_request()
            time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
