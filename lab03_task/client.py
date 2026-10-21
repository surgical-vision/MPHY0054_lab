#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time

# Import the Rotmat2quat service type from lab03_task.srv.
############################################################################
# BEGIN STUDENT CODE: service_imports






# END STUDENT CODE: service_imports
############################################################################


class Rot2QuatClient(Node):
    def __init__(self):
        super().__init__('rot2quat_client', namespace='lab03_task')

        # Create self.client for the relative service name 'rotmat2quat'.
        ############################################################################
        # BEGIN STUDENT CODE: client_initialization



        raise NotImplementedError("Lab 03: initialize the rotation conversion client")



        # END STUDENT CODE: client_initialization
        ############################################################################
        assert self.client in self.clients, "Store the client as self.client"

        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        """Create a proper 3x3 rotation matrix, send it and display the response."""
        ############################################################################
        # BEGIN STUDENT CODE: send_request



        raise NotImplementedError("Lab 03: create, send and display a service request")



        # END STUDENT CODE: send_request
        ############################################################################


############################################################################
# BEGIN STUDENT CODE: helpers






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
