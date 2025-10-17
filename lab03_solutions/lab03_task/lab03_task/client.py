#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request
from lab03_ser.srv import Rotmat2quat  # ROS2 naming convention (CamelCase)

class Rot2QuatClient(Node):
    def __init__(self):
        super().__init__('rot2quat_client')

        #TODO: Initialise the ROS service client. It takes two arguments: The name of the service, and the service definition.
        self.client = self.create_client(Rotmat2quat, 'rotmat2quat')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        #TODO: create a random request matrix
        request = Rotmat2quat.Request()

        request.r.data = np.random.uniform(-1, 1, 9).tolist()

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'Request: {request.r.data}')

        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
    rclpy.init(args=args)
    node = Rot2QuatClient()

    try:
        while rclpy.ok():
            node.send_request()
            time.sleep(3)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
