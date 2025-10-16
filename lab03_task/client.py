#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time

#TODO: import the SRV file from its corresponding folder, as well as its Request


class Rot2QuatClient(Node):
    def __init__(self):
        super().__init__('rot2quat_client')

        #TODO: Initialise the ROS service client. It takes two arguments: The name of the service, and the service definition.
        

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_request(self):
        #TODO: create a random request matrix
        


        

        

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
