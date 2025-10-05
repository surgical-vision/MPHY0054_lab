import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('hello_node')
    node.get_logger().info('Hello from ROS 2 Foxy!')
    node.destroy_node()
    rclpy.shutdown()

