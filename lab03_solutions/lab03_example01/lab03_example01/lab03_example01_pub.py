#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
from lab03_example_msg.msg import Test  
from std_msgs.msg import Float64


def rot_publisher():
    rclpy.init(args=None)
    node = Node('rotation_publisher')

    pub = node.create_publisher(Test, 'publish_rotation', 100)

    rot_msg = Test()

    
    def timer_callback():
        rot_msg.rotx = Float64()
        rot_msg.roty = Float64()
        rot_msg.rotz = Float64()
        rot_msg.rotx.data = random.uniform(-2.0, 2.0)
        rot_msg.roty.data = random.uniform(-1.0, 1.0)
        rot_msg.rotz.data = random.uniform(-2.1, 1.8)

        pub.publish(rot_msg)
        node.get_logger().info(
            f'Publishing: rotx={rot_msg.rotx.data:.2f}, roty={rot_msg.roty.data:.2f}, rotz={rot_msg.rotz.data:.2f}'
        )

    timer_period = 0.1  
    timer = node.create_timer(timer_period, timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    try:
        rot_publisher()  
    except KeyboardInterrupt:
        pass
    
if __name__ == '__main__':
    main()
