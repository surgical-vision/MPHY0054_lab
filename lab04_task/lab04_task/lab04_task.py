#!/usr/bin/env python3
# ROS2 Foxy – Forward Kinematics (Student Template, NO answers)
# NOTE:
#   - Keep DH geometry separate from encoder zero offsets.
#   - Offsets are listed below but NOT applied; you must decide how/where to use them.

import rclpy
from rclpy.node import Node
from math import pi
import numpy as np
from numpy import arctan2
from geometry_msgs.msg import Quaternion

# TODO: Import the message type that holds data describing robot joint angles (ROS2)
# from sensor_msgs.msg import JointState

# TODO: Import the class that publishes coordinate frame transform information (ROS2)
# from tf2_ros import TransformBroadcaster

# TODO: Import the message type that expresses a transform from one frame to another (ROS2)
# from geometry_msgs.msg import TransformStamped


# ==================== DH PARAMETERS ====================
# The OpenManipulator DH parameters
# a1 is 0.012 rather than 0 to align with the URDF file
a = [0.012, 0.13, 0.124, 0.07]
alpha = [pi/2, 0.0, 0.0, 0.0]
d = [0.075, 0.0, 0.0, 0.0]
theta = [0.0, -1.385, 1.385, 0.0]

# ==================== OFFSETS (LIST ONLY, DO NOT APPLY HERE) ====================
# TODO: Provide encoder zero offsets (example values to be replaced by students)

# TODO: If you have prismatic joints, you may also define d_offset
# d_offset = [0.0, 0.0, 0.0, 0.0]


# TODO: Define the frame names (ROS2)
# parent_frame = 'world'
# link_names = ['fkine_link_1', 'fkine_link_2', 'fkine_link_3', 'fkine_link_4']


def forward_kinematics(a, alpha, d, theta):
    """
    Compute a single homogeneous transformation matrix
    following the standard DH convention.
    """
    T = np.eye(4)
    T[0, 0] = np.cos(theta)
    T[0, 1] = -np.sin(theta) * np.cos(alpha)
    T[0, 2] = np.sin(theta) * np.sin(alpha)
    T[0, 3] = a * np.cos(theta)

    T[1, 0] = np.sin(theta)
    T[1, 1] = np.cos(theta) * np.cos(alpha)
    T[1, 2] = -np.cos(theta) * np.sin(alpha)
    T[1, 3] = a * np.sin(theta)

    T[2, 1] = np.sin(alpha)
    T[2, 2] = np.cos(alpha)
    T[2, 3] = d
    return T



def rotmat2q(T):
    """
    Convert a 3×3 rotation matrix to geometry_msgs/Quaternion.
    """
    q = Quaternion()
    trace = T[0, 0] + T[1, 1] + T[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q.w = 0.25 / s
        q.x = (T[2, 1] - T[1, 2]) * s
        q.y = (T[0, 2] - T[2, 0]) * s
        q.z = (T[1, 0] - T[0, 1]) * s
    else:
        if T[0, 0] > T[1, 1] and T[0, 0] > T[2, 2]:
            s = 2.0 * np.sqrt(1.0 + T[0, 0] - T[1, 1] - T[2, 2])
            q.w = (T[2, 1] - T[1, 2]) / s
            q.x = 0.25 * s
            q.y = (T[0, 1] + T[1, 0]) / s
            q.z = (T[0, 2] + T[2, 0]) / s
        elif T[1, 1] > T[2, 2]:
            s = 2.0 * np.sqrt(1.0 + T[1, 1] - T[0, 0] - T[2, 2])
            q.w = (T[0, 2] - T[2, 0]) / s
            q.x = (T[0, 1] + T[1, 0]) / s
            q.y = 0.25 * s
            q.z = (T[1, 2] + T[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + T[2, 2] - T[0, 0] - T[1, 1])
            q.w = (T[1, 0] - T[0, 1]) / s
            q.x = (T[0, 2] + T[2, 0]) / s
            q.y = (T[1, 2] + T[2, 1]) / s
            q.z = 0.25 * s
    return q


class FKinePublisher(Node):
    def __init__(self):
        super().__init__('open_forward_kinematic_node')

        # TODO: Initialise the broadcaster (ROS2)
        # self.br = TransformBroadcaster(self)

        # TODO: Initialise the subscriber (ROS2)
        # self.sub = self.create_subscription(JointState, '/joint_states', self.fkine_callback, 10)

        # Optional: info log
        # self.get_logger().info('FKinePublisher node started.')

    def fkine_callback(self, joint_msg):
        # TODO: Fill in this callback function (ROS2)
        # Access joint_msg.position (list of joint values)

        # This loop should iterate through all joints. In this simplified example, we are only using joint frame 1.
        for i in range(1):

            # TODO: For every joint, calculate the forward kinematics -> here hard-coded
            # T_i = forward_kinematics(a[i], alpha[i], d[i], theta[i])

            # Here we define the transform from joint frame 0 to joint frame 1 using the tf2 broadcaster.
            # TODO: Create and fill a TransformStamped (ROS2)
            # transform = TransformStamped()

            # TODO: Define the transform timestamp (ROS2)
            # transform.header.stamp = self.get_clock().now().to_msg()

            # TODO: Define the transform header frame (parent frame)
            # transform.header.frame_id = parent_frame

            # TODO: Define the transform child frame
            # transform.child_frame_id = link_names[i]

            # TODO: Populate the transform field. It consists of translation and rotation.
            # transform.transform.translation.x = float(T_i[0, 3])
            # transform.transform.translation.y = float(T_i[1, 3])
            # transform.transform.translation.z = float(T_i[2, 3])
            # transform.transform.rotation = rotmat2q(T_i)

            # TODO: Broadcast the transform to tf2 (ROS2)
            # self.br.sendTransform(transform)
            pass


def main(args=None):
    rclpy.init(args=args)

    # TODO: Create node instance
    # node = FKinePublisher()

    # TODO: Spin ROS2 executor
    # rclpy.spin(node)

    # Shutdown
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

